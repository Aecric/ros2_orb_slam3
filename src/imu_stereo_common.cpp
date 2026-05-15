// ORB-SLAM3 IMU_STEREO wrapper for Intel RealSense D435i.
//
// Subscribes to:
//   - left  IR image  (default /camera/cam_head/infra1/image_rect_raw)
//   - right IR image  (default /camera/cam_head/infra2/image_rect_raw)
//   - IMU            (default /camera/cam_head/imu, expects unite_imu_method:=2)
//
// Stereo pairs are synchronized with message_filters::ApproximateTime. The IMU runs at a
// higher rate than the cameras, so its samples are buffered in a thread-safe deque and
// drained up to the current stereo timestamp before each TrackStereo call.

#include "ros2_orb_slam3/imu_stereo_common.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

ImuStereoMode::ImuStereoMode() : Node("imu_stereo_node_cpp")
{
    this->declare_parameter<std::string>("voc_file_arg", "");
    this->declare_parameter<std::string>("settings_file_path_arg", "");
    this->declare_parameter<std::string>("left_image_topic",
                                         "/camera/cam_head/infra1/image_rect_raw");
    this->declare_parameter<std::string>("right_image_topic",
                                         "/camera/cam_head/infra2/image_rect_raw");
    this->declare_parameter<std::string>("imu_topic", "/camera/cam_head/imu");
    this->declare_parameter<bool>("enable_pangolin", true);
    this->declare_parameter<int>("sync_queue_size", 30);

    vocFilePath_      = this->get_parameter("voc_file_arg").as_string();
    settingsFilePath_ = this->get_parameter("settings_file_path_arg").as_string();
    leftTopic_        = this->get_parameter("left_image_topic").as_string();
    rightTopic_       = this->get_parameter("right_image_topic").as_string();
    imuTopic_         = this->get_parameter("imu_topic").as_string();
    enablePangolin_   = this->get_parameter("enable_pangolin").as_bool();
    const int syncQueueSize = this->get_parameter("sync_queue_size").as_int();

    const char * homeEnv = std::getenv("HOME");
    const std::string homeDir = homeEnv ? std::string(homeEnv) : std::string();
    const std::string packageRoot = homeDir + "/orb_slam3/src/ros2_orb_slam3";
    if (vocFilePath_.empty()) {
        vocFilePath_ = packageRoot + "/orb_slam3/Vocabulary/ORBvoc.txt.bin";
    }
    if (settingsFilePath_.empty()) {
        settingsFilePath_ = packageRoot + "/orb_slam3/config/Stereo-Inertial/RealSense_D435i.yaml";
    }

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 IMU_STEREO node starting");
    RCLCPP_INFO(this->get_logger(), "  voc_file      : %s", vocFilePath_.c_str());
    RCLCPP_INFO(this->get_logger(), "  settings_file : %s", settingsFilePath_.c_str());
    RCLCPP_INFO(this->get_logger(), "  left_topic    : %s", leftTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  right_topic   : %s", rightTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  imu_topic     : %s", imuTopic_.c_str());

    pAgent_ = std::make_unique<ORB_SLAM3::System>(
        vocFilePath_, settingsFilePath_, ORB_SLAM3::System::IMU_STEREO, enablePangolin_);

    // IMU subscriber: SensorDataQoS is best-effort + small queue, matches realsense2_camera.
    imuSub_ = this->create_subscription<ImuMsg>(
        imuTopic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuStereoMode::imuCallback, this, _1));

    // Stereo subscribers + ApproximateTime sync.
    leftSub_.subscribe(this, leftTopic_);
    rightSub_.subscribe(this, rightTopic_);
    sync_ = std::make_shared<message_filters::Synchronizer<StereoSyncPolicy>>(
        StereoSyncPolicy(syncQueueSize), leftSub_, rightSub_);
    sync_->registerCallback(std::bind(&ImuStereoMode::stereoCallback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Waiting for synchronized stereo + IMU stream ...");
}

ImuStereoMode::~ImuStereoMode()
{
    if (pAgent_) {
        pAgent_->Shutdown();
    }
}

void ImuStereoMode::imuCallback(const ImuMsg::ConstSharedPtr msg)
{
    const double t = static_cast<double>(msg->header.stamp.sec) +
                     static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    ORB_SLAM3::IMU::Point sample(
        static_cast<float>(msg->linear_acceleration.x),
        static_cast<float>(msg->linear_acceleration.y),
        static_cast<float>(msg->linear_acceleration.z),
        static_cast<float>(msg->angular_velocity.x),
        static_cast<float>(msg->angular_velocity.y),
        static_cast<float>(msg->angular_velocity.z),
        t);

    std::lock_guard<std::mutex> lock(imuMutex_);
    imuBuffer_.push_back(sample);
}

std::vector<ORB_SLAM3::IMU::Point> ImuStereoMode::drainImuUntil(double image_time)
{
    std::vector<ORB_SLAM3::IMU::Point> out;
    std::lock_guard<std::mutex> lock(imuMutex_);
    while (!imuBuffer_.empty() && imuBuffer_.front().t <= image_time) {
        out.push_back(imuBuffer_.front());
        imuBuffer_.pop_front();
    }
    return out;
}

void ImuStereoMode::stereoCallback(const ImageMsg::ConstSharedPtr & left_msg,
                                   const ImageMsg::ConstSharedPtr & right_msg)
{
    cv_bridge::CvImageConstPtr left_ptr;
    cv_bridge::CvImageConstPtr right_ptr;
    try {
        left_ptr  = cv_bridge::toCvShare(left_msg,  sensor_msgs::image_encodings::MONO8);
        right_ptr = cv_bridge::toCvShare(right_msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge stereo conversion failed: %s", e.what());
        return;
    }

    const double timestamp = static_cast<double>(left_msg->header.stamp.sec) +
                             static_cast<double>(left_msg->header.stamp.nanosec) * 1e-9;

    std::vector<ORB_SLAM3::IMU::Point> imu_meas = drainImuUntil(timestamp);

    if (imu_meas.empty()) {
        // Without IMU samples ORB-SLAM3 cannot initialise the inertial state. Skip the
        // frame; the next one will pick up the accumulated IMU readings.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No IMU samples available for stereo frame @ %.6f s — skipping", timestamp);
        return;
    }

    pAgent_->TrackStereo(left_ptr->image, right_ptr->image, timestamp, imu_meas);
}
