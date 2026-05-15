// ORB-SLAM3 RGB-D wrapper for Intel RealSense D435 / D435i.
// Subscribes to a color stream and an aligned depth stream, runs them through
// message_filters::ApproximateTime, and feeds each synchronized pair into
// ORB_SLAM3::System::TrackRGBD.

#include "ros2_orb_slam3/rgbd_common.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

RgbdMode::RgbdMode() : Node("rgbd_node_cpp")
{
    this->declare_parameter<std::string>("node_name_arg", "rgbd_slam_cpp");
    this->declare_parameter<std::string>("voc_file_arg", "");
    this->declare_parameter<std::string>("settings_file_path_arg", "");
    this->declare_parameter<std::string>("rgb_image_topic", "/camera/cam_head/color/image_raw");
    this->declare_parameter<std::string>("depth_image_topic",
                                         "/camera/cam_head/aligned_depth_to_color/image_raw");
    this->declare_parameter<bool>("enable_pangolin", true);
    this->declare_parameter<int>("sync_queue_size", 30);

    nodeName_ = this->get_parameter("node_name_arg").as_string();
    vocFilePath_ = this->get_parameter("voc_file_arg").as_string();
    settingsFilePath_ = this->get_parameter("settings_file_path_arg").as_string();
    rgbTopic_ = this->get_parameter("rgb_image_topic").as_string();
    depthTopic_ = this->get_parameter("depth_image_topic").as_string();
    enablePangolin_ = this->get_parameter("enable_pangolin").as_bool();
    const int syncQueueSize = this->get_parameter("sync_queue_size").as_int();

    // Mirror the path convention used by the existing mono node: workspace must be ~/orb_slam3.
    const char * homeEnv = std::getenv("HOME");
    const std::string homeDir = homeEnv ? std::string(homeEnv) : std::string();
    const std::string packageRoot = homeDir + "/orb_slam3/src/ros2_orb_slam3";
    if (vocFilePath_.empty()) {
        vocFilePath_ = packageRoot + "/orb_slam3/Vocabulary/ORBvoc.txt.bin";
    }
    if (settingsFilePath_.empty()) {
        settingsFilePath_ = packageRoot + "/orb_slam3/config/RGB-D/RealSense_D435i.yaml";
    }

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 RGB-D node starting");
    RCLCPP_INFO(this->get_logger(), "  node_name      : %s", nodeName_.c_str());
    RCLCPP_INFO(this->get_logger(), "  voc_file       : %s", vocFilePath_.c_str());
    RCLCPP_INFO(this->get_logger(), "  settings_file  : %s", settingsFilePath_.c_str());
    RCLCPP_INFO(this->get_logger(), "  rgb_topic      : %s", rgbTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  depth_topic    : %s", depthTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  pangolin viewer: %s", enablePangolin_ ? "on" : "off");

    pAgent_ = std::make_unique<ORB_SLAM3::System>(
        vocFilePath_, settingsFilePath_, ORB_SLAM3::System::RGBD, enablePangolin_);

    rgbSub_.subscribe(this, rgbTopic_);
    depthSub_.subscribe(this, depthTopic_);
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(syncQueueSize), rgbSub_, depthSub_);
    sync_->registerCallback(std::bind(&RgbdMode::grabRgbd, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Waiting for synchronized RGB + depth pairs ...");
}

RgbdMode::~RgbdMode()
{
    if (pAgent_) {
        pAgent_->Shutdown();
    }
}

void RgbdMode::grabRgbd(const ImageMsg::ConstSharedPtr & rgb_msg,
                        const ImageMsg::ConstSharedPtr & depth_msg)
{
    cv_bridge::CvImageConstPtr rgb_ptr;
    cv_bridge::CvImageConstPtr depth_ptr;

    try {
        rgb_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge RGB conversion failed: %s", e.what());
        return;
    }

    try {
        // Keep the depth in its native encoding (16UC1 mm for aligned_depth_to_color).
        // ORB-SLAM3 applies RGBD.DepthMapFactor internally.
        depth_ptr = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge depth conversion failed: %s", e.what());
        return;
    }

    const double timestamp = static_cast<double>(rgb_msg->header.stamp.sec) +
                             static_cast<double>(rgb_msg->header.stamp.nanosec) * 1e-9;

    pAgent_->TrackRGBD(rgb_ptr->image, depth_ptr->image, timestamp);
}
