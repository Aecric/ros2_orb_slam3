#ifndef IMU_STEREO_COMMON_HPP
#define IMU_STEREO_COMMON_HPP

#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

#include "System.h"
#include "ImuTypes.h"

class ImuStereoMode : public rclcpp::Node
{
public:
    ImuStereoMode();
    ~ImuStereoMode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg = sensor_msgs::msg::Imu;
    using StereoSyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

    // Parameters
    std::string vocFilePath_;
    std::string settingsFilePath_;
    std::string leftTopic_;
    std::string rightTopic_;
    std::string imuTopic_;
    bool enablePangolin_;

    // SLAM
    std::unique_ptr<ORB_SLAM3::System> pAgent_;

    // Subscribers
    message_filters::Subscriber<ImageMsg> leftSub_;
    message_filters::Subscriber<ImageMsg> rightSub_;
    std::shared_ptr<message_filters::Synchronizer<StereoSyncPolicy>> sync_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imuSub_;

    // IMU buffer drained at each stereo frame
    std::mutex imuMutex_;
    std::deque<ORB_SLAM3::IMU::Point> imuBuffer_;

    void imuCallback(const ImuMsg::ConstSharedPtr msg);
    void stereoCallback(const ImageMsg::ConstSharedPtr & left_msg,
                        const ImageMsg::ConstSharedPtr & right_msg);
    std::vector<ORB_SLAM3::IMU::Point> drainImuUntil(double image_time);
};

#endif  // IMU_STEREO_COMMON_HPP
