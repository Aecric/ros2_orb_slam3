#ifndef RGBD_COMMON_HPP
#define RGBD_COMMON_HPP

#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

#include "System.h"

class RgbdMode : public rclcpp::Node
{
public:
    RgbdMode();
    ~RgbdMode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

    std::string nodeName_;
    std::string vocFilePath_;
    std::string settingsFilePath_;
    std::string rgbTopic_;
    std::string depthTopic_;
    bool enablePangolin_;

    std::unique_ptr<ORB_SLAM3::System> pAgent_;

    message_filters::Subscriber<ImageMsg> rgbSub_;
    message_filters::Subscriber<ImageMsg> depthSub_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

    void grabRgbd(const ImageMsg::ConstSharedPtr & rgb_msg,
                  const ImageMsg::ConstSharedPtr & depth_msg);
};

#endif  // RGBD_COMMON_HPP
