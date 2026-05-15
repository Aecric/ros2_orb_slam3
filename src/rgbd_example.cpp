// Entry point for the ORB-SLAM3 RGB-D node.

#include "ros2_orb_slam3/rgbd_common.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
