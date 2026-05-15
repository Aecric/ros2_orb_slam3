#include "ros2_orb_slam3/imu_stereo_common.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuStereoMode>();

    // IMU runs at ~200 Hz, stereo at 30 Hz — use a MultiThreadedExecutor so the IMU
    // callback is never starved by image processing.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
