#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "gnss_imu_wheel_localizer/gnss_imu_wheel_localizer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<gnss_imu_wheel_localizer::GnssImuWheelLocalizerNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
