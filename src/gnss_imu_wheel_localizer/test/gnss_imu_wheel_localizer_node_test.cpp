#include <memory>

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define private public
#include "gnss_imu_wheel_localizer/gnss_imu_wheel_localizer_node.hpp"
#undef private

namespace giwl = gnss_imu_wheel_localizer;

class GnssImuWheelLocalizerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(GnssImuWheelLocalizerNodeTest, EkfInitializesAfterSensorMessages)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-p", "publish_tf:=false"});

  auto node = std::make_shared<giwl::GnssImuWheelLocalizerNode>(options);

  auto wheel_msg = std::make_shared<nav_msgs::msg::Odometry>();
  wheel_msg->header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  wheel_msg->twist.twist.linear.x = 2.0;
  node->handleWheelOdometry(wheel_msg);

  auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
  imu_msg->header.stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.3);
  imu_msg->orientation = tf2::toMsg(q);
  imu_msg->angular_velocity.z = 0.0;
  imu_msg->linear_acceleration.x = 0.0;
  node->handleImu(imu_msg);

  auto gnss_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  gnss_msg->header.stamp = rclcpp::Time(3, 0, RCL_ROS_TIME);
  gnss_msg->latitude = 35.0;
  gnss_msg->longitude = 139.0;
  gnss_msg->altitude = 10.0;
  gnss_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  node->handleGnss(gnss_msg);

  ASSERT_TRUE(node->ekf_.isInitialized());
  const auto & state = node->ekf_.getState();

  EXPECT_NEAR(state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::X)), 0.0, 1.0e-6);
  EXPECT_NEAR(state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::Y)), 0.0, 1.0e-6);
  EXPECT_NEAR(state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::YAW)), 0.3, 1.0e-2);
  EXPECT_NEAR(state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::VELOCITY)), 2.0, 1.0e-6);
}
