#pragma once

#include <array>
#include <fstream>
#include <memory>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>

#include "gnss_imu_wheel_localizer/extended_kalman_filter.hpp"

namespace gnss_imu_wheel_localizer
{

class GnssImuWheelLocalizerNode : public rclcpp::Node
{
public:
  explicit GnssImuWheelLocalizerNode(const rclcpp::NodeOptions & options);

private:
  struct Parameters
  {
    std::string gnss_topic;
    std::string imu_topic;
    std::string wheel_odom_topic;
    std::string velocity_report_topic;

    std::string map_frame;
    std::string odom_frame;
    std::string base_link_frame;

    bool publish_tf{true};
    bool use_3d_position{false};
    bool use_attitude{false};

    bool override_origin{false};
    double origin_latitude{0.0};
    double origin_longitude{0.0};
    double origin_altitude{0.0};

    bool enable_pose_csv_logging{false};
    std::string pose_csv_path;

    std::array<double, ExtendedKalmanFilter::kStateDim> process_noise_diagonal{};
    std::array<double, ExtendedKalmanFilter::kStateDim> initial_covariance_diagonal{};

    std::array<double, 3> gnss_position_noise{};
    double wheel_velocity_noise{1.0};
    double imu_yaw_noise{0.1};
    double imu_attitude_noise{0.1};
  };

  void declareAndLoadParameters();

  void handleGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void handleWheelOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleVelocityReport(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

  void initializeOriginIfNeeded(const sensor_msgs::msg::NavSatFix & msg);
  void maybeInitializeFilter(
    const sensor_msgs::msg::NavSatFix & msg,
    double enu_x,
    double enu_y,
    double enu_z,
    const rclcpp::Time & stamp);

  void predictToStamp(const rclcpp::Time & stamp);
  void publishOutputs(const rclcpp::Time & stamp);
  void maybeWritePoseCsv(
    const rclcpp::Time & stamp,
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw,
    double velocity);

  ExtendedKalmanFilter::CovarianceMatrix makeDiagonalMatrix(
    const std::array<double, ExtendedKalmanFilter::kStateDim> & diagonal) const;

  Parameters params_;

  ExtendedKalmanFilter ekf_;
  ExtendedKalmanFilter::ProcessModelInput process_input_{};
  ExtendedKalmanFilter::CovarianceMatrix process_noise_;
  ExtendedKalmanFilter::CovarianceMatrix initial_covariance_;

  GeographicLib::LocalCartesian local_cartesian_;
  bool origin_initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double latest_wheel_velocity_{0.0};
  bool wheel_velocity_available_{false};

  double latest_roll_{0.0};
  double latest_pitch_{0.0};
  double latest_yaw_{0.0};
  bool attitude_available_{false};

  std::ofstream pose_csv_stream_;
  bool pose_csv_header_written_{false};
};

}  // namespace gnss_imu_wheel_localizer
