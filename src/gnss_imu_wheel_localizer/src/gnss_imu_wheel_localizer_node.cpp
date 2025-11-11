#include "gnss_imu_wheel_localizer/gnss_imu_wheel_localizer_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gnss_imu_wheel_localizer
{

namespace
{
constexpr double kLargeVariance = 1.0e6;

bool isValidGnssFix(const sensor_msgs::msg::NavSatFix & msg)
{
  const bool finite_position = std::isfinite(msg.latitude) && std::isfinite(msg.longitude) &&
                               std::isfinite(msg.altitude);
  const bool has_fix = msg.status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  return finite_position && has_fix;
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  tf2::Matrix3x3 mat(q);
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  mat.getRPY(roll, pitch, yaw);
  return yaw;
}

std::tuple<double, double, double> quaternionToRPY(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  tf2::Matrix3x3 mat(q);
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  mat.getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

}  // namespace

GnssImuWheelLocalizerNode::GnssImuWheelLocalizerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("gnss_imu_wheel_localizer", options)
{
  declareAndLoadParameters();

  process_noise_ = makeDiagonalMatrix(params_.process_noise_diagonal);
  initial_covariance_ = makeDiagonalMatrix(params_.initial_covariance_diagonal);

  if (params_.override_origin) {
    local_cartesian_.Reset(
      params_.origin_latitude, params_.origin_longitude, params_.origin_altitude);
    origin_initialized_ = true;
  }

  const auto sensor_qos = rclcpp::SensorDataQoS();

  gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    params_.gnss_topic, sensor_qos,
    std::bind(&GnssImuWheelLocalizerNode::handleGnss, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    params_.imu_topic, sensor_qos,
    std::bind(&GnssImuWheelLocalizerNode::handleImu, this, std::placeholders::_1));

  wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    params_.wheel_odom_topic, sensor_qos,
    std::bind(&GnssImuWheelLocalizerNode::handleWheelOdometry, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 10);

  if (params_.publish_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
}

void GnssImuWheelLocalizerNode::declareAndLoadParameters()
{
  params_.gnss_topic = declare_parameter<std::string>("gnss_topic", "/sensing/gnss/fix");
  params_.imu_topic = declare_parameter<std::string>("imu_topic", "/sensing/imu/imu_data");
  params_.wheel_odom_topic = declare_parameter<std::string>("wheel_odom_topic", "/localization/kinematic_state");

  params_.map_frame = declare_parameter<std::string>("map_frame", "map");
  params_.odom_frame = declare_parameter<std::string>("odom_frame", "odom");
  params_.base_link_frame = declare_parameter<std::string>("base_link_frame", "base_link");

  params_.publish_tf = declare_parameter<bool>("publish_tf", true);
  params_.use_3d_position = declare_parameter<bool>("use_3d_position", false);
  params_.use_attitude = declare_parameter<bool>("use_attitude", false);

  params_.override_origin = declare_parameter<bool>("override_origin", false);
  params_.origin_latitude = declare_parameter<double>("origin_latitude", 0.0);
  params_.origin_longitude = declare_parameter<double>("origin_longitude", 0.0);
  params_.origin_altitude = declare_parameter<double>("origin_altitude", 0.0);

  const auto process_noise_diagonal = declare_parameter<std::vector<double>>(
    "process_noise_diagonal", {0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 1.0});
  const auto initial_covariance_diagonal = declare_parameter<std::vector<double>>(
    "initial_covariance_diagonal", {5.0, 5.0, 5.0, 0.5, 0.5, 0.5, 1.0});
  const auto gnss_position_noise = declare_parameter<std::vector<double>>(
    "gnss_position_noise", {1.0, 1.0, 2.0});

  params_.wheel_velocity_noise = declare_parameter<double>("wheel_velocity_noise", 0.5);
  params_.imu_yaw_noise = declare_parameter<double>("imu_yaw_noise", 0.1);
  params_.imu_attitude_noise = declare_parameter<double>("imu_attitude_noise", 0.1);

  const auto assignDiagonal = [](auto & target_array, const std::vector<double> & source) {
    const std::size_t n = std::min(target_array.size(), source.size());
    std::copy_n(source.begin(), n, target_array.begin());
    for (std::size_t i = n; i < target_array.size(); ++i) {
      target_array[i] = 1.0;
    }
  };

  assignDiagonal(params_.process_noise_diagonal, process_noise_diagonal);
  assignDiagonal(params_.initial_covariance_diagonal, initial_covariance_diagonal);

  params_.gnss_position_noise.fill(1.0);
  for (std::size_t i = 0; i < std::min<std::size_t>(params_.gnss_position_noise.size(), gnss_position_noise.size()); ++i) {
    params_.gnss_position_noise[i] = gnss_position_noise[i];
  }
}

void GnssImuWheelLocalizerNode::handleGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!isValidGnssFix(*msg)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received invalid GNSS fix");
    return;
  }

  initializeOriginIfNeeded(*msg);
  if (!origin_initialized_) {
    RCLCPP_WARN(get_logger(), "GNSS origin is not initialized yet.");
    return;
  }

  double enu_x{0.0};
  double enu_y{0.0};
  double enu_z{0.0};
  local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, enu_x, enu_y, enu_z);

  maybeInitializeFilter(*msg, enu_x, enu_y, enu_z, msg->header.stamp);
  if (!ekf_.isInitialized()) {
    return;
  }

  predictToStamp(msg->header.stamp);

  if (params_.use_3d_position) {
    Eigen::Matrix<double, 3, 1> measurement;
    measurement << enu_x, enu_y, enu_z;

    Eigen::Matrix<double, 3, ExtendedKalmanFilter::kStateDim> H = Eigen::Matrix<double, 3, ExtendedKalmanFilter::kStateDim>::Zero();
    H(0, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::X)) = 1.0;
    H(1, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Y)) = 1.0;
    H(2, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Z)) = 1.0;

    Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();
    for (int i = 0; i < 3; ++i) {
      R(i, i) = params_.gnss_position_noise[i];
    }

    ekf_.update(measurement, H, R);
  } else {
    Eigen::Matrix<double, 2, 1> measurement;
    measurement << enu_x, enu_y;

    Eigen::Matrix<double, 2, ExtendedKalmanFilter::kStateDim> H = Eigen::Matrix<double, 2, ExtendedKalmanFilter::kStateDim>::Zero();
    H(0, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::X)) = 1.0;
    H(1, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Y)) = 1.0;

    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Zero();
    R(0, 0) = params_.gnss_position_noise[0];
    R(1, 1) = params_.gnss_position_noise[1];

    ekf_.update(measurement, H, R);
  }

  publishOutputs(msg->header.stamp);
}

void GnssImuWheelLocalizerNode::handleImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  process_input_.yaw_rate = msg->angular_velocity.z;
  process_input_.longitudinal_acceleration = msg->linear_acceleration.x;

  const auto [roll, pitch, yaw] = quaternionToRPY(msg->orientation);
  latest_roll_ = roll;
  latest_pitch_ = pitch;
  latest_yaw_ = yaw;
  attitude_available_ = true;

  if (!ekf_.isInitialized()) {
    return;
  }

  predictToStamp(msg->header.stamp);

  Eigen::Matrix<double, 1, 1> measurement;
  measurement << yaw;

  Eigen::Matrix<double, 1, ExtendedKalmanFilter::kStateDim> H = Eigen::Matrix<double, 1, ExtendedKalmanFilter::kStateDim>::Zero();
  H(0, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::YAW)) = 1.0;

  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = params_.imu_yaw_noise;

  ekf_.update(measurement, H, R);

  if (params_.use_attitude) {
    Eigen::Matrix<double, 2, 1> attitude_measurement;
    attitude_measurement << roll, pitch;

    Eigen::Matrix<double, 2, ExtendedKalmanFilter::kStateDim> H_att =
      Eigen::Matrix<double, 2, ExtendedKalmanFilter::kStateDim>::Zero();
    H_att(0, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::ROLL)) = 1.0;
    H_att(1, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::PITCH)) = 1.0;

    Eigen::Matrix<double, 2, 2> R_att = Eigen::Matrix<double, 2, 2>::Zero();
    R_att(0, 0) = params_.imu_attitude_noise;
    R_att(1, 1) = params_.imu_attitude_noise;

    ekf_.update(attitude_measurement, H_att, R_att);
  }

  publishOutputs(msg->header.stamp);
}

void GnssImuWheelLocalizerNode::handleWheelOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double velocity = msg->twist.twist.linear.x;
  if (std::isfinite(velocity)) {
    process_input_.linear_velocity = velocity;
    latest_wheel_velocity_ = velocity;
    wheel_velocity_available_ = true;
  }

  if (!ekf_.isInitialized()) {
    return;
  }

  predictToStamp(msg->header.stamp);

  Eigen::Matrix<double, 1, 1> measurement;
  measurement << latest_wheel_velocity_;

  Eigen::Matrix<double, 1, ExtendedKalmanFilter::kStateDim> H = Eigen::Matrix<double, 1, ExtendedKalmanFilter::kStateDim>::Zero();
  H(0, static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::VELOCITY)) = 1.0;

  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = params_.wheel_velocity_noise;

  ekf_.update(measurement, H, R);

  publishOutputs(msg->header.stamp);
}

void GnssImuWheelLocalizerNode::initializeOriginIfNeeded(const sensor_msgs::msg::NavSatFix & msg)
{
  if (origin_initialized_) {
    return;
  }

  if (params_.override_origin) {
    local_cartesian_.Reset(
      params_.origin_latitude, params_.origin_longitude, params_.origin_altitude);
    origin_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Origin overridden by parameters: lat=%.8f lon=%.8f alt=%.2f",
      params_.origin_latitude, params_.origin_longitude, params_.origin_altitude);
    return;
  }

  if (!isValidGnssFix(msg)) {
    return;
  }

  local_cartesian_.Reset(msg.latitude, msg.longitude, msg.altitude);
  origin_initialized_ = true;
  RCLCPP_INFO(get_logger(), "GNSS origin initialized at lat=%.8f lon=%.8f alt=%.2f",
    msg.latitude, msg.longitude, msg.altitude);
}

void GnssImuWheelLocalizerNode::maybeInitializeFilter(
  const sensor_msgs::msg::NavSatFix & msg,
  const double enu_x,
  const double enu_y,
  const double enu_z,
  const rclcpp::Time & stamp)
{
  if (ekf_.isInitialized()) {
    return;
  }

  ExtendedKalmanFilter::StateVector state;
  state.setZero();
  state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::X)) = enu_x;
  state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Y)) = enu_y;
  state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Z)) = params_.use_3d_position ? enu_z : 0.0;
  state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::ROLL)) =
    params_.use_attitude ? latest_roll_ : 0.0;
  state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::PITCH)) =
    params_.use_attitude ? latest_pitch_ : 0.0;
  state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::VELOCITY)) =
    wheel_velocity_available_ ? latest_wheel_velocity_ : 0.0;

  if (attitude_available_) {
    state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::YAW)) = latest_yaw_;
  }

  ekf_.initialize(state, initial_covariance_, stamp);
  ekf_.setStateTime(stamp);

  RCLCPP_INFO(get_logger(), "EKF initialized at time %.3f", stamp.seconds());

  (void)msg;  // msg kept for potential future use (e.g., covariance)
}

void GnssImuWheelLocalizerNode::predictToStamp(const rclcpp::Time & stamp)
{
  if (!ekf_.isInitialized()) {
    return;
  }

  const auto current_stamp = ekf_.getStateTime();
  if (stamp <= current_stamp) {
    return;
  }

  const double dt = (stamp - current_stamp).seconds();
  ekf_.predict(process_input_, dt, process_noise_);
  ekf_.setStateTime(stamp);
}

void GnssImuWheelLocalizerNode::publishOutputs(const rclcpp::Time & stamp)
{
  if (!ekf_.isInitialized()) {
    return;
  }

  const auto & state = ekf_.getState();
  const auto & covariance = ekf_.getCovariance();

  const double x = state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::X));
  const double y = state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Y));
  const double z = params_.use_3d_position ? state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Z)) : 0.0;
  const double yaw = state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::YAW));
  const double velocity = state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::VELOCITY));

  double roll = params_.use_attitude ?
    state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::ROLL)) : 0.0;
  double pitch = params_.use_attitude ?
    state(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::PITCH)) : 0.0;

  if (params_.use_attitude) {
    if (!std::isfinite(roll)) {
      roll = latest_roll_;
    }
    if (!std::isfinite(pitch)) {
      pitch = latest_pitch_;
    }
  }

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion orientation_msg = tf2::toMsg(q);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = params_.map_frame;
  pose_msg.pose.pose.position.x = x;
  pose_msg.pose.pose.position.y = y;
  pose_msg.pose.pose.position.z = z;
  pose_msg.pose.pose.orientation = orientation_msg;

  std::array<double, 36> pose_covariance{};
  pose_covariance.fill(0.0);

  const auto assign_cov = [&pose_covariance](std::size_t row, std::size_t col, double value) {
    pose_covariance[row * 6 + col] = value;
    if (row != col) {
      pose_covariance[col * 6 + row] = value;
    }
  };

  assign_cov(0, 0, covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::X),
    static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::X)));
  assign_cov(1, 1, covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Y),
    static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Y)));
  assign_cov(0, 1, covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::X),
    static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Y)));

  if (params_.use_3d_position) {
    assign_cov(2, 2, covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Z),
      static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Z)));
  } else {
    assign_cov(2, 2, kLargeVariance);
  }

  assign_cov(5, 5, covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::YAW),
    static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::YAW)));
  assign_cov(0, 5, covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::X),
    static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::YAW)));
  assign_cov(1, 5, covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::Y),
    static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::YAW)));

  assign_cov(3, 3, params_.use_attitude ?
    covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::ROLL),
      static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::ROLL)) : kLargeVariance);
  assign_cov(4, 4, params_.use_attitude ?
    covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::PITCH),
      static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::PITCH)) : kLargeVariance);

  pose_msg.pose.covariance = pose_covariance;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = params_.odom_frame;
  odom_msg.child_frame_id = params_.base_link_frame;
  odom_msg.pose = pose_msg.pose;
  odom_msg.pose.pose.position.z = z;

  odom_msg.twist.twist.linear.x = velocity;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.z = process_input_.yaw_rate;

  std::array<double, 36> twist_covariance{};
  twist_covariance.fill(0.0);
  twist_covariance[0] = covariance(static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::VELOCITY),
    static_cast<std::size_t>(ExtendedKalmanFilter::StateIndex::VELOCITY));
  twist_covariance[5 * 6 + 5] = params_.imu_yaw_noise;
  odom_msg.twist.covariance = twist_covariance;

  pose_pub_->publish(pose_msg);
  odom_pub_->publish(odom_msg);

  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = params_.odom_frame;
    transform.child_frame_id = params_.base_link_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation = orientation_msg;
    tf_broadcaster_->sendTransform(transform);
  }
}

ExtendedKalmanFilter::CovarianceMatrix GnssImuWheelLocalizerNode::makeDiagonalMatrix(
  const std::array<double, ExtendedKalmanFilter::kStateDim> & diagonal) const
{
  ExtendedKalmanFilter::CovarianceMatrix matrix =
    ExtendedKalmanFilter::CovarianceMatrix::Zero();
  for (std::size_t i = 0; i < ExtendedKalmanFilter::kStateDim; ++i) {
    matrix(i, i) = diagonal[i];
  }
  return matrix;
}

}  // namespace gnss_imu_wheel_localizer

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gnss_imu_wheel_localizer::GnssImuWheelLocalizerNode)
