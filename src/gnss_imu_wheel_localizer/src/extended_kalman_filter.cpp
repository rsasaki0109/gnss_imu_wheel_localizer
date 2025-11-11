#include "gnss_imu_wheel_localizer/extended_kalman_filter.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double normalizeAngle(const double angle)
{
  double normalized = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }
  return normalized - M_PI;
}

}  // namespace

namespace gnss_imu_wheel_localizer
{

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
  state_.setZero();
  covariance_.setIdentity();
}

void ExtendedKalmanFilter::initialize(
  const StateVector & state, const CovarianceMatrix & covariance, const rclcpp::Time & stamp)
{
  state_ = state;
  covariance_ = covariance;
  stamp_ = stamp;
  initialized_ = true;
}

bool ExtendedKalmanFilter::isInitialized() const
{
  return initialized_;
}

void ExtendedKalmanFilter::predict(
  const ProcessModelInput & input, const double dt, const CovarianceMatrix & process_noise)
{
  if (!initialized_ || dt <= 0.0) {
    return;
  }

  using SI = StateIndex;

  const auto yaw_index = static_cast<std::size_t>(SI::YAW);
  const auto velocity_index = static_cast<std::size_t>(SI::VELOCITY);
  const auto x_index = static_cast<std::size_t>(SI::X);
  const auto y_index = static_cast<std::size_t>(SI::Y);

  const double yaw = state_(yaw_index);
  const double velocity = state_(velocity_index);

  const double yaw_rate = input.yaw_rate;
  const double acceleration = input.longitudinal_acceleration;
  const double linear_velocity = std::isfinite(input.linear_velocity) ? input.linear_velocity : velocity;

  const double next_velocity = linear_velocity + acceleration * dt;
  const double average_velocity = 0.5 * (linear_velocity + next_velocity);

  const double yaw_delta = yaw_rate * dt;
  const double next_yaw = normalizeAngle(yaw + yaw_delta);
  const double average_yaw = normalizeAngle(yaw + 0.5 * yaw_delta);

  const double distance = average_velocity * dt;

  state_(x_index) += distance * std::cos(average_yaw);
  state_(y_index) += distance * std::sin(average_yaw);
  state_(yaw_index) = next_yaw;
  state_(velocity_index) = next_velocity;

  // For now, keep Z, roll, and pitch states as-is. They remain placeholders for future expansion.

  CovarianceMatrix F = CovarianceMatrix::Identity();
  F(x_index, yaw_index) = -distance * std::sin(average_yaw);
  F(x_index, velocity_index) = dt * std::cos(average_yaw);

  F(y_index, yaw_index) = distance * std::cos(average_yaw);
  F(y_index, velocity_index) = dt * std::sin(average_yaw);

  covariance_ = F * covariance_ * F.transpose() + process_noise;
}

const ExtendedKalmanFilter::StateVector & ExtendedKalmanFilter::getState() const
{
  return state_;
}

const ExtendedKalmanFilter::CovarianceMatrix & ExtendedKalmanFilter::getCovariance() const
{
  return covariance_;
}

void ExtendedKalmanFilter::setStateTime(const rclcpp::Time & stamp)
{
  stamp_ = stamp;
}

const rclcpp::Time & ExtendedKalmanFilter::getStateTime() const
{
  return stamp_;
}

}  // namespace gnss_imu_wheel_localizer
