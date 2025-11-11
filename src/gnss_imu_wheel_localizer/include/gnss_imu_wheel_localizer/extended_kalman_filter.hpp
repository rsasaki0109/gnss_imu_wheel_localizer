#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <rclcpp/time.hpp>

namespace gnss_imu_wheel_localizer
{

class ExtendedKalmanFilter
{
public:
  static constexpr std::size_t kStateDim = 7;  // x, y, z, roll, pitch, yaw, velocity

  enum class StateIndex : std::size_t
  {
    X = 0,
    Y = 1,
    Z = 2,
    ROLL = 3,
    PITCH = 4,
    YAW = 5,
    VELOCITY = 6
  };

  using StateVector = Eigen::Matrix<double, kStateDim, 1>;
  using CovarianceMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;

  struct ProcessModelInput
  {
    double linear_velocity{0.0};
    double longitudinal_acceleration{0.0};
    double yaw_rate{0.0};
  };

  ExtendedKalmanFilter();

  void initialize(const StateVector & state, const CovarianceMatrix & covariance, const rclcpp::Time & stamp);
  bool isInitialized() const;

  void predict(const ProcessModelInput & input, double dt, const CovarianceMatrix & process_noise);

  template<std::size_t MeasurementDim>
  void update(
    const Eigen::Matrix<double, MeasurementDim, 1> & measurement,
    const Eigen::Matrix<double, MeasurementDim, kStateDim> & observation_matrix,
    const Eigen::Matrix<double, MeasurementDim, MeasurementDim> & observation_noise)
  {
    if (!initialized_) {
      return;
    }

    const auto innovation = measurement - observation_matrix * state_;
    const auto S = observation_matrix * covariance_ * observation_matrix.transpose() + observation_noise;
    const auto K = covariance_ * observation_matrix.transpose() * S.inverse();

    state_ = state_ + K * innovation;
    const auto identity = CovarianceMatrix::Identity();
    covariance_ = (identity - K * observation_matrix) * covariance_;
  }

  const StateVector & getState() const;
  const CovarianceMatrix & getCovariance() const;

  void setStateTime(const rclcpp::Time & stamp);
  const rclcpp::Time & getStateTime() const;

private:
  StateVector state_;
  CovarianceMatrix covariance_;
  rclcpp::Time stamp_;
  bool initialized_{false};
};

}  // namespace gnss_imu_wheel_localizer
