#include <gtest/gtest.h>

#include "gnss_imu_wheel_localizer/extended_kalman_filter.hpp"

namespace giwl = gnss_imu_wheel_localizer;

namespace
{
constexpr double kTolerance = 1.0e-6;
}  // namespace

TEST(ExtendedKalmanFilterTest, InitializeSetsStateAndCovariance)
{
  giwl::ExtendedKalmanFilter ekf;

  giwl::ExtendedKalmanFilter::StateVector state =
    giwl::ExtendedKalmanFilter::StateVector::Zero();
  state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::X)) = 1.0;
  state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::Y)) = -2.0;
  state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::YAW)) = 0.5;
  state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::VELOCITY)) = 3.0;

  giwl::ExtendedKalmanFilter::CovarianceMatrix covariance =
    giwl::ExtendedKalmanFilter::CovarianceMatrix::Identity();
  covariance *= 0.5;

  const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);

  ekf.initialize(state, covariance, stamp);

  ASSERT_TRUE(ekf.isInitialized());
  const auto & stored_state = ekf.getState();
  const auto & stored_covariance = ekf.getCovariance();

  for (std::size_t i = 0; i < giwl::ExtendedKalmanFilter::kStateDim; ++i) {
    EXPECT_DOUBLE_EQ(stored_state(i), state(i));
    for (std::size_t j = 0; j < giwl::ExtendedKalmanFilter::kStateDim; ++j) {
      EXPECT_DOUBLE_EQ(stored_covariance(i, j), covariance(i, j));
    }
  }
  EXPECT_EQ(ekf.getStateTime(), stamp);
}

TEST(ExtendedKalmanFilterTest, PredictAdvancesPose)
{
  giwl::ExtendedKalmanFilter ekf;
  giwl::ExtendedKalmanFilter::StateVector state =
    giwl::ExtendedKalmanFilter::StateVector::Zero();
  state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::VELOCITY)) = 1.0;

  giwl::ExtendedKalmanFilter::CovarianceMatrix covariance =
    giwl::ExtendedKalmanFilter::CovarianceMatrix::Identity();

  ekf.initialize(state, covariance, rclcpp::Time(0, 0, RCL_ROS_TIME));

  giwl::ExtendedKalmanFilter::ProcessModelInput input;
  input.linear_velocity = 1.0;
  input.longitudinal_acceleration = 0.0;
  input.yaw_rate = 0.1;

  giwl::ExtendedKalmanFilter::CovarianceMatrix process_noise =
    giwl::ExtendedKalmanFilter::CovarianceMatrix::Identity() * 1.0e-3;

  const double dt = 1.0;
  ekf.predict(input, dt, process_noise);

  const auto & updated_state = ekf.getState();

  const double expected_yaw = 0.1;
  const double expected_x = std::cos(expected_yaw / 2.0);
  const double expected_y = std::sin(expected_yaw / 2.0);

  EXPECT_NEAR(updated_state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::X)),
    expected_x, 1.0e-2);
  EXPECT_NEAR(updated_state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::Y)),
    expected_y, 1.0e-2);
  EXPECT_NEAR(updated_state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::YAW)),
    expected_yaw, 1.0e-3);
  EXPECT_NEAR(updated_state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::VELOCITY)),
    1.0, 1.0e-6);
}

TEST(ExtendedKalmanFilterTest, UpdateCorrectsStateWithMeasurement)
{
  giwl::ExtendedKalmanFilter ekf;
  giwl::ExtendedKalmanFilter::StateVector state =
    giwl::ExtendedKalmanFilter::StateVector::Zero();
  state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::X)) = 0.0;
  state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::Y)) = 0.0;

  giwl::ExtendedKalmanFilter::CovarianceMatrix covariance =
    giwl::ExtendedKalmanFilter::CovarianceMatrix::Identity() * 0.5;

  ekf.initialize(state, covariance, rclcpp::Time(0, 0, RCL_ROS_TIME));

  Eigen::Matrix<double, 2, 1> measurement;
  measurement << 2.0, 0.5;

  Eigen::Matrix<double, 2, giwl::ExtendedKalmanFilter::kStateDim> observation_matrix =
    Eigen::Matrix<double, 2, giwl::ExtendedKalmanFilter::kStateDim>::Zero();
  observation_matrix(0, static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::X)) = 1.0;
  observation_matrix(1, static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::Y)) = 1.0;

  Eigen::Matrix<double, 2, 2> observation_noise = Eigen::Matrix<double, 2, 2>::Identity() * 0.1;

  ekf.update(measurement, observation_matrix, observation_noise);

  const auto & updated_state = ekf.getState();
  // Kalman gain K = P * H^T * (H * P * H^T + R)^-1 = 0.5 / (0.5 + 0.1) = 0.833...
  // updated_x = 0 + K * (2.0 - 0) = 1.666...
  // updated_y = 0 + K * (0.5 - 0) = 0.416...
  const double expected_x = 2.0 * 0.5 / (0.5 + 0.1);
  const double expected_y = 0.5 * 0.5 / (0.5 + 0.1);
  EXPECT_NEAR(updated_state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::X)),
    expected_x, 1.0e-6);
  EXPECT_NEAR(updated_state(static_cast<std::size_t>(giwl::ExtendedKalmanFilter::StateIndex::Y)),
    expected_y, 1.0e-6);
}
