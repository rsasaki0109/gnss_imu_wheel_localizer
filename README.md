# gnss_imu_wheel_localizer

A sensor fusion localizer for Autoware that integrates GNSS, IMU, and wheel odometry using an Extended Kalman Filter (EKF). The state vector is designed as 7-dimensional (x, y, z, roll, pitch, yaw, velocity) to facilitate future 6DoF expansion. Currently implements 2D planar motion with optional 3D position and attitude estimation via `use_3d_position` and `use_attitude` parameters.

## Features

- **Sensor Fusion**: Fuses GNSS (`sensor_msgs/msg/NavSatFix`), IMU (`sensor_msgs/msg/Imu`), and wheel odometry (`nav_msgs/msg/Odometry`) using EKF
- **Coordinate Conversion**: Converts GNSS lat/lon to ENU coordinates using `GeographicLib::LocalCartesian`
- **Output**: Publishes `nav_msgs/msg/Odometry` and `geometry_msgs/msg/PoseWithCovarianceStamped`
- **TF Broadcasting**: Optional `odom` → `base_link` transform broadcasting
- **CSV Logging**: Built-in CSV export for pose data analysis and visualization
- **Configurable**: YAML-based parameter and covariance configuration with 6DoF expansion support

## Key Design Decisions

### Yaw Angle Estimation
**Important**: This localizer does **not** use IMU orientation (quaternion) for yaw observation. Instead, yaw angle is estimated by the EKF through:
- **Prediction**: Integration of yaw rate (IMU angular velocity) and vehicle velocity
- **Correction**: GNSS position updates indirectly constrain yaw through motion consistency

This approach is suitable for IMUs that provide only angular velocity without absolute orientation (common in automotive-grade IMUs).

## Dependencies

- `rclcpp`, `rclcpp_components`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`
- `Eigen3`
- `GeographicLib`

## Build

```bash
cd /path/to/your/workspace
colcon build --packages-select gnss_imu_wheel_localizer
source install/setup.bash
```

> **Note**: Requires ROS 2 build environment. Install `ros-humble-colcon-common-extensions` if `colcon` is not available.

## Usage

### Basic Launch

```bash
ros2 launch gnss_imu_wheel_localizer gnss_imu_wheel_localizer.launch.py
```

By default, loads `gnss_imu_wheel_localizer::GnssImuWheelLocalizerNode` into a multi-threaded `rclcpp_components` container. For standalone node execution, uncomment the `standalone_node` section in the launch file.

### Launch with Rosbag Playback

```bash
ros2 launch gnss_imu_wheel_localizer gnss_imu_wheel_localizer.launch.py \
  play_rosbag:=true \
  rosbag_path:=/path/to/your/rosbag.db3
```

### CSV Logging

Enable CSV logging to record pose data:

```bash
ros2 launch gnss_imu_wheel_localizer gnss_imu_wheel_localizer.launch.py \
  enable_pose_csv_logging:=true \
  pose_csv_path:=/tmp/pose.csv
```

CSV format: `stamp_sec,stamp_nanosec,x,y,z,roll,pitch,yaw,velocity`

### Visualization

Visualize logged pose data using the included Python script:

```bash
python3 src/gnss_imu_wheel_localizer/scripts/plot_pose.py /tmp/pose.csv
```

**Example Output:**

![Pose Plot Example](docs/pose_plot_example.png)

The plot shows:
- **Top-left**: Vehicle trajectory (X-Y)
- **Top-right**: Velocity over time
- **Bottom-left**: Roll & Pitch angles
- **Bottom-right**: Yaw (heading) angle

## Topics

### Subscribed
- `gnss_topic` (default: `/sensing/gnss/ublox/nav_sat_fix`) - GNSS position
- `imu_topic` (default: `/sensing/imu/tamagawa/imu_raw`) - IMU data (angular velocity, linear acceleration)
- `wheel_odom_topic` (default: `/localization/kinematic_state`) - Wheel odometry

### Published
- `~/odometry` → `localization/gnss_imu_wheel_localizer/odometry` (`nav_msgs/msg/Odometry`)
- `~/pose` → `localization/gnss_imu_wheel_localizer/pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `TF` (`odom` → `base_link`) - if `publish_tf` is `true`

## Parameters

Key parameters in `config/gnss_imu_wheel_localizer.param.yaml`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `publish_tf` | Broadcast TF transform | `true` |
| `use_3d_position` | Include Z in state/observation | `false` |
| `use_attitude` | Enable roll/pitch observation | `false` |
| `enable_pose_csv_logging` | Enable CSV output | `true` |
| `pose_csv_path` | CSV output path | `/tmp/gnss_imu_wheel_localizer/pose.csv` |
| `process_noise_diagonal` | Process noise (7D) | `[0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 1.0]` |
| `initial_covariance_diagonal` | Initial covariance (7D) | `[5.0, 5.0, 5.0, 0.5, 0.5, 0.5, 1.0]` |
| `gnss_position_noise` | GNSS observation noise (x,y,z) | `[1.0, 1.0, 2.0]` |
| `wheel_velocity_noise` | Wheel velocity noise | `0.5` |
| `override_origin` | Manually set ENU origin | `false` |
| `origin_latitude` | ENU origin latitude | `0.0` |
| `origin_longitude` | ENU origin longitude | `0.0` |
| `origin_altitude` | ENU origin altitude | `0.0` |

## Architecture

### EKF State Vector (7D)
```
[x, y, z, roll, pitch, yaw, velocity]
```

### Process Model
- **Position**: Updated using velocity and yaw angle (constant velocity model)
- **Yaw**: Updated using yaw rate (constant yaw rate model)
- **Velocity**: Updated using linear acceleration

### Observation Model
- **GNSS**: Observes position (x, y) or (x, y, z) if `use_3d_position` is enabled
- **Wheel Odometry**: Observes velocity
- **IMU**: Provides yaw rate and acceleration for prediction (orientation not used for observation)

## Future Expansion

The design supports easy extension to full 6DoF:
- State vector size (`ExtendedKalmanFilter::kStateDim`) is already 7D
- `ExtendedKalmanFilter::StateIndex` enum provides unified state access
- Enable `use_3d_position` and `use_attitude` to activate 3D position and roll/pitch estimation

## Development Tips

- Logging uses `RCLCPP_INFO/WARN_THROTTLE` to minimize output while providing initialization feedback
- For simulation/replay, visualize `/localization/gnss_imu_wheel_localizer/odometry` and `/tf` to verify state convergence
- Use CSV logging and visualization script for detailed analysis of estimator performance

## License

This package is licensed under Apache 2.0.
