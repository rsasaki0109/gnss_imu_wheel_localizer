# gnss_imu_wheel_localizer

Autoware 用 GNSS / IMU / ホイールオドメトリ統合ローカライザーパッケージです。将来的に 6 軸 (6DoF) 拡張が容易になるよう、状態ベクトルを 7 次元 (x, y, z, roll, pitch, yaw, velocity) で設計しています。現状は 2D 平面走行を前提とした EKF 実装としつつ、`use_3d_position`・`use_attitude` パラメータを有効化することで ENU z 成分やロール・ピッチも取り扱える準備が整っています。

## 主な機能

* GNSS (`sensor_msgs/msg/NavSatFix`)、IMU (`sensor_msgs/msg/Imu`)、ホイールオドメトリ (`nav_msgs/msg/Odometry`) を融合する EKF。
* GNSS から ENU 座標を生成するため `GeographicLib::LocalCartesian` を利用。
* `nav_msgs/msg/Odometry` と `geometry_msgs/msg/PoseWithCovarianceStamped` を出力。
* 任意で `odom` → `base_link` の TF を配信。
* パラメータ／共分散設定を YAML ファイルで管理。6DoF 拡張を見据えたノイズパラメータを先行定義。

## 依存関係

* `rclcpp`
* `rclcpp_components`
* `sensor_msgs`
* `nav_msgs`
* `geometry_msgs`
* `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`
* `Eigen3`
* `GeographicLib`

## ビルド方法

```bash
cd /home/autoware/ros2_ws
colcon build --packages-select gnss_imu_wheel_localizer
source install/setup.bash
```

> **Note:** 上記コマンドは ROS 2 ビルド環境が整っている前提です。`colcon` コマンドが利用できない場合は、`ros-humble-colcon-common-extensions` などをインストールしてください。

## 実行方法

### コンポーネントとして起動

```bash
ros2 launch gnss_imu_wheel_localizer gnss_imu_wheel_localizer.launch.py
```

デフォルトでは `rclcpp_components` のマルチスレッドコンテナに `gnss_imu_wheel_localizer::GnssImuWheelLocalizerNode` をロードします。スタンドアロンノードで起動したい場合は、`launch/gnss_imu_wheel_localizer.launch.py` 内の `standalone_node` 部分のコメントアウトを外してください。

### トピック・出力

* 入力
  * `gnss_topic` (初期値: `/sensing/gnss/fix`)
  * `imu_topic` (初期値: `/sensing/imu/imu_data`)
  * `wheel_odom_topic` (初期値: `/localization/kinematic_state`)
* 出力
  * `~/odometry` (`nav_msgs/msg/Odometry`) → デフォルトリマップ: `localization/gnss_imu_wheel_localizer/odometry`
  * `~/pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`) → デフォルトリマップ: `localization/gnss_imu_wheel_localizer/pose`
  * `TF` (`odom` → `base_link`) ※ `publish_tf` が `true` の場合

## 主なパラメータ (`config/gnss_imu_wheel_localizer.param.yaml`)

* `publish_tf`: TF を出力するかどうか。
* `use_3d_position`: ENU Z 方向を状態・観測に含めるか。
* `use_attitude`: ロール・ピッチ観測を有効にするか。
* `process_noise_diagonal` / `initial_covariance_diagonal`: 状態ノイズと初期共分散対角要素。
* `gnss_position_noise`: GNSS 観測ノイズ (x, y, z)。
* `wheel_velocity_noise`: ホイール速度観測ノイズ。
* `imu_yaw_noise` / `imu_attitude_noise`: IMU のヨー、ロール・ピッチ観測ノイズ。
* `override_origin` と緯度経度高度: GNSS ENU 原点を手動設定するためのパラメータ。

## 将来拡張のための設計ポイント

* 状態ベクトルサイズ (`ExtendedKalmanFilter::kStateDim`) を 7 に設定し、将来 roll/pitch/z を積極的に推定できるようにしています。
* `ExtendedKalmanFilter::StateIndex` 列挙で状態アクセスを統一し、インデックスのハードコーディングを回避しています。
* `use_3d_position` / `use_attitude` を有効化するだけで 3D 位置やロール・ピッチ観測が EKF に反映される構造です。

## 開発・テストのヒント

* ログ出力は `RCLCPP_INFO/WARN_THROTTLE` で最小限としつつ、初期化状況が把握できるようにしています。
* シミュレーションやリプレイ用に、`bag` データを再生しながら `/localization/gnss_imu_wheel_localizer/odometry`、`/tf` を可視化すると状態の収束確認が容易です。

## ライセンス

本パッケージは Apache 2.0 ライセンスです。
