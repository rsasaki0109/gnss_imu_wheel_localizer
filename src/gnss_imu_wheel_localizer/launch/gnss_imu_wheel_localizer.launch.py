import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_name = 'gnss_imu_wheel_localizer'
    pkg_share = get_package_share_directory(package_name)

    default_param_file = os.path.join(
        pkg_share, 'config', 'gnss_imu_wheel_localizer.param.yaml')

    param_file_arg = DeclareLaunchArgument(
        'params',
        default_value=default_param_file,
        description='Path to parameter file for GNSS IMU wheel localizer')

    params = LaunchConfiguration('params')

    container = ComposableNodeContainer(
        name='localization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=package_name,
                plugin='gnss_imu_wheel_localizer::GnssImuWheelLocalizerNode',
                name='gnss_imu_wheel_localizer',
                parameters=[params],
                remappings=[
                    ('~/odometry', 'localization/gnss_imu_wheel_localizer/odometry'),
                    ('~/pose', 'localization/gnss_imu_wheel_localizer/pose'),
                ],
            ),
        ],
        output='screen',
    )

    standalone_node = Node(
        package=package_name,
        executable='gnss_imu_wheel_localizer_node',
        name='gnss_imu_wheel_localizer',
        parameters=[params],
        remappings=[
            ('~/odometry', 'localization/gnss_imu_wheel_localizer/odometry'),
            ('~/pose', 'localization/gnss_imu_wheel_localizer/pose'),
        ],
        emulate_tty=True,
        output='screen',
        condition=None,
    )

    return LaunchDescription([
        param_file_arg,
        container,
        # Comment out the following line if only component mode is required.
        # standalone_node,
    ])
