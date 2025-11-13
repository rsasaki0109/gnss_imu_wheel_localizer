import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
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

    play_rosbag_arg = DeclareLaunchArgument(
        'play_rosbag',
        default_value='false',
        description='Whether to play a rosbag alongside the localizer')

    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value=os.path.join(
            '/home/autoware/autoware_map/sample-rosbag',
            'sample-rosbag_0.db3'),
        description='Path to rosbag to play when play_rosbag is true')

    rosbag_loop_arg = DeclareLaunchArgument(
        'rosbag_loop',
        default_value='false',
        description='Loop rosbag playback when true')

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

    rosbag_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('play_rosbag')),
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    LaunchConfiguration('rosbag_path'),
                    '--loop'
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('rosbag_loop')),
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    LaunchConfiguration('rosbag_path'),
                ],
                output='screen',
                condition=UnlessCondition(LaunchConfiguration('rosbag_loop')),
            ),
        ],
    )

    return LaunchDescription([
        param_file_arg,
        play_rosbag_arg,
        rosbag_path_arg,
        rosbag_loop_arg,
        container,
        rosbag_group,
        # Comment out the following line if only component mode is required.
        # standalone_node,
    ])
