#!/usr/bin/env python3
import csv
import os
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseCsvLogger(Node):
    def __init__(self):
        super().__init__('pose_csv_logger')

        csv_path = self.declare_parameter('pose_csv_path', '/tmp/gnss_imu_wheel_localizer_pose.csv').get_parameter_value().string_value
        os.makedirs(Path(csv_path).parent, exist_ok=True)
        self._file = open(csv_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            'stamp_sec', 'stamp_nanosec', 'x', 'y', 'z',
            'orient_x', 'orient_y', 'orient_z', 'orient_w'
        ])

        pose_topic = self.declare_parameter('pose_topic', '/localization/gnss_imu_wheel_localizer/pose').get_parameter_value().string_value
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self._pose_callback, 10)

        self.get_logger().info('Logging pose to %s', csv_path)

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        stamp = msg.header.stamp
        pose = msg.pose.pose
        self._writer.writerow([
            stamp.sec,
            stamp.nanosec,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])

    def destroy_node(self):
        if not self._file.closed:
            self._file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
