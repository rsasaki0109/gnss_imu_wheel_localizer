#!/usr/bin/env python3
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import math

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle in radians"""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def main():
    db_path = '/home/autoware/autoware_map/sample-rosbag/sample-rosbag_0.db3'
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get topic id for IMU
    cursor.execute("SELECT id FROM topics WHERE name='/sensing/imu/tamagawa/imu_raw'")
    topic_id = cursor.fetchone()[0]
    
    # Get messages
    cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={topic_id} ORDER BY timestamp LIMIT 20")
    
    msg_type = get_message('sensor_msgs/msg/Imu')
    
    print("First 20 IMU messages:")
    print("timestamp, qx, qy, qz, qw, yaw(deg), angular_vel_z")
    
    for timestamp, data in cursor.fetchall():
        msg = deserialize_message(data, msg_type)
        q = msg.orientation
        yaw_rad = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        yaw_deg = math.degrees(yaw_rad)
        print(f"{timestamp}, {q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f}, {yaw_deg:.2f}, {msg.angular_velocity.z:.6f}")
    
    conn.close()

if __name__ == '__main__':
    main()
