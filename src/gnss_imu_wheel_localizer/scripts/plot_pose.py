#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def main():
    csv_path = '/tmp/pose.csv'
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]

    x_list = []
    y_list = []
    velocity_list = []
    roll_list = []
    pitch_list = []
    yaw_list = []

    if not os.path.exists(csv_path):
        print(f"Error: File {csv_path} not found.")
        return

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x_list.append(float(row['x']))
                y_list.append(float(row['y']))
                velocity_list.append(float(row['velocity']))
                roll_list.append(np.degrees(float(row['roll'])))
                pitch_list.append(np.degrees(float(row['pitch'])))
                yaw_list.append(np.degrees(float(row['yaw'])))
            except ValueError:
                continue

    if not x_list:
        print("No valid data found in CSV.")
        return

    fig = plt.figure(figsize=(14, 10))
    
    # Plot Trajectory
    plt.subplot(2, 2, 1)
    plt.plot(x_list, y_list, label='Trajectory', linewidth=1.5)
    plt.title('Vehicle Trajectory (X-Y)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    # Plot Velocity
    plt.subplot(2, 2, 2)
    plt.plot(velocity_list, label='Velocity', color='orange', linewidth=1.5)
    plt.title('Vehicle Velocity')
    plt.xlabel('Sample')
    plt.ylabel('Velocity [m/s]')
    plt.grid(True)
    plt.legend()

    # Plot Attitude (Roll, Pitch, Yaw)
    plt.subplot(2, 2, 3)
    plt.plot(roll_list, label='Roll', color='red', linewidth=1.5, alpha=0.7)
    plt.plot(pitch_list, label='Pitch', color='green', linewidth=1.5, alpha=0.7)
    plt.title('Vehicle Attitude (Roll & Pitch)')
    plt.xlabel('Sample')
    plt.ylabel('Angle [deg]')
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(yaw_list, label='Yaw', color='blue', linewidth=1.5)
    plt.title('Vehicle Heading (Yaw)')
    plt.xlabel('Sample')
    plt.ylabel('Angle [deg]')
    plt.grid(True)
    plt.legend()

    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pose_plot.png')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Plot saved to {output_path}")

if __name__ == '__main__':
    main()
