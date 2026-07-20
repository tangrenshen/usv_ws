#!/usr/bin/env python3
import sys
from mcap_ros2.reader import read_ros2_messages

def extract_lidar_joints(mcap_path):
    urdf_data = None
    for msg in read_ros2_messages(mcap_path):
        if msg.channel.topic == '/wamv/robot_description':
            urdf_data = msg.ros_msg.data
            break
    
    if not urdf_data:
        print("robot_description not found")
        return
    
    print("=== 所有lidar相关的joint和origin行 ===")
    lines = urdf_data.split('\n')
    
    in_joint = False
    current_joint = ""
    
    for i, line in enumerate(lines):
        if '<joint name=' in line and 'lidar' in line.lower():
            in_joint = True
            current_joint = line.strip()
            print(f"\n{current_joint}")
        
        if in_joint:
            if '<origin' in line:
                print(f"  {line.strip()}")
            if '</joint>' in line:
                in_joint = False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 extract_all_lidar_joints.py <mcap_file>")
        sys.exit(1)
    extract_lidar_joints(sys.argv[1])