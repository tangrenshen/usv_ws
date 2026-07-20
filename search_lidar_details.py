#!/usr/bin/env python3
import sys
import re
from mcap_ros2.reader import read_ros2_messages

def search_lidar_joints(mcap_path):
    urdf_data = None
    for msg in read_ros2_messages(mcap_path):
        if msg.channel.topic == '/wamv/robot_description':
            urdf_data = msg.ros_msg.data
            break
    
    if not urdf_data:
        print("robot_description not found")
        return
    
    print("=== 搜索所有包含lidar的joint ===")
    joint_pattern = re.compile(r'<joint name="([^"]*lidar[^"]*)"', re.IGNORECASE)
    for match in joint_pattern.finditer(urdf_data):
        print(f"  Joint: {match.group(1)}")
    
    print("\n=== 搜索所有包含lidar的link ===")
    link_pattern = re.compile(r'<link name="([^"]*lidar[^"]*)"', re.IGNORECASE)
    for match in link_pattern.finditer(urdf_data):
        print(f"  Link: {match.group(1)}")
    
    print("\n=== 输出front_lidar附近50行 ===")
    lines = urdf_data.split('\n')
    for i, line in enumerate(lines):
        if 'front_lidar' in line:
            start = max(0, i-5)
            end = min(len(lines), i+50)
            for j in range(start, end):
                print(f"{j+1}: {lines[j]}")
            break

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 search_lidar_details.py <mcap_file>")
        sys.exit(1)
    search_lidar_joints(sys.argv[1])