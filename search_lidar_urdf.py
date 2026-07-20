#!/usr/bin/env python3
import sys
import re
from mcap_ros2.reader import read_ros2_messages

def search_lidar_in_urdf(mcap_path):
    urdf_data = None
    for msg in read_ros2_messages(mcap_path):
        if msg.channel.topic == '/wamv/robot_description':
            urdf_data = msg.ros_msg.data
            break
    
    if not urdf_data:
        print("robot_description not found")
        return
    
    print("=== URDF中所有包含lidar的行 ===")
    lines = urdf_data.split('\n')
    lidar_lines = []
    for i, line in enumerate(lines):
        if 'lidar' in line.lower():
            lidar_lines.append((i+1, line))
    
    for line_num, line in lidar_lines:
        print(f"{line_num}: {line.strip()}")
    
    print(f"\n共找到 {len(lidar_lines)} 行")
    
    print("\n=== 搜索origin标签(包含xyz和rpy) ===")
    origin_pattern = re.compile(r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"')
    for match in origin_pattern.finditer(urdf_data):
        print(f"  xyz={match.group(1)}, rpy={match.group(2)}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 search_lidar_urdf.py <mcap_file>")
        sys.exit(1)
    search_lidar_in_urdf(sys.argv[1])