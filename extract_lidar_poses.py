#!/usr/bin/env python3
import sys
import re
from mcap_ros2.reader import read_ros2_messages

LIDAR_NAMES = [
    "front_lidar",
    "right_lidar", 
    "right2_lidar",
    "back_lidar",
    "left2_lidar",
    "left_lidar",
    "forward_lidar"
]

def extract_lidar_urdf_poses(mcap_path):
    urdf_data = None
    for msg in read_ros2_messages(mcap_path):
        if msg.channel.topic == '/wamv/robot_description':
            urdf_data = msg.ros_msg.data
            break
    
    if not urdf_data:
        print("robot_description not found")
        return
    
    print("=== 7路雷达URDF真实位姿 ===")
    print()
    
    for lidar_name in LIDAR_NAMES:
        pattern = f'<joint name="wamv/{lidar_name}_joint"(.*?)</joint>'
        match = re.search(pattern, urdf_data, re.DOTALL)
        
        if match:
            joint_content = match.group(1)
            
            xyz_match = re.search(r'<origin xyz="([\d\.\-e\s]+)"', joint_content)
            rpy_match = re.search(r'rpy="([\d\.\-e\s]+)"', joint_content)
            
            if xyz_match and rpy_match:
                xyz = xyz_match.group(1).strip()
                rpy = rpy_match.group(1).strip()
                print(f"[{lidar_name}]")
                print(f"  xyz: {xyz}")
                print(f"  rpy: {rpy}")
                print()
            else:
                print(f"[{lidar_name}] - 未找到origin")
                print()
        else:
            print(f"[{lidar_name}] - 未找到joint")
            print()
    
    print("=== Nominal位姿(代码中) ===")
    print()
    print("[front_lidar]")
    print("  xyz: 7.0 0.0 2.9")
    print("  rpy: 0.0 115.0 0.0")
    print()
    print("[right_lidar]")
    print("  xyz: 0.0 1.626 2.9")
    print("  rpy: 0.0 115.0 -90.0")
    print()
    print("[right2_lidar]")
    print("  xyz: -3.5 1.626 2.9")
    print("  rpy: 0.0 115.0 -90.0")
    print()
    print("[back_lidar]")
    print("  xyz: -7.0 0.0 2.9")
    print("  rpy: 0.0 115.0 180.0")
    print()
    print("[left2_lidar]")
    print("  xyz: -3.5 -1.626 2.9")
    print("  rpy: 0.0 115.0 90.0")
    print()
    print("[left_lidar]")
    print("  xyz: 0.0 -1.626 2.9")
    print("  rpy: 0.0 115.0 90.0")
    print()
    print("[forward_lidar]")
    print("  xyz: 7.0 0.0 2.0")
    print("  rpy: 0.0 0.0 0.0")
    print()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 extract_lidar_poses.py <mcap_file>")
        sys.exit(1)
    extract_lidar_urdf_poses(sys.argv[1])