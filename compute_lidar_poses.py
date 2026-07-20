#!/usr/bin/env python3
import sys
import math
from mcap_ros2.reader import read_ros2_messages

def parse_xyz_rpy(xyz_str, rpy_str):
    xyz = [float(v) for v in xyz_str.strip().split()]
    rpy = [float(v) for v in rpy_str.strip().split()]
    return xyz, rpy

def compose_transform(xyz1, rpy1, xyz2, rpy2):
    rx, ry, rz = rpy1
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    
    R = [
        [cy*cz, -cy*sz, sy],
        [cx*sz + sx*sy*cz, cx*cz - sx*sy*sz, -sx*cy],
        [sx*sz - cx*sy*cz, sx*cz + cx*sy*sz, cx*cy]
    ]
    
    tx = R[0][0]*xyz2[0] + R[0][1]*xyz2[1] + R[0][2]*xyz2[2] + xyz1[0]
    ty = R[1][0]*xyz2[0] + R[1][1]*xyz2[1] + R[1][2]*xyz2[2] + xyz1[1]
    tz = R[2][0]*xyz2[0] + R[2][1]*xyz2[1] + R[2][2]*xyz2[2] + xyz1[2]
    
    rpy_new = [rpy1[0]+rpy2[0], rpy1[1]+rpy2[1], rpy1[2]+rpy2[2]]
    return [tx, ty, tz], rpy_new

def extract_lidar_poses(mcap_path):
    urdf_data = None
    for msg in read_ros2_messages(mcap_path):
        if msg.channel.topic == '/wamv/robot_description':
            urdf_data = msg.ros_msg.data
            break
    
    if not urdf_data:
        print("robot_description not found")
        return
    
    lidar_configs = {
        "front_lidar": ["base_to_front_lidar_post", "front_lidar_post_to_front_lidar_post_arm", "front_lidar_post_arm_to_front_lidar"],
        "back_lidar": ["base_to_back_lidar_post", "back_lidar_post_to_back_lidar_post_arm", "back_lidar_post_arm_to_back_lidar"],
        "left_lidar": ["base_to_left_lidar_post", "left_lidar_post_to_left_lidar_post_arm", "left_lidar_post_arm_to_left_lidar"],
        "left2_lidar": ["base_to_left2_lidar_post", "left2_lidar_post_to_left2_lidar_post_arm", "left2_lidar_post_arm_to_left2_lidar"],
        "right_lidar": ["base_to_right_lidar_post", "right_lidar_post_to_right_lidar_post_arm", "right_lidar_post_arm_to_right_lidar"],
        "right2_lidar": ["base_to_right2_lidar_post", "right2_lidar_post_to_right2_lidar_post_arm", "right2_lidar_post_arm_to_right2_lidar"],
        "forward_lidar": ["base_to_forward_lidar_post", "forward_lidar_post_to_forward_lidar_post_arm", "forward_lidar_post_arm_to_forward_lidar"]
    }
    
    nominal_poses = {
        "front_lidar": {"xyz": [7.0, 0.0, 2.9], "rpy_deg": [0.0, 115.0, 0.0]},
        "back_lidar": {"xyz": [-7.0, 0.0, 2.9], "rpy_deg": [0.0, 115.0, 180.0]},
        "left_lidar": {"xyz": [0.0, -1.626, 2.9], "rpy_deg": [0.0, 115.0, 90.0]},
        "left2_lidar": {"xyz": [-3.5, -1.626, 2.9], "rpy_deg": [0.0, 115.0, 90.0]},
        "right_lidar": {"xyz": [0.0, 1.626, 2.9], "rpy_deg": [0.0, 115.0, -90.0]},
        "right2_lidar": {"xyz": [-3.5, 1.626, 2.9], "rpy_deg": [0.0, 115.0, -90.0]},
        "forward_lidar": {"xyz": [7.0, 0.0, 2.0], "rpy_deg": [0.0, 0.0, 0.0]}
    }
    
    lines = urdf_data.split('\n')
    
    print("="*80)
    print("=== 7路雷达真实URDF位姿 vs Nominal位姿 ===")
    print("="*80)
    print()
    
    for lidar_name, joint_names in lidar_configs.items():
        current_xyz = [0, 0, 0]
        current_rpy = [0, 0, 0]
        
        for joint_name in joint_names:
            in_joint = False
            joint_xyz = None
            joint_rpy = None
            
            for line in lines:
                if f'<joint name="wamv/{joint_name}_joint"' in line:
                    in_joint = True
                if in_joint and '<origin' in line:
                    xyz_match = line.find('xyz="')
                    rpy_match = line.find('rpy="')
                    if xyz_match != -1 and rpy_match != -1:
                        xyz_end = line.find('"', xyz_match + 5)
                        rpy_end = line.find('"', rpy_match + 5)
                        xyz_str = line[xyz_match+5:xyz_end]
                        rpy_str = line[rpy_match+5:rpy_end]
                        joint_xyz, joint_rpy = parse_xyz_rpy(xyz_str, rpy_str)
                    in_joint = False
                    break
            
            if joint_xyz and joint_rpy:
                current_xyz, current_rpy = compose_transform(current_xyz, current_rpy, joint_xyz, joint_rpy)
        
        nom = nominal_poses[lidar_name]
        xyz_diff = [current_xyz[i] - nom["xyz"][i] for i in range(3)]
        rpy_diff_deg = [(current_rpy[i] * 180/math.pi) - nom["rpy_deg"][i] for i in range(3)]
        
        print(f"[{lidar_name}]")
        print(f"  URDF真实:")
        print(f"    xyz: ({current_xyz[0]:.4f}, {current_xyz[1]:.4f}, {current_xyz[2]:.4f})")
        print(f"    rpy: ({current_rpy[0]*180/math.pi:.2f}°, {current_rpy[1]*180/math.pi:.2f}°, {current_rpy[2]*180/math.pi:.2f}°)")
        print(f"  Nominal:")
        print(f"    xyz: ({nom['xyz'][0]}, {nom['xyz'][1]}, {nom['xyz'][2]})")
        print(f"    rpy: ({nom['rpy_deg'][0]}°, {nom['rpy_deg'][1]}°, {nom['rpy_deg'][2]}°)")
        print(f"  差异(真实 - nominal):")
        print(f"    xyz: ({xyz_diff[0]:.4f}m, {xyz_diff[1]:.4f}m, {xyz_diff[2]:.4f}m)")
        print(f"    rpy: ({rpy_diff_deg[0]:.2f}°, {rpy_diff_deg[1]:.2f}°, {rpy_diff_deg[2]:.2f}°)")
        print()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 compute_lidar_poses.py <mcap_file>")
        sys.exit(1)
    extract_lidar_poses(sys.argv[1])