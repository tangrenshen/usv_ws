#!/usr/bin/env python3
import math

def multiply_transform(R1, t1, R2, t2):
    R = [[sum(R1[i][k] * R2[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    t = [sum(R1[i][k] * t2[k] for k in range(3)) + t1[i] for i in range(3)]
    return R, t

def rpy_to_rotation(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    
    R = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ]
    return R

def rotation_to_rpy(R):
    sy = math.sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0])
    singular = sy < 1e-6
    
    if not singular:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0
    
    return roll, pitch, yaw

JOINTS = {
    "front_lidar": [
        {"name": "base_to_front_lidar_post", "xyz": [6.93, 0.0, 1.54825], "rpy": [0, 0, 0]},
        {"name": "front_lidar_post_to_front_lidar_post_arm", "xyz": [0.03, 0, 0.25175], "rpy": [0, 0, 0]},
        {"name": "front_lidar_post_arm_to_front_lidar", "xyz": [0.04, 0, 0.05], "rpy": [0, 2.007128639793479, 0]},
    ],
    "back_lidar": [
        {"name": "base_to_back_lidar_post", "xyz": [-6.93, 0, 1.54825], "rpy": [0, 0, math.pi]},
        {"name": "back_lidar_post_to_back_lidar_post_arm", "xyz": [0.03, 0, 0.25175], "rpy": [0, 0, 0]},
        {"name": "back_lidar_post_arm_to_back_lidar", "xyz": [0.04, 0, 0.05], "rpy": [0, 2.007128639793479, 0]},
    ],
    "left_lidar": [
        {"name": "base_to_left_lidar_post", "xyz": [-6.0, 1.5564, 1.54825], "rpy": [0, 0, math.pi/2]},
        {"name": "left_lidar_post_to_left_lidar_post_arm", "xyz": [0.03, 0, 0.25175], "rpy": [0, 0, 0]},
        {"name": "left_lidar_post_arm_to_left_lidar", "xyz": [0.04, 0, 0.05], "rpy": [0, 2.007128639793479, 0]},
    ],
    "left2_lidar": [
        {"name": "base_to_left2_lidar_post", "xyz": [6.0, 1.5564, 1.54825], "rpy": [0, 0, math.pi/2]},
        {"name": "left2_lidar_post_to_left2_lidar_post_arm", "xyz": [0.03, 0, 0.25175], "rpy": [0, 0, 0]},
        {"name": "left2_lidar_post_arm_to_left2_lidar", "xyz": [0.04, 0, 0.05], "rpy": [0, 2.007128639793479, 0]},
    ],
    "right_lidar": [
        {"name": "base_to_right_lidar_post", "xyz": [-6.0, -1.5564, 1.54825], "rpy": [0, 0, -math.pi/2]},
        {"name": "right_lidar_post_to_right_lidar_post_arm", "xyz": [0.03, 0, 0.25175], "rpy": [0, 0, 0]},
        {"name": "right_lidar_post_arm_to_right_lidar", "xyz": [0.04, 0, 0.05], "rpy": [0, 2.007128639793479, 0]},
    ],
    "right2_lidar": [
        {"name": "base_to_right2_lidar_post", "xyz": [6.0, -1.5564, 1.54825], "rpy": [0, 0, -math.pi/2]},
        {"name": "right2_lidar_post_to_right2_lidar_post_arm", "xyz": [0.03, 0, 0.25175], "rpy": [0, 0, 0]},
        {"name": "right2_lidar_post_arm_to_right2_lidar", "xyz": [0.04, 0, 0.05], "rpy": [0, 2.007128639793479, 0]},
    ],
    "forward_lidar": [
        {"name": "base_to_forward_lidar_post", "xyz": [6.93, 0.0, 1.41325], "rpy": [0, 0, 0]},
        {"name": "forward_lidar_post_to_forward_lidar_post_arm", "xyz": [0.03, 0, 0.11675], "rpy": [0, 0, 0]},
        {"name": "forward_lidar_post_arm_to_forward_lidar", "xyz": [0.04, 0, 0.05], "rpy": [0, 0, 0]},
    ],
}

NOMINAL = {
    "front_lidar": {"xyz": [7.0, 0.0, 2.9], "rpy_deg": [0.0, 115.0, 0.0]},
    "back_lidar": {"xyz": [-7.0, 0.0, 2.9], "rpy_deg": [0.0, 115.0, 180.0]},
    "left_lidar": {"xyz": [0.0, -1.626, 2.9], "rpy_deg": [0.0, 115.0, 90.0]},
    "left2_lidar": {"xyz": [-3.5, -1.626, 2.9], "rpy_deg": [0.0, 115.0, 90.0]},
    "right_lidar": {"xyz": [0.0, 1.626, 2.9], "rpy_deg": [0.0, 115.0, -90.0]},
    "right2_lidar": {"xyz": [-3.5, 1.626, 2.9], "rpy_deg": [0.0, 115.0, -90.0]},
    "forward_lidar": {"xyz": [7.0, 0.0, 2.0], "rpy_deg": [0.0, 0.0, 0.0]},
}

print("="*90)
print("=== URDF逐级变换链复核 ===")
print("="*90)
print()

for lidar, joints in JOINTS.items():
    R_total = [[1,0,0], [0,1,0], [0,0,1]]
    t_total = [0, 0, 0]
    
    print(f"[{lidar}]")
    for j, joint in enumerate(joints):
        R_joint = rpy_to_rotation(joint["rpy"][0], joint["rpy"][1], joint["rpy"][2])
        t_joint = joint["xyz"]
        
        R_total, t_total = multiply_transform(R_total, t_total, R_joint, t_joint)
        
        rpy_deg = [joint["rpy"][i] * 180/math.pi for i in range(3)]
        print(f"  [{j+1}] {joint['name']}")
        print(f"    xyz: ({joint['xyz'][0]:.4f}, {joint['xyz'][1]:.4f}, {joint['xyz'][2]:.4f})")
        print(f"    rpy: ({rpy_deg[0]:.2f}°, {rpy_deg[1]:.2f}°, {rpy_deg[2]:.2f}°)")
        print(f"    累积: xyz=({t_total[0]:.4f}, {t_total[1]:.4f}, {t_total[2]:.4f})")
    
    roll, pitch, yaw = rotation_to_rpy(R_total)
    print(f"  ──────────────────────────────────")
    print(f"  URDF真实累积:")
    print(f"    xyz: ({t_total[0]:.4f}, {t_total[1]:.4f}, {t_total[2]:.4f})")
    print(f"    rpy: ({roll*180/math.pi:.2f}°, {pitch*180/math.pi:.2f}°, {yaw*180/math.pi:.2f}°)")
    
    nom = NOMINAL[lidar]
    xyz_diff = [t_total[i] - nom["xyz"][i] for i in range(3)]
    rpy_diff = [(roll*180/math.pi)-nom["rpy_deg"][0], 
                (pitch*180/math.pi)-nom["rpy_deg"][1], 
                (yaw*180/math.pi)-nom["rpy_deg"][2]]
    print(f"  Nominal代码值:")
    print(f"    xyz: ({nom['xyz'][0]:.4f}, {nom['xyz'][1]:.4f}, {nom['xyz'][2]:.4f})")
    print(f"    rpy: ({nom['rpy_deg'][0]:.2f}°, {nom['rpy_deg'][1]:.2f}°, {nom['rpy_deg'][2]:.2f}°)")
    print(f"  差异(真实 - nominal):")
    print(f"    xyz: ({xyz_diff[0]:.4f}m, {xyz_diff[1]:.4f}m, {xyz_diff[2]:.4f}m)")
    print(f"    rpy: ({rpy_diff[0]:.2f}°, {rpy_diff[1]:.2f}°, {rpy_diff[2]:.2f}°)")
    print()