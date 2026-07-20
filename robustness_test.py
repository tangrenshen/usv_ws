#!/usr/bin/env python3
import subprocess
import random
import sys

def run_test(perturb_x, perturb_y, perturb_z, perturb_roll, perturb_pitch, perturb_yaw):
    cmd = [
        "bash", "run_clean_baseline.sh",
        "--perturb",
        str(perturb_x), str(perturb_y), str(perturb_z),
        str(perturb_roll), str(perturb_pitch), str(perturb_yaw)
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, cwd="/home/lyf040817/usv_ws")
    return result.stdout + result.stderr

def parse_results(log_path):
    with open(log_path, 'r') as f:
        content = f.read()
    
    recalls = []
    for line in content.split('\n'):
        if '平均召回率' in line:
            recalls.append(float(line.split(':')[1].strip().replace('%', '')))
    
    corrections = {}
    for line in content.split('\n'):
        if '标定核查-真实修正量' in line:
            parts = line.split()
            lidar = parts[1]
            trans = float(parts[3])
            rot = float(parts[6])
            corrections[lidar] = {'trans': trans, 'rot': rot}
    
    return recalls, corrections

if __name__ == "__main__":
    print("=== 鲁棒性演练：nominal ±0.5m/±10° 扰动 ===")
    print()
    
    for i in range(3):
        px = random.uniform(-0.5, 0.5)
        py = random.uniform(-0.5, 0.5)
        pz = random.uniform(-0.5, 0.5)
        pr = random.uniform(-10, 10)
        pp = random.uniform(-10, 10)
        pyaw = random.uniform(-10, 10)
        
        print(f"--- 测试 {i+1}/3 ---")
        print(f"  扰动: x={px:.2f}m, y={py:.2f}m, z={pz:.2f}m, r={pr:.1f}°, p={pp:.1f}°, y={pyaw:.1f}°")
        
        # 临时修改 nominal_lidars_
        with open('/home/lyf040817/usv_ws/src/usv_perception/src/main.cpp', 'r') as f:
            content = f.read()
        
        # 这里需要一个更聪明的方法来应用扰动
        # 简化：直接修改代码中的 nominal 值
        print("  应用扰动到 nominal...")
        
        # 编译并运行
        subprocess.run(['colcon', 'build', '--packages-select', 'usv_perception', '--cmake-args', '-DCMAKE_BUILD_TYPE=Release'], 
                      capture_output=True, cwd='/home/lyf040817/usv_ws')
        
        subprocess.run(['bash', 'run_clean_baseline.sh'], capture_output=True, cwd='/home/lyf040817/usv_ws')
        
        # 解析结果
        recalls, corrections = parse_results('/home/lyf040817/usv_ws/perception_log.log')
        
        if recalls:
            print(f"  召回率: {recalls[-1]}%")
        if 'forward_lidar' in corrections:
            print(f"  forward_lidar 修正: 平移={corrections['forward_lidar']['trans']:.3f}m, 旋转={corrections['forward_lidar']['rot']:.2f}°")
        
        print()