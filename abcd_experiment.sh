#!/bin/bash
# A/B/C/D 四组对照实验脚本
# 用法: bash abcd_experiment.sh {A|B|C|D}
# 每组只跑bag + 计数，不评估召回率（吞吐量实验，不依赖gt对齐）

set -e
GROUP=${1:-A}
cd "$(dirname "${BASH_SOURCE[0]}")"

echo "=== 实验 $GROUP ==="

# 切换onLidar代码到对应版本
case $GROUP in
  A)
    cp main.cpp.backup_A_full src/usv_perception/src/main.cpp
    echo "已切换到A组（完整处理: reserve + for循环 + move + 锁）"
    ;;
  B)
    # B组: 保留锁+reserve+move空vector，去掉for循环emplace
    # 测"大块内存分配+move+锁"开销（去掉循环写入）
    cp main.cpp.backup_A_full src/usv_perception/src/main.cpp
    python3 -c "
import re
with open('src/usv_perception/src/main.cpp', 'r') as f:
    content = f.read()
# 把for循环注释掉
old = '''    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      pts.emplace_back(*ix, *iy, *iz);
    }'''
new = '''    // [B组] for循环跳过，pts保持空，测'reserve+move空vector+锁'开销
    (void)ix; (void)iy; (void)iz;'''
content = content.replace(old, new)
with open('src/usv_perception/src/main.cpp', 'w') as f:
    f.write(content)
print('已切换到B组（去掉for循环，保留reserve+move+锁）')
"
    ;;
  C)
    # C组: 只计数，最小基线
    cp main.cpp.backup_A_full src/usv_perception/src/main.cpp
    python3 -c "
with open('src/usv_perception/src/main.cpp', 'r') as f:
    content = f.read()
old = '''    std::vector<cv::Point3f> pts;
    pts.reserve(static_cast<size_t>(msg->width) * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, \"x\"), iy(*msg, \"y\"), iz(*msg, \"z\");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      pts.emplace_back(*ix, *iy, *iz);
    }

    const double sensor_stamp_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    std::lock_guard<std::mutex> lk(*lidar_mtx_[idx]);
    latest_lidars_[idx]    = std::move(pts);
    lidar_wall_time_[idx]  = std::chrono::steady_clock::now();
    lidar_stamp_sec_[idx]  = sensor_stamp_sec;
    lidar_has_data_[idx]   = true;
    lidar_msg_count_[idx]++;'''
new = '''    // [C组] 只计数，最小基线，无任何处理
    lidar_msg_count_[idx]++;
    lidar_has_data_[idx] = true;'''
content = content.replace(old, new)
with open('src/usv_perception/src/main.cpp', 'w') as f:
    f.write(content)
print('已切换到C组（只计数，最小基线）')
"
    ;;
  D)
    # D组: 保留锁+for循环遍历，不做reserve、不做move
    # 测"纯for循环遍历+锁"开销（去掉大块分配+move析构）
    cp main.cpp.backup_A_full src/usv_perception/src/main.cpp
    python3 -c "
with open('src/usv_perception/src/main.cpp', 'r') as f:
    content = f.read()
old = '''    std::vector<cv::Point3f> pts;
    pts.reserve(static_cast<size_t>(msg->width) * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, \"x\"), iy(*msg, \"y\"), iz(*msg, \"z\");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      pts.emplace_back(*ix, *iy, *iz);
    }

    const double sensor_stamp_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    std::lock_guard<std::mutex> lk(*lidar_mtx_[idx]);
    latest_lidars_[idx]    = std::move(pts);
    lidar_wall_time_[idx]  = std::chrono::steady_clock::now();
    lidar_stamp_sec_[idx]  = sensor_stamp_sec;
    lidar_has_data_[idx]   = true;
    lidar_msg_count_[idx]++;'''
new = '''    // [D组] 保留for循环遍历+锁，不做reserve、不做move
    // 测'纯for循环遍历+锁'开销（去掉大块分配+move析构）
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, \"x\"), iy(*msg, \"y\"), iz(*msg, \"z\");
    volatile float sum = 0.0f;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      sum += *ix + *iy + *iz;
    }
    (void)sum;

    const double sensor_stamp_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    std::lock_guard<std::mutex> lk(*lidar_mtx_[idx]);
    // 不写latest_lidars_，只更新元数据
    lidar_wall_time_[idx]  = std::chrono::steady_clock::now();
    lidar_stamp_sec_[idx]  = sensor_stamp_sec;
    lidar_has_data_[idx]   = true;
    lidar_msg_count_[idx]++;'''
content = content.replace(old, new)
with open('src/usv_perception/src/main.cpp', 'w') as f:
    f.write(content)
print('已切换到D组（保留for循环+锁，去掉reserve+move）')
"
    ;;
  *)
    echo "用法: bash abcd_experiment.sh {A|B|C|D}"
    exit 1
    ;;
esac

echo ""
echo "[编译]"
cd /home/lyf040817/usv_ws
colcon build --packages-select usv_perception --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3

echo ""
echo "[清理进程]"
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 3

echo "[启动感知节点]"
rm -f perception_log_${GROUP}.log
source install/setup.bash
nohup ros2 run usv_perception perception_node > perception_log_${GROUP}.log 2>&1 &
PERCEPTION_PID=$!
sleep 3

echo "[播放bag（正常速度）]"
ros2 bag play ~/bags/bag_09_03_17/ --clock > /dev/null 2>&1
echo "  ✅ bag播放完成"

echo "[等待处理完剩余数据]"
sleep 8

echo "[停止进程]"
kill $PERCEPTION_PID 2>/dev/null || true
sleep 2
pkill -9 -f "perception_node" 2>/dev/null || true
sleep 1

echo ""
echo "=== 实验 $GROUP 结果 ==="
echo "[QoS消息计数]"
grep "QoS" perception_log_${GROUP}.log | grep -E "lidar|相机消息数"

echo ""
echo "[QoS雷达总丢失率]"
python3 -c "
import re
with open('perception_log_${GROUP}.log') as f:
    content = f.read()
counts = re.findall(r'\[QoS\]\s+(\w+):\s+(\d+)条\s+\(丢失([\d.]+)%\)', content)
lidar_total = 0
lidar_lost = 0
for name, cnt, lost in counts:
    if 'lidar' in name:
        c = int(cnt)
        l = float(lost)/100
        lidar_total += c
        # 丢失率反推期望: c / (1 - l)
print(f'雷达总接收: {lidar_total}条 (7路, 期望768×7=5376条)')
"

echo ""
echo "=== 实验 $GROUP 完成 ==="
