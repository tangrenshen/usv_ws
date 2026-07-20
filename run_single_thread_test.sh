#!/bin/bash
# SingleThreadedExecutor对照实验v2
# 修复：1)定期QoS输出 2)优雅退出确保on_shutdown触发

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "========================================"
echo "SingleThreadedExecutor对照实验v2"
echo "========================================"
echo ""

echo "[1/5] 清理所有进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 3

echo ""
echo "[2/5] 清理旧文件..."
rm -f perception_log.log

echo ""
echo "[3/5] 启动perception_node（SingleThreadedExecutor）..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
nohup ros2 run usv_perception perception_node > perception_log.log 2>&1 &
PERCEPTION_PID=$!
echo "  perception_node PID: $PERCEPTION_PID"

for i in $(seq 1 30); do
  if grep -q "初始化完毕" perception_log.log 2>/dev/null; then
    echo "  ✅ perception_node已启动 (${i}秒)"
    break
  fi
  sleep 1
done

echo ""
echo "[4/5] 播放bag（正常速度 --rate 1.0）..."
echo "  开始时间: $(date '+%H:%M:%S')"
START_SEC=$(date +%s)
ros2 bag play ~/bags/bag_09_03_17/ --clock --rate 1.0
END_SEC=$(date +%s)
echo "  结束时间: $(date '+%H:%M:%S')"
echo "  实际播放时长: $((END_SEC - START_SEC))秒"

echo ""
echo "[清理] 等待处理完剩余数据..."
sleep 5

echo ""
echo "[5/5] 优雅停止perception_node（SIGTERM + 等待15秒）..."
kill -TERM $PERCEPTION_PID 2>/dev/null
for i in $(seq 1 15); do
  if ! kill -0 $PERCEPTION_PID 2>/dev/null; then
    echo "  ✅ 节点已退出 (${i}秒)"
    break
  fi
  sleep 1
done
if kill -0 $PERCEPTION_PID 2>/dev/null; then
  echo "  ⚠️ 节点未响应SIGTERM，发送SIGKILL"
  kill -9 $PERCEPTION_PID 2>/dev/null
  sleep 2
fi

echo ""
echo "========================================"
echo "SingleThreadedExecutor结果"
echo "========================================"

echo ""
echo "[所有QoS计数输出]"
grep -n "QoS消息计数\|定期QoS" perception_log.log

echo ""
echo "[最终QoS计数（on_shutdown或最后一次定期输出）]"
grep -A 15 "QoS消息计数" perception_log.log | tail -16
echo "---"
grep "定期QoS" perception_log.log | tail -5

echo ""
echo "[front_lidar对比]"
echo "  独立观察者qos_verify: 729条 (丢失5.1%) - DDS层"
echo "  MultiThreadedExecutor: 16条 (标定完成时, 丢失97.9%) - 前11秒"
PERCEPTION_COUNT=$(grep "QoS.*front_lidar.*条" perception_log.log | tail -1 | grep -oP '\d+(?=条)' | head -1)
PERIODIC_COUNT=$(grep "定期QoS" perception_log.log | tail -1 | grep -oP 'front_lidar=\K\d+')
echo "  SingleThreadedExecutor(标定完成时): ${PERCEPTION_COUNT:-未知}条"
echo "  SingleThreadedExecutor(最后一次定期输出): ${PERIODIC_COUNT:-未知}条"

echo ""
echo "[标定结果]"
grep -E "标定.*尝试|标定NaN防护|标定完成" perception_log.log | head -3

echo ""
echo "[processLidarBranch耗时统计]"
grep -oP '耗时\] \K[\d.]+(?=ms)' perception_log.log | python3 -c "
import sys
times = [float(x) for x in sys.stdin]
if times:
    print(f'  样本数: {len(times)}, 平均: {sum(times)/len(times):.2f}ms, 最大: {max(times):.2f}ms')
"
