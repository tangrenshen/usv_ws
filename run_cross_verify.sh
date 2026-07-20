#!/bin/bash
# 交叉验证v2：独立观察者(qos_verify_node) vs perception_node 的接收率对比
# 目的：定位丢包发生在DDS层还是executor/回调调度层
# 修复：增加启动等待，确保perception_node完全启动后再播放bag

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "========================================"
echo "交叉验证v2：DDS层 vs executor层丢包定位"
echo "========================================"
echo ""

echo "[1/7] 清理所有进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "qos_verify" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 3
ps aux | grep -E "perception_node|boat_logger|qos_verify|ros2 bag play" | grep -v grep || echo "  ✅ 无残留进程"

echo ""
echo "[2/7] 清理旧文件..."
rm -f boat_log.jsonl perception_log.log qos_verify.log /tmp/qos_verify_count.txt

echo ""
echo "[3/7] 启动perception_node（正常速度，当前代码状态）..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
nohup ros2 run usv_perception perception_node > perception_log.log 2>&1 &
PERCEPTION_PID=$!
echo "  perception_node PID: $PERCEPTION_PID"

echo ""
echo "[4/7] 等待perception_node完全启动（最多30秒）..."
for i in $(seq 1 30); do
  if grep -q "初始化完毕" perception_log.log 2>/dev/null; then
    echo "  ✅ perception_node已启动 (${i}秒)"
    break
  fi
  sleep 1
  if [ $i -eq 30 ]; then
    echo "  ❌ perception_node启动超时"
    echo "  日志内容："
    cat perception_log.log
    exit 1
  fi
done

echo ""
echo "[5/7] 启动独立观察者 qos_verify_node（订阅front_lidar）..."
nohup python3 qos_verify_node.py > qos_verify.log 2>&1 &
QOS_PID=$!
echo "  qos_verify_node PID: $QOS_PID"
sleep 3

echo ""
echo "[6/7] 播放bag（正常速度 --rate 1.0，完整播放约90秒）..."
echo "  开始时间: $(date '+%H:%M:%S')"
START_SEC=$(date +%s)
ros2 bag play ~/bags/bag_09_03_17/ --clock --rate 1.0
END_SEC=$(date +%s)
echo "  结束时间: $(date '+%H:%M:%S')"
echo "  实际播放时长: $((END_SEC - START_SEC))秒"
echo "  ✅ bag播放完成"

echo ""
echo "[清理] 等待处理完剩余数据..."
sleep 10

echo ""
echo "[7/7] 停止进程..."
kill $PERCEPTION_PID 2>/dev/null || true
kill $QOS_PID 2>/dev/null || true
sleep 3
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "qos_verify" 2>/dev/null || true
sleep 1

echo ""
echo "========================================"
echo "交叉验证结果"
echo "========================================"

echo ""
echo "[A] 独立观察者 qos_verify_node 收到的 front_lidar 消息数："
QOS_COUNT=$(cat /tmp/qos_verify_count.txt 2>/dev/null || echo "0")
echo "  $QOS_COUNT 条"

echo ""
echo "[B] perception_node 的 QoS 计数："
grep -A 10 "QoS消息计数" perception_log.log | head -12

echo ""
echo "[C] perception_node front_lidar 计数："
grep "QoS.*front_lidar.*条" perception_log.log || echo "  (未找到QoS计数)"

echo ""
echo "[D] 标定结果："
grep -E "标定.*尝试|标定NaN防护|标定完成" perception_log.log | head -5

echo ""
echo "========================================"
echo "判定"
echo "========================================"
echo "期望bag消息数: 768条 (front_lidar)"
echo "  独立观察者: $QOS_COUNT 条"
PERCEPTION_COUNT=$(grep "QoS.*front_lidar.*条" perception_log.log | grep -oP '\d+(?=条)' | head -1)
echo "  perception_node: ${PERCEPTION_COUNT:-未知} 条"
echo ""
if [ -n "$QOS_COUNT" ] && [ "$QOS_COUNT" -gt 0 ] 2>/dev/null; then
  QOS_LOSS=$(python3 -c "print(f'{(1-$QOS_COUNT/768)*100:.1f}%')")
  echo "  独立观察者丢失率: $QOS_LOSS"
fi
if [ -n "$PERCEPTION_COUNT" ] && [ "$PERCEPTION_COUNT" -gt 0 ] 2>/dev/null; then
  PERC_LOSS=$(python3 -c "print(f'{(1-$PERCEPTION_COUNT/768)*100:.1f}%')")
  echo "  perception_node丢失率: $PERC_LOSS"
fi

echo ""
echo "========================================"
echo "判定逻辑"
echo "========================================"
if [ -n "$QOS_COUNT" ] && [ -n "$PERCEPTION_COUNT" ] && [ "$QOS_COUNT" -gt 0 ] 2>/dev/null && [ "$PERCEPTION_COUNT" -gt 0 ] 2>/dev/null; then
  if [ "$QOS_COUNT" -gt 300 ] && [ "$PERCEPTION_COUNT" -lt 50 ]; then
    echo "→ 独立观察者收到多、perception_node收到少"
    echo "→ 丢包在executor/回调调度层"
    echo "→ 下一步：SingleThreadedExecutor对照实验"
  elif [ "$QOS_COUNT" -lt 100 ]; then
    echo "→ 独立观察者也丢包严重"
    echo "→ 丢包在DDS层"
    echo "→ 下一步：QoS/RMW排查（换RMW实现）"
  else
    echo "→ 需要进一步分析"
  fi
fi
