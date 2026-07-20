#!/bin/bash
# 标定重试实验：验证NaN防护改进+帧数门槛后，标定是否能在数据充足时收敛
# 改动：1)NaN时不放弃,允许重试 2)每路至少10帧才开始标定
# 跑完后分析：标定结果、QoS计数、召回率、cluster_diagnostic

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "========================================"
echo "标定重试实验：NaN防护改进+帧数门槛"
echo "========================================"
echo ""
echo "改动说明:"
echo "  1. NaN时不置位calib_done_,允许下一个周期重试"
echo "  2. NaN时回退calib_attempts_(不消耗重试次数)"
echo "  3. 增加nan_retry_count_(上限30次)"
echo "  4. 帧数门槛:每路至少10帧才开始标定(修复min_points=30的点数门槛设计缺陷)"
echo ""

echo "[1/7] 清理所有进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "qos_verify" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 3

echo ""
echo "[2/7] 清理旧文件..."
rm -f boat_log.jsonl perception_log.log boat_logger.out

echo ""
echo "[3/7] 启动perception_node..."
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
echo "[4/7] 启动boat_logger..."
nohup python3 boat_logger.py > boat_logger.out 2>&1 &
LOGGER_PID=$!
echo "  boat_logger PID: $LOGGER_PID"
sleep 2

echo ""
echo "[5/7] 播放bag（正常速度 --rate 1.0）..."
echo "  开始时间: $(date '+%H:%M:%S')"
START_SEC=$(date +%s)
ros2 bag play ~/bags/bag_09_03_17/ --clock --rate 1.0
END_SEC=$(date +%s)
echo "  结束时间: $(date '+%H:%M:%S')"
echo "  实际播放时长: $((END_SEC - START_SEC))秒"

echo ""
echo "[清理] 等待处理完剩余数据..."
sleep 10

echo ""
echo "[6/7] 优雅停止perception_node..."
kill -TERM $PERCEPTION_PID 2>/dev/null
for i in $(seq 1 15); do
  if ! kill -0 $PERCEPTION_PID 2>/dev/null; then
    echo "  ✅ 节点已退出 (${i}秒)"
    break
  fi
  sleep 1
done
kill $LOGGER_PID 2>/dev/null || true
sleep 2
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true

echo ""
echo "========================================"
echo "实验结果分析"
echo "========================================"

echo ""
echo "[A] 标定过程（关键）"
echo "  --- 帧数门槛等待日志 ---"
grep "等待数据积累" perception_log.log | head -3
echo "  --- 标定尝试日志 ---"
grep "标定.*第.*次尝试" perception_log.log
echo "  --- NaN防护日志 ---"
grep "标定NaN防护" perception_log.log | head -10
echo "  --- 标定完成/放弃 ---"
grep -E "标定完成|放弃标定|达到最大尝试" perception_log.log | head -5

echo ""
echo "[B] 标定核查（修正量）"
grep "标定核查-真实修正量" perception_log.log | head -7

echo ""
echo "[C] QoS最终计数"
grep "定期QoS" perception_log.log | tail -3
echo "---"
grep -A 10 "QoS消息计数" perception_log.log | tail -12

echo ""
echo "[D] boat_log记录统计"
python3 -c "
import json
types = {}
with open('boat_log.jsonl') as f:
    for line in f:
        try:
            r = json.loads(line)
            t = r.get('type', 'unknown')
            types[t] = types.get(t, 0) + 1
        except: pass
for t, c in sorted(types.items()):
    print(f'  {t}: {c}')
"

echo ""
echo "[E] 召回率评估"
python3 boat_evaluator.py 2>&1 | tail -15

echo ""
echo "[F] boat簇分析（size_z分布）"
python3 analyze_boat_clusters.py 2>&1 | head -20
