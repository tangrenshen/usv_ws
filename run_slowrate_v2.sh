#!/bin/bash
# 降速实验v2：完整bag + NaN检查 + 自动重跑
# 用法: bash run_slowrate_v2.sh
# 最多重跑3次，直到标定无NaN

cd "$(dirname "${BASH_SOURCE[0]}")"
MAX_RETRIES=3

run_once() {
    local attempt=$1
    echo ""
    echo "========================================"
    echo "降速实验 第 ${attempt} 次尝试"
    echo "========================================"

    echo "[1/5] 清理所有进程..."
    pkill -9 -f "perception_node" 2>/dev/null || true
    pkill -9 -f "boat_logger" 2>/dev/null || true
    pkill -9 -f "ros2 bag play" 2>/dev/null || true
    sleep 3

    echo "[2/5] 检查残留进程..."
    ps aux | grep -E "perception_node|boat_logger|ros2 bag play" | grep -v grep || echo "  ✅ 无残留进程"

    echo "[3/5] 清理旧文件..."
    rm -f boat_log.jsonl perception_log.log

    echo "[4/5] 启动感知节点(定时器周期×10, stale_sec×10)..."
    source install/setup.bash
    nohup ros2 run usv_perception perception_node \
      --ros-args \
      -p image_timer_period_sec:=2.0 \
      -p lidar_timer_period_sec:=1.0 \
      -p calib_timer_period_sec:=10.0 \
      -p image_stale_sec:=5.0 \
      -p lidar_stale_sec:=5.0 \
      > perception_log.log 2>&1 &
    PERCEPTION_PID=$!
    echo "  perception_node PID: $PERCEPTION_PID"

    sleep 5

    echo "[4b/5] 启动日志记录器..."
    nohup python3 boat_logger.py > boat_logger.out 2>&1 &
    LOGGER_PID=$!
    sleep 1

    echo "[5/5] 播放bag(--rate 0.1, 完整播放)..."
    echo "  预计耗时约15分钟，请耐心等待..."
    ros2 bag play ~/bags/bag_09_03_17/ --clock --rate 0.1
    echo "  ✅ bag播放完成"

    echo "[清理] 等待处理完剩余数据..."
    sleep 30

    echo "[清理] 停止进程..."
    kill $PERCEPTION_PID 2>/dev/null || true
    kill $LOGGER_PID 2>/dev/null || true
    sleep 3
    pkill -9 -f "perception_node" 2>/dev/null || true
    pkill -9 -f "boat_logger" 2>/dev/null || true
    sleep 1

    echo ""
    echo "[NaN检查] 检查标定是否有NaN..."
    if grep -q "标定NaN防护\|平移修正=-nan\|旋转修正=nan" perception_log.log; then
        echo "  ❌ 检测到标定NaN！本次运行作废"
        grep "标定NaN防护\|平移修正=-nan\|旋转修正=nan" perception_log.log | head -10
        return 1
    else
        echo "  ✅ 标定无NaN，本次运行有效"
        return 0
    fi
}

# 最多重跑MAX_RETRIES次
for i in $(seq 1 $MAX_RETRIES); do
    if run_once $i; then
        echo ""
        echo "========================================"
        echo "✅ 降速实验成功（第 ${i} 次尝试）"
        echo "========================================"

        echo ""
        echo "[QoS消息计数]"
        grep "QoS" perception_log.log

        echo ""
        echo "[标定结果]"
        grep -E "\[标定\]|标定完成|标定NaN" perception_log.log

        echo ""
        echo "[odom记录数]"
        grep '"type":"odom"' boat_log.jsonl | wc -l

        echo ""
        echo "[召回率评估]"
        python3 boat_evaluator.py

        echo ""
        echo "[boat簇分析]"
        python3 analyze_boat_clusters.py 2>&1 | head -25

        echo ""
        echo "[转换链路验证]"
        python3 verify_transform_v2.py 2>&1 | tail -15

        exit 0
    else
        echo "  第 ${i} 次失败，准备重跑..."
        sleep 5
    fi
done

echo ""
echo "========================================"
echo "❌ 降速实验失败：${MAX_RETRIES}次尝试都检测到标定NaN"
echo "========================================"
echo "这说明标定NaN是高频问题，需要先修SVD退化问题"
