#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

for run in 1 2 3 4 5; do
    echo "=== 第 $run 次测试 ==="
    
    echo "[1/5] 清理所有进程..."
    pkill -9 -f "perception_node" 2>/dev/null || true
    pkill -9 -f "boat_logger" 2>/dev/null || true
    pkill -9 -f "ros2 bag play" 2>/dev/null || true
    sleep 3
    
    echo "[2/5] 检查残留进程..."
    ps aux | grep -E "perception_node|boat_logger|ros2 bag play" | grep -v grep || echo "  ✅ 无残留进程"
    
    echo "[3/5] 清理旧文件..."
    rm -f boat_log.jsonl perception_log.log
    
    echo "[4/5] 启动感知节点..."
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    nohup ros2 run usv_perception perception_node > perception_log.log 2>&1 &
    PERCEPTION_PID=$!
    echo "  perception_node PID: $PERCEPTION_PID"
    
    echo "[4b/5] 等待节点初始化..."
    sleep 3
    
    echo "[4c/5] 启动日志记录器..."
    nohup python3 boat_logger.py > boat_logger.out 2>&1 &
    LOGGER_PID=$!
    echo "  boat_logger PID: $LOGGER_PID"
    sleep 1
    
    echo "[5/5] 播放bag..."
    ros2 bag play ~/bags/bag_09_03_17/ --clock
    echo "  ✅ bag播放完成"
    
    echo "[清理] 等待处理完剩余数据..."
    sleep 8
    
    echo "[清理] 发送SIGTERM停止进程..."
    kill $PERCEPTION_PID 2>/dev/null || true
    kill $LOGGER_PID 2>/dev/null || true
    
    echo "[清理] 等待进程优雅退出(最多10秒)..."
    for i in {1..10}; do
        sleep 1
        ps -p $PERCEPTION_PID > /dev/null 2>&1 || break
        echo "  等待中... ($i/10)"
    done
    
    echo "[清理] 检查是否需要强制终止..."
    ps -p $PERCEPTION_PID > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "  ⚠️ perception_node未优雅退出，执行强制终止"
        pkill -9 -f "perception_node" 2>/dev/null || true
        pkill -9 -f "boat_logger" 2>/dev/null || true
    else
        echo "  ✅ perception_node已优雅退出"
    fi
    sleep 1
    
    echo "[统计] 分析结果..."
    echo -n "  gt_records: "
    gt_count=$(grep -c '"type": "gt"' boat_log.jsonl 2>/dev/null || echo "0")
    echo "$gt_count"
    echo -n "  det_records: "
    det_count=$(grep -c '"type": "det"' boat_log.jsonl 2>/dev/null || echo "0")
    echo "$det_count"
    
    echo "[评估] 运行boat_evaluator..."
    python3 boat_evaluator.py
    
    echo "[QoS] 提取消息计数..."
    grep "QoS消息计数" perception_log.log
    grep "\[QoS\]" perception_log.log | grep -v "QoS消息计数" | head -15
    
    echo "[保存] 保存本次测试数据..."
    cp boat_log.jsonl "boat_log_run${run}.jsonl"
    cp perception_log.log "perception_log_run${run}.log"
    
    echo ""
done

echo "=== 全部5次测试完成 ==="
