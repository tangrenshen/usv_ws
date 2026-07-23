#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash

run_single_test() {
    echo "[1/5] 清理所有进程..."
    pkill -9 -f "perception_node" 2>/dev/null || true
    pkill -9 -f "boat_logger" 2>/dev/null || true
    pkill -9 -f "ros2 bag play" 2>/dev/null || true
    sleep 3

    echo "[2/5] 清理旧文件..."
    rm -f boat_log.jsonl perception_log.log

    echo "[3/5] 启动感知节点..."
    source install/setup.bash
    nohup ros2 run usv_perception perception_node > perception_log.log 2>&1 &
    PERCEPTION_PID=$!
    sleep 3

    echo "[4/5] 启动日志记录器..."
    nohup python3 boat_logger.py > boat_logger.out 2>&1 &
    LOGGER_PID=$!
    sleep 1

    echo "[5/5] 播放bag..."
    ros2 bag play ~/bags/bag_09_03_17/ --clock > /dev/null 2>&1
    echo "  ✅ bag播放完成"

    echo "[清理] 等待处理完剩余数据..."
    sleep 8

    echo "[清理] 停止进程..."
    kill $PERCEPTION_PID 2>/dev/null || true
    kill $LOGGER_PID 2>/dev/null || true
    sleep 2

    echo "[清理] 确保进程已停止..."
    pkill -9 -f "perception_node" 2>/dev/null || true
    pkill -9 -f "boat_logger" 2>/dev/null || true
    sleep 1

    echo "[评估] 运行boat_evaluator..."
    python3 boat_evaluator.py
}

run_test() {
    local name="$1"
    local iterations="$2"

    echo "=== 运行 ${name} ${iterations} 次 ==="

    local recalls=()
    local fprs=()

    for i in $(seq 1 $iterations); do
        echo "--- ${name} 第 ${i} 次 ---"
        RESULT=$(run_single_test 2>&1)
        echo "$RESULT"

        RECALL=$(echo "$RESULT" | grep "平均召回率" | sed 's/.*: \([0-9.]*\)%/\1/')
        FPR=$(echo "$RESULT" | grep "平均误检率" | sed 's/.*: \([0-9.]*\)%/\1/')

        recalls+=("$RECALL")
        fprs+=("$FPR")

        echo "第 ${i} 次: 召回率=${RECALL}% 误检率=${FPR}%"
    done

    IFS=$'\n' sorted_recalls=($(sort -g <<<"${recalls[*]}"))
    IFS=$'\n' sorted_fprs=($(sort -g <<<"${fprs[*]}"))
    unset IFS

    local n=${#sorted_recalls[@]}
    local median_recall=${sorted_recalls[$((n/2))]}
    local median_fpr=${sorted_fprs[$((n/2))]}

    echo "=== ${name} 结果汇总 ==="
    echo "召回率: ${recalls[*]}% → 中值=${median_recall}%"
    echo "误检率: ${fprs[*]}% → 中值=${median_fpr}%"

    echo "${name}_recall_median=${median_recall}" >> comparison_results.txt
    echo "${name}_fpr_median=${median_fpr}" >> comparison_results.txt

    return 0
}

echo "[确认] 当前main.cpp状态："
diff -q src/usv_perception/src/main.cpp src/usv_perception/src/main.cpp.bak > /dev/null 2>&1 && echo "  警告：与main.cpp.bak相同（历史代码）！" || echo "  确认为当前代码"

echo "[构建] 编译当前代码..."
colcon build --symlink-install > /dev/null 2>&1

run_test "当前代码" 3

echo ""
echo "=== 最终对比结果（含历史+当前）==="
cat comparison_results.txt

HIST_RECALL=$(grep "历史代码_recall_median" comparison_results.txt | tail -1 | sed 's/.*=//')
CURR_RECALL=$(grep "当前代码_recall_median" comparison_results.txt | tail -1 | sed 's/.*=//')
HIST_FPR=$(grep "历史代码_fpr_median" comparison_results.txt | tail -1 | sed 's/.*=//')
CURR_FPR=$(grep "当前代码_fpr_median" comparison_results.txt | tail -1 | sed 's/.*=//')

RECALL_DIFF=$(echo "$CURR_RECALL - $HIST_RECALL" | bc)
FPR_DIFF=$(echo "$CURR_FPR - $HIST_FPR" | bc)

echo ""
echo "召回率差异: 当前(${CURR_RECALL}%) - 历史(${HIST_RECALL}%) = ${RECALL_DIFF}pp"
echo "误检率差异: 当前(${CURR_FPR}%) - 历史(${HIST_FPR}%) = ${FPR_DIFF}pp"
