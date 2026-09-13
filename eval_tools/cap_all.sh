#!/bin/bash
# $1=标签  其余=节点参数；5 个 run 依次回放录制到 /home/lyf040817/boatmiss/<标签>/runN
TAG=$1; shift
D=/home/lyf040817/boatmiss/$TAG; mkdir -p $D
for i in 1 2 3 4 5; do
  echo "== $TAG run$i $(date +%T)"
  /home/lyf040817/boatmiss/cap.sh $i $D/run$i "$@" | grep -E "Duration|check|boats "
  echo "   残留节点: $(pgrep -xc perception_node)  残留回放/录制: $(pgrep -c -f ros2.bag)"
done
