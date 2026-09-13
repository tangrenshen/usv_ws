#!/bin/bash
for p in $(pgrep -f "bag record -o /home/lyf040817/boatmiss"); do [ "$p" != "$$" ] && kill -TERM $p; done
sleep 5
for p in $(pgrep -f "bag record -o /home/lyf040817/boatmiss"); do [ "$p" != "$$" ] && kill -9 $p; done
sleep 1
echo "剩余录制器: $(pgrep -fc 'bag record -o /home/lyf040817/boatmiss' ) (含本脚本自身匹配时为1)"
