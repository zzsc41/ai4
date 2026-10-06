#!/usr/bin/env bash
# 模块 8 状态机 —— 事件驱动全链路 E2E（方案 B：/detections 为节拍）
#
# 编排：roscore → state_machine_node → check_state（先订阅 /state）
#                                  → fake_sensors（脚本五段场景，驱动切换）
# 断言节点**先于**数据源启动，确保不错过早期切换（如 VISUAL_LOCK）。
# 跑完自动清理后端进程；日志落在 test/state_machine/e2e_test/logs/。
#
# 用法：bash test/state_machine/e2e_test/run_e2e.sh
set -u

REPO="/home/demo/桌面/ai4"
WS="$REPO/catkin_ws"
E2E="$REPO/test/state_machine/e2e_test"
LOGDIR="$E2E/logs"
mkdir -p "$LOGDIR"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

CORE_PID=""
SM_PID=""
CHECK_PID=""
FAKE_PID=""

cleanup() {
  [ -n "$FAKE_PID" ] && kill "$FAKE_PID" 2>/dev/null
  [ -n "$CHECK_PID" ] && kill "$CHECK_PID" 2>/dev/null
  [ -n "$SM_PID" ] && kill "$SM_PID" 2>/dev/null
  [ -n "$CORE_PID" ] && kill "$CORE_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

wait_master() { for _ in $(seq 1 50); do rosnode list >/dev/null 2>&1 && return 0; sleep 0.2; done; return 1; }
wait_topic()  { for _ in $(seq 1 50); do rostopic list 2>/dev/null | grep -qx "$1" && return 0; sleep 0.2; done; return 1; }

echo "=== [1/5] 启动 roscore ==="
if rosnode list >/dev/null 2>&1; then
  echo "  master 已在运行，复用"
else
  nohup roscore > "$LOGDIR/roscore.log" 2>&1 &
  CORE_PID=$!
  if wait_master; then
    echo "  roscore 已启动 (pid=$CORE_PID)"
  else
    echo "FAIL: roscore 未启动"
    exit 1
  fi
fi

echo "=== [2/5] 启动 state_machine_node ==="
rosrun uav_tracking state_machine_node.py \
  _conf_thresh:=0.8 _area_thresh:=0.02 \
  _n_visual_lock:=3 _m_visual_lost:=3 _k_radar_lost:=5 \
  > "$LOGDIR/state_machine.log" 2>&1 &
SM_PID=$!
sleep 1
echo "  state_machine_node pid=$SM_PID"

echo "=== [3/5] 先启动 check_state（订阅 /state，观测 16s）==="
python3 "$E2E/check_state.py" _duration:=16.0 \
  > "$LOGDIR/check_state.log" 2>&1 &
CHECK_PID=$!
sleep 1.5   # 确保订阅已建立，不错过早期切换
echo "  check_state pid=$CHECK_PID"

echo "=== [4/5] 启动 fake_sensors（五段场景, 20Hz, 每段 2.5s）==="
python3 "$E2E/fake_sensors.py" _rate:=20.0 _phase_seconds:=2.5 \
  > "$LOGDIR/fake_sensors.log" 2>&1 &
FAKE_PID=$!
sleep 0.5
echo "  fake_sensors pid=$FAKE_PID"

echo "=== [5/5] 等待断言结束 ==="
wait "$CHECK_PID"
RC=$?

echo "---- check_state 输出 ----"
cat "$LOGDIR/check_state.log"
echo "----------------------------------------"
if [ "$RC" -eq 0 ]; then
  echo "E2E RESULT: PASS"
  exit 0
else
  echo "E2E RESULT: FAIL (rc=$RC)  日志见 $LOGDIR"
  exit 1
fi
