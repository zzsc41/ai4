#!/usr/bin/env bash
# 模块 7 控制仲裁 —— 事件转发全链路 E2E（方案 A：两路 50Hz 定频 + /state 定时切换）
#
# 编排：roscore → control_mux_node → [fake_mux_input(两路 50Hz, state 每 4s 切换)
#                                     + check_mux_output 断言]
# 跑完自动清理后端进程；日志落在 test/control_mux/e2e_test/logs/。
#
# 用法：bash test/control_mux/e2e_test/run_e2e.sh
set -u

REPO="/home/demo/桌面/ai4"
WS="$REPO/catkin_ws"
E2E="$REPO/test/control_mux/e2e_test"
LOGDIR="$E2E/logs"
mkdir -p "$LOGDIR"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

CORE_PID=""
FAKE_PID=""
MUX_PID=""

cleanup() {
  [ -n "$FAKE_PID" ] && kill "$FAKE_PID" 2>/dev/null
  [ -n "$MUX_PID" ] && kill "$MUX_PID" 2>/dev/null
  [ -n "$CORE_PID" ] && kill "$CORE_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

wait_master() { for _ in $(seq 1 50); do rosnode list >/dev/null 2>&1 && return 0; sleep 0.2; done; return 1; }
wait_topic()  { for _ in $(seq 1 50); do rostopic list 2>/dev/null | grep -qx "$1" && return 0; sleep 0.2; done; return 1; }

echo "=== [1/4] 启动 roscore ==="
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

echo "=== [2/4] 启动 control_mux_node ==="
rosrun uav_tracking control_mux_node.py > "$LOGDIR/control_mux.log" 2>&1 &
MUX_PID=$!
sleep 1
echo "  control_mux_node pid=$MUX_PID"

echo "=== [3/4] 启动 fake_mux_input（两路 50Hz, /state 每 4s 切换）==="
python3 "$E2E/fake_mux_input.py" _rate:=50.0 _phase_seconds:=4.0 \
  > "$LOGDIR/fake_mux_input.log" 2>&1 &
FAKE_PID=$!
wait_topic /ctrl/radar               || echo "  (警告) /ctrl/radar 未及时出现"
wait_topic /ctrl/visual              || echo "  (警告) /ctrl/visual 未及时出现"
wait_topic /mavros/setpoint_raw/local || echo "  (警告) /mavros/setpoint_raw/local 未及时出现"
sleep 0.5

echo "=== [4/4] 断言 /mavros/setpoint_raw/local（12s ≈ 3 个状态阶段）==="
python3 "$E2E/check_mux_output.py" _duration:=12.0 _min_msgs:=300
RC=$?

echo "----------------------------------------"
if [ "$RC" -eq 0 ]; then
  echo "E2E RESULT: PASS"
  exit 0
else
  echo "E2E RESULT: FAIL (rc=$RC)  日志见 $LOGDIR"
  exit 1
fi
