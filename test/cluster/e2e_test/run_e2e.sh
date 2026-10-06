#!/usr/bin/env bash
# 模块 1 点迹聚类 —— 实际数据流全打通 E2E 测试
#
# 编排：roscore → cluster_node → [阶段A: fake_radar(有目标) + 断言]
#                              → [阶段B: fake_radar(无目标) + 断言]
# 跑完自动清理后端进程；日志落在 test/cluster/e2e_test/logs/。
#
# 用法：bash test/cluster/e2e_test/run_e2e.sh
set -u

REPO="/home/demo/桌面/ai4"
WS="$REPO/catkin_ws"
E2E="$REPO/test/cluster/e2e_test"
LOGDIR="$E2E/logs"
mkdir -p "$LOGDIR"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

CORE_PID=""
RADAR_PID=""
CLUSTER_PID=""

cleanup() {
  [ -n "$RADAR_PID" ] && kill "$RADAR_PID" 2>/dev/null
  [ -n "$CLUSTER_PID" ] && kill "$CLUSTER_PID" 2>/dev/null
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

echo "=== [2/4] 启动 cluster_node ==="
rosrun uav_tracking cluster_node.py _z_thresh:=0.3 _eps:=0.5 _min_samples:=5 \
  > "$LOGDIR/cluster.log" 2>&1 &
CLUSTER_PID=$!
sleep 1
echo "  cluster_node pid=$CLUSTER_PID"

run_phase() {
  local phase="$1" with_target="$2" mode="$3" rc=0
  echo "=== $phase: fake_radar(with_target=$with_target) + check(mode=$mode) ==="
  python3 "$E2E/fake_radar.py" _with_target:="$with_target" \
    _x:=8.0 _y:=3.0 _z:=2.0 _rate:=10.0 \
    > "$LOGDIR/fake_radar_$mode.log" 2>&1 &
  RADAR_PID=$!
  wait_topic /radar/points   || echo "  (警告) /radar/points 未及时出现"
  wait_topic /radar/centroid || echo "  (警告) /radar/centroid 未及时出现"
  sleep 0.5
  python3 "$E2E/check_centroid.py" _mode:="$mode" _duration:=6.0 _min_msgs:=20
  rc=$?
  kill "$RADAR_PID" 2>/dev/null
  wait "$RADAR_PID" 2>/dev/null
  RADAR_PID=""
  sleep 0.5
  return $rc
}

echo "=== [3/4] 阶段 A（有目标）==="
run_phase "阶段A" true target
RC_A=$?

echo "=== [4/4] 阶段 B（无目标）==="
run_phase "阶段B" false empty
RC_B=$?

echo "----------------------------------------"
if [ "$RC_A" -eq 0 ] && [ "$RC_B" -eq 0 ]; then
  echo "E2E RESULT: PASS  (A=0, B=0)"
  exit 0
else
  echo "E2E RESULT: FAIL  (A=$RC_A, B=$RC_B)  日志见 $LOGDIR"
  exit 1
fi
