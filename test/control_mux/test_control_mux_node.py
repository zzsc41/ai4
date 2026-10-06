"""模块 7 节点壳（``/control_mux``）单元测试。

覆盖 ``scripts/control_mux_node.py`` 中**不依赖 ROS master** 的部分（同 cluster 粒度）：

- ``state_msg_to_str``：``std_msgs/String`` → ``str``；
- 模块级常量（话题名 / 初值 / 队列）；
- 用**真实** ``mavros_msgs/PositionTarget`` 串起 ``ControlMux`` 的放行 / 丢弃逻辑，
  验证「壳 ↔ 核」接口契约（指令为不透明对象、原样转发）。

运行前提：已 ``source /opt/ros/noetic/setup.bash`` 与
``source catkin_ws/devel/setup.bash``（需能 import ``rospy`` 与 ``mavros_msgs``）；
否则本文件整体跳过，不影响纯逻辑测试。
"""
import os
import sys

import pytest

# 将 catkin 包 scripts/ 加入 import 路径，使 control_mux_node 可被 import。
_SCRIPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "catkin_ws", "src", "uav_tracking", "scripts",
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# 依赖 ROS（rospy + mavros_msgs）；缺失则本文件整体跳过
pytest.importorskip("rospy")
pytest.importorskip("mavros_msgs.msg")

from mavros_msgs.msg import PositionTarget  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import control_mux_node  # noqa: E402
from control_mux import (  # noqa: E402
    CHANNEL_RADAR,
    CHANNEL_VISUAL,
    STATE_VISUAL_LOCK,
    ControlMux,
)


def _target(marker, frame_id):
    """构造一条带来源标记的 PositionTarget（mux 不解析内容，仅原样转发）。"""
    m = PositionTarget()
    m.header.frame_id = frame_id
    m.position.x = marker
    return m


# ---------------------------------------------------------------------------
# state_msg_to_str / 模块常量
# ---------------------------------------------------------------------------
def test_state_msg_to_str_reads_data():
    assert control_mux_node.state_msg_to_str(
        String(data="VISUAL_LOCK")) == "VISUAL_LOCK"


def test_state_msg_to_str_empty_passthrough():
    # 空串按未知状态处理（下游 select_channel 回退雷达）
    assert control_mux_node.state_msg_to_str(String(data="")) == ""


def test_module_constants():
    assert control_mux_node.STATE_TOPIC == "/state"
    assert control_mux_node.SETPOINT_TOPIC == "/mavros/setpoint_raw/local"
    assert control_mux_node.INITIAL_STATE == "RADAR_GUIDE"
    assert control_mux_node.QUEUE_SIZE == 1


# ---------------------------------------------------------------------------
# 壳 ↔ 核 契约：真实 PositionTarget 的放行 / 丢弃
# ---------------------------------------------------------------------------
def test_radar_forwarded_in_radar_state():
    mux = ControlMux()
    msg = _target(1.0, "radar_src")
    assert mux.on_command(CHANNEL_RADAR, msg) is msg


def test_visual_dropped_in_radar_state():
    mux = ControlMux()
    assert mux.on_command(CHANNEL_VISUAL, _target(2.0, "visual_src")) is None


def test_switch_to_visual_changes_forwarding():
    mux = ControlMux()
    r = _target(1.0, "radar_src")
    v = _target(2.0, "visual_src")
    assert mux.on_command(CHANNEL_RADAR, r) is r

    mux.on_state(STATE_VISUAL_LOCK)
    assert mux.on_command(CHANNEL_RADAR, r) is None
    assert mux.on_command(CHANNEL_VISUAL, v) is v


def test_visual_command_forwarded_unchanged():
    # 只转发不生成：返回同一对象，内容不被修改
    mux = ControlMux(STATE_VISUAL_LOCK)
    v = _target(2.0, "visual_src")
    out = mux.on_command(CHANNEL_VISUAL, v)
    assert out is v
    assert out.header.frame_id == "visual_src"
    assert out.position.x == 2.0
