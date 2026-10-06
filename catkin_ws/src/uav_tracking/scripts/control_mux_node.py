#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/control_mux 节点壳：``/state`` + ``/ctrl/radar`` + ``/ctrl/visual`` → ``/mavros/setpoint_raw/local``。

对应《python代码移植到ros平台方法整理.md》第五节与《design/函数接口.md》模块 7：

- **壳**（本文件）：订阅 / 发布 / 类型转换 / 日志，**不含仲裁算法**；
- **核**（``lib/control_mux.py``，指向 ``code/`` 的软链）：``ControlMux`` /
  ``select_channel`` 纯逻辑，零改动。

设计约定（``design/函数接口.md`` 模块 7 与「决策端设计原则」）
--------------------------------------------------------
- **只转发不生成**：收到哪路原样转哪路，不生成、不修改指令；
- **事件转发、无定时器**：收到即转；指令流连续性由上游 ``/navigator`` /
  ``/pid_controller`` 的固定时钟保证；
- **每路只留最新一条**（``queue_size=1``）；
- **不做 ``/state`` 超时检测**（方案 B）：状态机挂死由 ``/pid_controller`` 的
  ``stop_setpoint`` 与 PX4 OFFBOARD failsafe 两层兜底。
"""
from __future__ import annotations

import os
import sys

import rospy
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

# 把 lib/ 加入 import 路径（纯逻辑与壳解耦；lib/ 内为指向 code/ 的软链）
_LIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib")
if _LIB_DIR not in sys.path:
    sys.path.insert(0, _LIB_DIR)

from control_mux import (  # noqa: E402  纯逻辑，零改动
    CHANNEL_RADAR,
    CHANNEL_VISUAL,
    ControlMux,
)

__all__ = ["state_msg_to_str", "ControlMuxNode"]

# --- 模块级常量（代码书写原则第 6 条：默认值 / 未指定参数集中定义）---
STATE_TOPIC = "/state"
SETPOINT_TOPIC = "/mavros/setpoint_raw/local"
INITIAL_STATE = "RADAR_GUIDE"  # 未收到 /state 时回退雷达（数据类型.md 三）
QUEUE_SIZE = 1                 # 每路只留最新一条


def state_msg_to_str(msg):
    """``std_msgs/String`` → ``str``（纯转换，可单测）。

    直接取 ``msg.data``；空串按「未知状态」处理（``select_channel`` 回退雷达）。
    """
    return msg.data


class ControlMuxNode:
    """``/control_mux`` 节点：事件订阅 → ``ControlMux`` 仲裁 → 转发。"""

    def __init__(self):
        self.mux = ControlMux(INITIAL_STATE)

        # 唯一出口：转发到飞控 setpoint 话题（下游只看最新）
        self.pub = rospy.Publisher(
            SETPOINT_TOPIC, PositionTarget, queue_size=QUEUE_SIZE)
        # 输入三路：状态 + 两路指令（各只保最新一条）
        rospy.Subscriber(STATE_TOPIC, String, self.on_state, queue_size=QUEUE_SIZE)
        rospy.Subscriber(
            CHANNEL_RADAR, PositionTarget, self.on_radar, queue_size=QUEUE_SIZE)
        rospy.Subscriber(
            CHANNEL_VISUAL, PositionTarget, self.on_visual, queue_size=QUEUE_SIZE)
        rospy.loginfo("[mux] ready: initial_state=%s output=%s",
                      self.mux.state, SETPOINT_TOPIC)

    # ---- 事件层：收到即判、收到即转（本模块无定时器）----
    def on_state(self, msg):
        self.mux.on_state(state_msg_to_str(msg))

    def on_radar(self, msg):
        self._forward(CHANNEL_RADAR, msg)

    def on_visual(self, msg):
        self._forward(CHANNEL_VISUAL, msg)

    def _forward(self, channel, msg):
        """调核仲裁：选中 → 原样转发；否则丢弃。"""
        out = self.mux.on_command(channel, msg)
        if out is not None:
            self.pub.publish(out)


def main():
    rospy.init_node("control_mux")
    ControlMuxNode()
    rospy.spin()


if __name__ == "__main__":
    main()
