#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假控制源（E2E 测试用）：持续发布两路指令 + 按时间切换 ``/state``。

无 ``/navigator`` / ``/pid_controller`` / ``/state_machine`` 时，用它打通
``/state + /ctrl/radar + /ctrl/visual → (/control_mux) → /mavros/setpoint_raw/local`` 链路。

行为
----
- ``/ctrl/radar``、``/ctrl/visual`` 两路**均以 ``~rate``（默认 50Hz）定频发布**；
- ``/state`` 按 ``~states`` 顺序、每 ``~phase_seconds`` 秒切换一次并循环；
- 指令用 ``header.frame_id``（``radar_src`` / ``visual_src``）与 ``position.x``
  标记来源，便于下游 checker 辨认「这条到底来自哪一路」。

参数（rosparam，均可选）
------------------------
:~rate           float  两路指令发布频率 Hz     默认 50.0
:~state_rate     float  ``/state`` 重复发布频率  默认 20.0
:~phase_seconds  float  每个状态持续秒数        默认 4.0
:~states         str    状态序列（逗号分隔）     默认 "RADAR_GUIDE,VISUAL_LOCK,SEARCH/RTH"
"""
from __future__ import annotations

import rospy
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

RADAR_FRAME = "radar_src"
VISUAL_FRAME = "visual_src"
RADAR_MARKER = 1.0
VISUAL_MARKER = 2.0

DEFAULT_STATES = "RADAR_GUIDE,VISUAL_LOCK,SEARCH/RTH"


def _make_msg(frame_id, marker):
    m = PositionTarget()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = frame_id
    m.position.x = marker
    return m


class FakeMuxInput:
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.state_rate_hz = rospy.get_param("~state_rate", 20.0)
        self.phase_seconds = rospy.get_param("~phase_seconds", 4.0)
        states = rospy.get_param("~states", DEFAULT_STATES)
        self.states = [s.strip() for s in states.split(",") if s.strip()]

        self.radar_pub = rospy.Publisher("/ctrl/radar", PositionTarget, queue_size=1)
        self.visual_pub = rospy.Publisher("/ctrl/visual", PositionTarget, queue_size=1)
        self.state_pub = rospy.Publisher("/state", String, queue_size=1)
        rospy.loginfo("[fake_mux_input] rate=%.1fHz phase=%.1fs states=%s",
                      self.rate_hz, self.phase_seconds, self.states)

    def _state_at(self, t):
        idx = int(t // self.phase_seconds) % len(self.states)
        return self.states[idx]

    def spin(self):
        start = rospy.Time.now().to_sec()
        last_state_t = -1.0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - start
            self.radar_pub.publish(_make_msg(RADAR_FRAME, RADAR_MARKER))
            self.visual_pub.publish(_make_msg(VISUAL_FRAME, VISUAL_MARKER))
            if t - last_state_t >= 1.0 / self.state_rate_hz:
                self.state_pub.publish(String(data=self._state_at(t)))
                last_state_t = t
            rate.sleep()


def main():
    rospy.init_node("fake_mux_input")
    FakeMuxInput().spin()


if __name__ == "__main__":
    main()
