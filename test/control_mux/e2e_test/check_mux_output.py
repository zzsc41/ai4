#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E2E 断言节点：校验 ``/mavros/setpoint_raw/local`` 只放行当前 ``/state`` 对应通道。

策略（方案 A：两路定频 + ``/state`` 定时切换 + 窗口断言）
--------------------------------------------------------
- 订阅 ``/state`` 记录状态时间线，订阅 ``/mavros/setpoint_raw/local`` 记录每条转发指令；
- 由指令的 ``header.frame_id`` 辨认来源通道（``radar_src`` → 雷达，``visual_src`` → 视觉）；
- 断言：任一刻转发出来的指令都应属于「该时刻 ``/state`` 对应通道」（切换瞬间留
  ``~grace`` 秒容差）；且雷达 / 视觉都被放行过、三种状态都出现过。

参数（rosparam）
----------------
:~duration   float  采集时长 s                             默认 12.0
:~min_msgs   int    最少收到转发指令数                     默认 300
:~grace      float  状态切换后容差 s（忽略过渡期内的判定）   默认 0.2

退出码：0 = PASS，1 = FAIL。
"""
from __future__ import annotations

import bisect
import sys
import time

import rospy
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

RADAR_FRAME = "radar_src"
VISUAL_FRAME = "visual_src"
CH_RADAR = "/ctrl/radar"
CH_VISUAL = "/ctrl/visual"

#: 期望出现的三种状态（覆盖视觉锁定 + 雷达回退）
EXPECTED_STATES = ("RADAR_GUIDE", "VISUAL_LOCK", "SEARCH/RTH")


def _expected_channel(state):
    """状态 → 期望放行通道（与 design/函数接口.md 7.1 仲裁规则一致）。"""
    return CH_VISUAL if state == "VISUAL_LOCK" else CH_RADAR


class Checker:
    def __init__(self):
        self.duration = rospy.get_param("~duration", 12.0)
        self.min_msgs = int(rospy.get_param("~min_msgs", 300))
        self.grace = rospy.get_param("~grace", 0.2)

        self.state_log = []   # [(t, state)]，仅记录变化
        self.cmd_log = []     # [(t, channel)]，每条转发指令
        self.seen_states = set()

        rospy.Subscriber("/state", String, self.on_state, queue_size=20)
        rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget,
                         self.on_cmd, queue_size=200)

    def on_state(self, msg):
        state = msg.data
        self.seen_states.add(state)
        if not self.state_log or self.state_log[-1][1] != state:
            self.state_log.append((rospy.get_time(), state))

    def on_cmd(self, msg):
        fid = msg.header.frame_id
        if fid == VISUAL_FRAME:
            ch = CH_VISUAL
        elif fid == RADAR_FRAME:
            ch = CH_RADAR
        else:
            ch = "unknown:" + fid
        self.cmd_log.append((rospy.get_time(), ch))

    @staticmethod
    def _fail(reason):
        print("FAIL: " + reason)
        return 1

    def _state_at(self, t):
        """查 t 时刻最近一次状态（状态时间线此前最后一次变化）。"""
        if not self.state_log:
            return None
        ts = [x[0] for x in self.state_log]
        i = bisect.bisect_right(ts, t) - 1
        return self.state_log[0][1] if i < 0 else self.state_log[i][1]

    def run(self):
        deadline = time.time() + self.duration
        rate = rospy.Rate(20)
        while time.time() < deadline and not rospy.is_shutdown():
            rate.sleep()
        return self._assert()

    def _assert(self):
        n = len(self.cmd_log)
        print("[check] 收到转发指令 %d 条（期望 >= %d）" % (n, self.min_msgs))
        if n < self.min_msgs:
            return self._fail("转发指令数据流不足（%d < %d）" % (n, self.min_msgs))
        if not self.state_log:
            return self._fail("未收到任何 /state")

        change_ts = [t for t, _ in self.state_log]
        mismatch = []
        for t, ch in self.cmd_log:
            if ch == _expected_channel(self._state_at(t)):
                continue
            # 切换瞬间容差：附近有状态变化则跳过（避免跨话题时序抖动误判）
            if any(abs(t - ct) <= self.grace for ct in change_ts):
                continue
            mismatch.append((t, ch, _expected_channel(self._state_at(t))))

        forwarded = {ch for _, ch in self.cmd_log}
        print("[check] 放行通道 = %s" % sorted(forwarded))
        print("[check] 出现状态 = %s" % sorted(self.seen_states))
        print("[check] 违规放行 = %d 条" % len(mismatch))

        if mismatch:
            t, ch, exp = mismatch[0]
            return self._fail("状态与放行通道不符：t=%.3f 收到 %s 期望 %s（共 %d 条）"
                              % (t, ch, exp, len(mismatch)))
        if CH_RADAR not in forwarded:
            return self._fail("从未放行雷达通道 %s" % CH_RADAR)
        if CH_VISUAL not in forwarded:
            return self._fail("从未放行视觉通道 %s" % CH_VISUAL)
        missing = set(EXPECTED_STATES) - self.seen_states
        if missing:
            return self._fail("未出现状态 %s" % sorted(missing))

        print("PASS: /control_mux 事件转发正确 —— 始终只放行当前 /state 对应通道")
        return 0


def main():
    rospy.init_node("e2e_check_mux")
    sys.exit(Checker().run())


if __name__ == "__main__":
    main()
