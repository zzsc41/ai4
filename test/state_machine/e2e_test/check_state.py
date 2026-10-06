#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""断言节点（E2E 测试用）：订阅 ``/state``，跑 ``~duration`` 秒后校验状态变化序列。

期望（方案 B：``/detections`` 为节拍，N/M/K=3/3/5），即 fake_sensors 五段场景
对应的切换序列::

    VISUAL_LOCK → RADAR_GUIDE → SEARCH/RTH → RADAR_GUIDE

> 说明：``/state`` 按变化发布，节点启动时的初值 ``RADAR_GUIDE`` 早于本订阅者，
> 可能收不到，故期望只取**有意义的切换链**（不含开头的初值态）。

校验方式：把收到的 ``/state`` 序列**去重**后，判断期望序列是否为其**有序子序列**。

参数
----
~duration  float  观测时长 s，默认 16.0

退出码：0=通过，1=失败
"""
from __future__ import annotations

import sys

import rospy
from std_msgs.msg import String

EXPECTED = ["VISUAL_LOCK", "RADAR_GUIDE", "SEARCH/RTH", "RADAR_GUIDE"]


def _dedupe(seq):
    """去掉相邻重复项（``/state`` 按变化发布，仍可能因订阅时机重复）。"""
    out = []
    for item in seq:
        if not out or out[-1] != item:
            out.append(item)
    return out


def _is_subsequence(sub, seq):
    """``sub`` 是否为 ``seq`` 的有序子序列。"""
    it = iter(seq)
    return all(any(x == s for x in it) for s in sub)


def main():
    rospy.init_node("check_state")
    duration = rospy.get_param("~duration", 16.0)

    observed = []

    def on_state(msg):
        observed.append(msg.data)

    rospy.Subscriber("/state", String, on_state, queue_size=100)

    deadline = rospy.Time.now() + rospy.Duration(duration)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        rate.sleep()

    seq = _dedupe(observed)
    print("[check] 收到 %d 条 /state，去重后序列: %s" % (len(observed), seq))

    if observed and _is_subsequence(EXPECTED, seq):
        print("PASS: 状态切换序列符合预期 %s" % (EXPECTED,))
        sys.exit(0)
    print("FAIL: 期望包含有序子序列 %s，实际 %s" % (EXPECTED, seq))
    sys.exit(1)


if __name__ == "__main__":
    main()
