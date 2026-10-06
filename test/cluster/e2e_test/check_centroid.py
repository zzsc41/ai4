#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E2E 断言节点：订阅 ``/radar/centroid``，校验数据流与内容。

参数（rosparam）
----------------
~mode      str    "target" | "empty"（默认 "target"）
~x/~y/~z   float  目标真值（target 模式断言用，默认 8 / 3 / 2）
~tol       float  位置容差 m（默认 0.3）
~duration  float  采集时长 s（默认 6.0）
~min_msgs  int    最少收到消息数（默认 20，证明话题确实在流）
~frame_id  str    期望坐标系（默认 "map"；空串则不校验）

退出码：0 = PASS，1 = FAIL。
"""
from __future__ import annotations

import sys
import time

import rospy
from uav_tracking.msg import CentroidArray


class Checker:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "target")
        self.target = (rospy.get_param("~x", 8.0),
                       rospy.get_param("~y", 3.0),
                       rospy.get_param("~z", 2.0))
        self.tol = rospy.get_param("~tol", 0.3)
        self.duration = rospy.get_param("~duration", 6.0)
        self.min_msgs = int(rospy.get_param("~min_msgs", 20))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.msgs = []
        rospy.Subscriber("/radar/centroid", CentroidArray, self.msgs.append, queue_size=50)

    @staticmethod
    def _fail(reason):
        print("FAIL: " + reason)
        return 1

    def run(self):
        deadline = time.time() + self.duration
        rate = rospy.Rate(20)
        while time.time() < deadline and not rospy.is_shutdown():
            rate.sleep()
        return self._assert()

    def _assert(self):
        n = len(self.msgs)
        print("[check] mode=%s 收到 %d 条 /radar/centroid（期望 >= %d）"
              % (self.mode, n, self.min_msgs))
        if n < self.min_msgs:
            return self._fail("话题 /radar/centroid 数据流不足（%d < %d）"
                              % (n, self.min_msgs))
        last = self.msgs[-1]
        k = len(last.centroids)
        if self.mode == "empty":
            if k != 0:
                return self._fail("期望空数组，实际 %d 个质心" % k)
            print("PASS: empty 阶段 —— 话题在流且数组为空（空也发成立）")
            return 0

        # mode == target
        if k != 1:
            return self._fail("期望 1 个质心，实际 %d 个" % k)
        p = last.centroids[0]
        got = (p.point.x, p.point.y, p.point.z)
        dist = ((got[0] - self.target[0]) ** 2
                + (got[1] - self.target[1]) ** 2
                + (got[2] - self.target[2]) ** 2) ** 0.5
        print("[check] 质心 = (%.3f, %.3f, %.3f)  真值 = %s  偏差 = %.3f m"
              % (got[0], got[1], got[2], self.target, dist))
        if dist > self.tol:
            return self._fail("质心偏差 %.3f m > 容差 %.3f m" % (dist, self.tol))
        if self.frame_id and p.header.frame_id != self.frame_id:
            return self._fail("坐标系 %r != 期望 %r"
                              % (p.header.frame_id, self.frame_id))
        print("PASS: target 阶段 —— 数据流正常且质心与真值一致（frame=%s）"
              % p.header.frame_id)
        return 0


def main():
    rospy.init_node("e2e_check_centroid")
    sys.exit(Checker().run())


if __name__ == "__main__":
    main()
