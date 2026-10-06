#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/state_machine 节点壳：``/detections`` + ``/visual/tracks`` + ``/radar/tracks`` → ``/state``。

对应《python代码移植到ros平台方法整理.md》第五节与《design/函数接口.md》模块 8：

- **壳**（本文件）：订阅 / 发布 / 类型转换 / 日志，**不含状态转移算法**；
- **核**（``lib/state_machine.py``，指向 ``code/`` 的软链）：``evaluate_visual_lock`` /
  ``evaluate_radar_lost`` / ``transition`` 纯逻辑，零改动。

节拍（方案 B）
--------------
状态机**无发布时钟**（设计原则第 4 条）：以 ``/detections`` 的到达为**唯一节拍**——
每收到一帧检测即算「一拍」，用三路缓存判定并推进 ``transition``；
``/visual/tracks`` 与 ``/radar/tracks`` **仅刷新各自缓存**（不触发判定），
从而「连续 N 帧」= 「连续 N 个检测帧」，语义稳定、不会因快慢不同步误判。
``/state`` **按变化发布**（启动先发一次初值）。

跨帧状态（当前状态 + N/M/K 计数）住在节点成员变量里，纯函数保持无状态。

设计约定
--------
- **空也发**：三路话题无内容时发布空数组，本壳照常接收并参与判定
  （「空 = 没目标（业务正常）、静默 = 故障」）；
- **参数外置**：阈值走 rosparam（launch / 命令行可调），不写死。
"""
from __future__ import annotations

import os
import sys

import rospy
from std_msgs.msg import String

# 把 lib/ 加入 import 路径（纯逻辑与壳解耦；lib/ 内为指向 code/ 的软链）
_LIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib")
if _LIB_DIR not in sys.path:
    sys.path.insert(0, _LIB_DIR)

import state_machine as sm  # noqa: E402  纯逻辑，零改动
from datatypes import DetectedTarget, Track, VisualTrack  # noqa: E402
# 自定义消息与 datatypes 同名，起别名模块避免冲突
from uav_tracking.msg import Detections  # noqa: E402
from uav_tracking.msg import TrackArray, VisualTrackArray  # noqa: E402

__all__ = [
    "detections_from_msg",
    "visual_tracks_from_msg",
    "radar_tracks_from_msg",
    "state_to_msg",
    "step",
    "StateMachineNode",
]

# --- 模块级常量（代码书写原则第 6 条：默认值 / 未指定参数集中定义）---
TOPIC_DETECTIONS = "/detections"
TOPIC_VISUAL_TRACKS = "/visual/tracks"
TOPIC_RADAR_TRACKS = "/radar/tracks"
TOPIC_STATE = "/state"
QUEUE_SIZE = 1                      # 各路只保最新
INITIAL_STATE = sm.STATE_RADAR_GUIDE
CONF_THRESH_DEFAULT = 0.8           # 与 evaluate_visual_lock 默认一致
AREA_THRESH_DEFAULT = 0.02


# ---------------------------------------------------------------------------
# 纯转换 / 单拍函数（不依赖 ROS master，供单测 / notebook 复用）
# ---------------------------------------------------------------------------
def detections_from_msg(msg):
    """``Detections`` → ``list[DetectedTarget]``（纯转换，可单测）。"""
    return [
        DetectedTarget(class_id=int(t.class_id),
                       confidence=float(t.confidence),
                       bbox=tuple(float(v) for v in t.bbox))
        for t in msg.targets
    ]


def visual_tracks_from_msg(msg):
    """``VisualTrackArray`` → ``list[VisualTrack]``（纯转换，可单测）。"""
    return [
        VisualTrack(id=int(t.id),
                    bbox=tuple(float(v) for v in t.bbox),
                    area_ratio=float(t.area_ratio))
        for t in msg.tracks
    ]


def radar_tracks_from_msg(msg):
    """``TrackArray`` → ``list[Track]``（纯转换，可单测）。"""
    return [
        Track(id=int(t.id),
              position=tuple(float(v) for v in t.position),
              velocity=tuple(float(v) for v in t.velocity),
              covariance=tuple(float(v) for v in t.covariance),
              age=int(t.age))
        for t in msg.tracks
    ]


def state_to_msg(state):
    """``str`` → ``std_msgs/String``（纯转换，可单测）。"""
    return String(data=state)


def step(state, counters, detections, visual_tracks, radar_tracks,
         conf_thresh=CONF_THRESH_DEFAULT, area_thresh=AREA_THRESH_DEFAULT):
    """单拍判定（纯函数，可单测）：三路缓存 → ``evaluate_*`` → ``transition``。

    返回 ``(new_state, visual_lock, radar_lost)``；``counters`` **原地更新**。
    """
    visual_lock = sm.evaluate_visual_lock(
        detections, visual_tracks, conf_thresh, area_thresh)
    radar_lost = sm.evaluate_radar_lost(radar_tracks)
    new_state = sm.transition(state, visual_lock, radar_lost, counters)
    return new_state, visual_lock, radar_lost


class StateMachineNode:
    """``/state_machine`` 节点：事件订阅（``/detections`` 为节拍）→ 纯逻辑 → 发布。"""

    def __init__(self):
        # 算法参数一律走 rosparam（launch / 命令行可调，不写死）
        self.conf_thresh = rospy.get_param("~conf_thresh", CONF_THRESH_DEFAULT)
        self.area_thresh = rospy.get_param("~area_thresh", AREA_THRESH_DEFAULT)
        # N/M/K 覆盖模块级常量（纯逻辑零改动，见《函数接口.md》模块 8）
        sm.N_VISUAL_LOCK = rospy.get_param("~n_visual_lock", sm.N_VISUAL_LOCK)
        sm.M_VISUAL_LOST = rospy.get_param("~m_visual_lost", sm.M_VISUAL_LOST)
        sm.K_RADAR_LOST = rospy.get_param("~k_radar_lost", sm.K_RADAR_LOST)

        # 跨帧状态：当前状态 + N/M/K 计数（住节点，纯函数无状态）
        self.state = INITIAL_STATE
        self.counters = {key: 0 for key in sm.COUNTER_KEYS}
        # 三路缓存（``/detections`` 到达即用当前三路缓存判定）
        self._detections = []
        self._visual_tracks = []
        self._radar_tracks = []

        self.pub = rospy.Publisher(TOPIC_STATE, String, queue_size=QUEUE_SIZE)
        rospy.Subscriber(TOPIC_DETECTIONS, Detections, self.on_detections,
                         queue_size=QUEUE_SIZE)
        rospy.Subscriber(TOPIC_VISUAL_TRACKS, VisualTrackArray,
                         self.on_visual_tracks, queue_size=QUEUE_SIZE)
        rospy.Subscriber(TOPIC_RADAR_TRACKS, TrackArray,
                         self.on_radar_tracks, queue_size=QUEUE_SIZE)

        # 启动先发一次初值（之后按变化发布）
        self.pub.publish(state_to_msg(self.state))
        rospy.loginfo(
            "[state_machine] ready: N/M/K=%d/%d/%d conf=%.2f area=%.3f state=%s",
            sm.N_VISUAL_LOCK, sm.M_VISUAL_LOST, sm.K_RADAR_LOST,
            self.conf_thresh, self.area_thresh, self.state)

    # ---- 事件层：三路回调只刷新缓存；``/detections`` 额外作为节拍推进一步 ----
    def on_detections(self, msg):
        self._detections = detections_from_msg(msg)
        self.advance()

    def on_visual_tracks(self, msg):
        self._visual_tracks = visual_tracks_from_msg(msg)

    def on_radar_tracks(self, msg):
        self._radar_tracks = radar_tracks_from_msg(msg)

    def advance(self):
        """一拍：用当前三路缓存判定并推进状态；**变化才发布**。"""
        new_state, visual_lock, radar_lost = step(
            self.state, self.counters,
            self._detections, self._visual_tracks, self._radar_tracks,
            self.conf_thresh, self.area_thresh)
        if new_state != self.state:
            rospy.loginfo(
                "[state_machine] %s -> %s (visual_lock=%s radar_lost=%s)",
                self.state, new_state, visual_lock, radar_lost)
            self.state = new_state
            self.pub.publish(state_to_msg(new_state))


def main():
    rospy.init_node("state_machine")
    StateMachineNode()
    rospy.spin()


if __name__ == "__main__":
    main()
