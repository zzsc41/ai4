#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假传感器源（E2E 测试用）：按脚本五段发布 ``/detections`` + ``/visual/tracks`` + ``/radar/tracks``。

以 ``/detections`` 为节拍（状态机方案 B），故**每拍三路都发**——无内容时发空数组，
落实「空也发」约定。场景每段约 ``~phase_seconds`` 秒：

======  =================  ==========================================
阶段     名称                状态机期望
======  =================  ==========================================
A       仅雷达             RADAR_GUIDE
B       视觉锁定条件        （连续 N 拍后）VISUAL_LOCK
C       视觉丢失            （连续 M 拍后）RADAR_GUIDE
D       雷达丢失            （连续 K 拍后）SEARCH/RTH
E       雷达恢复            RADAR_GUIDE
======  =================  ==========================================

参数（rosparam，均可选）
------------------------
~rate            float  发布频率 Hz    默认 20.0
~phase_seconds   float  每段时长 s     默认 2.0
~conf            float  检测置信度     默认 0.9（> 0.8 达标）
~area            float  视觉面积比     默认 0.05（> 0.02 达标）
"""
from __future__ import annotations

import rospy
from uav_tracking.msg import DetectedTarget, Detections, Track, TrackArray
from uav_tracking.msg import VisualTrack, VisualTrackArray

#: 五段场景：(名称, 是否有雷达航迹, 是否满足视觉锁定条件)
PHASES = (
    ("A_radar_only", True, False),
    ("B_visual_lock", True, True),
    ("C_visual_lost", True, False),
    ("D_radar_lost", False, False),
    ("E_radar_back", True, False),
)


def _detections(conf):
    m = Detections()
    t = DetectedTarget()
    t.class_id = 0
    t.confidence = conf
    t.bbox = [0.0, 0.0, 100.0, 100.0]
    m.targets.append(t)
    return m


def _visual_tracks(area):
    m = VisualTrackArray()
    t = VisualTrack()
    t.id = 1
    t.bbox = [0.0, 0.0, 100.0, 100.0]
    t.area_ratio = area
    m.tracks.append(t)
    return m


def _radar_tracks(present):
    m = TrackArray()
    if present:
        t = Track()
        t.id = 1
        t.position = [8.0, 3.0, 2.0]
        t.velocity = [0.0, 0.0, 0.0]
        t.covariance = [0.0] * 36
        t.age = 1
        m.tracks.append(t)
    return m


def main():
    rospy.init_node("fake_sensors")
    rate_hz = rospy.get_param("~rate", 20.0)
    phase_seconds = rospy.get_param("~phase_seconds", 2.0)
    conf = rospy.get_param("~conf", 0.9)
    area = rospy.get_param("~area", 0.05)

    pub_det = rospy.Publisher("/detections", Detections, queue_size=1)
    pub_vt = rospy.Publisher("/visual/tracks", VisualTrackArray, queue_size=1)
    pub_rt = rospy.Publisher("/radar/tracks", TrackArray, queue_size=1)
    rospy.loginfo("[fake_sensors] rate=%.1fHz phase=%.1fs conf=%.2f area=%.2f",
                  rate_hz, phase_seconds, conf, area)

    t0 = rospy.Time.now()
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - t0).to_sec()
        idx = min(int(elapsed / phase_seconds), len(PHASES) - 1)
        _name, has_radar, visual_ok = PHASES[idx]
        # 先发非节拍两路刷新缓存，最后发 /detections 触发一拍
        pub_rt.publish(_radar_tracks(has_radar))
        pub_vt.publish(_visual_tracks(area) if visual_ok else VisualTrackArray())
        pub_det.publish(_detections(conf) if visual_ok else Detections())
        rate.sleep()


if __name__ == "__main__":
    main()
