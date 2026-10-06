#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假雷达数据源（E2E 测试用）：发布 ``/radar/points``（``sensor_msgs/PointCloud2``）。

在无 Gazebo 雷达插件时，用「地面杂波 + 可选目标簇」打通
``/radar/points → /cluster → /radar/centroid`` 链路。

参数（rosparam，均可选）
------------------------
~rate         float  发布频率 Hz      默认 10.0
~frame_id     str    点云坐标系       默认 "map"
~with_target  bool   是否包含目标簇   默认 True
~x / ~y / ~z  float  目标簇中心真值   默认 8.0 / 3.0 / 2.0
~n_ground     int    地面杂波点数     默认 400
~n_target     int    目标簇点数       默认 60
~ground_std   float  地面 z 抖动      默认 0.03（远小于 z_thresh，保证被滤除）
~target_std   float  目标簇三向抖动   默认 0.3
~seed         int    随机种子         默认 0（可复现）
"""
from __future__ import annotations

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
]


class FakeRadar:
    def __init__(self):
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.with_target = rospy.get_param("~with_target", True)
        self.target = (rospy.get_param("~x", 8.0),
                       rospy.get_param("~y", 3.0),
                       rospy.get_param("~z", 2.0))
        self.n_ground = int(rospy.get_param("~n_ground", 400))
        self.n_target = int(rospy.get_param("~n_target", 60))
        self.ground_std = rospy.get_param("~ground_std", 0.03)
        self.target_std = rospy.get_param("~target_std", 0.3)
        self.rng = np.random.default_rng(rospy.get_param("~seed", 0))

        self.pub = rospy.Publisher("/radar/points", PointCloud2, queue_size=1)
        rospy.loginfo("[fake_radar] rate=%.1fHz with_target=%s target=%s frame=%s",
                      self.rate_hz, self.with_target, self.target, self.frame_id)

    def _rows(self):
        # 地面杂波：z 在 0 附近抖动（远低于 z_thresh，会被 /cluster 滤掉）
        gxy = self.rng.uniform(-15.0, 15.0, (self.n_ground, 2))
        gz = np.abs(self.rng.normal(0.0, self.ground_std, self.n_ground))
        rows = [(float(x), float(y), float(z), 0.0)
                for (x, y), z in zip(gxy, gz)]
        # 目标簇：围绕真值三向高斯抖动
        if self.with_target:
            t = self.rng.normal(0.0, self.target_std, (self.n_target, 3)) + np.array(self.target)
            rows += [(float(x), float(y), float(z), 10.0) for x, y, z in t]
        return rows

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.frame_id
            self.pub.publish(pc2.create_cloud(header, _FIELDS, self._rows()))
            rate.sleep()


def main():
    rospy.init_node("fake_radar")
    FakeRadar().spin()


if __name__ == "__main__":
    main()
