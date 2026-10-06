#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/cluster 节点壳：``sensor_msgs/PointCloud2`` → 滤地 + DBSCAN → ``CentroidArray``。

对应《python代码移植到ros平台方法整理.md》第五节，职责边界：

- **壳**（本文件）：订阅 / 发布 / 类型转换 / 日志，**不含算法**；
- **核**（``lib/cluster.py``）：``filter_ground`` / ``dbscan_cluster`` 纯逻辑，零改动。

可测性
------
两个类型转换被拆成模块级纯函数 :func:`cloud_to_points` 与
:func:`clusters_to_centroid_array`——它们不依赖 ROS master（只 import 消息类），
供 notebook / 单元测试直接复用；``ClusterNode`` 只负责 rospy 接线。

设计约定（``design/函数接口.md`` 设计原则）
------------------------------------------
- **空也发**：无簇时发布空 ``CentroidArray``（「空 = 没目标、静默 = 故障」）；
- 传感器大消息 ``queue_size=1``，只保最新帧。
"""
from __future__ import annotations

import os
import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

# 把 lib/ 加入 import 路径（纯逻辑与壳解耦；lib/ 内为指向 code/ 的软链）
_LIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib")
if _LIB_DIR not in sys.path:
    sys.path.insert(0, _LIB_DIR)

from cluster import filter_ground, dbscan_cluster  # noqa: E402  纯逻辑，零改动
from datatypes import Point3D  # noqa: E402
from uav_tracking.msg import CentroidArray  # noqa: E402  自定义消息

__all__ = ["cloud_to_points", "clusters_to_centroid_array", "ClusterNode"]


def cloud_to_points(msg):
    """``sensor_msgs/PointCloud2`` → ``list[Point3D]``（纯转换，可单测）。

    点云含 ``intensity`` 字段时取其值，否则强度置 ``0.0``；``NaN`` 点跳过。
    """
    available = {f.name for f in msg.fields}
    if "intensity" in available:
        rows = pc2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        return [Point3D(x, y, z, i) for x, y, z, i in rows]
    rows = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    return [Point3D(x, y, z, 0.0) for x, y, z in rows]


def clusters_to_centroid_array(clusters, header=None):
    """``list[Cluster]`` → ``CentroidArray``（纯转换，可单测）。

    ``clusters`` 为空时返回**空数组**（不静默，落实「空也发」约定）。
    ``header`` 非空时逐条写入 ``PointStamped.header``（继承点云时间戳 / 坐标系）。
    """
    out = CentroidArray()
    for c in clusters:
        ps = PointStamped()
        if header is not None:
            ps.header = header
        ps.point.x, ps.point.y, ps.point.z = c.centroid
        out.centroids.append(ps)
    return out


class ClusterNode:
    """``/cluster`` 节点：事件订阅 → 纯逻辑 → 发布。"""

    def __init__(self):
        # 算法参数一律走 rosparam（launch 可调，不写死）
        self.z_thresh = rospy.get_param("~z_thresh", 0.3)
        self.eps = rospy.get_param("~eps", 0.5)
        self.min_samples = rospy.get_param("~min_samples", 5)

        self.pub = rospy.Publisher("/radar/centroid", CentroidArray, queue_size=1)
        rospy.Subscriber("/radar/points", PointCloud2, self.on_points, queue_size=1)
        rospy.loginfo(
            "[cluster] ready: eps=%.2f min_samples=%d z_thresh=%.2f",
            self.eps, self.min_samples, self.z_thresh)

    def on_points(self, msg):
        """事件层：收到一帧点云处理一帧（本模块无定时器）。"""
        points = cloud_to_points(msg)
        clusters = dbscan_cluster(
            filter_ground(points, self.z_thresh), self.eps, self.min_samples)
        self.pub.publish(clusters_to_centroid_array(clusters, msg.header))


def main():
    rospy.init_node("cluster")
    ClusterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
