"""模块 1 节点壳（``/cluster``）单元测试。

覆盖 ``scripts/cluster_node.py`` 中两个**可测的纯转换函数**（不依赖 ROS master）：

- ``cloud_to_points``：``PointCloud2`` → ``list[Point3D]``（含 intensity / 无 intensity / 空云）；
- ``clusters_to_centroid_array``：``list[Cluster]`` → ``CentroidArray``（含「空也发」、header 继承）；
- 一次端到端：合成点云 → 滤地 + 聚类 → 消息，数值一致。

运行前提：已 ``source /opt/ros/noetic/setup.bash`` 与
``source catkin_ws/devel/setup.bash``（需能 import ``rospy`` 与生成的
``uav_tracking.msg``）；否则本文件整体跳过，不影响纯逻辑测试。
"""
import os
import sys

import pytest

# 将 catkin 包 scripts/ 加入 import 路径，使 cluster_node 可被 import。
_SCRIPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "catkin_ws", "src", "uav_tracking", "scripts",
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# 依赖 ROS（rospy + 生成的 uav_tracking.msg），缺失则本文件整体跳过
pytest.importorskip("rospy")
pytest.importorskip("uav_tracking.msg")

import sensor_msgs.point_cloud2 as pc2  # noqa: E402
from sensor_msgs.msg import PointField  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from uav_tracking.msg import CentroidArray  # noqa: E402

import cluster_node  # noqa: E402
from cluster import dbscan_cluster, filter_ground  # noqa: E402
from datatypes import Cluster, Point3D  # noqa: E402


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def make_cloud(rows, with_intensity=True, frame_id="map"):
    """用给定 ``(x, y, z[, intensity])`` 行构造 ``PointCloud2``。"""
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    if with_intensity:
        fields.append(PointField("intensity", 12, PointField.FLOAT32, 1))
    header = Header()
    header.frame_id = frame_id
    return pc2.create_cloud(header, fields, rows)


# ---------------------------------------------------------------------------
# cloud_to_points
# ---------------------------------------------------------------------------
def test_cloud_to_points_reads_xyz_intensity():
    msg = make_cloud([(1.0, 2.0, 3.0, 10.0), (4.0, 5.0, 6.0, 20.0)])
    pts = cluster_node.cloud_to_points(msg)
    assert pts == [Point3D(1.0, 2.0, 3.0, 10.0), Point3D(4.0, 5.0, 6.0, 20.0)]


def test_cloud_to_points_without_intensity_defaults_zero():
    msg = make_cloud([(1.0, 2.0, 3.0)], with_intensity=False)
    assert cluster_node.cloud_to_points(msg) == [Point3D(1.0, 2.0, 3.0, 0.0)]


def test_cloud_to_points_empty_cloud_returns_empty():
    assert cluster_node.cloud_to_points(make_cloud([])) == []


def test_cloud_to_points_returns_point3d_instances():
    pts = cluster_node.cloud_to_points(make_cloud([(0.0, 0.0, 1.0, 5.0)]))
    assert all(isinstance(p, Point3D) for p in pts)


# ---------------------------------------------------------------------------
# clusters_to_centroid_array
# ---------------------------------------------------------------------------
def _clusters():
    a = Cluster(id=0, centroid=(1.0, 2.0, 3.0), points=[], size=0)
    b = Cluster(id=1, centroid=(4.0, 5.0, 6.0), points=[], size=0)
    return [a, b]


def test_clusters_to_array_centroids_match():
    out = cluster_node.clusters_to_centroid_array(_clusters())
    assert isinstance(out, CentroidArray)
    got = [(p.point.x, p.point.y, p.point.z) for p in out.centroids]
    assert got == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]


def test_clusters_to_array_empty_is_published_not_silent():
    # 「空也发」：无簇时长度为 0，但仍是一个可发布的 CentroidArray（而非静默）
    out = cluster_node.clusters_to_centroid_array([])
    assert isinstance(out, CentroidArray)
    assert len(out.centroids) == 0


def test_clusters_to_array_inherits_header():
    header = Header()
    header.frame_id = "radar_link"
    out = cluster_node.clusters_to_centroid_array(_clusters(), header)
    assert all(p.header.frame_id == "radar_link" for p in out.centroids)


# ---------------------------------------------------------------------------
# 端到端：合成点云 → 滤地 + 聚类 → 消息
# ---------------------------------------------------------------------------
def test_end_to_end_cloud_to_centroid_array():
    # 两个空中簇（z=5）+ 一批地面杂波（z≈0）
    rows = [
        (0.0, 0.0, 5.0, 1.0), (0.2, 0.0, 5.0, 1.0), (0.0, 0.2, 5.0, 1.0),
        (10.0, 10.0, 5.0, 1.0), (10.2, 10.0, 5.0, 1.0), (10.0, 10.2, 5.0, 1.0),
        (1.0, 1.0, 0.0, 1.0), (2.0, 2.0, 0.05, 1.0),
    ]
    msg = make_cloud(rows)
    points = cluster_node.cloud_to_points(msg)
    clusters = dbscan_cluster(filter_ground(points, 1.0), eps=0.5, min_samples=2)
    out = cluster_node.clusters_to_centroid_array(clusters, msg.header)
    assert len(out.centroids) == 2
    xs = sorted(round(p.point.x, 3) for p in out.centroids)
    assert xs == [round(0.2 / 3, 3), round(30.2 / 3, 3)]
