"""模块 1（``/cluster``）单元测试。

覆盖：
- ``filter_ground``：丢低留高、边界值丢弃、空输入、非法参数（dev/onboard 两阶段）；
- ``dbscan_cluster``：两簇分离、噪声点排除、全噪声、空输入、簇编号/质心、
  非法参数（dev/onboard 两阶段）。

阶段开关通过 monkeypatch 修改 ``cluster.ONBOARD`` 模拟，不依赖真实环境变量。
"""
import pytest

import cluster
from datatypes import Cluster, Point3D


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def _two_blobs():
    """构造两个相距很远、各自 3 点的点簇。"""
    a = [Point3D(0.0, 0.0, 5.0), Point3D(0.2, 0.0, 5.0), Point3D(0.0, 0.2, 5.0)]
    b = [Point3D(10.0, 10.0, 5.0), Point3D(10.2, 10.0, 5.0), Point3D(10.0, 10.2, 5.0)]
    return a + b


# ---------------------------------------------------------------------------
# filter_ground
# ---------------------------------------------------------------------------
def test_filter_ground_drops_ground_keeps_air():
    points = [Point3D(0, 0, 0.0), Point3D(1, 1, 0.5), Point3D(2, 2, 3.0)]
    out = cluster.filter_ground(points, z_thresh=1.0)
    assert [p.z for p in out] == [3.0]


def test_filter_ground_boundary_is_dropped():
    # z == z_thresh 位于阈值上，按约定丢弃
    assert cluster.filter_ground([Point3D(0, 0, 2.0)], z_thresh=2.0) == []


def test_filter_ground_preserves_order_and_identity():
    points = [Point3D(0, 0, 5.0), Point3D(0, 0, 5.0)]
    out = cluster.filter_ground(points, z_thresh=1.0)
    assert out == points


def test_filter_ground_empty_returns_empty():
    assert cluster.filter_ground([], z_thresh=1.0) == []


def test_filter_ground_non_numeric_raises_in_dev(monkeypatch):
    monkeypatch.setattr(cluster, "ONBOARD", False)
    with pytest.raises(ValueError):
        cluster.filter_ground([Point3D(0, 0, 1.0)], z_thresh=None)


def test_filter_ground_non_numeric_onboard_returns_empty(monkeypatch):
    monkeypatch.setattr(cluster, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = cluster.filter_ground([Point3D(0, 0, 1.0)], z_thresh=None)
    assert out == []


# ---------------------------------------------------------------------------
# dbscan_cluster
# ---------------------------------------------------------------------------
def test_dbscan_splits_two_clusters():
    clusters = cluster.dbscan_cluster(_two_blobs(), eps=0.5, min_samples=2)
    assert len(clusters) == 2
    assert [c.id for c in clusters] == [0, 1]
    assert [c.size for c in clusters] == [3, 3]
    assert clusters[0].centroid == pytest.approx(
        (0.2 / 3, 0.2 / 3, 5.0), abs=1e-9
    )
    # 目标 B：x 均值 = (10 + 10.2 + 10) / 3，y 均值 = (10 + 10 + 10.2) / 3
    assert clusters[1].centroid == pytest.approx(
        (30.2 / 3, 30.2 / 3, 5.0), abs=1e-9
    )


def test_dbscan_centroid_are_plain_floats_and_len3():
    clusters = cluster.dbscan_cluster(_two_blobs(), eps=0.5, min_samples=2)
    for c in clusters:
        assert isinstance(c.centroid, tuple)
        assert len(c.centroid) == 3
        assert all(isinstance(v, float) for v in c.centroid)


def test_dbscan_size_equals_len_points():
    clusters = cluster.dbscan_cluster(_two_blobs(), eps=0.5, min_samples=2)
    for c in clusters:
        assert c.size == len(c.points)
        assert isinstance(c, Cluster)


def test_dbscan_excludes_noise_point():
    noise = Point3D(100.0, 100.0, 100.0)
    clusters = cluster.dbscan_cluster(_two_blobs() + [noise], eps=0.5, min_samples=2)
    assert len(clusters) == 2
    assert sum(c.size for c in clusters) == 6  # 噪声点不计入
    members = [p for c in clusters for p in c.points]
    assert noise not in members


def test_dbscan_all_noise_returns_no_cluster():
    pts = [Point3D(0, 0, 0), Point3D(10, 10, 0)]
    assert cluster.dbscan_cluster(pts, eps=0.1, min_samples=2) == []


def test_dbscan_empty_returns_empty():
    assert cluster.dbscan_cluster([], eps=0.5, min_samples=2) == []


def test_dbscan_min_samples_one_each_point_own_cluster():
    pts = [Point3D(0, 0, 0), Point3D(5, 5, 5)]
    clusters = cluster.dbscan_cluster(pts, eps=0.1, min_samples=1)
    assert len(clusters) == 2
    assert all(c.size == 1 for c in clusters)


@pytest.mark.parametrize(
    "eps, min_samples",
    [(-1.0, 2), (0.0, 2), (0.5, 0), (0.5, -3)],
)
def test_dbscan_invalid_params_raise_in_dev(monkeypatch, eps, min_samples):
    monkeypatch.setattr(cluster, "ONBOARD", False)
    with pytest.raises(ValueError):
        cluster.dbscan_cluster([Point3D(0, 0, 0)], eps=eps, min_samples=min_samples)


def test_dbscan_non_int_min_samples_raises_in_dev(monkeypatch):
    monkeypatch.setattr(cluster, "ONBOARD", False)
    with pytest.raises(ValueError):
        cluster.dbscan_cluster([Point3D(0, 0, 0)], eps=1.0, min_samples=2.0)


@pytest.mark.parametrize(
    "eps, min_samples",
    [(-1.0, 2), (0.5, 0), (0.5, 2.0)],
)
def test_dbscan_invalid_params_onboard_returns_empty(monkeypatch, eps, min_samples):
    monkeypatch.setattr(cluster, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = cluster.dbscan_cluster(
            [Point3D(0, 0, 0)], eps=eps, min_samples=min_samples
        )
    assert out == []
