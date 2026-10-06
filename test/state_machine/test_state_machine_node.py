"""模块 8 节点壳（``/state_machine``）单元测试。

覆盖 ``scripts/state_machine_node.py`` 中**不依赖 ROS master** 的部分（同 cluster 粒度）：

- ``detections_from_msg`` / ``visual_tracks_from_msg`` / ``radar_tracks_from_msg``：
  ROS 消息 → ``datatypes``（含空数组）；
- ``state_to_msg``：``str`` → ``std_msgs/String``；
- ``step``：单拍判定（缓存 → ``evaluate_*`` → ``transition``），覆盖 N/M/K 切换、
  视觉优先、计数原地更新；
- 一次端到端：真实消息 → 转换 → ``step``，数值语义一致。

运行前提：已 ``source /opt/ros/noetic/setup.bash`` 与
``source catkin_ws/devel/setup.bash``（需能 import ``rospy`` 与生成的
``uav_tracking.msg``）；否则本文件整体跳过，不影响纯逻辑测试。
"""
import os
import sys

import pytest

# 将 catkin 包 scripts/ 加入 import 路径，使 state_machine_node 可被 import。
_SCRIPTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "catkin_ws", "src", "uav_tracking", "scripts",
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# 依赖 ROS（rospy + 生成的 uav_tracking.msg），缺失则本文件整体跳过
pytest.importorskip("rospy")
pytest.importorskip("uav_tracking.msg")

import datatypes  # noqa: E402
import state_machine as sm  # noqa: E402
import state_machine_node as node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from uav_tracking.msg import DetectedTarget as DetectedTargetMsg  # noqa: E402
from uav_tracking.msg import Detections  # noqa: E402
from uav_tracking.msg import Track as TrackMsg  # noqa: E402
from uav_tracking.msg import TrackArray  # noqa: E402
from uav_tracking.msg import VisualTrack as VisualTrackMsg  # noqa: E402
from uav_tracking.msg import VisualTrackArray  # noqa: E402


# ---------------------------------------------------------------------------
# 辅助：构造 ROS 消息
# ---------------------------------------------------------------------------
def make_detections(items):
    """``items``: ``[(class_id, confidence, (x, y, w, h)), ...]``。"""
    msg = Detections()
    for class_id, confidence, bbox in items:
        t = DetectedTargetMsg()
        t.class_id = class_id
        t.confidence = confidence
        t.bbox = list(bbox)
        msg.targets.append(t)
    return msg


def make_visual_track_array(items):
    """``items``: ``[(id, (x, y, w, h), area_ratio), ...]``。"""
    msg = VisualTrackArray()
    for track_id, bbox, area_ratio in items:
        t = VisualTrackMsg()
        t.id = track_id
        t.bbox = list(bbox)
        t.area_ratio = area_ratio
        msg.tracks.append(t)
    return msg


def make_track_array(ids):
    """按给定 id 列表造等长 ``TrackArray``（位置 / 速度为定值，协方差 36 维）。"""
    msg = TrackArray()
    for track_id in ids:
        t = TrackMsg()
        t.id = track_id
        t.position = [8.0, 3.0, 2.0]
        t.velocity = [0.0, 0.0, 0.0]
        t.covariance = [0.0] * 36
        t.age = 1
        msg.tracks.append(t)
    return msg


def _counters():
    return {key: 0 for key in sm.COUNTER_KEYS}


def _det_dataclass(confidence=0.9):
    return datatypes.DetectedTarget(class_id=0, confidence=confidence,
                                    bbox=(0.0, 0.0, 10.0, 10.0))


def _vt_dataclass(area_ratio=0.05):
    return datatypes.VisualTrack(id=1, bbox=(0.0, 0.0, 10.0, 10.0),
                                 area_ratio=area_ratio)


def _rt_dataclass():
    return datatypes.Track(id=1, position=(8.0, 3.0, 2.0),
                           velocity=(0.0, 0.0, 0.0),
                           covariance=tuple([0.0] * 36), age=1)


# ---------------------------------------------------------------------------
# 转换函数
# ---------------------------------------------------------------------------
def test_detections_from_msg_reads_fields():
    out = node.detections_from_msg(
        make_detections([(0, 0.5, (1.0, 2.0, 3.0, 4.0))]))
    assert len(out) == 1
    assert isinstance(out[0], datatypes.DetectedTarget)
    assert out[0].class_id == 0
    assert out[0].confidence == pytest.approx(0.5)
    assert out[0].bbox == (1.0, 2.0, 3.0, 4.0)


def test_detections_from_msg_empty():
    assert node.detections_from_msg(Detections()) == []


def test_visual_tracks_from_msg_reads_fields():
    out = node.visual_tracks_from_msg(
        make_visual_track_array([(1, (0.0, 0.0, 5.0, 5.0), 0.25)]))
    assert len(out) == 1
    assert isinstance(out[0], datatypes.VisualTrack)
    assert out[0].id == 1
    assert out[0].area_ratio == pytest.approx(0.25)
    assert out[0].bbox == (0.0, 0.0, 5.0, 5.0)


def test_visual_tracks_from_msg_empty():
    assert node.visual_tracks_from_msg(VisualTrackArray()) == []


def test_radar_tracks_from_msg_reads_fields():
    out = node.radar_tracks_from_msg(make_track_array([7]))
    assert len(out) == 1
    assert isinstance(out[0], datatypes.Track)
    assert out[0].id == 7
    assert out[0].position == (8.0, 3.0, 2.0)
    assert out[0].velocity == (0.0, 0.0, 0.0)
    assert len(out[0].covariance) == 36
    assert out[0].age == 1


def test_radar_tracks_from_msg_empty():
    assert node.radar_tracks_from_msg(TrackArray()) == []


def test_state_to_msg_wraps_string():
    msg = node.state_to_msg(sm.STATE_VISUAL_LOCK)
    assert isinstance(msg, String)
    assert msg.data == "VISUAL_LOCK"


# ---------------------------------------------------------------------------
# 模块常量
# ---------------------------------------------------------------------------
def test_module_constants():
    assert node.TOPIC_DETECTIONS == "/detections"
    assert node.TOPIC_VISUAL_TRACKS == "/visual/tracks"
    assert node.TOPIC_RADAR_TRACKS == "/radar/tracks"
    assert node.TOPIC_STATE == "/state"
    assert node.QUEUE_SIZE == 1
    assert node.INITIAL_STATE == "RADAR_GUIDE"


# ---------------------------------------------------------------------------
# step：单拍判定
# ---------------------------------------------------------------------------
def test_step_enters_visual_lock_after_n_frames():
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE
    dets, vts, rts = [_det_dataclass()], [_vt_dataclass()], [_rt_dataclass()]
    for _ in range(sm.N_VISUAL_LOCK - 1):
        state, visual_lock, radar_lost = node.step(state, counters, dets, vts, rts)
        assert state == sm.STATE_RADAR_GUIDE
    state, visual_lock, radar_lost = node.step(state, counters, dets, vts, rts)
    assert state == sm.STATE_VISUAL_LOCK
    assert visual_lock is True and radar_lost is False


def test_step_enters_search_after_k_frames():
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE
    for _ in range(sm.K_RADAR_LOST - 1):
        state, _, radar_lost = node.step(state, counters, [], [], [])
        assert state == sm.STATE_RADAR_GUIDE
        assert radar_lost is True
    state, _, radar_lost = node.step(state, counters, [], [], [])
    assert state == sm.STATE_SEARCH_RTH


def test_step_exits_visual_after_m_frames():
    counters = _counters()
    state = sm.STATE_VISUAL_LOCK
    for _ in range(sm.M_VISUAL_LOST - 1):
        state, visual_lock, _ = node.step(state, counters, [], [], [_rt_dataclass()])
        assert state == sm.STATE_VISUAL_LOCK
        assert visual_lock is False
    state, _, _ = node.step(state, counters, [], [], [_rt_dataclass()])
    assert state == sm.STATE_RADAR_GUIDE


def test_step_visual_priority_over_radar_lost(monkeypatch):
    # 视觉锁定与雷达丢失同时成立：视觉优先（在 N 帧进 VISUAL_LOCK）
    monkeypatch.setattr(sm, "N_VISUAL_LOCK", 2)
    monkeypatch.setattr(sm, "K_RADAR_LOST", 2)
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE
    state, _, _ = node.step(state, counters, [_det_dataclass()], [_vt_dataclass()], [])
    assert state == sm.STATE_RADAR_GUIDE
    state, _, _ = node.step(state, counters, [_det_dataclass()], [_vt_dataclass()], [])
    assert state == sm.STATE_VISUAL_LOCK


def test_step_updates_counters_in_place():
    counters = _counters()
    node.step(sm.STATE_RADAR_GUIDE, counters, [_det_dataclass()], [_vt_dataclass()],
              [_rt_dataclass()])
    assert counters["visual_lock"] == 1
    assert counters["radar_lost"] == 0


def test_step_custom_thresholds():
    counters = _counters()
    # 视觉条件不达标 → 不进 VISUAL_LOCK
    state, visual_lock, _ = node.step(
        sm.STATE_RADAR_GUIDE, counters, [_det_dataclass(0.5)], [_vt_dataclass(0.05)],
        [_rt_dataclass()], conf_thresh=0.8)
    assert visual_lock is False


# ---------------------------------------------------------------------------
# 端到端：真实消息 → 转换 → step
# ---------------------------------------------------------------------------
def test_end_to_end_msg_to_transition(monkeypatch):
    monkeypatch.setattr(sm, "N_VISUAL_LOCK", 2)
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE

    def one_frame():
        dets = node.detections_from_msg(make_detections([(0, 0.9, (0.0, 0.0, 100.0, 100.0))]))
        vts = node.visual_tracks_from_msg(make_visual_track_array([(1, (0.0, 0.0, 100.0, 100.0), 0.05)]))
        rts = node.radar_tracks_from_msg(make_track_array([1]))
        return node.step(state, counters, dets, vts, rts)

    state, _, _ = one_frame()
    assert state == sm.STATE_RADAR_GUIDE
    state, visual_lock, radar_lost = one_frame()
    assert state == sm.STATE_VISUAL_LOCK
    assert visual_lock is True and radar_lost is False
