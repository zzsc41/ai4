"""模块 8（``/state_machine``）单元测试。

覆盖：

- ``evaluate_visual_lock``：双条件满足、置信度 / 面积任缺、严格大于边界、
  空输入、自定义阈值、非 ``list`` 参数（dev / onboard 两阶段）；
- ``evaluate_radar_lost``：空 → 丢失、非空 → 未丢失、非法类型（dev / onboard）；
- ``transition``：进入 / 退出 ``VISUAL_LOCK``（N/M 帧）、进入 / 退出
  ``SEARCH/RTH``（K 帧）、计数清零、视觉优先于雷达、原地更新 ``counters``、
  阈值常量可覆盖（monkeypatch）、非法参数（dev / onboard）。

阶段开关通过 ``monkeypatch`` 修改 ``state_machine.ONBOARD`` 模拟，不依赖真实
环境变量；N/M/K 阈值亦用 ``monkeypatch`` 覆盖模块级常量。
"""
import pytest

import state_machine as sm
from datatypes import DetectedTarget, Track, VisualTrack


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------
def _det(confidence):
    """构造一条检测（bbox 无关紧要，仅取置信度）。"""
    return DetectedTarget(class_id=0, confidence=confidence, bbox=(10.0, 10.0, 20.0, 20.0))


def _vtrack(area_ratio):
    """构造一条视觉航迹（仅取面积比）。"""
    return VisualTrack(id=1, bbox=(10.0, 10.0, 20.0, 20.0), area_ratio=area_ratio)


def _rtrack(track_id=1):
    """构造一条雷达航迹。"""
    return Track(
        id=track_id,
        position=(1.0, 2.0, 3.0),
        velocity=(0.0, 0.0, 0.0),
        covariance=tuple([0.0] * 36),
        age=1,
    )


def _counters():
    """新建一份 N/M/K 计数器。"""
    return {"visual_lock": 0, "visual_lost": 0, "radar_lost": 0}


# ---------------------------------------------------------------------------
# evaluate_visual_lock
# ---------------------------------------------------------------------------
def test_visual_lock_both_conditions_satisfied():
    assert sm.evaluate_visual_lock([_det(0.9)], [_vtrack(0.05)]) is True


def test_visual_lock_confidence_missing():
    # 面积达标但置信度不足 → False
    assert sm.evaluate_visual_lock([_det(0.5)], [_vtrack(0.05)]) is False


def test_visual_lock_area_missing():
    # 置信度达标但面积不足 → False
    assert sm.evaluate_visual_lock([_det(0.9)], [_vtrack(0.01)]) is False


def test_visual_lock_both_missing():
    assert sm.evaluate_visual_lock([_det(0.5)], [_vtrack(0.01)]) is False


def test_visual_lock_empty_detections():
    assert sm.evaluate_visual_lock([], [_vtrack(0.05)]) is False


def test_visual_lock_empty_tracks():
    assert sm.evaluate_visual_lock([_det(0.9)], []) is False


def test_visual_lock_empty_both():
    assert sm.evaluate_visual_lock([], []) is False


def test_visual_lock_multi_target_any_satisfies():
    # 单目标场景下的存在性判定：多目标中任一同时满足即算（此处按两条分别存在）
    dets = [_det(0.3), _det(0.95)]
    tracks = [_vtrack(0.001), _vtrack(0.1)]
    assert sm.evaluate_visual_lock(dets, tracks) is True


def test_visual_lock_strict_greater_boundary():
    # 严格大于：等于阈值不算满足
    assert sm.evaluate_visual_lock([_det(0.8)], [_vtrack(0.02)]) is False
    assert sm.evaluate_visual_lock([_det(0.8 + 1e-9)], [_vtrack(0.02 + 1e-9)]) is True


def test_visual_lock_custom_thresholds():
    assert sm.evaluate_visual_lock([_det(0.6)], [_vtrack(0.03)],
                                   conf_thresh=0.5, area_thresh=0.02) is True
    assert sm.evaluate_visual_lock([_det(0.6)], [_vtrack(0.03)],
                                   conf_thresh=0.7, area_thresh=0.02) is False


@pytest.mark.parametrize("dets, tracks", [(None, []), ([], None), ("x", [])])
def test_visual_lock_invalid_types_raise_in_dev(monkeypatch, dets, tracks):
    monkeypatch.setattr(sm, "ONBOARD", False)
    with pytest.raises(ValueError):
        sm.evaluate_visual_lock(dets, tracks)


@pytest.mark.parametrize("dets, tracks", [(None, []), ([], None)])
def test_visual_lock_invalid_types_onboard_returns_false(monkeypatch, dets, tracks):
    monkeypatch.setattr(sm, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = sm.evaluate_visual_lock(dets, tracks)
    assert out is False


@pytest.mark.parametrize("conf, area", [(None, 0.02), (0.8, None)])
def test_visual_lock_invalid_thresholds_raise_in_dev(monkeypatch, conf, area):
    monkeypatch.setattr(sm, "ONBOARD", False)
    with pytest.raises(ValueError):
        sm.evaluate_visual_lock([_det(0.9)], [_vtrack(0.05)], conf_thresh=conf, area_thresh=area)


@pytest.mark.parametrize("conf, area", [(None, 0.02), (0.8, None)])
def test_visual_lock_invalid_thresholds_onboard_returns_false(monkeypatch, conf, area):
    monkeypatch.setattr(sm, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = sm.evaluate_visual_lock([_det(0.9)], [_vtrack(0.05)], conf_thresh=conf, area_thresh=area)
    assert out is False


# ---------------------------------------------------------------------------
# evaluate_radar_lost
# ---------------------------------------------------------------------------
def test_radar_lost_when_empty():
    assert sm.evaluate_radar_lost([]) is True


def test_radar_not_lost_when_present():
    assert sm.evaluate_radar_lost([_rtrack()]) is False


def test_radar_lost_invalid_type_raises_in_dev(monkeypatch):
    monkeypatch.setattr(sm, "ONBOARD", False)
    with pytest.raises(ValueError):
        sm.evaluate_radar_lost(None)


def test_radar_lost_invalid_type_onboard_treats_as_lost(monkeypatch):
    monkeypatch.setattr(sm, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        assert sm.evaluate_radar_lost(None) is True


# ---------------------------------------------------------------------------
# transition：进入 VISUAL_LOCK（连续 N 帧）
# ---------------------------------------------------------------------------
def test_transition_enter_visual_after_n_frames(monkeypatch):
    monkeypatch.setattr(sm, "N_VISUAL_LOCK", 3)
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE
    # 前 N-1 帧保持 RADAR_GUIDE
    for _ in range(2):
        state = sm.transition(state, True, False, counters)
        assert state == sm.STATE_RADAR_GUIDE
    # 第 N 帧进入 VISUAL_LOCK
    state = sm.transition(state, True, False, counters)
    assert state == sm.STATE_VISUAL_LOCK


def test_transition_visual_lock_counter_resets_on_gap():
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE
    # 累积 2 帧后中断
    state = sm.transition(state, True, False, counters)
    state = sm.transition(state, True, False, counters)
    assert counters["visual_lock"] == 2
    state = sm.transition(state, False, False, counters)
    assert counters["visual_lock"] == 0
    assert state == sm.STATE_RADAR_GUIDE


def test_transition_enter_visual_uses_module_constant(monkeypatch):
    # 覆盖阈值为 2，验证常量在调用时被读取
    monkeypatch.setattr(sm, "N_VISUAL_LOCK", 2)
    counters = _counters()
    assert sm.transition(sm.STATE_RADAR_GUIDE, True, False, counters) == sm.STATE_RADAR_GUIDE
    assert sm.transition(sm.STATE_RADAR_GUIDE, True, False, counters) == sm.STATE_VISUAL_LOCK


# ---------------------------------------------------------------------------
# transition：退出 VISUAL_LOCK（连续 M 帧丢失）
# ---------------------------------------------------------------------------
def test_transition_exit_visual_after_m_lost(monkeypatch):
    monkeypatch.setattr(sm, "M_VISUAL_LOST", 3)
    counters = _counters()
    state = sm.STATE_VISUAL_LOCK
    for _ in range(2):
        state = sm.transition(state, False, False, counters)
        assert state == sm.STATE_VISUAL_LOCK
    state = sm.transition(state, False, False, counters)
    assert state == sm.STATE_RADAR_GUIDE


def test_transition_visual_lost_counter_resets_when_relocked():
    counters = _counters()
    state = sm.STATE_VISUAL_LOCK
    state = sm.transition(state, False, False, counters)
    assert counters["visual_lost"] == 1
    # 重新锁定 → 丢失计数清零
    state = sm.transition(state, True, False, counters)
    assert counters["visual_lost"] == 0
    assert state == sm.STATE_VISUAL_LOCK


# ---------------------------------------------------------------------------
# transition：进入 / 退出 SEARCH/RTH（连续 K 帧无航迹）
# ---------------------------------------------------------------------------
def test_transition_enter_search_after_k_frames(monkeypatch):
    monkeypatch.setattr(sm, "K_RADAR_LOST", 5)
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE
    for _ in range(4):
        state = sm.transition(state, False, True, counters)
        assert state == sm.STATE_RADAR_GUIDE
    state = sm.transition(state, False, True, counters)
    assert state == sm.STATE_SEARCH_RTH


def test_transition_radar_lost_counter_resets_when_recovered():
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE
    state = sm.transition(state, False, True, counters)
    state = sm.transition(state, False, True, counters)
    assert counters["radar_lost"] == 2
    state = sm.transition(state, False, False, counters)  # 雷达恢复
    assert counters["radar_lost"] == 0
    assert state == sm.STATE_RADAR_GUIDE


def test_transition_exit_search_on_radar_recovery():
    counters = _counters()
    assert sm.transition(sm.STATE_SEARCH_RTH, False, False, counters) == sm.STATE_RADAR_GUIDE
    assert sm.transition(sm.STATE_SEARCH_RTH, False, True, counters) == sm.STATE_SEARCH_RTH


# ---------------------------------------------------------------------------
# transition：视觉优先于雷达丢失
# ---------------------------------------------------------------------------
def test_transition_visual_priority_over_radar_lost(monkeypatch):
    # N=3 < K=5，二者同时持续成立时应在第 3 帧进 VISUAL_LOCK（而非 SEARCH/RTH）
    monkeypatch.setattr(sm, "N_VISUAL_LOCK", 3)
    monkeypatch.setattr(sm, "K_RADAR_LOST", 3)
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE
    for _ in range(2):
        state = sm.transition(state, True, True, counters)
        assert state == sm.STATE_RADAR_GUIDE
    state = sm.transition(state, True, True, counters)
    assert state == sm.STATE_VISUAL_LOCK


# ---------------------------------------------------------------------------
# transition：计数清零与原地更新
# ---------------------------------------------------------------------------
def test_transition_resets_all_counters_on_switch(monkeypatch):
    monkeypatch.setattr(sm, "N_VISUAL_LOCK", 2)
    counters = _counters()
    counters["radar_lost"] = 3  # 遗留计数
    state = sm.transition(sm.STATE_RADAR_GUIDE, True, False, counters)
    state = sm.transition(state, True, False, counters)
    assert state == sm.STATE_VISUAL_LOCK
    assert counters == {"visual_lock": 0, "visual_lost": 0, "radar_lost": 0}


def test_transition_updates_counters_in_place():
    counters = _counters()
    returned = counters  # 同一对象引用
    sm.transition(sm.STATE_RADAR_GUIDE, True, False, counters)
    assert counters is returned
    assert counters["visual_lock"] == 1


def test_transition_no_switch_keeps_state_and_counts_visually():
    counters = _counters()
    out = sm.transition(sm.STATE_VISUAL_LOCK, True, False, counters)
    assert out == sm.STATE_VISUAL_LOCK
    assert counters["visual_lost"] == 0


# ---------------------------------------------------------------------------
# transition：非法参数（dev / onboard）
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("state", ["UNKNOWN", "", None, 123])
def test_transition_invalid_state_raises_in_dev(monkeypatch, state):
    monkeypatch.setattr(sm, "ONBOARD", False)
    with pytest.raises(ValueError):
        sm.transition(state, True, False, _counters())


@pytest.mark.parametrize("state", ["UNKNOWN", "", None, 123])
def test_transition_invalid_state_onboard_returns_unchanged(monkeypatch, state):
    monkeypatch.setattr(sm, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = sm.transition(state, True, False, _counters())
    assert out == state


def test_transition_non_bool_flags_raise_in_dev(monkeypatch):
    monkeypatch.setattr(sm, "ONBOARD", False)
    with pytest.raises(ValueError):
        sm.transition(sm.STATE_RADAR_GUIDE, 1, False, _counters())


def test_transition_non_bool_flags_onboard_coerced(monkeypatch):
    monkeypatch.setattr(sm, "ONBOARD", True)
    counters = _counters()
    with pytest.warns(RuntimeWarning):
        out = sm.transition(sm.STATE_RADAR_GUIDE, 1, 0, counters)
    assert out == sm.STATE_RADAR_GUIDE
    assert counters["visual_lock"] == 1  # bool(1) → True


def test_transition_non_dict_counters_raise_in_dev(monkeypatch):
    monkeypatch.setattr(sm, "ONBOARD", False)
    with pytest.raises(ValueError):
        sm.transition(sm.STATE_RADAR_GUIDE, True, False, None)


def test_transition_non_dict_counters_onboard_returns_unchanged(monkeypatch):
    monkeypatch.setattr(sm, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        out = sm.transition(sm.STATE_VISUAL_LOCK, True, False, None)
    assert out == sm.STATE_VISUAL_LOCK


# ---------------------------------------------------------------------------
# 端到端时序（三态闭环）
# ---------------------------------------------------------------------------
def test_end_to_end_state_cycle(monkeypatch):
    monkeypatch.setattr(sm, "N_VISUAL_LOCK", 2)
    monkeypatch.setattr(sm, "M_VISUAL_LOST", 2)
    monkeypatch.setattr(sm, "K_RADAR_LOST", 3)
    counters = _counters()
    state = sm.STATE_RADAR_GUIDE

    assert sm.transition(state, True, False, counters) == sm.STATE_RADAR_GUIDE
    state = sm.transition(state, True, False, counters)
    assert state == sm.STATE_VISUAL_LOCK

    assert sm.transition(state, False, False, counters) == sm.STATE_VISUAL_LOCK
    state = sm.transition(state, False, False, counters)
    assert state == sm.STATE_RADAR_GUIDE

    assert sm.transition(state, False, True, counters) == sm.STATE_RADAR_GUIDE
    assert sm.transition(state, False, True, counters) == sm.STATE_RADAR_GUIDE
    state = sm.transition(state, False, True, counters)
    assert state == sm.STATE_SEARCH_RTH

    state = sm.transition(state, False, False, counters)  # 雷达恢复
    assert state == sm.STATE_RADAR_GUIDE
