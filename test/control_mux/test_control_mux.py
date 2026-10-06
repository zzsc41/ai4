"""模块 7（``/control_mux``）单元测试。

覆盖：

- ``select_channel``：``RADAR_GUIDE``→雷达、``VISUAL_LOCK``→视觉、
  ``SEARCH/RTH``→回退雷达、未知字符串→回退雷达、非 ``str``（dev 抛错 /
  onboard 告警+回退）；
- ``ControlMux.on_command``：选中通道转发、非选中丢弃、原样返回、未知通道处理；
- ``ControlMux.latest``：每路只留最新一条；
- ``ControlMux.on_state``：状态更新、变化日志、非法类型（dev / onboard）；
- 阶段开关通过 ``monkeypatch`` 修改 ``control_mux.ONBOARD`` 模拟，不依赖真实环境变量。
"""
import logging

import pytest

import control_mux
from control_mux import (
    CHANNEL_RADAR,
    CHANNEL_VISUAL,
    ControlMux,
    select_channel,
)


def _cmd(name):
    """构造一条不透明「指令」对象（mux 不解析内容，仅原样转发）。"""
    return {"name": name}


# ---------------------------------------------------------------------------
# select_channel
# ---------------------------------------------------------------------------
def test_select_channel_radar_guide():
    assert select_channel(control_mux.STATE_RADAR_GUIDE) == CHANNEL_RADAR


def test_select_channel_visual_lock():
    assert select_channel(control_mux.STATE_VISUAL_LOCK) == CHANNEL_VISUAL


@pytest.mark.parametrize("state", ["SEARCH/RTH", "", "WHATEVER", "radar_guide"])
def test_select_channel_fallback_to_radar(state):
    # 其他值（含 SEARCH/RTH）一律回退雷达
    assert select_channel(state) == CHANNEL_RADAR


@pytest.mark.parametrize("state", [None, 123, ["RADAR_GUIDE"]])
def test_select_channel_non_str_raises_in_dev(monkeypatch, state):
    monkeypatch.setattr(control_mux, "ONBOARD", False)
    with pytest.raises(ValueError):
        select_channel(state)


@pytest.mark.parametrize("state", [None, 123, ["RADAR_GUIDE"]])
def test_select_channel_non_str_onboard_falls_back(monkeypatch, state):
    monkeypatch.setattr(control_mux, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        assert select_channel(state) == CHANNEL_RADAR


# ---------------------------------------------------------------------------
# ControlMux 初始状态
# ---------------------------------------------------------------------------
def test_default_initial_state_is_radar():
    mux = ControlMux()
    assert mux.state == control_mux.STATE_RADAR_GUIDE
    assert mux.channel == CHANNEL_RADAR


def test_initial_state_can_be_visual():
    mux = ControlMux(control_mux.STATE_VISUAL_LOCK)
    assert mux.channel == CHANNEL_VISUAL


def test_initial_state_non_str_raises_in_dev(monkeypatch):
    monkeypatch.setattr(control_mux, "ONBOARD", False)
    with pytest.raises(ValueError):
        ControlMux(None)


def test_initial_state_non_str_onboard_defaults_radar(monkeypatch):
    monkeypatch.setattr(control_mux, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        mux = ControlMux(None)
    assert mux.state == control_mux.STATE_RADAR_GUIDE


# ---------------------------------------------------------------------------
# ControlMux.on_command：转发 / 丢弃
# ---------------------------------------------------------------------------
def test_radar_command_forwarded_in_radar_state():
    mux = ControlMux()
    c = _cmd("R1")
    assert mux.on_command(CHANNEL_RADAR, c) is c


def test_visual_command_dropped_in_radar_state():
    mux = ControlMux()
    assert mux.on_command(CHANNEL_VISUAL, _cmd("V1")) is None


def test_visual_command_forwarded_in_visual_state():
    mux = ControlMux(control_mux.STATE_VISUAL_LOCK)
    c = _cmd("V1")
    assert mux.on_command(CHANNEL_VISUAL, c) is c


def test_radar_command_dropped_in_visual_state():
    mux = ControlMux(control_mux.STATE_VISUAL_LOCK)
    assert mux.on_command(CHANNEL_RADAR, _cmd("R1")) is None


def test_command_forwarded_unchanged():
    # 只转发不生成：原样返回同一对象
    mux = ControlMux()
    c = _cmd("R1")
    out = mux.on_command(CHANNEL_RADAR, c)
    assert out is c and out == c


@pytest.mark.parametrize("channel", ["/ctrl/bad", "", None, 1])
def test_on_command_unknown_channel_raises_in_dev(monkeypatch, channel):
    monkeypatch.setattr(control_mux, "ONBOARD", False)
    with pytest.raises(ValueError):
        ControlMux().on_command(channel, _cmd("X"))


@pytest.mark.parametrize("channel", ["/ctrl/bad", "", None, 1])
def test_on_command_unknown_channel_onboard_drops(monkeypatch, channel):
    monkeypatch.setattr(control_mux, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        assert ControlMux().on_command(channel, _cmd("X")) is None


# ---------------------------------------------------------------------------
# ControlMux.latest：每路只留最新一条
# ---------------------------------------------------------------------------
def test_latest_none_before_any_command():
    mux = ControlMux()
    assert mux.latest(CHANNEL_RADAR) is None
    assert mux.latest(CHANNEL_VISUAL) is None


def test_latest_keeps_only_newest_per_channel():
    mux = ControlMux()
    mux.on_command(CHANNEL_RADAR, _cmd("R1"))
    mux.on_command(CHANNEL_RADAR, _cmd("R2"))
    mux.on_command(CHANNEL_RADAR, _cmd("R3"))
    assert mux.latest(CHANNEL_RADAR) == _cmd("R3")


def test_latest_caches_dropped_channel_too():
    # 视觉态下收到雷达指令会被丢弃，但仍缓存为该路最新（queue_size=1 语义）
    mux = ControlMux(control_mux.STATE_VISUAL_LOCK)
    mux.on_command(CHANNEL_RADAR, _cmd("R1"))
    assert mux.latest(CHANNEL_RADAR) == _cmd("R1")


def test_latest_unknown_channel_raises_in_dev(monkeypatch):
    monkeypatch.setattr(control_mux, "ONBOARD", False)
    with pytest.raises(ValueError):
        ControlMux().latest("/ctrl/bad")


def test_latest_unknown_channel_onboard_returns_none(monkeypatch):
    monkeypatch.setattr(control_mux, "ONBOARD", True)
    with pytest.warns(RuntimeWarning):
        assert ControlMux().latest("/ctrl/bad") is None


# ---------------------------------------------------------------------------
# ControlMux.on_state：状态更新与日志
# ---------------------------------------------------------------------------
def test_on_state_updates_channel():
    mux = ControlMux()
    mux.on_state(control_mux.STATE_VISUAL_LOCK)
    assert mux.state == control_mux.STATE_VISUAL_LOCK
    assert mux.channel == CHANNEL_VISUAL


def test_on_state_logs_on_change(caplog):
    mux = ControlMux()
    with caplog.at_level(logging.INFO, logger="control_mux"):
        mux.on_state(control_mux.STATE_VISUAL_LOCK)
    assert "[mux] state -> VISUAL_LOCK" in caplog.text


def test_on_state_same_value_no_log(caplog):
    mux = ControlMux()
    with caplog.at_level(logging.INFO, logger="control_mux"):
        mux.on_state(control_mux.STATE_RADAR_GUIDE)  # 与初值相同
    assert caplog.text == ""


def test_on_state_non_str_raises_in_dev(monkeypatch):
    monkeypatch.setattr(control_mux, "ONBOARD", False)
    with pytest.raises(ValueError):
        ControlMux().on_state(None)


def test_on_state_non_str_onboard_ignores(monkeypatch):
    monkeypatch.setattr(control_mux, "ONBOARD", True)
    mux = ControlMux(control_mux.STATE_VISUAL_LOCK)
    with pytest.warns(RuntimeWarning):
        mux.on_state(None)
    # 保持原状态，避免误切换通道
    assert mux.state == control_mux.STATE_VISUAL_LOCK


# ---------------------------------------------------------------------------
# 端到端时序（切换后放行通道随之改变）
# ---------------------------------------------------------------------------
def test_forwarding_switches_with_state():
    mux = ControlMux()
    assert mux.on_command(CHANNEL_RADAR, _cmd("R1")) is not None
    assert mux.on_command(CHANNEL_VISUAL, _cmd("V1")) is None

    mux.on_state(control_mux.STATE_VISUAL_LOCK)
    assert mux.on_command(CHANNEL_RADAR, _cmd("R2")) is None
    assert mux.on_command(CHANNEL_VISUAL, _cmd("V2")) is not None

    mux.on_state("SEARCH/RTH")  # 回退雷达
    assert mux.channel == CHANNEL_RADAR
    assert mux.on_command(CHANNEL_RADAR, _cmd("R3")) is not None
