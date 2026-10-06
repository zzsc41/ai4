"""运行阶段小配置（全项目共用，代码书写原则第 1 条）。

代码分两个运行阶段，由环境变量 ``UAV_STAGE`` 选择：

==========  ==============  ==================================================
取值         阶段            非法输入处理
==========  ==============  ==================================================
``dev``      开发（默认）    直接抛 ``ValueError``，问题尽早暴露、停止运行
``onboard``  上机            发出 ``RuntimeWarning``，由各模块给默认值兜底
==========  ==============  ==================================================

用法：各模块统一 ``from config import STAGE, ONBOARD``，并在自身 ``_reject``
中按 ``ONBOARD`` 决定「抛错」还是「告警 + 默认值」。

.. note::
   阶段在 **import 时** 读取一次；测试中可 ``monkeypatch.setattr(模块, "ONBOARD", ...)``
   模拟，无需改环境变量。若运行中修改 ``UAV_STAGE``，因本模块已被缓存，需
   ``importlib.reload(config)`` 后再 ``reload`` 使用方模块方可生效。
"""
from __future__ import annotations

import os

__all__ = ["STAGE", "ONBOARD"]

#: 运行阶段名（小写）：``"dev"`` 或 ``"onboard"``。
STAGE = os.environ.get("UAV_STAGE", "dev").strip().lower()

#: 是否处于上机阶段（``True`` 时非法输入告警并兜底，不抛异常）。
ONBOARD = STAGE == "onboard"
