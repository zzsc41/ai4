"""pytest 全局配置。

将仓库 ``code/`` 目录加入 ``sys.path``，使测试文件可直接 ``import cluster`` /
``import datatypes``，无需为 ``code/`` 添加 ``__init__.py``。

目录结构假定::

    ai4/
    ├── code/
    │   ├── cluster.py
    │   └── datatypes.py
    └── test/
        ├── conftest.py          <- 本文件
        └── cluster/
            └── test_cluster.py
"""
import os
import sys

_CODE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "code"
)
if _CODE_DIR not in sys.path:
    sys.path.insert(0, _CODE_DIR)
