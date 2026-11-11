# M0-02 默认 GUI 启动复验

从 ai4 根目录，在有桌面和本地 socket 权限的终端运行；先停止现有实例：

```bash
/usr/bin/python3 test/flow/M0-02/python/test_readiness.py
/usr/bin/python3 test/flow/M0-02/python/verify_startup.py --suite
```

真实套件顺序执行两次默认启动（各监测至少 320 秒墙钟）和一次 `gui:=false`（180 秒）。约需 15 分钟。仅向本次 `sensor_session.py` 发送 SIGINT，保留正常退出日志；不全局杀进程。终端打印每次全新运行目录。

运行目录保留 `console.log`、`ros/`、`startup_health.json`、`spectator_view.txt`、`shutdown.json`、`launcher_exit_code.txt`、`acceptance_samples.jsonl`、`acceptance_streams.json`、`acceptance_result.json` 和 `desktop_*.png`。GUI 进程证据由订阅探针保存在 `acceptance_gui_process.json`。订阅探针统计原始相机/点云内容哈希；显示计数、双机位置、窗口存在和旁观相机凭证独立核查。

结束后，对终端打印的实际目录执行（替换路径）：

```bash
/usr/bin/python3 test/flow/M0-02/python/analyze_acceptance.py data/radar/<GUI第一轮> data/radar/<GUI第二轮>
/usr/bin/python3 test/flow/M0-02/python/analyze_acceptance.py --no-gui --minimum 180 data/radar/<无Gazebo窗口轮>
```

自动核查不能代替人工查看 `desktop_*.png`。`gui:=false` 仍有相机和雷达数据窗口，需要桌面；不算完整 Gazebo GUI 验收。旧实例的就绪文件不会跨目录使用；单元回归还检查旧时间戳、旧格式、其他实例进程和已退出进程的凭证被拒绝。

本轮原始现象未复现，不能仅凭长时间运行成功宣称历史故障根因已确认。详细调查与未完成项见 `design_module/M0/模块说明文档/M0-02启动调查.md`。
