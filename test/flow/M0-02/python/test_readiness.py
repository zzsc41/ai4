#!/usr/bin/python3
import json
import sys
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[4]
spec=importlib.util.spec_from_file_location('watchdog',ROOT/'catkin_ws/src/uav_sim/scripts/start_sensor_clock.py')
watchdog=importlib.util.module_from_spec(spec);spec.loader.exec_module(watchdog)

class ReceiptTest(unittest.TestCase):
    def test_current_process_run_and_verified_pose_required(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);manifest=root/'run.json';manifest.write_text('{}')
            receipt=root/'spectator_view.txt'
            self.assertFalse(watchdog.spectator_readiness(root)[0])
            receipt.write_text('spectator view applied after both model visuals arrived\n')
            self.assertFalse(watchdog.spectator_readiness(root)[0])
            process=subprocess.Popen(['sleep','60'],env=dict(os.environ,AI4_VIEW_STATUS=str(receipt)))
            try:
                receipt.write_text('spectator view verified after both model visuals arrived\npid=%d\n'%process.pid)
                self.assertTrue(watchdog.spectator_readiness(root)[0])
                os.utime(receipt,ns=(1,1))
                self.assertFalse(watchdog.spectator_readiness(root)[0])
                receipt.write_text('spectator view verified after both model visuals arrived\npid=%d\n'%os.getpid())
                self.assertFalse(watchdog.spectator_readiness(root)[0])
                receipt.write_text('spectator view verified after both model visuals arrived\npid=%d\n'%process.pid)
            finally:
                process.terminate();process.wait()
            self.assertFalse(watchdog.spectator_readiness(root)[0])

class SessionExitTest(unittest.TestCase):
    def test_watchdog_failure_overrides_roslaunch_zero_exit(self):
        for failed in (False, True):
            with self.subTest(failed=failed), tempfile.TemporaryDirectory() as folder:
                root=Path(folder);(root/'run.json').write_text('{}')
                if failed:(root/'startup_error.txt').write_text('GUI readiness timeout')
                result=subprocess.run([sys.executable,str(ROOT/'catkin_ws/src/uav_sim/scripts/sensor_session.py'),str(root),sys.executable,'-c','pass'],env=dict(os.environ,AI4_RECORDING='false'),timeout=10)
                self.assertEqual(result.returncode,1 if failed else 0)
                manifest=json.loads((root/'run.json').read_text())
                self.assertEqual(manifest['status'],'failed' if failed else 'stopped_live')
                if failed:self.assertEqual(manifest['exit_reason'],'GUI readiness timeout')

if __name__=='__main__':unittest.main()
