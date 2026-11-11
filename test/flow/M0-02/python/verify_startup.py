#!/usr/bin/python3
"""Real desktop acceptance: unique default launches, wall-clock evidence, owned shutdown."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]


def probe(run):
    import hashlib
    import threading
    import rospy
    from sensor_msgs.msg import Image, PointCloud2
    lock = threading.Lock()
    data = {k: dict(count=0, hashes=set(), frames=set()) for k in ('camera', 'radar')}
    def receive(kind, msg):
        with lock:
            entry = data[kind]
            entry['count'] += 1
            entry['hashes'].add(hashlib.sha256(msg.data).hexdigest())
            entry['frames'].add(msg.header.frame_id)
    rospy.init_node('m0_02_evidence', anonymous=True)
    subs = [rospy.Subscriber('/camera/image_raw', Image, lambda m: receive('camera', m), queue_size=1),
            rospy.Subscriber('/radar/points', PointCloud2, lambda m: receive('radar', m), queue_size=1)]
    while not rospy.is_shutdown():
        with lock:
            payload = {k: dict(count=v['count'], distinct_data=len(v['hashes']), frames=sorted(v['frames'])) for k,v in data.items()}
        (run/'acceptance_streams.json').write_text(json.dumps(payload, indent=2))
        if (run/'spectator_view.txt').exists() and not (run/'acceptance_gui_process.json').exists():
            try:
                receipt=(run/'spectator_view.txt').read_text()
                pid=int(next(line[4:] for line in receipt.splitlines() if line.startswith('pid=')))
                proc=Path('/proc')/str(pid)
                maps=[line for line in (proc/'maps').read_text().splitlines() if 'libai4_scene_view.so' in line]
                env=[v.decode() for v in (proc/'environ').read_bytes().split(b'\0') if v.startswith(b'AI4_VIEW_')]
                evidence=dict(receipt=receipt,cmdline=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(),plugin_maps=maps,view_environment=env)
                (run/'acceptance_gui_process.json').write_text(json.dumps(evidence,indent=2))
            except (OSError,ValueError,StopIteration):pass
        time.sleep(1)


def session_pid(run):
    for item in Path('/proc').iterdir():
        if not item.name.isdigit(): continue
        try:
            args = (item/'cmdline').read_bytes().split(b'\0')
            if any(a.endswith(b'/sensor_session.py') for a in args) and os.fsencode(str(run)) in args:
                return int(item.name)
        except OSError: pass
    raise RuntimeError('owned sensor_session not found: '+str(run))


def check(run, seconds, process=None):
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    started = time.monotonic()
    log = (run/'acceptance_samples.jsonl').open('a', buffering=1)
    command = 'source /opt/ros/noetic/setup.bash\nsource "$1/connection.bash"\nexec /usr/bin/python3 "$2" --probe "$1"'
    observer = subprocess.Popen(['bash','-c',command,'m0-02',str(run),str(Path(__file__).resolve())])
    failures = []
    initial_receipt = (run/'spectator_view.txt').exists()
    try:
        while True:
            elapsed = time.monotonic()-started
            try: health = json.loads((run/'startup_health.json').read_text())
            except (OSError, ValueError): health = {}
            tree = subprocess.run(['xwininfo','-root','-tree'], capture_output=True,text=True,timeout=5)
            windows = [line.strip() for line in tree.stdout.splitlines() if '"Gazebo"' in line or '"AI4 Camera + 3D Radar' in line]
            entry = dict(wall_elapsed_s=elapsed, utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), health=health, windows=windows)
            log.write(json.dumps(entry)+'\n')
            app.primaryScreen().grabWindow(0).save(str(run/('desktop_%03d.png'%elapsed)))
            if process and process.poll() is not None:
                failures.append('launcher exited early: '+str(process.returncode));break
            if elapsed >= 150 and health.get('status') != 'running': failures.append('not running at %.1fs'%elapsed)
            if elapsed >= seconds: break
            time.sleep(min(30, seconds-elapsed))
    finally:
        observer.send_signal(signal.SIGINT)
        observer.wait(timeout=15)
        log.close()
    measured = time.monotonic()-started
    created = datetime.datetime.fromisoformat(json.loads((run/'run.json').read_text())['created_utc'])
    lifetime = (datetime.datetime.now(datetime.timezone.utc)-created).total_seconds()
    before = json.loads((run/'startup_health.json').read_text()) if (run/'startup_health.json').exists() else {}
    os.kill(session_pid(run), signal.SIGINT)
    if process: code = process.wait(timeout=70)
    else:
        end=time.monotonic()+70
        while not (run/'launcher_exit_code.txt').exists() and time.monotonic()<end: time.sleep(.5)
        code=(run/'launcher_exit_code.txt').read_text().strip()
    result=dict(run=str(run), measured_wall_s=measured, since_run_created_wall_s=lifetime, initial_receipt_present=initial_receipt,
                final_health=before, exit_code=code, failures=failures)
    (run/'acceptance_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


def launch(seconds, gui):
    # The EXIT trap can write its result just before Bash releases fd 9.
    deadline=time.monotonic()+15
    with (ROOT/'data/radar/instance.lock').open('a') as handle:
        while True:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                fcntl.flock(handle, fcntl.LOCK_UN)
                break
            except BlockingIOError:
                if time.monotonic()>deadline:raise RuntimeError('previous launcher still owns instance lock')
                time.sleep(.2)
    old=set((ROOT/'data/radar').iterdir())
    args=['bash','catkin_ws/src/uav_sim/scripts/start_mounted_radar.sh']
    if not gui:args.append('gui:=false')
    process=subprocess.Popen(args,cwd=ROOT,stdout=open('/tmp/m0_02_acceptance_console.log','a'),stderr=subprocess.STDOUT)
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        runs=[p for p in (ROOT/'data/radar').iterdir() if p not in old and p.is_dir() and (p/'run.json').exists()]
        if runs:break
        if process.poll() is not None:raise RuntimeError('launch failed')
        time.sleep(.1)
    else:raise RuntimeError('new run directory missing')
    print('RUN '+str(runs[0]),flush=True)
    check(runs[0],seconds,process)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--probe',type=Path);p.add_argument('--baseline',type=Path);p.add_argument('--suite',action='store_true');a=p.parse_args()
    if a.probe:probe(a.probe)
    elif a.baseline:check(a.baseline.resolve(),1)
    elif a.suite:
        launch(320,True)
        launch(320,True)
        launch(180,False)
