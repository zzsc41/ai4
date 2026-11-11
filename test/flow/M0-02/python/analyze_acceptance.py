#!/usr/bin/python3
"""Read-only acceptance checks; data motion, GUI presence and view receipt are separate."""
import argparse
import datetime
import json
from pathlib import Path


def analyze(run, gui=True, minimum=300):
    samples=[json.loads(line) for line in (run/'acceptance_samples.jsonl').read_text().splitlines()]
    result=json.loads((run/'acceptance_result.json').read_text())
    streams=json.loads((run/'acceptance_streams.json').read_text())
    steady=[s for s in samples if s['wall_elapsed_s']>=150]
    errors=[]
    def check(condition, description):
        if not condition:errors.append(description)
    check(result['measured_wall_s']>=minimum,'wall duration too short')
    check(samples[-1]['wall_elapsed_s']>=minimum,'final sample too early')
    check(int(result['exit_code'])==0,'nonzero exit')
    check(not result['failures'],'runner reported failures')
    check(bool(steady),'no samples after original failure point')
    check(all(s['health'].get('status')=='running' for s in steady),'health not running')
    check(all(set(s['health'].get('moving_models',[]))=={'static_uav','target_uav'} for s in steady),'both models not moving')
    for model in ('static_uav','target_uav'):
        positions={tuple(s['health'].get('actual_positions',{}).get(model,[])) for s in steady}
        check(len(positions)>1,model+' position did not change')
    for kind in ('camera','radar'):
        check(streams[kind]['distinct_data']>1,kind+' content unchanged')
        counts=[s['health'].get('viewer_frames',{}).get(kind,0) for s in steady]
        check(len(counts)>1 and counts[-1]>counts[0],kind+' display stopped')
    check(streams['radar']['frames']==['world'],'radar is not world frame')
    if gui:
        receipt=run/'spectator_view.txt'
        check(not result['initial_receipt_present'],'receipt already present at first observation')
        check(receipt.exists(),'no GUI receipt')
        if receipt.exists():
            created=datetime.datetime.fromisoformat(json.loads((run/'run.json').read_text())['created_utc']).timestamp()
            check(receipt.stat().st_mtime>=created,'stale GUI receipt')
            check('spectator view verified after both model visuals arrived' in receipt.read_text(),'no real pose verification')
        check(all(any('"Gazebo"' in w for w in s['windows']) for s in steady),'Gazebo window missing')
        check(all(s['health'].get('gui_view_ready') for s in steady),'GUI view not verified')
    else:
        check(not (run/'spectator_view.txt').exists(),'unexpected GUI receipt for gui=false')
        check(all(not any('"Gazebo"' in w for w in s['windows']) for s in steady),'Gazebo unexpectedly present')
    layout=run/'layout.log'
    check(not layout.exists() or 'Traceback' not in layout.read_text(),'window layout helper raised an exception')
    report=dict(run=str(run),passed=not errors,errors=errors,duration_wall_s=result['measured_wall_s'],streams=streams,gui_required=gui)
    (run/'acceptance_analysis.json').write_text(json.dumps(report,indent=2))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path);p.add_argument('--no-gui',action='store_true');p.add_argument('--minimum',type=float,default=300);a=p.parse_args()
    reports=[analyze(run,not a.no_gui,a.minimum) for run in a.runs]
    print(json.dumps(reports,ensure_ascii=False,indent=2))
    raise SystemExit(0 if all(r['passed'] for r in reports) else 1)
