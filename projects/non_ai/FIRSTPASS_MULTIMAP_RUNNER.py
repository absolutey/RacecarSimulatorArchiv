#!/usr/bin/env python3
"""Isolated ICCAS/Spielberg SOLO/H2H execution matrix, explicit opt-in.

Dry-run by default. If --execute is given, verifies environment/map SHA,
starts one ROS launch at a time with HUD+independent rosbag, waits for live
preflight before requesting /race_start. Never reports G0-G9 PASS based on
START acceptance; runtime may reject and physical results require later audit.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import zipfile

import yaml
ROOT=Path(__file__).resolve().parent
CASES={
    'iccas2025_solo':('iccas2025',True,'SOLO_MPC_EXPERIMENTAL','scp'),
    'spielberg_solo':('spielberg',True,'SOLO_SPIELBERG_MPC_SCP_EXPERIMENTAL','scp'),
    'iccas2025_h2h':('iccas2025',False,'H2H_MPC_DYNAMIC_EXPERIMENTAL','scp'),
    'spielberg_h2h':('spielberg',False,'H2H_SPIELBERG_MPC_SCP_EXPERIMENTAL','scp'),
}
SIM=ROOT/'src/environment/f1tenth_racing_sim'
REF=ROOT/'src/platform/f1tenth_racing_reference'


def identity(case):
    if case not in CASES:
        raise ValueError('UNKNOWN_EXECUTION_CASE')
    track,solo,profile,candidate=CASES[case]
    envfile=SIM/'config'/('spielberg_environment.yaml' if track=='spielberg' else 'competition_environment.yaml')
    env=yaml.safe_load(envfile.read_text())
    if env['environment_identity']['active_track']!=track:
        raise ValueError('RUNNER_TRACK_MISMATCH')
    image=SIM/env['map']['image_file']
    centerline=SIM/env['map']['centerline_file']
    for path,digest in ((image,env['map']['image_sha256']),
                        (centerline,env['map']['centerline_sha256'])):
        if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('RUNNER_MAP_CENTERLINE_SHA_MISMATCH:'+str(path))
    policy=REF/'config'/('spielberg_nominal_reference.json' if track=='spielberg' else 'nominal_reference.json')
    p=json.loads(policy.read_text())
    if (p.get('track_id')!=track or
            p.get('map_image_sha256')!=env['map']['image_sha256'] or
            candidate not in p.get('candidates',{})):
        raise ValueError('RUNNER_POLICY_CANDIDATE_MAP_MISMATCH')
    args=['ros2','launch','f1tenth_racing_bringup','race_foundation.launch.py',
          'solo:='+str(solo).lower(),'use_r_b0:=true','use_rviz:=false',
          'use_flight_recorder:=true','use_spectator_hud:=true','record_rosbag:=false',
          'controller_kind:=mpc','review_phase:=R-B',
          'execution_profile:='+profile,'trajectory_candidate:='+candidate,
          'sim_online_adaptive:=true',
          'speed_cap_mps:=AUTO','fallback_speed_cap_mps:=AUTO',
          'use_opponent_perception:='+str(not solo).lower(),
          'use_opponent_prediction:='+str(not solo).lower(),
          'use_overtaking:='+str(not solo).lower(),
          'dynamic_red_guard_enabled:='+str(not solo).lower(),
          'use_native_guard_ros:=false',
          'environment_file:='+str(envfile),
          'map_file:='+str(image),'centerline_file:='+str(centerline),
          'nominal_policy_file:='+str(policy),
          'expected_track_id:='+track,
          'expected_map_sha256:='+env['map']['image_sha256']]
    return {'case':case,'track':track,'solo':solo,'map_sha256':env['map']['image_sha256'],
            'profile':profile,'command':args}


def _end(process):
    if process is None or process.poll() is not None:return
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL)
        process.wait(timeout=4)


def _popen(args,out,*,env):
    return subprocess.Popen(args,stdout=out,stderr=subprocess.STDOUT,
                            cwd=ROOT,env=env,start_new_session=True)


def execute(case, desc, output, duration_s):
    if not (ROOT/'install/setup.bash').is_file() or shutil.which('ros2') is None:
        raise RuntimeError('RUNNER_ROS_OVERLAY_NOT_SOURCED')
    output.mkdir(parents=True,exist_ok=False)
    runtime=dict(os.environ)
    runtime.setdefault('ROS_DOMAIN_ID','169')
    runtime['ROS_LOG_DIR']=str(output/'ros_logs')
    runtime['F1TENTH_FORENSIC_TRACE_DIR']=str(output/'actor_traces')
    run_id='RAB_FIRSTPASS_'+case+'_'+str(int(time.time()))
    runtime['F1TENTH_RUN_ID']=run_id
    (output/'ros_logs').mkdir();(output/'actor_traces').mkdir()
    desc=dict(desc,run_id=run_id,duration_s=duration_s,
              qualification='UNPROVEN_UNTIL_INDEPENDENT_REVIEW')
    desc['command'] += ['telemetry_run_id:='+run_id,
                        'telemetry_run_root:='+str(output/'runs')]
    (output/'identity.json').write_text(json.dumps(desc,indent=2)+'\n')
    processes=[]
    accepted=False
    try:
        with (output/'launch.log').open('w') as f:
            launch=_popen(desc['command'],f,env=runtime);processes.append(launch)
        time.sleep(7)
        if launch.poll() is not None:raise RuntimeError('LAUNCH_EXITED_BEFORE_PREFLIGHT')
        # Independent rosbag and HUD are *both* required, no optional off switch.
        bagdir=output/'rosbag'
        with (output/'rosbag.log').open('w') as f:
            bag=_popen(['ros2','bag','record','-a','-o',str(bagdir)],f,env=runtime)
            processes.append(bag)
        with (output/'capture.log').open('w') as f:
            collector=_popen([sys.executable,'-B',str(ROOT/'capture_full_topics.py'),
                              '--out',str(output/'full_topic_trace')],f,env=runtime)
            processes.append(collector)
        time.sleep(3)
        if collector.poll() is not None or bag.poll() is not None:
            raise RuntimeError('BAG_OR_HUD_CAPTURE_PREREQUISITE_EXITED')
        preflight=subprocess.run([sys.executable,'-B',str(ROOT/'PREFLIGHT_LIVE.py'),
                                  str(output/'preflight_live.json')],
            cwd=ROOT,env=runtime,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
            text=True,timeout=20)
        (output/'preflight.log').write_text(preflight.stdout)
        if preflight.returncode!=0:
            raise RuntimeError('LIVE_PREFLIGHT_REJECTED_START_NOT_SENT')
        start=subprocess.run(['ros2','service','call','/race_start',
                              'std_srvs/srv/Trigger','{}'],env=runtime,
                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                              text=True,timeout=10)
        (output/'start.log').write_text(start.stdout)
        accepted=(start.returncode==0 and bool(re.search(r'success\s*:\s*True',start.stdout)))
        if not accepted:raise RuntimeError('START_SERVICE_NOT_ACCEPTED')
        time.sleep(duration_s)
    except Exception as exc:
        (output/'blocked_reason.txt').write_text(type(exc).__name__+':'+str(exc)+'\n')
    finally:
        for p in reversed(processes):_end(p)
        result={'case':case,'start_accepted':accepted,
                'runtime_qualification':'NOT_EVALUATED',
                'evidence_dir':str(output)}
        (output/'execution_result.json').write_text(json.dumps(result,indent=2)+'\n')
        bundle=output.with_suffix('.zip')
        with zipfile.ZipFile(bundle,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=4) as z:
            for f in sorted(output.rglob('*')):
                if f.is_file():z.write(f,arcname=str(Path(output.name)/f.relative_to(output)))
        print('RUN_EVIDENCE_ZIP='+str(bundle),flush=True)
    return accepted


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--cases',nargs='+',choices=CASES.keys(),default=list(CASES))
    ap.add_argument('--execute',action='store_true',help='Actually launch isolated ROS simulations')
    ap.add_argument('--duration-s',type=int,default=30)
    ap.add_argument('--output-root',type=Path,default=Path.home()/'Downloads')
    args=ap.parse_args()
    if not (10<=args.duration_s<=180):ap.error('duration must be 10..180 seconds')
    descriptions=[identity(k) for k in args.cases]
    if not args.execute:
        print(json.dumps({'status':'DRY_RUN_ONLY','cases':descriptions},indent=2))
        return 0
    passed=[]
    for item in descriptions:
        folder=args.output_root/('F1TENTH_FIRSTPASS_'+item['case']+'_'+str(int(time.time())))
        ok=execute(item['case'],item,folder,args.duration_s)
        passed.append(ok)
    return 0 if all(passed) else 2

if __name__=='__main__':
    raise SystemExit(main())
