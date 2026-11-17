#!/usr/bin/env python3
"""Read-only 4-layer wiring audit. Structural observations are NOT flight safety proof."""
from pathlib import Path
import ast, argparse, hashlib, json, re, xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parent
P=lambda p: ROOT/p
T=lambda p: P(p).read_text()
checks=[]
def chk(layer,label,ok,paths,qualification='STATIC'):
    checks.append({'layer':layer,'contract':label,
                   'status':'PASS_'+qualification if ok else 'FAIL_'+qualification,
                   'source_paths':paths})

def has(path,*need):
    if not P(path).is_file():return False
    contents=T(path)
    return all(s in contents for s in need)

base='src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/'
native='src/platform/f1tenth_racing_native_proof/'
launch='src/platform/f1tenth_racing_bringup/launch/race_foundation.launch.py'
sim='src/environment/f1tenth_racing_sim/f1tenth_racing_sim/simulator.py'
ref='src/platform/f1tenth_racing_reference/f1tenth_racing_reference/node.py'
re='src/driving/algorithms/f1tenth_racing_overtaking/f1tenth_racing_overtaking/'
cpp=native+'src/native_guard_ros.cpp';header=native+'src/native_guard_intent_bounds.hpp'
packages=list(P('src').rglob('package.xml'))
chk('inventory','17 unique ROS package manifests',len(packages)==17 and
    len({ET.parse(x).getroot().findtext('name') for x in packages})==17,['src'])
chk('safety','Python/C++ Guards mutually exclusive in launch',
    has(launch,'UnlessCondition(LaunchConfiguration("use_native_guard_ros"))',
       'IfCondition(LaunchConfiguration("use_native_guard_ros"))'),[launch])
chk('safety','C++ Guard shares exact single publisher identity',
    has(cpp,'Node("actuator_guard0")') and has(sim,"node_name!='actuator_guard0'"),[cpp,sim])
chk('safety','Native ROS Guard SOLO only and H2H denied',
    has(launch,'NATIVE_CPP_GUARD_H2H_NOT_QUALIFIED') and has(cpp,'if(!solo||'),[launch,cpp])
chk('safety','C++ proof has bounded monotonic 15ms deadline',
    has(cpp,'deadline=begun+15000000','f1_evaluate_native_stop(',
        'steady_ns()>deadline','stop_certificate_verified=true'),[cpp])
chk('safety','C++ Guard enforces physical actuator and intent lease limits before proof',
    has(cpp,'native_guard_intent_bounds::admissible(',
            'steering_limit_=parameters[10]','acceleration_limit_=parameters[13]',
            'braking_limit_=parameters[14]') and
    has(header,'kMaxCreatedAgeNs = 75','kMaxIntentLeaseNs = 100',
        'std::abs(steer) <= max_steer'),[cpp,header])
chk('safety','C++ wall watchdog ignores one repeated ROS tick but latches regression/stall',
    has(cpp,'now<last_ros_ns_','now>last_ros_ns_',
        'st-last_clock_progress_steady_ns_>75000000') and
    'if(now<=last_ros_ns_&&last_ros_ns_>0)' not in T(cpp),[cpp])
chk('safety','C++ fault brake cannot assert certified stop',
    has(cpp,'cmd.stop_certificate_verified=false;',
        'MINIMUM_RISK_BRAKE','latched_fault_'),[cpp])
chk('safety','Simulator has separate local Guard and watchdog command veto',
    has(sim,'GUARD_STATUS_PUBLISHER_MISMATCH','ATOMIC_EXTERNAL_WATCHDOG_REJECT'),[sim])
chk('rb','R-B state QoS is latest-only after QoS recovery',
    has(base+'node.py','ReliabilityPolicy.BEST_EFFORT',
        'LatestOnlyControlWorker('),[base+'node.py'])
chk('rb','Native and Python GuardAhead preserve 7ms fail-closed admission',
    has(base+'guard_ahead.py','MAX_EXTRA_NS = 7_000_000',
       'GUARD_AHEAD_RESPONSE_DEADLINE_UNPROVEN','evaluate_dual('),[base+'guard_ahead.py'])
chk('rb','MPC subprocess uses spawn and can never publish actuator commands',
    has(base+'process_primary_worker.py',"mp.get_context('spawn')",
       'MPC_PROCESS_RESULT_EXPIRED') and
    'create_publisher(' not in T(base+'process_primary_worker.py'),
    [base+'process_primary_worker.py'])
chk('rb','IPC write or read failures request a new process generation',
    has(base+'process_primary_worker.py','MPC_PROCESS_CHANNEL_WRITE_FAILED',
        'MPC_PROCESS_CHANNEL_READ_FAILED','self._schedule_recycle_locked(recycle_reason)'),
    [base+'process_primary_worker.py'])
chk('rb','Primary request is optional and cannot bypass fallback Gate',
    has(base+'node.py','primary_process_enabled',
        'IsolatedMpcProcessWorker(', 'recoverability.recoverable',
        'guard_ahead=self.guard_ahead') and
    has(base+'final_preflight.py','guard_ahead.choose('),
    [base+'node.py',base+'final_preflight.py'])
chk('rb','No arbitrary inherited fixed 5m/s execution cap in launch AUTO mode',
    has(launch,'speed_cap_mps", default_value="AUTO"',
        'physics_max=float(environment', 'sim_online_adaptive'),[launch])
chk('ra','ICCAS and Spielberg have distinct verified environment identities',
    P('src/environment/f1tenth_racing_sim/config/competition_environment.yaml').is_file() and
    P('src/environment/f1tenth_racing_sim/config/spielberg_environment.yaml').is_file(),
    ['src/environment/f1tenth_racing_sim/config'])
chk('ra','Nominal reference has active override arbitration',
    has(ref,'active_target_trajectory','H2H_ABORT_RECOVERY_STATIONARY_REJOIN'),[ref])
chk('ra','Spielberg policy registers both R-A3 candidates without promoting experimental',
    {'scp','scp_true_joint_experimental'}<=set(json.loads(T(
        'src/platform/f1tenth_racing_reference/config/spielberg_nominal_reference.json'))['candidates']) and
    'OFFLINE_EXPERIMENTAL_UNQUALIFIED' in T('src/driving/algorithms/f1tenth_racing_raceline/data/spielberg_r_a3_true_joint_experimental_racing_trajectory.json'),
    ['src/platform/f1tenth_racing_reference/config/spielberg_nominal_reference.json'])
chk('ra','Four map×mode launch cases available, default dry run only',
    has('FIRSTPASS_MULTIMAP_RUNNER.py','iccas2025_solo','spielberg_solo',
        'iccas2025_h2h','spielberg_h2h','if not args.execute'),
    ['FIRSTPASS_MULTIMAP_RUNNER.py'])
chk('h2h','R-C and R-D ROS nodes participate in H2H launch',
    has(launch,'opponent_perception','opponent_prediction','use_overtaking'),[launch])
chk('h2h','R-E requires post-ABORT fresh Red epoch',
    has(re+'successor_node.py','required_fresh_red_after_abort_ns',
        'accept_post_abort_red_observation('),[re+'successor_node.py'])
chk('h2h','H2H enforces Red guard in launch',
    has(launch,'H2H_GUARD_RED_PROOF_REQUIRED',
        'dynamic_red_guard_enabled'),[launch])
chk('h2h','Recovery is fresh nominal and Red not legacy committed pass reuse',
    has(ref,'H2H_ABORT_RECOVERY_STATIONARY_REJOIN') and
    has(re+'successor_core.py','acknowledge_abort_recovery'),[ref,re+'successor_core.py'])
chk('h2h','G8 cannot pass on a single overtake without ABORT fresh Red and retry',
    has('src/verification/qualification_gates/gate_model.py',
        'successful_retries','fresh_red_after_abort_verified',
        'retry_used_new_plan_identity'),['src/verification/qualification_gates/gate_model.py'])
chk('gates','G0-G9 distinguish per-map G6-G8 scope',
    has('src/verification/qualification_gates/gate_model.py',
        "'G6':('iccas2025','spielberg')", "'G8':('iccas2025','spielberg')"),
    ['src/verification/qualification_gates/gate_model.py'])
chk('gates','Gates require source and evidence-bound metrics, not just test count',
    has('src/verification/qualification_gates/gate_model.py',
        'source_tree_sha256','_validate_gate_evidence','NO_VERIFIABLE_EVIDENCE'),
    ['src/verification/qualification_gates/gate_model.py'])
for suffix,parser in (('.py',lambda s:ast.parse(s)),('.json',json.loads),('.yaml',yaml.safe_load)):
    files=list(P('src').rglob('*'+suffix));fail=[]
    for f in files:
        try:parser(f.read_text())
        except Exception as e:fail.append(str(f.relative_to(ROOT))+':'+str(e))
    chk('inventory','source '+suffix+' parses',not fail,fail or [str(len(files))+' files'])

out={'schema':'RAB_FOUR_LAYER_CODE_CLOSURE_AUDIT_V1',
     'basis':'isolation 698 PASS frozen source + verified new code diff',
     'only_source_and_offline':True,'physical_tests_executed':False,
     'ros_package_count':len(packages),'static_checks':checks,
     'static_pass':sum(x['status']=='PASS_STATIC' for x in checks),
     'static_fail':sum(x['status']=='FAIL_STATIC' for x in checks),
     'unresolved_qualification':[
        'C++ Guard H2H blocked intentionally; Python H2H Guard is selected',
        '7ms GuardAhead and 15ms Guard OS/ROS worst-case timing not proven',
        'R-A3 true-joint remains experimental and not formally optimal/physically qualified',
        'high-speed MPC v(s) applied to Ego not proven end-to-end',
        'H2H full Red deviation/ABORT/MERGE/FOLLOW/REJOIN/retry not proven',
        'G0-G9 official promotion lacks live evidence and is not granted',
        'new 17-package Ubuntu regression must run again after this patch']}
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--json-out');args=ap.parse_args()
    if args.json_out:Path(args.json_out).write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
    print('AUDIT_PACKAGES',len(packages),'STATIC',out['static_pass'],'/',len(checks))
    for c in checks:
        if not c['status'].startswith('PASS'):print('FAIL',c['layer'],c['contract'],c['source_paths'])
    print('SIMULATION_NOT_PERFORMED;G0_G9_NOT_QUALIFIED')
    raise SystemExit(bool(out['static_fail']))
