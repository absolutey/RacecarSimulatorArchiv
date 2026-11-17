#!/usr/bin/env python3
"""Deterministic 3rd pass static architecture audit, NOT ROS runtime qualification."""
import ast, hashlib, json, re, sys, xml.etree.ElementTree as ET
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parent
SRC=ROOT/'src'
checks=[]
def check(group, name, good, evidence):
    checks.append({'group':group,'name':name,'result':'PASS_STATIC' if good else 'FAIL_STATIC','evidence':evidence})
    return good
def txt(path): return (ROOT/path).read_text()
def sha(path): return hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
paths=list(SRC.rglob('*'))
packages=sorted(SRC.rglob('package.xml'))
package_inventory=[]
for p in packages:
    obj=ET.parse(p).getroot()
    name=obj.findtext('name')
    package_inventory.append({'name':name,'path':p.parent.relative_to(ROOT).as_posix(),
        'source_files':len([f for f in p.parent.rglob('*') if f.is_file()]),
        'declared_dependencies':[e.text for e in obj if e.tag in ('depend','exec_depend','build_depend')]})
check('inventory','all_17_packages',len(packages)==17 and len({p['name'] for p in package_inventory})==17,
    f'{len(packages)} package.xml')
for suffix,parser in (('.py',lambda s:ast.parse(s)),('.yaml',yaml.safe_load),('.json',json.loads)):
    selected=[p for p in ROOT.rglob('*'+suffix) if 'build' not in p.parts and 'install' not in p.parts]
    failed=[]
    for f in selected:
        try:parser(f.read_text())
        except Exception as exc:failed.append(f'{f.relative_to(ROOT)}: {exc}')
    check('syntax','parse_'+suffix,not failed,f'{len(selected)} files, errors={failed[:3]}')
for p in packages:ET.parse(p)
cpp=txt('src/platform/f1tenth_racing_native_proof/src/native_guard_ros.cpp')
launch=txt('src/platform/f1tenth_racing_bringup/launch/race_foundation.launch.py')
sim=txt('src/environment/f1tenth_racing_sim/f1tenth_racing_sim/simulator.py')
watch=txt('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/independent_watchdog_node.py')
coord=txt('src/platform/f1tenth_racing_bringup/scripts/race_coordinator.py')
check('guard','native_guard_exact_ros_identity',
    'explicit NativeGuard():Node("actuator_guard0")' in cpp and
    'name="actuator_guard0",output="screen"' in launch and
    all("node_name!='actuator_guard0'" in x for x in (sim,watch)) and
    'node_name!="actuator_guard0"' in coord,
    'native Guard = simulator = independent watchdog = Race Coordinator = actuator_guard0')
check('guard','exclusive_python_or_cpp_guard',
    'condition=UnlessCondition(LaunchConfiguration("use_native_guard_ros"))' in launch and
    'condition=IfCondition(LaunchConfiguration("use_native_guard_ros"))' in launch,
    'mutually exclusive ROS launch conditions')
check('guard','cpp_guard_only_solo',
    'NATIVE_CPP_GUARD_H2H_NOT_QUALIFIED' in launch and 'if(!solo||' in cpp,
    'H2H C++ authority prohibited')
check('guard','cpp_stop_and_deadline_15ms',
    all(v in cpp for v in ('deadline=begun+15000000','f1_evaluate_native_stop(',
                          'stop_certificate_verified=true','NATIVE_PRE_PUBLISH_LEASE_EXPIRED')),
    'full stop, monotonic deadline and final lease check')
check('guard','cpp_minimum_risk_does_not_forge_cert',
    'cmd.stop_certificate_verified=false;' in cpp and 'MINIMUM_RISK_BRAKE' in cpp,
    'brake is UNQUALIFIED authority mode')
check('guard','simulator_final_veto_and_deadman',
    all(v in sim for v in ('ATOMIC_GUARD_PUBLISHER_MISMATCH',
       'ATOMIC_EXTERNAL_WATCHDOG_REJECT','GUARD_STATUS_PUBLISHER_MISMATCH')),
    'simulator retains distinct Guard status / watchdog veto')
rb=txt('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/node.py')
ga=txt('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/guard_ahead.py')
process=txt('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/process_primary_worker.py')
check('rb','fresh_rb_qos_latest_only',
    'depth=1, reliability=ReliabilityPolicy.BEST_EFFORT' in rb and
    'LatestOnlyControlWorker(' in rb and '75_000_000' in rb,
    'BEST_EFFORT/depth 1 plus original 75ms state contract')
check('rb','guardahead_dual_tick_7ms_gate',
    all(v in ga for v in ('MAX_EXTRA_NS = 7_000_000','evaluate_dual(',
        'GUARD_AHEAD_BUDGET_UNPROVEN','GUARD_AHEAD_RESPONSE_DEADLINE_UNPROVEN')),
    'current/next tick, no time overrun can confer proof')
check('rb','isolated_mpc_spawn_deadline',
    all(v in process for v in ("mp.get_context('spawn')",'self._ready.wait(timeout=10.)',
        'MPC_PROCESS_RESULT_EXPIRED','MPC_PROCESS_REPLY_REPLAY_OR_REORDER')),
    'separate spawned solver, no late result adopted')
check('rb','mpc_process_no_ros_publisher',
    not re.search(r'create_(?:publisher|subscription)\(',process) and
    'GuardedActuatorCommand' not in process,
    'process is not an actuator authority')
check('rb','selectable_process_mpc',
    'IsolatedMpcProcessWorker(' in rb and 'primary_process_enabled' in launch,
    'optional isolation, not implicitly enabled')
reference=txt('src/platform/f1tenth_racing_reference/f1tenth_racing_reference/node.py')
successor=txt('src/driving/algorithms/f1tenth_racing_overtaking/f1tenth_racing_overtaking/successor_node.py')
check('h2h','abort_parked_gate_and_fresh_red',
    'H2H_ABORT_RECOVERY_STATIONARY_REJOIN' in reference and
    'required_fresh_red_after_abort_ns' in successor and
    'accept_post_abort_red_observation(' in successor,
    'parked S0-R recovery + fresh Red observation epoch')
check('h2h','default_h2h_dynamic_guard',
    'H2H_GUARD_RED_PROOF_REQUIRED' in launch and
    'dynamic_red_guard_enabled' in launch,
    'no H2H without Red collision proof')
policy=json.loads(txt('src/platform/f1tenth_racing_reference/config/spielberg_nominal_reference.json'))
check('ra3','spielberg_4_registered_candidate_identities',
    {'baseline','centerline_preliminary','scp','scp_true_joint_experimental'}<=set(policy['candidates']) and
    all(re.fullmatch(r'sha256:[0-9a-f]{64}',x['source_trajectory_id']) for x in policy['candidates'].values()),
    'R-A2 and R-A3 SCP/true-joint distinct identities')
runner=txt('FIRSTPASS_MULTIMAP_RUNNER.py')
check('map','four_independent_run_profiles',
    all(x in runner for x in ('iccas2025_solo','spielberg_solo','iccas2025_h2h','spielberg_h2h')) and
    "'--execute'" in runner and 'ros2\',\'bag\',\'record\'' in runner,
    '4 map/mode combinations, opt-in execution; separate rosbag evidence')
gate=txt('src/verification/qualification_gates/gate_model.py')
receipt=txt('BUILD_FIRSTPASS_G0_G9_RECEIPTS.py')
check('gates','g0_g9_no_auto_promotion',
    'product_promoted\':False' in receipt and 'DRAFT_NOT_PRODUCT_PROMOTION' in gate and
    'source_tree_sha256' in gate and 'successful_retries' in gate,
    'no fabricated qualification, unique run/source and post-ABORT retry')
# Explicit non-passing observations (important: not software source failures).
open_observations=[
    {'id':'O1','severity':'HIGH','topic':'C++ Guard runtime','reason':'ROS binary can compile but C++ ROS node end-to-end authority, reliable status and physical brake not exercised; username handoff postponed simulation'},
    {'id':'O2','severity':'HIGH','topic':'GuardAhead 7ms','reason':'Most recent SOLO has 186/1221 cycles >7ms, and C++ fast path is selectable not yet qualified under that workload'},
    {'id':'O3','severity':'HIGH','topic':'Guard 15ms','reason':'static 15ms deadline rejects late proof but does not prove operating-system/ROS worst-case response time'},
    {'id':'O4','severity':'MEDIUM','topic':'MPC process recovery','reason':'on one take timeout active job is not retired by later submissions, so Primary may remain permanently unavailable until worker replacement; Fallback remains fail-closed'},
    {'id':'O5','severity':'HIGH','topic':'H2H closed loop','reason':'moving ABORT, stationary rejoin and fresh-red retry behavior require combined multi-node fault-injection; static presence != functional qualification'},
    {'id':'O6','severity':'MEDIUM','topic':'R-A3 optimization','reason':'864-point candidate generation exists, but convergence to mathematical optimality, velocity authority and physical tracking remain separate'},
    {'id':'O7','severity':'HIGH','topic':'G0-G9','reason':'assessment is provisional only, no signed physical WCET/H2H evidence, cannot be official PASS'},
]
out={'schema':'RAB_PASS3_SOURCE_AUDIT_V1','scope':'ALL_17_ROS_PACKAGES_PLUS_EXECUTION_GATE_TOOLS',
    'source_only':True,'simulation_executed':False,'packages':package_inventory,
    'static_checks':checks,'static_pass':sum(c['result']=='PASS_STATIC' for c in checks),
    'static_failed':sum(c['result']=='FAIL_STATIC' for c in checks),
    'observations_not_closed_by_static_tests':open_observations,
    'eligible_for_qualification_promotion':False}
if len(sys.argv)>1: Path(sys.argv[1]).write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print('PASS3_PACKAGES='+str(len(packages)))
print('PASS3_STATIC_CHECKS='+str(out['static_pass'])+'/'+str(len(checks)))
for c in checks:
    if c['result']!='PASS_STATIC':print('STATIC_FAILURE='+c['group']+':'+c['name']+' '+c['evidence'])
print('PASS3_SIMULATION_NOT_EXECUTED;G0_G9_NOT_QUALIFIED')
sys.exit(0 if out['static_failed']==0 else 1)
