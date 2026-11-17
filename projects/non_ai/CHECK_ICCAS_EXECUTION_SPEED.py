from pathlib import Path
from types import SimpleNamespace
import csv,importlib.util,sys,json,math
r=Path(__file__).resolve().parent
p=r/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/execution_speed_authority.py'
s=importlib.util.spec_from_file_location('exec_speed',p);mod=importlib.util.module_from_spec(s);sys.modules[s.name]=mod;s.loader.exec_module(mod)
data=r/'src/driving/algorithms/f1tenth_racing_raceline/data'
results={}
for fn in ('iccas2025_r_a2_racing_trajectory.csv','iccas2025_r_a3_scp_racing_trajectory.csv'):
 with (data/fn).open(newline='') as h:
  rows=[SimpleNamespace(**{k:float(v) for k,v in d.items()}) for d in csv.DictReader(h)]
 period=rows[-1].s_m+rows[-1].ds_to_next_m
 ref,a=mod.derive_closed_loop_execution_speed(rows,period,vehicle_speed_bound_mps=20.,braking_mps2=2.,acceleration_mps2=4.,proof_horizon_s=8.,reaction_s=.25,lateral_accel_bound_mps2=6.,physics_step_s=.025)
 assert len(ref)==len(rows) and not a.qualified
 assert a.planned_peak_mps>5.0 and a.planned_peak_mps<a.source_peak_mps
 assert all(0<=s.target_speed_mps<=row.target_speed_mps for s,row in zip(ref,rows))
 results[fn]={'authored_peak_mps':a.source_peak_mps,'execution_proposal_peak_mps':a.planned_peak_mps,'reduced_samples':a.reduced_samples,'period_m':period,'samples':len(rows),'independent_guard_certificate':'NOT_QUALIFIED'}
print('ICCAS_R_A2_R_A3_CLOSED_LOOP_SPEED_ENVELOPE_PASS',json.dumps(results,sort_keys=True))
