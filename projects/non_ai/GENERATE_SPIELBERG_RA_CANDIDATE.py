#!/usr/bin/env python3
"""Generate Spielberg R-A0→R-A1→R-A2→R-A3 with explicit map provenance.

Offline artifact generator only. Outputs are NOT installed into runtime policy
and cannot be activated by the ROS Guard without a separate admission step.
Refuses stages with missing/corrupt source artifacts and writes an auditable
result/status JSON even if a later optimizer stage rejects its candidate.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--sim-share',type=Path,required=True,
                    help='f1tenth_racing_sim package share or source root')
    ap.add_argument('--config-a0',type=Path)
    ap.add_argument('--config-a1',type=Path)
    ap.add_argument('--config-scp',type=Path)
    args=ap.parse_args()
    from f1tenth_racing_raceline.foundation_adapter import load_problem, solve_problem
    from f1tenth_racing_raceline.artifacts import write_artifacts, write_speed_artifacts
    from f1tenth_racing_raceline.speed_adapter import load_speed_problem, solve_speed_problem, validate_dense, centerline_comparator
    from f1tenth_racing_raceline.trajectory import compose_racing_trajectory, write_racing_trajectory, validate_racing_trajectory
    from f1tenth_racing_raceline.scp_adapter import load_scp_problem, solve_scp_problem, write_scp_candidate
    import f1tenth_racing_raceline.trajectory as trajectory_module
    import inspect

    output=args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    env=(args.sim_share/'config/spielberg_environment.yaml').resolve(strict=True)
    problem=load_problem(args.config_a0,environment_path_override=env)
    assert problem['environment']['environment_identity']['active_track']=='spielberg'
    trace={'schema':'RAB_SPIELBERG_OFFLINE_RA_PIPELINE_V1',
           'track_id':'spielberg','source_environment_sha256':digest(env),
           'map_sha256':problem['hashes']['map'],
           'centerline_sha256':problem['hashes']['centerline'],
           'status':'RUNNING','stages':[],'runtime_tracking_qualified':False,
           'Guard_authority':False}
    report=output/'SPIELBERG_RA_OFFLINE_REPORT.json'
    def step(name,files):
        for path in files:
            if not path.is_file() or not path.stat().st_size:
                raise RuntimeError('OFFLINE_RA_ARTIFACT_MISSING:'+str(path))
        trace['stages'].append({'stage':name,'files':{p.name:digest(p) for p in files}})
        report.write_text(json.dumps(trace,indent=2,sort_keys=True)+'\n')
    try:
        line,validation,centerline_metrics,line_metrics=solve_problem(problem)
        a0,a0meta,_=write_artifacts(output,line,problem,validation,centerline_metrics,line_metrics)
        step('R_A0_VERIFIED_GEOMETRY',[a0,a0meta])
        speed=load_speed_problem(args.config_a1,r_a0_csv_path=a0,
            r_a0_metadata_path=a0meta,environment_path_override=env)
        profile=solve_speed_problem(speed)
        dense=validate_dense(profile,speed['r_a0_metadata']['metrics']['centerline']['length'],
            int(speed['optimizer_data']['dense_samples_per_segment']))
        comparator,comp_sha=centerline_comparator(speed)
        a1,a1meta,_=write_speed_artifacts(output,profile,speed,dense,
            {'centerline_sha256':comp_sha,'comparator':{'lap_s':comparator.predicted_lap_time}})
        step('R_A1_PHYSICS_SPEED',[a1,a1meta])
        trajectory=compose_racing_trajectory(a0,a0meta,a1,a1meta,
            digest(inspect.getsourcefile(trajectory_module)),expected_track_id='spielberg')
        a2,a2meta,canonical_a2=write_racing_trajectory(output,trajectory,
            stem='spielberg_r_a2_racing_trajectory')
        validate_racing_trajectory(canonical_a2,a2)
        step('R_A2_VALIDATED_TRAJECTORY',[a2,a2meta])
        scp_problem=load_scp_problem(args.config_scp,baseline_csv=a2,
            baseline_metadata=a2meta,environment_path=env,
            r_a0_csv=a0,r_a0_metadata=a0meta)
        optimized,result=solve_scp_problem(scp_problem)
        a3,a3meta,product=write_scp_candidate(output,optimized)
        if result.accepted_steps<1:
            raise RuntimeError('SPIELBERG_SCP_NO_ACCEPTED_OPTIMIZATION')
        step('R_A3_SCP_OPTIMIZED_CANDIDATE',[a3,a3meta])
        trace.update({'status':'OFFLINE_CANDIDATE_GENERATED_UNQUALIFIED_RUNTIME',
                      'candidate_trajectory_id':product.trajectory_id,
                      'scp_accepted_steps':result.accepted_steps})
    except BaseException as e:
        trace.update({'status':'FAILED_CLOSED','failure':type(e).__name__+':'+str(e)})
        report.write_text(json.dumps(trace,indent=2,sort_keys=True)+'\n')
        raise
    report.write_text(json.dumps(trace,indent=2,sort_keys=True)+'\n')
    print('SPIELBERG_OFFLINE_RA_CANDIDATE_GENERATED_NOT_RUNTIME_QUALIFIED',report)


if __name__=='__main__':
    main()
