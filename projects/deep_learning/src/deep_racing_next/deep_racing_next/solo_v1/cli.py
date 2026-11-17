"""CLI for package smoke, real gym training, map features, planner teacher, ROS shadow."""
import argparse,json,sys
from pathlib import Path
from .l0_maps import load_course

def main(argv=None):
    p=argparse.ArgumentParser(description='Deep Racing 8-layer Solo V1. Research-only training, no actuator authorization.')
    p.add_argument('command',choices=['map-check','train','eval','planner-train','shadow','probe'])
    p.add_argument('--map-root',default=str(Path.home()/'F1TENTH_DEEP_RACING_WS/map_assets/external_f1tenth_racetracks_20261024'))
    p.add_argument('--map',default='iccas2025');p.add_argument('--backend',choices=['gym','test_double','original_pacejka'],default='gym')
    p.add_argument('--steps',type=int,default=512);p.add_argument('--updates',type=int,default=2)
    p.add_argument('--episodes',type=int,default=2);p.add_argument('--max-steps',type=int,default=1500)
    p.add_argument('--dt',type=float,default=.01);p.add_argument('--seed',type=int,default=202)
    p.add_argument('--out',default=str(Path.home()/'F1TENTH_DEEP_RACING_WS/runs/solo_v1'))
    p.add_argument('--checkpoint',default='');p.add_argument('--planner-checkpoint',default='');p.add_argument('--epochs',type=int,default=40)
    a=p.parse_args(argv)
    if a.steps<1 or a.updates<1 or a.episodes<1 or a.max_steps<1 or a.dt<=0: p.error('POSITIVE_PARAMS_REQUIRED')
    try:
        if a.command=='shadow':
            if not a.checkpoint:p.error('--checkpoint required')
            from .l6_ros import shadow_node
            shadow_node(a.checkpoint,a.map_root,a.map);return 0
        if a.command=='probe':
            import importlib.util
            result={n:bool(importlib.util.find_spec(n)) for n in ['torch','numpy','yaml','gym','f110_gym','rclpy']}
            result['original_pacejka_certified']=False
            result['original_pacejka_backend']='PRESENT_IN_V1_2_NOT_A_PHYSICS_CERTIFICATE'
            result['backend']='F110_GYM_AVAILABLE_NOT_PACEJKA_CERTIFIED' if result['gym'] and result['f110_gym'] else 'HOLD_NO_F110_GYM'
            print(json.dumps(result,indent=2));return 0
        if a.command=='planner-train':
            from .l2_planner import fit_planner
            dirs=sorted(Path(a.map_root).iterdir());courses=[];held={}
            for d in dirs:
                if not d.is_dir():continue
                try:courses.append(load_course(a.map_root,d.name))
                except Exception as e:held[d.name]=str(e)
            print(json.dumps({'planner':fit_planner(courses,Path(a.out)/'planner.pt',epochs=a.epochs),'map_input_hold':held},indent=2));return 0
        course=load_course(a.map_root,a.map)
        if a.command=='map-check': print(json.dumps({'map':course.name,'identity':course.fingerprint,'start':course.frame(0),'status':'INPUT_ONLY_NOT_PHYSICS_CERTIFIED'},indent=2));return 0
        if a.command=='train':
            from .l5_training import run_train
            result=run_train(course,a.backend,a.steps,a.updates,a.dt,Path(a.out)/a.map,seed=a.seed,max_steps=a.max_steps,planner_checkpoint=a.planner_checkpoint or None)
        elif a.command=='eval':
            if not a.checkpoint:p.error('--checkpoint required')
            from .l5_training import run_eval
            result=run_eval(course,a.checkpoint,a.backend,a.episodes,a.dt,max_steps=a.max_steps,planner_checkpoint=a.planner_checkpoint or None)
        print(json.dumps(result,indent=2,default=str));return 0
    except Exception as e:
        print(f'HOLD:{type(e).__name__}:{e}',file=sys.stderr)
        return 2

if __name__=='__main__':raise SystemExit(main())
