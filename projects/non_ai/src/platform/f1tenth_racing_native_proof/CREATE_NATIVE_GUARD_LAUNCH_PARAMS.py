"""Construct pinned native guard physics inputs from the already verified stage.

Only the isolated simulation launch may call this. Every input map/source is
verified before C++ authority is selected; this is not deployment permission.
"""
from pathlib import Path
import hashlib


def build_native_guard_parameters(env, map_file, *, epoch, producer, speed_cap):
    from f1tenth_racing_sim.world.map import OccupancyMap
    from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
    map_file = Path(map_file).resolve(strict=True)
    if hashlib.sha256(map_file.read_bytes()).hexdigest()!=env['map']['image_sha256']:
        raise ValueError('NATIVE_GUARD_MAP_IDENTITY_INVALID')
    occupancy=OccupancyMap(map_file,env['map'])
    s=StoppingEnvelope(env['dynamics'],occupancy,
        env['collision']['footprint_relative_to_base_link'],
        reaction_s=.25,uncertainty_padding_m=.03)
    c=s.dynamics.c;lim=c['limits'];ft=c['front_tire'];rt=c['rear_tire']
    low=c.get('low_speed_transition',{});fp=s.footprint;occ=s.map
    par=[c['mass_kg'],c['yaw_inertia_kg_m2'],c['front_axle_from_cg_m'],
         c['rear_axle_from_cg_m'],ft['pacejka_b'],ft['pacejka_c'],ft['pacejka_d_n'],
         rt['pacejka_b'],rt['pacejka_c'],rt['pacejka_d_n'],
         lim['steering_angle_abs_max_rad'],lim['steering_rate_abs_max_rad_s'],
         lim['speed_abs_max_m_s'],lim['acceleration_max_m_s2'],
         lim['deceleration_max_m_s2'],lim['jerk_abs_max_m_s3'],
         low.get('enter_mps',.5),low.get('exit_mps',1.5),low.get('response_time_s',.12),
         s.dt,s.max_time,s.brake,s.reaction,s.padding,s.slip_limit,s.lat_limit,
         fp['min_x_m'],fp['max_x_m'],fp['min_y_m'],fp['max_y_m'],
         occ.resolution,occ.origin_x,occ.origin_y,occ.origin_yaw,occ.threshold,float(occ.flip_y)]
    if len(par)!=36: raise ValueError('NATIVE_GUARD_PARAMETER_SCHEMA_INVALID')
    return dict(run_epoch=epoch,producer=producer,
                track_id=env['environment_identity']['active_track'],
                map_sha256=env['map']['image_sha256'],map_file=str(map_file),
                native_config=[float(v) for v in par],
                solo=True,max_speed_cap_mps=float(speed_cap),use_sim_time=True)
