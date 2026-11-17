"""Experimental C++ stopping-rollout adapter, NOT enabled for actuation.

This module must be explicitly instantiated with a locally compiled native
library and independently compared to the authoritative Python proof.
Neither this module nor its existence authorizes any Guard certificate.
"""
from ctypes import CDLL, POINTER, c_double, c_int, c_int64, c_uint8, c_void_p
from pathlib import Path
import ctypes

from .stopping_envelope import StopCertificate

_FIELDS = ('x','y','yaw','speed','vx','vy','yaw_rate','acceleration',
           'ax','ay','slip','requested_acceleration','jerk','steering','steering_rate')
_REASONS = {
    0:'CONDITIONAL_STOP_ONLY',
    1:'PROOF_BUDGET_UNPROVEN',
    2:'NONFINITE_INITIAL_STATE',
    3:'NONFINITE_COMMAND',
    4:'SPEED_OUTSIDE_MODEL',
    5:'MODEL_FAILED',
    6:'NONFINITE_ROLLOUT',
    7:'DYNAMIC_LIMIT',
    8:'SWEPT_COLLISION',
    9:'REVERSED_BEFORE_HOLD',
    10:'TIMEOUT_NO_FULL_STOP',
}

class ExperimentalNativeStopProof:
    """Owned native map/dynamics instance. Do not install into ROS by default."""
    def __init__(self, authoritative_python_envelope, library_path):
        s=authoritative_python_envelope
        c=s.dynamics.c
        lim=c['limits']
        low=c.get('low_speed_transition',{})
        ft=c['front_tire'];rt=c['rear_tire']; fp=s.footprint;occ=s.map
        par=[
            c['mass_kg'],c['yaw_inertia_kg_m2'],c['front_axle_from_cg_m'],
            c['rear_axle_from_cg_m'],ft['pacejka_b'],ft['pacejka_c'],ft['pacejka_d_n'],
            rt['pacejka_b'],rt['pacejka_c'],rt['pacejka_d_n'],
            lim['steering_angle_abs_max_rad'],lim['steering_rate_abs_max_rad_s'],
            lim['speed_abs_max_m_s'],lim['acceleration_max_m_s2'],
            lim['deceleration_max_m_s2'],lim['jerk_abs_max_m_s3'],
            low.get('enter_mps',0.5),low.get('exit_mps',1.5),low.get('response_time_s',0.12),
            s.dt,s.max_time,s.brake,s.reaction,s.padding,s.slip_limit,s.lat_limit,
            fp['min_x_m'],fp['max_x_m'],fp['min_y_m'],fp['max_y_m'],
            occ.resolution,occ.origin_x,occ.origin_y,occ.origin_yaw,occ.threshold,
            float(occ.flip_y),
        ]
        if len(par)!=36:raise RuntimeError('INVALID_CONFIG_SCHEMA')
        lib=CDLL(str(Path(library_path).resolve(strict=True)))
        lib.f1_create_native_stop.argtypes=[POINTER(c_double), POINTER(c_uint8),c_int,c_int]
        lib.f1_create_native_stop.restype=c_void_p
        lib.f1_evaluate_native_stop.argtypes=[c_void_p,POINTER(c_double),c_double,c_double,c_int64,POINTER(c_double)]
        lib.f1_evaluate_native_stop.restype=c_int
        lib.f1_evaluate_native_dualtick.argtypes=[c_void_p,POINTER(c_double),
            c_double,c_double,c_double,c_double,c_int64,POINTER(c_double)]
        lib.f1_evaluate_native_dualtick.restype=c_int
        lib.f1_destroy_native_stop.argtypes=[c_void_p]
        self._lib=lib
        config=(c_double*36)(*map(float,par))
        pixels=(c_uint8*len(occ.pixels)).from_buffer_copy(occ.pixels)
        self._handle=lib.f1_create_native_stop(config,pixels,occ.width,occ.height)
        if not self._handle:raise RuntimeError('NATIVE_PROOF_ENGINE_INIT_FAILED')

    def close(self):
        if getattr(self,'_handle',None):
            self._lib.f1_destroy_native_stop(self._handle)
            self._handle=None

    def __del__(self):
        self.close()

    def evaluate_dual(self, observed, steer, accel, applied_steer,
                      applied_accel, deadline_ns):
        if self._handle is None:raise RuntimeError('NATIVE_PROOF_CLOSED')
        vals=(c_double*15)(*(float(getattr(observed,k)) for k in _FIELDS))
        out=(c_double*6)()
        rc=self._lib.f1_evaluate_native_dualtick(self._handle,vals,float(steer),
            float(accel),float(applied_steer),float(applied_accel),
            int(deadline_ns),out)
        if rc != 0:raise RuntimeError('NATIVE_DUAL_TICK_UNPROVEN:'+str(rc))
        return (bool(out[0]),bool(out[1]),bool(out[2]),bool(out[3]))

    def evaluate(self, state, steer, accel, deadline_ns=None):
        if self._handle is None:raise RuntimeError('NATIVE_PROOF_CLOSED')
        vals=(c_double*15)(*(float(getattr(state,k)) for k in _FIELDS))
        out=(c_double*8)()
        err=self._lib.f1_evaluate_native_stop(
            self._handle,vals,float(steer),float(accel),
            0 if deadline_ns is None else int(deadline_ns),out)
        if err!=0:raise RuntimeError('NATIVE_PROOF_ENGINE_ERROR:'+str(err))
        reason_code=int(out[1]);reason=_REASONS.get(reason_code)
        if reason is None:raise RuntimeError('UNKNOWN_NATIVE_PROOF_REASON')
        return StopCertificate(bool(out[0]),reason,float(out[2]),float(out[3]),
                               float(out[4]),float(out[5]),float(out[6]),int(out[7]))

class ExperimentalNativeProofProxy:
    """Explicitly gated C++ full stopping-rollout backend.

    Dynamics stepping still uses the simulator's Python class. The Guard
    must independently instantiate its own engine from its own map/state and
    must never accept an R-B certificate as final actuator authority. Native
    deployment is optional and not a formal realtime certification.
    """
    def __init__(self, authoritative_python_envelope, library_path):
        self.dynamics=authoritative_python_envelope.dynamics
        self.native=ExperimentalNativeStopProof(authoritative_python_envelope,library_path)
    def evaluate_dual(self, observed, steer, accel, applied_steer,
                      applied_accel, deadline_ns):
        return self.native.evaluate_dual(observed,steer,accel,
                            applied_steer,applied_accel,deadline_ns)

    def evaluate(self,state,steer,accel,*,deadline_ns=None):
        return self.native.evaluate(state,steer,accel,deadline_ns)
    def evaluate_bounded(self,state,steer,accel,deadline_ns):
        return self.native.evaluate(state,steer,accel,deadline_ns)

def check_experimental_parity_receipt(library_path, receipt_path, *, expected_map_sha256=None, active_map_file=None):
    """Fail closed on mismatched native image or archived parity input source.

    This is a build/provenance gate, NOT a formal WCET or safety proof.
    """
    import hashlib
    import json
    receipt_file=Path(receipt_path).resolve(strict=True)
    receipt=json.loads(receipt_file.read_text())
    if (receipt.get('schema')!='F1TENTH_NATIVE_STOP_EXPERIMENT_V1' or
            receipt.get('result')!='OFFLINE_PARITY_PASS' or
            int(receipt.get('stop_cases',0))<1176 or
            int(receipt.get('guard_ahead_cases',0))<130 or
            int(receipt.get('native_dualtick_parity_cases',0))<130):
        raise ValueError('NATIVE_PARITY_RECEIPT_NOT_QUALIFIED')
    library_file=Path(library_path).resolve(strict=True)
    if (not library_file.is_file() or
            hashlib.sha256(library_file.read_bytes()).hexdigest()!=receipt.get('library_sha256')):
        raise ValueError('NATIVE_PARITY_LIBRARY_HASH_MISMATCH')
    entries=receipt.get('source_hashes',[])
    if not isinstance(entries,list) or len(entries)<9:
        raise ValueError('NATIVE_PARITY_SOURCE_MANIFEST_MISSING')
    recorded={}
    for entry in entries:
        candidate=Path(entry['path']).resolve(strict=True)
        if not candidate.is_file() or hashlib.sha256(candidate.read_bytes()).hexdigest()!=entry['sha256']:
            raise ValueError('NATIVE_PARITY_SOURCE_HASH_MISMATCH')
        recorded[candidate.name]=entry['sha256']
    # Independent safety activation MUST bind the parity receipt to the
    # actual active map. ICCAS parity evidence does not authorize a different
    # Spielberg occupancy map. Old 2-argument parity test callers remain
    # supported for pure offline testing, never by the deployed Guard/R-B.
    if expected_map_sha256 is not None or active_map_file is not None:
        if not expected_map_sha256 or not active_map_file:
            raise ValueError('NATIVE_ACTIVE_MAP_PROVENANCE_MISSING')
        active=Path(active_map_file).resolve(strict=True)
        actual_map=hashlib.sha256(active.read_bytes()).hexdigest()
        if actual_map!=expected_map_sha256:
            raise ValueError('NATIVE_ACTIVE_MAP_HASH_MISMATCH')
        if not any(Path(e['path']).name==active.name and
                   e.get('sha256')==actual_map for e in entries):
            raise ValueError('NATIVE_PARITY_NOT_RUN_FOR_ACTIVE_MAP')
    # The live imported simulator/rollout is also pinned. A passing test
    # against an entirely different extracted package must not qualify this
    # runtime's Guard or R-B module by accident.
    from f1tenth_racing_sim.physics import vehicle,pacejka
    from f1tenth_racing_sim.world import swept_collision,map as map_module
    from f1tenth_racing_sim.safety import stopping_envelope
    live=(Path(__file__),Path(vehicle.__file__),Path(pacejka.__file__),
          Path(swept_collision.__file__),Path(map_module.__file__),
          Path(stopping_envelope.__file__))
    for path in live:
        path=path.resolve(strict=True)
        if recorded.get(path.name)!=hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError('NATIVE_PARITY_ACTIVE_RUNTIME_SOURCE_MISMATCH')
    return True

# Backward compatibility for the experimental R-B preflight construction.
ExperimentalRBNativeProofProxy=ExperimentalNativeProofProxy
