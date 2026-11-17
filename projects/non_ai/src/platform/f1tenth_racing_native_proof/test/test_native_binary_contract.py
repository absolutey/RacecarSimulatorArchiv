"""V7 binary ABI/lifecycle and negative-input checks, not an RT certificate."""
import ctypes
import hashlib
import math
import os
from pathlib import Path
import pytest

LIB=Path(os.environ['F1_NATIVE_LIBRARY'])
COPIED=Path(os.environ['F1_NATIVE_IMPLEMENTATION_SOURCE'])
REFERENCE=Path(os.environ['F1_NATIVE_REFERENCE_SOURCE'])

def library():
    assert LIB.is_file(), 'NATIVE_ROS_SHARED_LIBRARY_NOT_COMPILED'
    lib=ctypes.CDLL(str(LIB))
    lib.f1_create_native_stop.argtypes=(ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_uint8),ctypes.c_int,ctypes.c_int)
    lib.f1_create_native_stop.restype=ctypes.c_void_p
    lib.f1_destroy_native_stop.argtypes=(ctypes.c_void_p,)
    lib.f1_evaluate_native_stop.argtypes=(ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),ctypes.c_double,ctypes.c_double,
        ctypes.c_int64,ctypes.POINTER(ctypes.c_double))
    lib.f1_evaluate_native_stop.restype=ctypes.c_int
    return lib

def params():
    p=[1.0]*36
    p[16]=0.5;p[17]=1.5;p[18]=0.12
    p[19]=0.01;p[20]=1.0;p[21]=1.0
    p[30]=0.05;p[34]=120;p[35]=0
    return (ctypes.c_double*36)(*p)

def new_engine(lib, p=None, width=32, height=32):
    pix=(ctypes.c_uint8*(width*height))(*([255]*(width*height)))
    return lib.f1_create_native_stop(p if p is not None else params(),pix,width,height)

def evaluate(lib,handle,values=None,steer=0.0,accel=0.0,deadline=0):
    state=(ctypes.c_double*15)(*([0.0]*15 if values is None else values))
    out=(ctypes.c_double*8)()
    code=lib.f1_evaluate_native_stop(handle,state,steer,accel,deadline,out)
    return code,list(out)

def test_file_integrity():
    assert hashlib.sha256(COPIED.read_bytes()).digest()==hashlib.sha256(REFERENCE.read_bytes()).digest()

def test_exported_symbols_present():
    lib=library()
    assert lib.f1_create_native_stop and lib.f1_destroy_native_stop and lib.f1_evaluate_native_stop

def test_null_config_denied():
    lib=library();pix=(ctypes.c_uint8*4)(255,255,255,255)
    assert not lib.f1_create_native_stop(None,pix,2,2)

def test_null_map_denied():
    assert not library().f1_create_native_stop(params(),None,2,2)

@pytest.mark.parametrize('dimension',[(0,2),(2,0),(-1,2),(2,-1)])
def test_bad_dimensions_denied(dimension):
    lib=library();pix=(ctypes.c_uint8*4)(255,255,255,255)
    assert not lib.f1_create_native_stop(params(),pix,*dimension)

@pytest.mark.parametrize('index,value',[(30,0.0),(19,0.0),(20,0.0),(21,0.0),(18,0.0),(17,0.1),(0,float('nan'))])
def test_invalid_parameters_denied(index,value):
    lib=library();p=params();p[index]=value
    assert not new_engine(lib,p)

def test_engine_lifecycle():
    lib=library();h=new_engine(lib)
    assert h
    lib.f1_destroy_native_stop(h)

@pytest.mark.parametrize('index,value',[(0,float('nan')),(3,float('inf')),(7,-float('inf'))])
def test_nonfinite_initial_state_fails_closed(index,value):
    lib=library();h=new_engine(lib);assert h
    try:
        s=[0.0]*15;s[index]=value
        code,out=evaluate(lib,h,s)
        assert code==0 and out[0]==0 and out[1]==2
    finally:lib.f1_destroy_native_stop(h)

@pytest.mark.parametrize('steer,accel',[(float('nan'),0.0),(0.0,float('inf'))])
def test_nonfinite_command_fails_closed(steer,accel):
    lib=library();h=new_engine(lib);assert h
    try:
        code,out=evaluate(lib,h,steer=steer,accel=accel)
        assert code==0 and out[0]==0 and out[1]==3
    finally:lib.f1_destroy_native_stop(h)

def test_expired_deadline_fails_closed():
    lib=library();h=new_engine(lib);assert h
    try:
        code,out=evaluate(lib,h,deadline=1)
        assert code==0 and out[0]==0 and out[1]==1
    finally:lib.f1_destroy_native_stop(h)

def test_null_engine_cannot_approve():
    lib=library();code,_=evaluate(lib,None)
    assert code != 0


def test_native_single_tick_abi_fail_closed_and_unchanged_on_failure():
    lib=library()
    lib.f1_step_native.argtypes=(ctypes.c_void_p,ctypes.POINTER(ctypes.c_double),
        ctypes.c_double,ctypes.c_double,ctypes.c_double)
    lib.f1_step_native.restype=ctypes.c_int
    handle=new_engine(lib)
    try:
        state=(ctypes.c_double*15)(*([0.0]*15))
        assert lib.f1_step_native(handle,state,0.,0.,-0.025)!=0
        assert list(state)==[0.0]*15
        state[3]=float('nan')
        assert lib.f1_step_native(handle,state,0.,0.,0.025)!=0
        assert math.isnan(state[3])
    finally:
        lib.f1_destroy_native_stop(handle)
