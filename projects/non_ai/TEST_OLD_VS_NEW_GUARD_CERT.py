#!/usr/bin/env python3
"""Compare the exact old and patched Guard certificate function on incident #252."""
import ast,math,time
from pathlib import Path
from TEST_GUARD_TICK_BINDING import make_case,load_method
root=Path(__file__).resolve().parent
src=(root/'ORIGINAL_P0_GUARD_CERTIFICATE.py.txt').read_text()
mod=ast.parse(src); assert len(mod.body)==1
ns={'math':math,'time':time,'ns':lambda t:int(t.sec)*1_000_000_000+int(t.nanosec)}
exec(compile(mod,'P0_ORIGINAL_Certificate','exec'),ns)
original=ns['_certificate']
old_g,old_msg,old_env,d=make_case()
new_g,new_msg,new_env,d=make_case()
old=original(old_g,old_msg)
new=load_method()(new_g,new_msg)
assert old is False and old_env.evaluate_calls==0,(old,old_env.evaluate_calls)
assert new is True and new_env.evaluate_calls==1,(new,new_env.evaluate_calls)
print('EXACT_ORIGINAL_GUARD_CERT_FALSE_BEFORE_PROOF; PATCHED_GUARD_CERT_TRUE_WITH_FULL_PROOF')
print('P0_INTENT_252_SAFE_BOUNDED_BINDING_REGRESSION_PASS; ROS_RUNTIME_UNQUALIFIED')
