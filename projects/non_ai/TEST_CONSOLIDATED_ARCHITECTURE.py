import importlib.util, unittest, math, random
from pathlib import Path
from types import SimpleNamespace
ROOT=Path('/mnt/data/F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018')
C=ROOT/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/execution_speed_authority.py'
H=ROOT/'src/driving/algorithms/f1tenth_racing_overtaking/f1tenth_racing_overtaking/dynamic_occupancy_certificate.py'
def load(p):
 import sys
 spec=importlib.util.spec_from_file_location('completion_'+p.stem,p);mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod);return mod
s=load(C);h=load(H)
def sample(pos,v,k=0.):return SimpleNamespace(s_m=pos,target_speed_mps=v,curvature_per_m=k,target_accel_mps2=0.,x_m=pos,y_m=0.,heading_rad=0.,lateral_offset_m=0.,ds_to_next_m=1.,sample_index=0)
def path(vals,curves=None):return tuple(sample(float(i),float(v),float((curves or [0]*len(vals))[i])) for i,v in enumerate(vals))
def plan(p):return s.derive_closed_loop_execution_speed(p,len(p),vehicle_speed_bound_mps=20.,braking_mps2=2.,acceleration_mps2=4.,proof_horizon_s=8.,reaction_s=.25,lateral_accel_bound_mps2=6.,physics_step_s=.025)
class Test(unittest.TestCase):
 def test_20_high_speed_physics_necessary_horizon(self):
  a,r=plan(path([20.]*150));self.assertAlmostEqual(r.horizon_necessary_speed_mps,15.45);self.assertEqual(len(a),150);self.assertTrue(all(q.target_speed_mps<=15.45 for q in a));self.assertFalse(r.qualified)
 def test_no_legacy_cap_at_5(self):
  a,r=plan(path([12.]*150));self.assertTrue(all(abs(q.target_speed_mps-12.)<1e-8 for q in a))
 def test_corner_brake_lookahead_ring(self):
  p=path([20.]*120,[0.]*80+[0.7]+[0.]*39);a,r=plan(p)
  self.assertLess(a[80].target_speed_mps,3.)
  self.assertLess(a[76].target_speed_mps,a[60].target_speed_mps)
  for i in range(len(a)):
   j=(i+1)%len(a);self.assertLessEqual(a[i].target_speed_mps**2,a[j].target_speed_mps**2+4.+1e-6)
   self.assertLessEqual(a[j].target_speed_mps**2,a[i].target_speed_mps**2+8.+1e-6)
 def test_accel_feasibility(self):
  a,r=plan(path([0]+[20]*59));self.assertLess(a[1].target_speed_mps,3.)
 def test_seam_brake_propagation(self):
  a,r=plan(path([1.]+[20.]*59));self.assertLess(a[-1].target_speed_mps,3.)
 def test_invalid(self):
  for bad in (path([0,math.nan,20]),path([-1,12,20]),path([0,12,20],[0,math.inf,0])):
   with self.assertRaises(ValueError):plan(bad)
  with self.assertRaises(ValueError):
   s.derive_closed_loop_execution_speed(path([1,2,3]),3.,vehicle_speed_bound_mps=20,braking_mps2=2,acceleration_mps2=4,proof_horizon_s=.1,reaction_s=.25,lateral_accel_bound_mps2=6,physics_step_s=.025)
 def test_reference_does_not_mutate(self):
  src=path([20]*30);a,r=plan(src);self.assertEqual(src[0].target_speed_mps,20);self.assertNotEqual(a[0].target_speed_mps,20)
 def test_red_well_separated(self):
  ego=[h.SpaceTimePoint(i,0,0,.3) for i in range(11)]
  red=[h.SpaceTimePoint(i,6,0,.5) for i in range(11)]
  self.assertTrue(h.check_dynamic_occupancy(ego,red,predicted_at_s=10,now_s=10,max_age_s=.2,required_horizon_s=10,margin_m=.3).safe)
 def test_red_crossing_inter_sample(self):
  ego=[h.SpaceTimePoint(0,-2,0,.3),h.SpaceTimePoint(1,2,0,.3)]
  red=[h.SpaceTimePoint(0,0,-2,.3),h.SpaceTimePoint(1,0,2,.3)]
  r=h.check_dynamic_occupancy(ego,red,predicted_at_s=10,now_s=10,max_age_s=.2,required_horizon_s=1,margin_m=.3)
  self.assertEqual(r.reason,'DYNAMIC_SWEPT_COLLISION')
 def test_uncovered_and_stale_fail_closed(self):
  e=[h.SpaceTimePoint(i,0,0,.3) for i in range(3)];r=[h.SpaceTimePoint(i,6,0,.5) for i in range(3)]
  self.assertFalse(h.check_dynamic_occupancy(e,r,predicted_at_s=10,now_s=11,max_age_s=.2,required_horizon_s=2,margin_m=.3).safe)
  self.assertFalse(h.check_dynamic_occupancy(e,r,predicted_at_s=10,now_s=10,max_age_s=.2,required_horizon_s=8,margin_m=.3).safe)
 def test_mutant_reference_rejected(self):
  src=list(path([1,2,3]));src[1].s_m=0
  with self.assertRaises(ValueError):plan(src)

# Additional independent evidence-level checks, without ROS imports.
class PhysicalLapChecks(unittest.TestCase):
    def setUp(self):
        import math
        base=ROOT/'src/verification/solo_champion/physical_lap_oracle.py'
        self.mod=load(base)
        self.pts=[(2*math.cos(2*math.pi*i/100),2*math.sin(2*math.pi*i/100)) for i in range(100)]
    def observations(self,laps):
        import math
        return [self.mod.Observation(i*.1,2*math.cos(2*math.pi*(.95+i/100)),2*math.sin(2*math.pi*(.95+i/100))) for i in range(laps*100+8)]
    def test_first_seam_only_never_completes(self):
        o=self.observations(0)
        self.assertEqual(self.mod.qualify_physical_laps(o,self.pts,corridor_half_width_m=.4,max_vehicle_speed_mps=20).qualified_laps,0)
    def test_one_full_lap_after_start_marker(self):
        o=self.observations(1)
        r=self.mod.qualify_physical_laps(o,self.pts,corridor_half_width_m=.4,max_vehicle_speed_mps=20)
        self.assertTrue(r.valid);self.assertEqual(r.qualified_laps,1)
    def test_collision_revokes(self):
        o=self.observations(1)
        o[60]=self.mod.Observation(o[60].time_s,o[60].x_m,o[60].y_m,True)
        self.assertFalse(self.mod.qualify_physical_laps(o,self.pts,corridor_half_width_m=.4,max_vehicle_speed_mps=20).valid)
    def test_reverse_rejected(self):
        o=self.observations(0)
        o[3],o[4]=o[4],o[3]
        self.assertFalse(self.mod.qualify_physical_laps(o,self.pts,corridor_half_width_m=.4,max_vehicle_speed_mps=20).valid)

if __name__=='__main__':unittest.main(verbosity=2)
