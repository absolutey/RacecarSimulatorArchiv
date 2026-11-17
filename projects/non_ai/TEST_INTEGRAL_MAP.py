#!/usr/bin/env python3
"""Exact cell threshold comparison for rotated, flipped and non-flipped maps."""
from pathlib import Path
import sys, tempfile, random
sys.path.insert(0,str(Path(__file__).resolve().parent/'src/environment/f1tenth_racing_sim'))
from f1tenth_racing_sim.world.map import OccupancyMap
from f1tenth_racing_sim.world.swept_collision import swept_footprint_collision,footprint_collision
from f1tenth_racing_sim.physics.vehicle import State
rng=random.Random(18923)
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'synthetic.pgm'
 for flip in [True,False]:
  for yaw in [0.0,0.45,1.75]:
   raw=bytes((0 if rng.random()<.25 else 254) for _ in range(40*60))
   p.write_bytes(b'P5\n40 60\n255\n'+raw)
   cfg={'resolution_m_per_pixel':.05,'origin':{'x_m':1.0,'y_m':-3.0,'yaw_rad':yaw},'runtime_pixel_occupied_below':128,'runtime_flip_y':flip}
   occ=OccupancyMap(p,cfg)
   for _ in range(750):
    i=rng.randrange(60);j=rng.randrange(i,60);c=rng.randrange(40);e=rng.randrange(c,40)
    direct=bool((occ.pixel_array[i:j+1,c:e+1]>=occ.threshold).all())
    assert occ.rectangle_is_free(i,j,c,e)==direct
   assert not occ.rectangle_is_free(-1,2,1,3)
   assert not occ.rectangle_is_free(0,60,1,3)
   # If occupancy threshold changes, the cached lookup must cease being used.
   occ.threshold=0
   assert occ.rectangle_is_free(0,59,0,39)
print('ROTATED_FLIPPED_RECTANGLE_EQUIVALENCE_PASS',6*750,'rectangles')
