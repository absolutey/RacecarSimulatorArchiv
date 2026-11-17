"""Standalone provisional R-A centerline candidate build from supplied mapped assets.
NO optimization/qualification claim; verification requires native ROS staged tests.
"""
from pathlib import Path
import csv,math,json,hashlib
from itertools import islice
R=Path(__file__).resolve().parents[1]
P=R/'src/environment/f1tenth_racing_sim/maps/spielberg/spielberg_centerline.csv'
A=R/'src/driving/algorithms/f1tenth_racing_raceline/data'
T='spielberg'
D=json.loads((R/'src/platform/f1tenth_racing_reference/config/nominal_reference.json').read_text())
# import YAML for environment only, no ROS used; sidecar hash compared to declared map assets
import yaml
Efile=R/'src/environment/f1tenth_racing_sim/config/spielberg_environment.yaml'
E=yaml.safe_load(Efile.read_text())
assert hashlib.sha256(P.read_bytes()).hexdigest()==E['map']['centerline_sha256']
rows=list(csv.DictReader(P.open(newline='')))
points=[(float(r['x_m']),float(r['y_m'])) for r in rows]
assert len(points)>20
n=len(points)
ds=[math.dist(points[i],points[(i+1)%n]) for i in range(n)]
assert min(ds)>0.000001
s=[0.]
for dist in ds[:-1]:s.append(s[-1]+dist)
length=sum(ds)
headings=[];curvatures=[]
for i in range(n):
 px,py=points[(i-1)%n];cx,cy=points[i];nx,ny=points[(i+1)%n]
 # central secant heading; approximate curvature from heading changes
 headings.append(math.atan2(ny-py,nx-px))
for i in range(n):
 k_prev=math.remainder(headings[i]-headings[(i-1)%n],2*math.pi)
 k_next=math.remainder(headings[(i+1)%n]-headings[i],2*math.pi)
 curvatures.append(0.5*(k_prev/ds[(i-1)%n]+k_next/ds[i]))
# physically motivated necessary bounds only, not an action authority
max_speed=min(float(E['dynamics']['limits']['speed_abs_max_m_s']),2.*(8.-0.25-0.025))
lat=6.;brake=2.;accel=float(E['dynamics']['limits']['acceleration_max_m_s2'])
v=[min(max_speed,math.sqrt(lat/max(abs(k),1e-8))) for k in curvatures]
for _ in range(n+2):
 updated=False
 for i in range(n-1,-1,-1):
  j=(i+1)%n
  q=math.sqrt(v[j]**2+2*brake*ds[i])
  if v[i]>q+1e-12:v[i]=q;updated=True
 for i in range(n):
  j=(i+1)%n
  q=math.sqrt(v[i]**2+2*accel*ds[i])
  if v[j]>q+1e-12:v[j]=q;updated=True
 if not updated:break
else:raise RuntimeError('SPEED_ENVELOPE_NOT_CONVERGED')
fields=('sample_index','s_m','x_m','y_m','heading_rad','curvature_per_m','lateral_offset_m','target_speed_mps','target_accel_mps2','ds_to_next_m')
name=f'{T}_centerline_preliminary_racing_trajectory'
path=A/(name+'.csv')
with path.open('w',newline='') as f:
 writer=csv.writer(f,lineterminator='\n');writer.writerow(fields)
 for i,(x,y) in enumerate(points):
  j=(i+1)%n
  a=(v[j]**2-v[i]**2)/(2*ds[i])
  writer.writerow([i]+[f'{x:.12f}' for x in (s[i],x,y,headings[i],curvatures[i],0.,v[i],a,ds[i])])
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
metadata=dict(
 schema_version=1, artifact_type='RacingTrajectory',frame_id='map',closed_loop=True,
 product_valid=True,runtime_tracking_qualified=False,sample_count=n,track_id=T,
 station_period_m=length,path_length_m=length,units='SI',
 canonical_direction=E['map']['direction'],
 qualification_state='OFFLINE_CENTERLINE_PRELIMINARY_UNQUALIFIED',
 geometry_qualification_status='NOT_YET_VERIFIED',
 speed_envelope_qualification_status='KINEMATIC_NECESSARY_ONLY',
 jerk_tracking_qualification='NOT_YET_VERIFIED',
 runtime_lap_time_status='NOT_VERIFIED',
 generation_method='CENTERLINE_FROM_REGISTERED_SPIELBERG_MAP_PRELIMINARY',
 trajectory_tier='UNQUALIFIED_EXPERIMENTAL',
 acceleration_semantics='KINEMATIC_REACHABILITY_OUTGOING_SEGMENT',
 generator_code_sha256=sha(__file__),
 csv_sha256=sha(path),
 source=dict(map_image_sha256=E['map']['image_sha256'],
             source_centerline_sha256=sha(P),
             environment_sha256=sha(Efile)),
)
meta_without_id=dict(metadata)
metadata['trajectory_id']='sha256:'+hashlib.sha256(path.read_bytes()+b'\0'+json.dumps(meta_without_id,sort_keys=True,separators=(',', ':'),allow_nan=False).encode()).hexdigest()
(A/(name+'.json')).write_text(json.dumps(metadata,indent=2,sort_keys=True)+'\n')
policy=dict(frame_id='map',track_id=T,map_image_sha256=E['map']['image_sha256'],default_candidate='baseline',candidates={
 'baseline':{'csv':name+'.csv','metadata':name+'.json','source_trajectory_id':metadata['trajectory_id']},
})
(R/'src/platform/f1tenth_racing_reference/config/spielberg_nominal_reference.json').write_text(json.dumps(policy,indent=2,sort_keys=True)+'\n')
print(json.dumps({'points':n,'length_m':length,'peak_mps':max(v),'minimum_mps':min(v),'path':str(path),'trajectory_id':metadata['trajectory_id']}))
