#!/usr/bin/env python3
"""Read-only runtime preflight after sourcing this stage's ROS overlay."""
from pathlib import Path
import json,sys
from ament_index_python.packages import get_package_share_directory
import f1tenth_racing_reference.contract_guard as contract_guard
from f1tenth_racing_reference.contract_guard import load_nominal_policy,_trusted_stage_provenance

ROOT=Path(__file__).resolve().parent
REF=Path(get_package_share_directory('f1tenth_racing_reference'))
LINE=Path(get_package_share_directory('f1tenth_racing_raceline'))
if not (str(REF).startswith(str(ROOT/'install')+'/') and
        str(LINE).startswith(str(ROOT/'install')+'/')):
    raise SystemExit('PREFLIGHT_PACKAGE_STAGE_MISMATCH')
POLICY=REF/'config/nominal_reference.json'
DATADIR=LINE/'data'
policy=json.loads(POLICY.read_text())
assert policy.get('track_id')=='iccas2025','unexpected map for ICCAS Solo'
assert _trusted_stage_provenance(ROOT, contract_guard.__file__), 'runtime code provenance mismatch'
loaded=load_nominal_policy(POLICY,track_id=policy['track_id'],
                           map_sha256=policy['map_image_sha256'],data_dir=DATADIR)
assert loaded['candidates']['baseline']['source_trajectory_id']
print('PREFLIGHT_S0R_ARTIFACT_AUTHORITY_PASS',
      json.dumps({'stage':str(ROOT),'map_sha256':loaded['map_image_sha256'],
                  'candidate':'baseline','source_trajectory_id':loaded['candidates']['baseline']['source_trajectory_id']},sort_keys=True))
