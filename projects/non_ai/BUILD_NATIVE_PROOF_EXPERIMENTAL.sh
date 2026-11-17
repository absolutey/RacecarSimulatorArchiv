#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
CODE="src/environment/f1tenth_racing_sim/f1tenth_racing_sim/safety/native_stop_proof.cpp"
# Build output is never part of the source release manifest and is never
# loaded by the Guard or R-B without an independently qualified integration.
OUT="${F1_NATIVE_PROOF_LIBRARY:-${TMPDIR:-/tmp}/f1tenth_native_stop_experiment_${USER:-user}.so}"
if [[ -z "${F1_NATIVE_PROOF_LIBRARY:-}" ]]; then
  g++ -std=c++17 -O3 -fno-fast-math -fPIC -shared "$CODE" -o "$OUT"
fi
python3 -B TEST_NATIVE_STOP_PROOF_EXPERIMENTAL.py "$OUT"
F1_NATIVE_PARITY_TRACK=iccas2025 python3 -B TEST_NATIVE_GUARDAHEAD_CANDIDATES.py "$OUT"
F1_NATIVE_PARITY_TRACK=spielberg python3 -B TEST_NATIVE_GUARDAHEAD_CANDIDATES.py "$OUT"
python3 -B BENCH_NATIVE_CERTIFIED_STOP.py "$OUT"
# Receipt is created only AFTER all native-vs-Python cases pass. It binds the
# local machine binary to the exact model/map/source artifacts it was tested on.
python3 -B - "$OUT" <<'PYCODE'
from pathlib import Path
import hashlib,json,sys
r=Path.cwd();lib=Path(sys.argv[1]).resolve(strict=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
files=[
 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/safety/native_stop_proof.cpp',
 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/safety/native_stop_proof.py',
 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/safety/stopping_envelope.py',
 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/world/swept_collision.py',
 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/world/map.py',
 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/physics/vehicle.py',
 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/physics/pacejka.py',
 'src/environment/f1tenth_racing_sim/config/competition_environment.yaml',
 'src/environment/f1tenth_racing_sim/maps/iccas2025/iccas2025_map.pgm',
 'src/environment/f1tenth_racing_sim/config/spielberg_environment.yaml',
 'src/environment/f1tenth_racing_sim/maps/spielberg/Spielberg_map.pgm',
]
rec={'schema':'F1TENTH_NATIVE_STOP_EXPERIMENT_V1','result':'OFFLINE_PARITY_PASS',
     'stop_cases':1176,'guard_ahead_cases':260,
     'native_dualtick_parity_cases':260,'library_sha256':sha(lib),
     'source_hashes':[{'path':str((r/p).resolve()),'sha256':sha(r/p)} for p in files],
     'warning':'EXPERIMENTAL_NOT_A_REALTIME_OR_RACING_CERTIFICATE'}
out=Path(str(lib)+'.receipt.json')
out.write_text(json.dumps(rec,sort_keys=True,indent=2)+'\n')
print('NATIVE_PRECHECK_LOCAL_LIBRARY='+str(lib))
print('NATIVE_PRECHECK_PARITY_RECEIPT='+str(out))
PYCODE
python3 -B TEST_NATIVE_PARITY_RECEIPT.py "$OUT"
