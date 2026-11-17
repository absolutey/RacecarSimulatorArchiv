#!/usr/bin/env python3
"""Read-only Solo evidence assessor. FAIL != unsafe; no qualification inferred.

Does not replace independent wall/vehicle collision geometry certification.
Does not issue actuator commands or change workspace source.
"""
import argparse
import csv
import io
import json
import math
from pathlib import Path
from zipfile import ZipFile


def summarize(rows, summary):
    speeds=[]
    safe_holds=0
    for row in rows:
        try:
            speed=float(row['blue_actual_speed_mps'])
            if math.isfinite(speed):
                speeds.append(speed)
        except (ValueError, KeyError, TypeError):
            pass
    physical=int(summary.get('physical_passes',0))
    wall=int(summary.get('wall_collisions',-1))
    car=bool(summary.get('car_collision',True))
    valid=bool(summary.get('valid',False))
    verdict = ('SOLO_CLEAN_LAP_EVIDENCE_PRESENT' if
               physical >= 1 and wall == 0 and not car and valid and speeds
               else 'SOLO_NOT_QUALIFIED')
    return {
        'verdict': verdict,
        'scope':'recorder_summary_sanity_only_not_full_physical_oracle',
        'physical_passes':physical,
        'wall_collisions':wall,
        'car_collision':car,
        'race_valid':valid,
        'rows':len(rows),
        'max_observed_speed_mps':max(speeds) if speeds else None,
        'note':'Must independently check footprint, lap crossing, Guard Applied and command continuity before PASS',
    }


def audit_zip(path):
    with ZipFile(path) as z:
        if z.testzip() is not None:
            raise ValueError('ZIP_CRC_FAIL')
        summary_files=[n for n in z.namelist() if n.endswith('/summary.json') and '/runs/' in n]
        csv_files=[n for n in z.namelist() if n.endswith('/telemetry.csv') and '/runs/' in n]
        if len(summary_files)!=1 or len(csv_files)!=1:
            raise ValueError('MISSING_OR_AMBIGUOUS_RECORDER_EVIDENCE')
        summary=json.loads(z.read(summary_files[0]))
        rows=list(csv.DictReader(io.StringIO(z.read(csv_files[0]).decode('utf-8-sig'))))
        return summarize(rows,summary)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--evidence-zip',required=True)
    args=p.parse_args()
    result=audit_zip(Path(args.evidence_zip))
    print(json.dumps(result,indent=2,ensure_ascii=False))
    raise SystemExit(0 if result['verdict']=='SOLO_CLEAN_LAP_EVIDENCE_PRESENT' else 2)


if __name__=='__main__':
    main()
