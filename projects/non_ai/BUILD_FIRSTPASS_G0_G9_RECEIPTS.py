#!/usr/bin/env python3
"""Assemble only EXISTING independently produced G0-G9 evidence.

No synthetic metrics, no unsigned official PASS, no vehicle command, and no
promotion. Evidence filenames and hashes are bound to one run/source/scope.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/verification/qualification_gates'))
from gate_model import expected_gate_keys,inspect_receipts


def assemble(directory, *, run_id, source_sha256):
    directory=Path(directory).resolve(strict=True)
    if (not isinstance(run_id,str) or not run_id or
            type(source_sha256)!=str or len(source_sha256)!=64 or
            not all(c in '0123456789abcdef' for c in source_sha256)):
        raise ValueError('INVALID_GATE_RUN_SOURCE_PROVENANCE')
    receipts=[]
    for gate,scope in expected_gate_keys():
        name=f'{gate}_{scope}.json'
        path=directory/name
        if path.is_symlink():
            raise ValueError('GATE_EVIDENCE_SYMLINK_FORBIDDEN:'+name)
        if not path.is_file():
            # Missing real evidence is faithfully reported as MISSING by the
            # assessor; never invent a zero, PASS, or placeholder file.
            continue
        metrics=json.loads(path.read_text())
        if (type(metrics)!=dict or metrics.get('schema')!='RAB_GATE_EVIDENCE_V2' or
                metrics.get('run_id')!=run_id or
                metrics.get('source_tree_sha256')!=source_sha256 or
                metrics.get('gate')!=gate or metrics.get('scope')!=scope):
            raise ValueError('GATE_EVIDENCE_IDENTITY_INVALID:'+name)
        receipts.append({
            'schema':'RAB_G0G9_PROVISIONAL_V2','gate':gate,'scope':scope,
            'run_id':run_id,'source_tree_sha256':source_sha256,
            'status':'PASS','metrics':metrics.get('metrics'),
            'evidence':[{'path':name,
                         'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}],
        })
    return receipts


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--evidence-root',required=True,type=Path)
    ap.add_argument('--run-id',required=True)
    ap.add_argument('--source-tree-sha256',required=True)
    ap.add_argument('--output',type=Path)
    args=ap.parse_args()
    receipts=assemble(args.evidence_root,run_id=args.run_id,
        source_sha256=args.source_tree_sha256)
    decision=inspect_receipts(receipts,args.evidence_root,
        run_id=args.run_id,source_tree_sha256=args.source_tree_sha256)
    out={'schema':'RAB_FIRSTPASS_G0_G9_REVIEW_V1',
         'receipts':receipts,'assessment':decision,
         'product_promoted':False,
         'qualification':'NOT_AUTHORIZED_FOR_PRODUCT_PROMOTION'}
    payload=json.dumps(out,indent=2,sort_keys=True,ensure_ascii=False)+'\n'
    if args.output:
        if args.output.resolve().is_relative_to(args.evidence_root.resolve()):
            raise ValueError('OUTPUT_INSIDE_EVIDENCE_ROOT_FORBIDDEN')
        args.output.write_text(payload)
    print(payload,end='')
    return 0 if decision['evidence_complete'] else 2

if __name__=='__main__':raise SystemExit(main())
