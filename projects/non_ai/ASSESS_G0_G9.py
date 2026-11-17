#!/usr/bin/env python3
"""Provisional G0-G9 evidence assessor; never authorizes vehicle movement."""
import argparse
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/verification/qualification_gates'))
from gate_model import inspect_receipts

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--receipts',type=Path,required=True)
    parser.add_argument('--evidence-root',type=Path,required=True)
    parser.add_argument('--source-tree-sha256',required=True)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if len(args.source_tree_sha256)!=64:
        parser.error('source-tree-sha256 must be a 64-character digest')
    records=json.loads(args.receipts.read_text())
    if not isinstance(records,list):
        parser.error('receipt input must be a JSON array')
    result=inspect_receipts(records,args.evidence_root,
        source_tree_sha256=args.source_tree_sha256,run_id=args.run_id)
    payload=json.dumps(result,ensure_ascii=False,indent=2,sort_keys=True)+'\n'
    if args.output:
        args.output.write_text(payload)
    print(payload,end='')
    return 0 if result['evidence_complete'] else 1

if __name__=='__main__':
    sys.exit(main())
