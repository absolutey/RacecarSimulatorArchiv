"""Actual NumPy MLP inference. No weights -> no action. No actuator publisher."""
from __future__ import annotations
import hashlib
from pathlib import Path
import numpy as np
from .contracts import ContractError, DriverProposal

class MLPDriver:
    def __init__(self, weights: list[tuple[np.ndarray, np.ndarray]], identity: str):
        if not weights or len(identity) != 64:
            raise ContractError('UNTRAINED_POLICY_REJECTED')
        self.weights=weights
        self.identity=identity
        prev=None
        for w,b in weights:
            if w.ndim!=2 or b.ndim!=1 or w.shape[1]!=b.shape[0] or (prev is not None and prev!=w.shape[0]):
                raise ContractError('POLICY_SHAPE_INVALID')
            if not np.isfinite(w).all() or not np.isfinite(b).all():
                raise ContractError('POLICY_NONFINITE')
            prev=w.shape[1]
        if prev!=2:
            raise ContractError('POLICY_OUTPUT_MUST_BE_TWO')
    @classmethod
    def from_npz(cls,path: str | Path) -> 'MLPDriver':
        p=Path(path)
        if not p.is_file() or p.is_symlink():
            raise ContractError('POLICY_CHECKPOINT_MISSING')
        raw=p.read_bytes()
        data=np.load(p,allow_pickle=False)
        weights=[]
        try:
            i=0
            while f'W{i}' in data.files and f'b{i}' in data.files:
                weights.append((np.asarray(data[f'W{i}'],dtype=float),np.asarray(data[f'b{i}'],dtype=float)))
                i+=1
            if len(data.files)!=2*i or not weights:
                raise ContractError('CHECKPOINT_SCHEMA_INVALID')
        finally:
            data.close()
        return cls(weights,hashlib.sha256(raw).hexdigest())
    def propose(self,features: np.ndarray,stamp_s: float)->DriverProposal:
        x=np.asarray(features,dtype=float)
        if x.ndim != 1 or len(x)!=self.weights[0][0].shape[0] or not np.isfinite(x).all():
            raise ContractError('OBSERVATION_INVALID')
        for j,(w,b) in enumerate(self.weights):
            x=x@w+b
            x=np.tanh(x) if j<len(self.weights)-1 else x
        if not np.isfinite(x).all():
            raise ContractError('OUTPUT_INVALID')
        return DriverProposal(float(x[0]),float(x[1]),float(stamp_s),self.identity)
