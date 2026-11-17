"""Shared run identity helpers; no ROS imports."""
import hashlib
import json
from pathlib import Path
import yaml


def read_environment(path, *, track_id, map_sha256):
    selected=Path(path).resolve(strict=True)
    environment=yaml.safe_load(selected.read_text())
    if not isinstance(environment,dict) or not isinstance(environment.get("dynamics"),dict):
        raise ValueError("invalid environment")
    identity=environment.get("environment_identity",{})
    map_info=environment.get("map",{})
    if identity.get("active_track") != track_id or map_info.get("image_sha256") != map_sha256:
        raise ValueError("controller environment differs from selected map")
    return environment,hashlib.sha256(selected.read_bytes()).hexdigest()


def vehicle_contract_sha256(environment):
    subset={"vehicle": environment["vehicle"], "dynamics": environment["dynamics"]}
    return hashlib.sha256(json.dumps(subset,sort_keys=True,separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()
