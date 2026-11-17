"""Fail-closed independently signed high-speed authority.

A PASS label is not evidence. This verifier checks a detached Ed25519
signature from a separately configured trusted public key, seven real
independent check artifacts, their file hashes and run/trajectory identity.
This is a gate for pre-existing external qualification, NOT an issuer.
No private key or self-signing shortcut is provided by this project.
"""
import base64
import hashlib
import json
import math
import time
from pathlib import Path

EVIDENCE_KINDS = (
    "swept_footprint", "guard_controller_kill", "guard_command_loss",
    "stability_cte", "jerk_and_braking", "collision_free_rosbag",
    "actual_controller_adoption",
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical(data):
    return json.dumps(data, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True, allow_nan=False).encode("utf8")


def _strict_hex(value):
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def verify_run_contract(path, *, track_id, map_sha, trajectory_id,
                        controller_kind, proposed_cap,
                        trusted_public_key_file="", trusted_public_key_sha256="",
                        environment_sha256="", vehicle_contract_sha256=""):
    """Verify outside-issued proof; raise ValueError on every uncertainty.

    Signed JSON format: {"payload":{...},"signature_ed25519_base64":"..."}.
    The public key is independent of the receipt; its raw file SHA256 must be
    pinned in the launch settings. Artifacts cannot be bare PASS strings.
    """
    if not trusted_public_key_file or not _strict_hex(trusted_public_key_sha256):
        raise ValueError("trusted high-speed issuer key and pinned SHA required")
    key_path=Path(trusted_public_key_file).resolve(strict=True)
    if sha(key_path) != trusted_public_key_sha256:
        raise ValueError("trusted issuer key SHA mismatch")
    envelope=json.loads(Path(path).read_text())
    if not isinstance(envelope,dict) or set(envelope)!={"payload", "signature_ed25519_base64"}:
        raise ValueError("signed receipt envelope required")
    data=envelope["payload"]
    if not isinstance(data,dict) or data.get("schema_version") != 2:
        raise ValueError("unsupported signed receipt schema")
    try:
        from cryptography.hazmat.primitives.serialization import load_pem_public_key
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
        public_key=load_pem_public_key(key_path.read_bytes())
        if not isinstance(public_key, Ed25519PublicKey):
            raise ValueError("issuer key must be Ed25519")
        signature=base64.b64decode(envelope["signature_ed25519_base64"], validate=True)
        if len(signature)!=64: raise ValueError("invalid Ed25519 signature size")
        public_key.verify(signature,_canonical(data))
    except Exception as exc:
        # No crypto module, forged signature, malformed base64 or wrong key: fail closed.
        raise ValueError("signed high-speed authority verification failed") from exc
    expected={"track_id":track_id,"map_image_sha256":map_sha,
              "trajectory_id":trajectory_id,"controller_kind":controller_kind}
    if not environment_sha256 or not vehicle_contract_sha256:
        raise ValueError("environment and vehicle identity required for high speed")
    expected["environment_sha256"]=environment_sha256
    expected["vehicle_contract_sha256"]=vehicle_contract_sha256
    for key,value in expected.items():
        if not value or data.get(key)!=value: raise ValueError("receipt identity mismatch: "+key)
    try:
        cap=float(proposed_cap); qualified=float(data["qualified_speed_cap_mps"])
    except (ValueError,TypeError,KeyError) as exc:
        raise ValueError("invalid qualified cap") from exc
    if not (math.isfinite(cap) and math.isfinite(qualified) and 0<cap<=qualified):
        raise ValueError("requested speed exceeds qualified receipt")
    # A previously valid run is not an unlimited future high-speed permit.
    now_ns=time.time_ns()
    issued=data.get("issued_at_unix_ns")
    expires=data.get("expires_at_unix_ns")
    if (not isinstance(issued,int) or isinstance(issued,bool) or
            not isinstance(expires,int) or isinstance(expires,bool) or
            issued<=0 or expires<=issued or expires-issued>90*86400*10**9 or
            not issued<=now_ns<=expires):
        raise ValueError("signed high-speed authority is missing, future dated, or expired")
    run_id=data.get("run_id")
    if not isinstance(run_id,str) or len(run_id)<8:
        raise ValueError("qualified run id missing")
    evidence=data.get("evidence")
    if not isinstance(evidence,dict) or set(evidence)!=set(EVIDENCE_KINDS):
        raise ValueError("seven independent evidence artifacts required")
    seen=set()
    for name in EVIDENCE_KINDS:
        item=evidence[name]
        if not isinstance(item,dict) or item.get("verdict")!="PASS" or item.get("check_id")!=name:
            raise ValueError("missing actual pass verdict: "+name)
        for bound in ("run_id", "track_id", "map_image_sha256", "trajectory_id", "controller_kind"):
            if item.get(bound)!=data.get(bound):
                raise ValueError("evidence identity mismatch: "+name+":"+bound)
        if not item.get("producer") or not _strict_hex(item.get("sha256")):
            raise ValueError("evidence producer / hash missing")
        actual_path=item.get("path")
        if not isinstance(actual_path,str) or not Path(actual_path).is_absolute():
            raise ValueError("absolute evidence path required")
        actual_file=Path(actual_path).resolve(strict=True)
        if not actual_file.is_file() or actual_file in seen or sha(actual_file)!=item["sha256"]:
            raise ValueError("evidence artifact missing / tampered / reused")
        seen.add(actual_file)
        # Evidence contents are separately reviewed by a trusted issuer.
        # A matching SHA on its own does not prove that an experiment was run.
    return True
