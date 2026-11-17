"""New 2026-10-11 postoperative negative tests; no safety assertions removed."""
import base64
import hashlib
import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from f1tenth_racing_control.execution_contract import EVIDENCE_KINDS, verify_run_contract
from f1tenth_racing_control.reference_admission import authorize_active_reference
from f1tenth_racing_control.environment_identity import read_environment, vehicle_contract_sha256


def ref(source="NOMINAL", id="sha256:approved", track="iccas2025", mapsha="a" * 64):
    return SimpleNamespace(source_kind=source, source_trajectory_id=id,
                           track_id=track, map_image_sha256=mapsha)


def accept(message, speed, trusted=False):
    return authorize_active_reference(message, requested_cap_mps=speed,
        qualified_trajectory_id="sha256:approved", expected_track_id="iccas2025",
        expected_map_sha256="a" * 64, trusted_high_speed=trusted)


def test_legacy_nominal_low_speed():
    assert accept(ref(), 5.0)


def test_legacy_overtake_distinct_plan_low_speed():
    assert accept(ref("OVERTAKE_OVERRIDE", "plan:1"), 5.0)


@pytest.mark.parametrize("message,speed,trusted", [
    (ref(id="sha256:other"),7.0,True),
    (ref(),7.0,False),
    (ref("OVERTAKE_OVERRIDE","plan:1"),7.0,True),
    (ref(mapsha="0"*64),5.0,False),
    (ref(track="Spielberg"),5.0,False),
    (ref(),float("nan"),True),
    (ref(),float("inf"),True),
    (ref(),0,True),
])
def test_bad_reference_or_cap_rejected(message,speed,trusted):
    with pytest.raises(ValueError):accept(message,speed,trusted)


def test_qualified_nominal_high_speed_identity():
    assert accept(ref(),7.0,True)


def qualified_fixture(tmp_path):
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    from cryptography.hazmat.primitives.serialization import Encoding,PublicFormat
    private=Ed25519PrivateKey.generate()
    public=private.public_key().public_bytes(Encoding.PEM,PublicFormat.SubjectPublicKeyInfo)
    key=tmp_path/"reviewed_key.pem";key.write_bytes(public)
    issued=time.time_ns()-60_000_000_000
    payload={"schema_version":2,"run_id":"safe-run-000001",
             "issued_at_unix_ns":issued,"expires_at_unix_ns":issued+86_400_000_000_000,
             "track_id":"iccas2025","map_image_sha256":"a"*64,
             "trajectory_id":"sha256:approved","controller_kind":"mpc",
             "qualified_speed_cap_mps":8.5,
             "environment_sha256":"e"*64,"vehicle_contract_sha256":"v"*64}
    artifacts={}
    for kind in EVIDENCE_KINDS:
        f=tmp_path/(kind+".json")
        f.write_text(json.dumps({"independent_measurement":kind}))
        artifacts[kind]={"path":str(f),"sha256":hashlib.sha256(f.read_bytes()).hexdigest(),
             "verdict":"PASS","producer":"external_test_fixture","check_id":kind,
             **{k:payload[k] for k in ("run_id","track_id","map_image_sha256",
                                      "trajectory_id","controller_kind")}}
    payload["evidence"]=artifacts
    receipt=tmp_path/"receipt.json"

    def sign(valid=True):
        msg=json.dumps(payload,sort_keys=True,separators=(",", ":"),
                       ensure_ascii=True,allow_nan=False).encode()
        sig=private.sign(msg) if valid else b"z"*64
        receipt.write_text(json.dumps({"payload":payload,
            "signature_ed25519_base64":base64.b64encode(sig).decode()}))

    def verify():
        return verify_run_contract(receipt,track_id="iccas2025",map_sha="a"*64,
            trajectory_id="sha256:approved",controller_kind="mpc",proposed_cap=8.0,
            trusted_public_key_file=key,
            trusted_public_key_sha256=hashlib.sha256(public).hexdigest(),
            environment_sha256="e"*64,vehicle_contract_sha256="v"*64)
    sign()
    return payload,sign,verify


def test_untrusted_legacy_pass_receipt_rejected(tmp_path):
    p=tmp_path/"old.json"
    p.write_text(json.dumps({"evidence":{k:"PASS" for k in EVIDENCE_KINDS}}))
    with pytest.raises(ValueError):
        verify_run_contract(p,track_id="iccas2025",map_sha="a"*64,
          trajectory_id="sha256:approved",controller_kind="mpc",proposed_cap=9.0)


def test_signed_receipt_all_artifacts_valid(tmp_path):
    _,_,verify=qualified_fixture(tmp_path)
    assert verify()


def test_signature_tampered_rejected(tmp_path):
    payload,sign,verify=qualified_fixture(tmp_path)
    sign(valid=False)
    with pytest.raises(ValueError):verify()


def test_missing_evidence_artifact_rejected(tmp_path):
    payload,_,verify=qualified_fixture(tmp_path)
    Path(payload["evidence"][EVIDENCE_KINDS[0]]["path"]).unlink()
    with pytest.raises((ValueError,FileNotFoundError)):verify()


def test_tampered_evidence_artifact_rejected(tmp_path):
    payload,_,verify=qualified_fixture(tmp_path)
    Path(payload["evidence"][EVIDENCE_KINDS[0]]["path"]).write_text("forged")
    with pytest.raises(ValueError):verify()


def test_signed_wrong_trajectory_rejected(tmp_path):
    payload,sign,verify=qualified_fixture(tmp_path)
    payload["trajectory_id"]="sha256:wrong"
    sign()
    with pytest.raises(ValueError,match="identity mismatch"):verify()


def test_environment_identity_and_vehicle_hash(tmp_path):
    data={"environment_identity":{"active_track":"iccas2025"},
          "map":{"image_sha256":"a"*64},
          "dynamics":{"limits":{"speed":5}},"vehicle":{"mass":1}}
    p=tmp_path/"env.yaml";p.write_text(yaml.safe_dump(data))
    assert read_environment(p,track_id="iccas2025",map_sha256="a"*64)[0]==data
    assert len(vehicle_contract_sha256(data))==64
    with pytest.raises(ValueError):read_environment(p,track_id="Spielberg",map_sha256="a"*64)


def test_expired_signed_receipt_rejected(tmp_path):
    payload,sign,verify=qualified_fixture(tmp_path)
    payload["expires_at_unix_ns"]=payload["issued_at_unix_ns"]+1
    sign()
    with pytest.raises(ValueError,match="expired"):
        verify()
