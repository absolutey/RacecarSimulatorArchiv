"""Fail-closed nominal execution and exact same-stage R-A artifact identity.

colcon --symlink-install links installed data_files to source files. Only
allow this specific installed raceline data -> source raceline data mapping
inside the selected isolated stage. Preserve ordinary direct-file usage and
cryptographic CSV/metadata integrity; do not allow arbitrary link targets.
"""
import hashlib
import json
import math
import os
from pathlib import Path


def content_sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def _permitted_stage_location(stage, home):
    """Recognize official source roots even under isolated Downloads staging.

    ROS verification unpacks a F1TENTH source stage into a temporary directory
    *inside* Downloads. Requiring stage.parent == Downloads rejects that
    correctly isolated workspace. Do not trust arbitrary roots or source
    names: a stage must be the expected named source directory under Downloads
    (at any nesting depth), or the explicitly supported canonical workspace.
    File-system and executing-module identity checks follow separately.
    """
    stage = Path(stage)
    home = Path(home)
    if stage in (home / 'F1TENTH_NEXT_WORKSPACE',
                 home / 'f1tenth_racing'):
        return True
    if not stage.name.startswith('F1TENTH_'):
        return False
    try:
        relative = stage.relative_to(home / 'Downloads')
    except ValueError:
        return False
    return bool(relative.parts)


def _trusted_stage_provenance(stage, module_file):
    """Accept only same-stage installed artifacts and the *executing* module.

    Neither ROS parameters nor an artifact registry may nominate a foreign
    stage as trusted. Paths must be canonical (no symlinked ancestors), and
    the executing reference module must resolve to this stage's source file.
    Contents and reference/map SHA are checked independently by the loader.
    """
    try:
        stage = Path(stage)
        module = Path(module_file).resolve(strict=True)
        permitted_location = _permitted_stage_location(stage, Path.home())
        expected = stage / ('src/platform/f1tenth_racing_reference/'
                            'f1tenth_racing_reference/contract_guard.py')
        return (permitted_location and not stage.is_symlink() and
                stage.is_dir() and stage.resolve(strict=True) == stage and
                expected.is_file() and not expected.is_symlink() and
                expected.resolve(strict=True) == expected and module == expected)
    except (ValueError, OSError, RuntimeError):
        return False


def _registered_artifact(base, entry, field):
    name = entry.get(field)
    if not isinstance(name, str) or not name or '/' in name or chr(92) in name:
        raise ValueError('invalid candidate artifact basename: ' + field)
    relative = Path(name)
    if relative.name in ('.','..') or relative.is_absolute() or len(relative.parts) != 1:
        raise ValueError('candidate artifact path traversal: ' + field)

    # Never silently resolve an installed directory symlink to an unrelated
    # workspace. The expected parent path is explicit, not user-supplied.
    suffix = ('install','f1tenth_racing_raceline','share','f1tenth_racing_raceline','data')
    installed = tuple(base.parts[-5:]) == suffix
    if installed:
        stage = base.parents[4]
        source_base = stage / 'src/driving/algorithms/f1tenth_racing_raceline/data'
        # Provenance is obtained from the *executing reference package code*,
        # not from the artifact data_dir and not from a mutable allowlist of
        # ZIP-version names. Same-stage source and installed data are mandatory.
        # An arbitrary foreign registry, cross-overlay import, and a symlinked
        # stage fail closed even if a candidate directory has a familiar name.
        if (not _trusted_stage_provenance(stage, Path(__file__)) or
                stage.is_symlink() or not stage.is_dir() or not source_base.is_dir() or
                source_base.resolve(strict=True) != source_base or
                base.resolve(strict=True) not in (base, source_base)):
            raise ValueError('candidate registry not inside executing verified stage')
        expected_targets = (base / relative, source_base / relative)
    else:
        # Existing offline direct-file functionality: files must remain inside
        # the exact caller-supplied directory, without a directory symlink.
        if base.resolve(strict=True) != base:
            raise ValueError('candidate registry directory is symlinked')
        expected_targets = (base / relative,)
    installed_path = base / relative
    try:
        resolved = installed_path.resolve(strict=True)
    except (OSError, RuntimeError) as ex:
        raise ValueError('candidate artifact missing/dangling: ' + field) from ex
    if resolved not in expected_targets or not resolved.is_file():
        raise ValueError('candidate artifact escaped registry: ' + field)
    if not installed_path.is_file():
        raise ValueError('candidate artifact not regular: ' + field)
    return installed_path


def load_nominal_policy(path, *, track_id, map_sha256, data_dir):
    path = Path(path)
    if not path.is_file():
        raise ValueError('nominal policy does not exist')
    policy = json.loads(path.read_text())
    if (not track_id or not map_sha256 or
            policy.get('track_id') != track_id or
            policy.get('map_image_sha256') != map_sha256):
        raise ValueError('nominal policy track or map identity mismatch')
    if policy.get('frame_id') != 'map':
        raise ValueError('unsupported reference frame')
    candidates = policy.get('candidates')
    if not isinstance(candidates, dict) or not candidates:
        raise ValueError('empty candidate registry')
    base = Path(os.path.abspath(data_dir))
    if not base.is_dir():
        raise ValueError('candidate registry data directory missing')
    for key, entry in candidates.items():
        if not key or not isinstance(entry, dict):
            raise ValueError('invalid candidate entry')
        for field in ('csv','metadata','source_trajectory_id'):
            if not entry.get(field):
                raise ValueError('missing candidate field ' + field)
        csv = _registered_artifact(base, entry, 'csv')
        metadata = _registered_artifact(base, entry, 'metadata')
        try:
            md = json.loads(metadata.read_text())
        except (OSError, UnicodeError, ValueError) as ex:
            raise ValueError('candidate metadata unreadable: ' + str(key)) from ex
        if (md.get('trajectory_id') != entry['source_trajectory_id'] or
                md.get('csv_sha256') != content_sha(csv)):
            raise ValueError('candidate CSV or trajectory fingerprint mismatch: ' + str(key))
    return policy


def validate_reference_message(msg, *, expected_track_id, expected_map_sha256):
    if not msg.active_reference_valid: raise ValueError('inactive reference')
    if msg.track_id != expected_track_id or msg.map_image_sha256 != expected_map_sha256:
        raise ValueError('controller map/track identity mismatch')
    if msg.frame_id != 'map': raise ValueError('controller reference frame mismatch')
    if not msg.active_reference_id or not msg.source_trajectory_id or int(msg.active_revision)<1:
        raise ValueError('reference identity/revision invalid')
    if msg.sample_count != len(msg.samples) or msg.sample_count<3:
        raise ValueError('reference sample count invalid')
    if msg.source_runtime_tracking_qualified is not False:
        raise ValueError('unexpected runtime qualification relabel')
    for sample in msg.samples:
        values=(sample.s_m,sample.x_m,sample.y_m,sample.curvature_per_m,
                sample.target_speed_mps,sample.target_accel_mps2,sample.ds_to_next_m)
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError('nonfinite reference')
        if sample.target_speed_mps<0 or sample.ds_to_next_m<=0:
            raise ValueError('invalid speed/geometry')
    return True
