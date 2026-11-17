"""Fail-closed stage-local asset and content fingerprint contract.

ROS2 colcon --symlink-install intentionally symlinks package data files into
install/share.  Permit only the declared asset path whose resolved regular
file remains INSIDE the same isolated workspace (src/install/build) and
whose bytes match the selected environment's SHA256.  Never follow arbitrary
external symlinks, even if their current contents match.
"""
import hashlib
import os
from pathlib import Path
import re
import stat


def _check_asset(config, filepath, label, share):
    key = 'image_file' if label == 'PGM' else 'centerline_file'
    sha_key = 'image_sha256' if label == 'PGM' else 'centerline_sha256'
    relative = Path(config['map'][key])
    expected_sha = config['map'][sha_key]
    if (relative.is_absolute() or '..' in relative.parts or
            len(relative.parts) != 3 or relative.parts[0] != 'maps' or
            not re.fullmatch('[0-9a-f]{64}', expected_sha)):
        raise ValueError(label + ' invalid asset identity contract')
    share = Path(os.path.abspath(share))
    if tuple(share.parts[-4:]) != ('install','f1tenth_racing_sim','share','f1tenth_racing_sim'):
        raise ValueError(label + ' nonisolated install package share refused')
    stage = share.parents[3]
    installed = share / relative
    sourced = stage / 'src/environment/f1tenth_racing_sim' / relative
    built = stage / 'build/f1tenth_racing_sim' / relative
    supplied = Path(os.path.abspath(filepath))
    if supplied not in (installed, sourced):
        raise ValueError(label + ' path outside declared selected-stage asset')
    try:
        resolved = supplied.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise ValueError(label + ' missing or dangling asset') from exc
    # Resolve using no user-controlled set of parents: exact canonical targets.
    # The stage's src/build copies may themselves NOT be external symlinks.
    if resolved not in (installed, sourced, built):
        raise ValueError(label + ' symlink escapes the selected isolated stage')
    if not resolved.is_file() or not stat.S_ISREG(resolved.stat().st_mode):
        raise ValueError(label + ' target is not a regular file')
    h = hashlib.sha256()
    with resolved.open('rb') as handle:
        for data in iter(lambda: handle.read(1 << 20), b''):
            h.update(data)
    if h.hexdigest() != expected_sha:
        raise ValueError(label + ' fingerprint does not match selected environment')
    return str(resolved)


def require_expected_source_fingerprints(config, map_file, centerline_file):
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('f1tenth_racing_sim')
    _check_asset(config, map_file, 'PGM', share)
    _check_asset(config, centerline_file, 'centerline', share)
