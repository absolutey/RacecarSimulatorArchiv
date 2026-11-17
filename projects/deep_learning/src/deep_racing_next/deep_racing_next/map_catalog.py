"""Safe map input resolution and provenance, not a full route extractor."""
from __future__ import annotations
import ast
import hashlib
import json
from pathlib import Path
from .contracts import MapIdentity, ContractError

def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def _safe_file(folder: Path, relative: str) -> Path:
    rel = Path(relative)
    if rel.is_absolute() or '..' in rel.parts:
        raise ContractError('UNSAFE_MAP_REFERENCE')
    p = folder / rel
    if p.is_symlink() or not p.is_file() or not p.resolve().is_relative_to(folder.resolve()):
        raise ContractError('MAP_FILE_MISSING_OR_UNSAFE')
    return p

def _read_yaml(path: Path) -> dict:
    # ROS map YAML is a simple mapping. Prefer safe parser, refuse unsupported files.
    try:
        import yaml
    except ImportError as e:
        raise ContractError('PYYAML_REQUIRED') from e
    d = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(d, dict):
        raise ContractError('BAD_MAP_YAML')
    return d

def inspect_map(folder: str | Path) -> dict:
    folder = Path(folder)
    if not folder.is_dir() or folder.is_symlink():
        raise ContractError('MAP_FOLDER_MISSING_OR_LINK')
    folder = folder.resolve()
    manifests = [p for p in folder.iterdir() if p.is_file() and not p.is_symlink() and p.suffix in ('.yaml', '.yml')]
    if len(manifests) != 1:
        raise ContractError('AMBIGUOUS_MAP_YAML')
    yml = manifests[0]
    m = _read_yaml(yml)
    required = {'image', 'resolution', 'origin'}
    if not required.issubset(m):
        raise ContractError('MAP_METADATA_MISSING')
    img = _safe_file(folder, str(m['image']))
    if img.suffix.lower() not in ('.png','.pgm'):
        raise ContractError('MAP_IMAGE_TYPE_UNSUPPORTED')
    origin = m['origin']
    if not isinstance(origin, list) or len(origin) != 3:
        raise ContractError('MAP_ORIGIN_INVALID')
    identity = MapIdentity(folder.name, sha256(img), sha256(yml), float(m['resolution']), tuple(float(x) for x in origin))
    centerlines = [p for p in folder.iterdir() if p.is_file() and not p.is_symlink() and 'centerline' in p.name.lower() and p.suffix.lower()=='.csv']
    return {'schema':'dr-v2.1','name':identity.name,'image':img.name,'image_sha256':identity.image_sha256,
        'yaml':yml.name,'yaml_sha256':identity.yaml_sha256,'resolution_m':identity.resolution_m,
        'origin_xyyaw':list(identity.origin_xyyaw),'centerline':centerlines[0].name if len(centerlines)==1 else None,
        'centerline_state': 'PRESENT' if len(centerlines)==1 else ('MISSING_NEEDS_EXTRACTION' if not centerlines else 'AMBIGUOUS_HOLD'),
        'qualification':'INPUT_ONLY_NOT_DRIVABLE'}

def catalog_maps(root: str | Path) -> dict:
    root = Path(root)
    if not root.is_dir():
        raise ContractError('MAP_ROOT_MISSING')
    out = {}
    for d in sorted(root.iterdir()):
        if d.is_dir() and not d.is_symlink():
            try: out[d.name] = inspect_map(d)
            except (ValueError, OSError) as e:
                out[d.name]={'name':d.name,'qualification':'HOLD','reason':str(e)}
    return {'maps':out,'map_count':len(out),'qualified_for_actuation':False}
