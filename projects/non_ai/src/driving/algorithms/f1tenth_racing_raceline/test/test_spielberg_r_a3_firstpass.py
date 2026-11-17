"""R-A3 actual numeric products must remain source-pinned, non-promoted."""
from pathlib import Path
import hashlib
import json
import importlib.util
import sys
import types

# ROS native environment resolves ament itself; the offline numeric check can
# import the same artifact parser without a ROS installation.
if importlib.util.find_spec('ament_index_python') is None:
    a = types.ModuleType('ament_index_python')
    b = types.ModuleType('ament_index_python.packages')
    b.get_package_share_directory = lambda _name: str(Path(__file__).resolve().parents[1])
    a.packages = b
    sys.modules['ament_index_python'] = a
    sys.modules['ament_index_python.packages'] = b

from f1tenth_racing_raceline.trajectory import load_racing_trajectory

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'data'
CONFIG = ROOT / 'config'
POLICY = ROOT.parents[2] / 'platform/f1tenth_racing_reference/config/spielberg_nominal_reference.json'


def test_spielberg_scp_geometry_real_optimized_product():
    stem = DATA / 'spielberg_r_a3_scp_racing_trajectory'
    obj = load_racing_trajectory(stem.with_suffix('.csv'), stem.with_suffix('.json'))
    scp = obj.metadata['scp']
    assert len(obj.samples) == 864
    assert scp['accepted_steps'] >= 3
    assert scp['final_predicted_time_s'] < scp['initial_predicted_time_s']
    assert scp['validation']['wall_collision_corner_hits'] == 0
    assert obj.metadata['runtime_tracking_qualified'] is False


def test_joint_path_speed_artifact_provenance_and_bounds():
    stem = DATA / 'spielberg_r_a3_true_joint_experimental_racing_trajectory'
    obj = load_racing_trajectory(stem.with_suffix('.csv'), stem.with_suffix('.json'))
    assert obj.metadata['generation_method'] == 'R_A3_SPARSE_SCP_TRUE_JOINT_EXPERIMENTAL'
    assert obj.metadata['scp']['joint_diagnostics']['joint_qp_accepted_steps'] >= 3
    assert obj.metadata['scp']['final_predicted_time_s'] < obj.metadata['scp']['initial_predicted_time_s']
    assert obj.metadata['scp']['physical_station_dense_validation']['maximum_global_speed_excess_m_s'] == 0.0
    config_path = CONFIG / 'spielberg_r_a3_true_joint_sparse_offline.json'
    assert json.loads(config_path.read_text())['use_joint_speed_profile'] is True
    assert obj.metadata['source']['r_a3_config_sha256'] == hashlib.sha256(config_path.read_bytes()).hexdigest()
    assert obj.metadata['runtime_tracking_qualified'] is False


def test_baseline_not_automatically_replaced_with_unqualified_scp():
    policy = json.loads(POLICY.read_text())
    assert policy['default_candidate'] == 'baseline'
    for candidate, kind in [('scp', 'R_A3_SCP_JOINT_PATH_SPEED'),
                            ('scp_true_joint_experimental', 'R_A3_SPARSE_SCP_TRUE_JOINT_EXPERIMENTAL')]:
        c = policy['candidates'][candidate]
        meta = json.loads((DATA / c['metadata']).read_text())
        assert meta['generation_method'] == kind
        assert meta['trajectory_id'] == c['source_trajectory_id']
        assert meta['runtime_tracking_qualified'] is False
