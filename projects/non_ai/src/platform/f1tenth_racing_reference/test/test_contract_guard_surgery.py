from types import SimpleNamespace as NS
import pytest
from f1tenth_racing_reference.contract_guard import validate_reference_message


def _ref(track='iccas2025',sha='abc'):
    return NS(active_reference_valid=True,track_id=track,map_image_sha256=sha,frame_id='map',
              active_reference_id='sha256:ref',source_trajectory_id='sha256:trajectory',
              active_revision=1,source_runtime_tracking_qualified=False,sample_count=3,
              samples=[NS(s_m=float(i),x_m=float(i),y_m=0.,curvature_per_m=0.,
                          target_speed_mps=1.,target_accel_mps2=0.,ds_to_next_m=1.)
                       for i in range(3)])


def test_nominal_map_identity_cannot_be_swapped():
    assert validate_reference_message(_ref(),expected_track_id='iccas2025',expected_map_sha256='abc')
    with pytest.raises(ValueError):
        validate_reference_message(_ref(track='spielberg'),expected_track_id='iccas2025',expected_map_sha256='abc')
    with pytest.raises(ValueError):
        validate_reference_message(_ref(sha='wrong'),expected_track_id='iccas2025',expected_map_sha256='abc')


def test_invalid_geometry_or_forged_runtime_label_rejected():
    r=_ref();r.samples[0].target_speed_mps=float('nan')
    with pytest.raises(ValueError): validate_reference_message(r,expected_track_id='iccas2025',expected_map_sha256='abc')
    r=_ref();r.source_runtime_tracking_qualified=True
    with pytest.raises(ValueError): validate_reference_message(r,expected_track_id='iccas2025',expected_map_sha256='abc')
