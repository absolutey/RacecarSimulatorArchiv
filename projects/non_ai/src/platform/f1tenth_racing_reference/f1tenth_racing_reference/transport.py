from dataclasses import fields

from control_msgs.msg import ActiveTargetTrajectory as ActiveTargetTrajectoryMsg
from control_msgs.msg import RacingTrajectorySample

from .core import ACTIVE_SCHEMA_VERSION, validate_active_target


def to_message(active):
    validate_active_target(active)
    message = ActiveTargetTrajectoryMsg()
    message.schema_version = ACTIVE_SCHEMA_VERSION
    message.active_reference_id = active.active_reference_id
    message.active_revision = active.active_revision
    message.source_kind = active.source_kind
    message.source_trajectory_id = active.source_trajectory_id
    message.source_generation_method = active.source_generation_method
    message.source_qualification = active.source_qualification
    message.source_product_valid = active.source_product_valid
    message.source_runtime_tracking_qualified = active.source_runtime_tracking_qualified
    message.active_reference_valid = active.active_reference_valid
    message.status_reason = active.status_reason
    message.track_id = active.track_id
    message.frame_id = active.frame_id
    message.map_image_sha256 = active.map_image_sha256
    message.closed_loop = active.closed_loop
    message.sample_count = active.sample_count
    message.station_period_m = active.station_period_m
    message.path_length_m = active.path_length_m
    if active.trajectory is not None:
        for source in active.trajectory.samples:
            sample = RacingTrajectorySample()
            for field in fields(source):
                setattr(sample, field.name, getattr(source, field.name))
            message.samples.append(sample)
    return message
