import bisect
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from f1tenth_racing_geometry import (
    ClosedTrackGeometry,
    interpolate_periodic_angle,
    interpolate_periodic_scalar,
    interpolate_periodic_vector,
)


SCHEMA_VERSION = 1
GENERATION_METHOD = "R_A0_MIN_CURVATURE_PLUS_R_A1_FOUNDATION_SPEED_ENVELOPE"
SAMPLE_FIELDS = (
    "sample_index",
    "s_m",
    "x_m",
    "y_m",
    "heading_rad",
    "curvature_per_m",
    "lateral_offset_m",
    "target_speed_mps",
    "target_accel_mps2",
    "ds_to_next_m",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _format(value):
    return f"{float(value):.12f}"


def _content_id(csv_path, metadata):
    canonical_metadata = json.dumps(metadata, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    digest = hashlib.sha256(Path(csv_path).read_bytes() + b"\0" + canonical_metadata).hexdigest()
    return f"sha256:{digest}"


@dataclass(frozen=True)
class TrajectorySample:
    sample_index: int
    s_m: float
    x_m: float
    y_m: float
    heading_rad: float
    curvature_per_m: float
    lateral_offset_m: float
    target_speed_mps: float
    target_accel_mps2: float
    ds_to_next_m: float


@dataclass(frozen=True)
class RacingTrajectory:
    samples: tuple
    metadata: dict

    @property
    def station_period_m(self):
        return float(self.metadata["station_period_m"])

    @property
    def trajectory_id(self):
        return self.metadata["trajectory_id"]

    def sample_at_s(self, query_s):
        query_s = float(query_s)
        if not math.isfinite(query_s):
            raise ValueError("query s must be finite")
        stations = tuple(sample.s_m for sample in self.samples)
        wrapped = query_s % self.station_period_m
        vectors = tuple((sample.x_m, sample.y_m) for sample in self.samples)
        x_m, y_m = interpolate_periodic_vector(stations, vectors, query_s, self.station_period_m)
        heading = interpolate_periodic_angle(
            stations,
            tuple(sample.heading_rad for sample in self.samples),
            query_s,
            self.station_period_m,
        )
        curvature = interpolate_periodic_scalar(
            stations,
            tuple(sample.curvature_per_m for sample in self.samples),
            query_s,
            self.station_period_m,
        )
        lateral_offset = interpolate_periodic_scalar(
            stations,
            tuple(sample.lateral_offset_m for sample in self.samples),
            query_s,
            self.station_period_m,
        )
        speed_squared = interpolate_periodic_scalar(
            stations,
            tuple(sample.target_speed_mps ** 2 for sample in self.samples),
            query_s,
            self.station_period_m,
        )
        segment = min(bisect.bisect_right(stations, wrapped) - 1, len(self.samples) - 1)
        if segment < 0:
            segment = len(self.samples) - 1
        source = self.samples[segment]
        return TrajectorySample(
            source.sample_index,
            wrapped,
            x_m,
            y_m,
            heading,
            curvature,
            lateral_offset,
            math.sqrt(max(0.0, speed_squared)),
            source.target_accel_mps2,
            source.ds_to_next_m,
        )


def _read_rows(path, required):
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or any(field not in rows[0] for field in required):
        raise ValueError(f"missing required fields: {required}")
    return rows


def _number(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid numeric field {field}") from error
    if not math.isfinite(value):
        raise ValueError(f"non-finite numeric field {field}")
    return value


def compose_racing_trajectory(r_a0_csv, r_a0_metadata, r_a1_csv, r_a1_metadata, generator_code_sha256, *, expected_track_id="iccas2025"):
    r_a0_csv, r_a0_metadata = Path(r_a0_csv), Path(r_a0_metadata)
    r_a1_csv, r_a1_metadata = Path(r_a1_csv), Path(r_a1_metadata)
    a0_meta = json.loads(r_a0_metadata.read_text())
    a1_meta = json.loads(r_a1_metadata.read_text())
    source_track = a0_meta.get("source_track")
    if not isinstance(source_track, str) or not source_track.strip():
        raise ValueError("missing R-A0 track identity")
    if not isinstance(expected_track_id, str) or not expected_track_id.strip() or source_track != expected_track_id:
        raise ValueError("unsupported or mismatched track identity")
    a0_hash, a1_hash = sha256(r_a0_csv), sha256(r_a1_csv)
    if a0_hash != a0_meta.get("output_csv_sha256"):
        raise ValueError("R-A0 CSV hash mismatch")
    if a1_hash != a1_meta.get("output_csv_sha256"):
        raise ValueError("R-A1 CSV hash mismatch")
    source = a1_meta.get("source_r_a0", {})
    if source.get("csv_sha256") != a0_hash or source.get("metadata_sha256") != sha256(r_a0_metadata):
        raise ValueError("R-A1 does not reference the exact R-A0 product")
    for field in ("centerline_sha256", "map_sha256", "map_metadata_sha256"):
        if source.get(field) != a0_meta.get(field):
            raise ValueError(f"R-A0/R-A1 provenance mismatch: {field}")

    a0_rows = _read_rows(
        r_a0_csv,
        ("s_m", "x_m", "y_m", "heading_rad", "curvature_1_m", "d_m"),
    )
    a1_rows = _read_rows(
        r_a1_csv,
        ("s_m", "target_speed_m_s", "implied_acceleration_m_s2"),
    )
    if len(a0_rows) != len(a1_rows) or len(a0_rows) != a0_meta.get("sample_count"):
        raise ValueError("R-A0/R-A1 sample count mismatch")
    if len(a1_rows) != a1_meta.get("profile", {}).get("sample_count"):
        raise ValueError("R-A1 metadata sample count mismatch")

    parsed = []
    stations = []
    points = []
    for index, (geometry, speed) in enumerate(zip(a0_rows, a1_rows)):
        station = _number(geometry, "s_m")
        if station != _number(speed, "s_m"):
            raise ValueError(f"station mismatch at sample {index}")
        stations.append(station)
        points.append((_number(geometry, "x_m"), _number(geometry, "y_m")))
        parsed.append((
            station,
            points[-1][0],
            points[-1][1],
            _number(geometry, "heading_rad"),
            _number(geometry, "curvature_1_m"),
            _number(geometry, "d_m"),
            _number(speed, "target_speed_m_s"),
            _number(speed, "implied_acceleration_m_s2"),
        ))
    station_period = float(a0_meta["metrics"]["centerline"]["length"])
    if stations[0] != 0.0 or any(a >= b for a, b in zip(stations, stations[1:])) or stations[-1] >= station_period:
        raise ValueError("trajectory stations must be unique and increasing in [0, period)")
    geometry = ClosedTrackGeometry(points)
    samples = []
    for index, values in enumerate(parsed):
        following = points[(index + 1) % len(points)]
        ds = math.dist(points[index], following)
        if ds <= 0.0:
            raise ValueError("trajectory contains a nonpositive segment")
        samples.append(TrajectorySample(index, *values, ds))
    for index, sample in enumerate(samples):
        following = samples[(index + 1) % len(samples)]
        expected_acceleration = (
            following.target_speed_mps ** 2 - sample.target_speed_mps ** 2
        ) / (2.0 * sample.ds_to_next_m)
        if not math.isclose(
            sample.target_accel_mps2,
            expected_acceleration,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError(f"R-A1 acceleration/segment mismatch at sample {index}")
    reconstructed_length = sum(sample.ds_to_next_m for sample in samples)
    expected_length = float(a1_meta["profile"]["path_length_m"])
    if not math.isclose(reconstructed_length, expected_length, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("R-A1 physical path length mismatch")

    metadata = {
        "acceleration_semantics": "ENVELOPE_DERIVED_REFERENCE_FOR_OUTGOING_SEGMENT",
        "artifact_type": "RacingTrajectory",
        "canonical_direction": "counterclockwise",
        "closed_loop": True,
        "frame_id": "map",
        "generation_method": GENERATION_METHOD,
        "generator_code_sha256": str(generator_code_sha256),
        "geometry_qualification_status": "VERIFIED",
        "jerk_tracking_qualification": "NOT_YET_VERIFIED",
        "offline_envelope_predicted_lap_time_s": float(
            a1_meta["profile"]["offline_envelope_predicted_lap_time_s"]
        ),
        "path_length_m": reconstructed_length,
        "product_valid": True,
        "qualification_state": "OFFLINE_BASELINE_QUALIFIED",
        "runtime_lap_time_status": "NOT_VERIFIED",
        "runtime_tracking_qualified": False,
        "sample_count": len(samples),
        "schema_version": SCHEMA_VERSION,
        "source": {
            "competition_environment_sha256": a0_meta["competition_environment_sha256"],
            "map_image_sha256": a0_meta["map_sha256"],
            "map_yaml_sha256": a0_meta["map_metadata_sha256"],
            "r_a0_csv_sha256": a0_hash,
            "r_a0_metadata_sha256": sha256(r_a0_metadata),
            "r_a1_csv_sha256": a1_hash,
            "r_a1_metadata_sha256": sha256(r_a1_metadata),
            "source_centerline_sha256": a0_meta["centerline_sha256"],
        },
        "speed_envelope_qualification_status": "VERIFIED",
        "station_period_m": station_period,
        "track_id": source_track,
        "trajectory_tier": "BASELINE",
        "units": "SI",
    }
    return RacingTrajectory(tuple(samples), metadata)


def write_racing_trajectory(output_dir, trajectory, stem="iccas2025_r_a2_racing_trajectory"):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"{stem}.csv"
    metadata_path = output_dir / f"{stem}.json"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(SAMPLE_FIELDS)
        for sample in trajectory.samples:
            writer.writerow((sample.sample_index,) + tuple(
                _format(getattr(sample, field)) for field in SAMPLE_FIELDS[1:]
            ))
    csv_hash = sha256(csv_path)
    metadata = dict(trajectory.metadata)
    metadata["csv_sha256"] = csv_hash
    metadata["trajectory_id"] = _content_id(csv_path, metadata)
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return csv_path, metadata_path, RacingTrajectory(trajectory.samples, metadata)


def validate_racing_trajectory(trajectory, csv_path=None):
    metadata = trajectory.metadata
    if metadata.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported RacingTrajectory schema version")
    if metadata.get("artifact_type") != "RacingTrajectory" or metadata.get("frame_id") != "map":
        raise ValueError("invalid RacingTrajectory identity/frame")
    if metadata.get("closed_loop") is not True or metadata.get("product_valid") is not True:
        raise ValueError("RacingTrajectory is not a valid closed-loop product")
    if metadata.get("runtime_tracking_qualified") is not False:
        raise ValueError("R-A2 product must not claim runtime tracking qualification")
    if len(trajectory.samples) != metadata.get("sample_count"):
        raise ValueError("RacingTrajectory sample count mismatch")
    if csv_path is not None:
        csv_hash = sha256(csv_path)
        identity_metadata = dict(metadata)
        trajectory_id = identity_metadata.pop("trajectory_id", None)
        if metadata.get("csv_sha256") != csv_hash or trajectory_id != _content_id(csv_path, identity_metadata):
            raise ValueError("RacingTrajectory content identity mismatch")
    stations = [sample.s_m for sample in trajectory.samples]
    if stations[0] != 0.0 or any(a >= b for a, b in zip(stations, stations[1:])):
        raise ValueError("RacingTrajectory stations are not strictly periodic/increasing")
    if stations[-1] >= trajectory.station_period_m:
        raise ValueError("RacingTrajectory station lies outside period")
    if [sample.sample_index for sample in trajectory.samples] != list(range(len(trajectory.samples))):
        raise ValueError("RacingTrajectory sample indices are not canonical")
    numeric = SAMPLE_FIELDS[1:]
    if any(
        not math.isfinite(getattr(sample, field))
        for sample in trajectory.samples
        for field in numeric
    ):
        raise ValueError("RacingTrajectory contains non-finite values")
    if any(sample.target_speed_mps < 0.0 or sample.ds_to_next_m <= 0.0 for sample in trajectory.samples):
        raise ValueError("RacingTrajectory contains invalid speed/segment length")
    geometry = ClosedTrackGeometry(tuple((sample.x_m, sample.y_m) for sample in trajectory.samples))
    for index, sample in enumerate(trajectory.samples):
        following = trajectory.samples[(index + 1) % len(trajectory.samples)]
        if not math.isclose(
            sample.ds_to_next_m,
            math.hypot(following.x_m - sample.x_m, following.y_m - sample.y_m),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError(f"RacingTrajectory segment length mismatch at sample {index}")
        expected_acceleration = (
            following.target_speed_mps ** 2 - sample.target_speed_mps ** 2
        ) / (2.0 * sample.ds_to_next_m)
        if not math.isclose(
            sample.target_accel_mps2,
            expected_acceleration,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError(f"RacingTrajectory acceleration mismatch at sample {index}")
    if not math.isclose(geometry.length, metadata["path_length_m"], rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("RacingTrajectory path length mismatch")
    start = trajectory.sample_at_s(0.0)
    seam = trajectory.sample_at_s(trajectory.station_period_m)
    for field in ("x_m", "y_m", "heading_rad", "curvature_per_m", "lateral_offset_m", "target_speed_mps"):
        if not math.isclose(getattr(start, field), getattr(seam, field), rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError(f"RacingTrajectory seam mismatch: {field}")
    return {
        "path_length_m": geometry.length,
        "product_valid": True,
        "sample_count": len(trajectory.samples),
        "seam_error": 0.0,
    }


def load_racing_trajectory(csv_path=None, metadata_path=None):
    if (csv_path is None) != (metadata_path is None):
        raise ValueError("CSV and metadata paths must be provided together")
    if csv_path is None:
        data = Path(get_package_share_directory("f1tenth_racing_raceline")) / "data"
        csv_path = data / "iccas2025_r_a2_racing_trajectory.csv"
        metadata_path = data / "iccas2025_r_a2_racing_trajectory.json"
    rows = _read_rows(csv_path, SAMPLE_FIELDS)
    samples = []
    for row in rows:
        try:
            index = int(row["sample_index"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("invalid sample_index") from error
        samples.append(TrajectorySample(index, *(_number(row, field) for field in SAMPLE_FIELDS[1:])))
    trajectory = RacingTrajectory(tuple(samples), json.loads(Path(metadata_path).read_text()))
    validate_racing_trajectory(trajectory, csv_path)
    return trajectory
