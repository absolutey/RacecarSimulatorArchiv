import csv
import hashlib
import json
import math
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from f1tenth_racing_geometry import (
    interpolate_periodic_angle,
    interpolate_periodic_scalar,
    interpolate_periodic_vector,
)

from f1tenth_racing_raceline.trajectory import (
    SCHEMA_VERSION,
    compose_racing_trajectory,
    load_racing_trajectory,
    validate_racing_trajectory,
    write_racing_trajectory,
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_csv(path, fields, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def source_products(tmp_path, a0_rows=None, a1_rows=None, a0_update=None, a1_update=None):
    tmp_path.mkdir(parents=True, exist_ok=True)
    a0_rows = a0_rows or [
        {"s_m": 0, "x_m": 0, "y_m": 0, "heading_rad": 3.1, "curvature_1_m": 1, "d_m": 0},
        {"s_m": 1, "x_m": 1, "y_m": 0, "heading_rad": -3.1, "curvature_1_m": 1, "d_m": 0.1},
        {"s_m": 2, "x_m": 1, "y_m": 1, "heading_rad": -1.5, "curvature_1_m": 1, "d_m": 0},
        {"s_m": 3, "x_m": 0, "y_m": 1, "heading_rad": 1.5, "curvature_1_m": 1, "d_m": -0.1},
    ]
    a1_rows = a1_rows or [
        {"s_m": 0, "target_speed_m_s": 2, "implied_acceleration_m_s2": 2.5},
        {"s_m": 1, "target_speed_m_s": 3, "implied_acceleration_m_s2": 3.5},
        {"s_m": 2, "target_speed_m_s": 4, "implied_acceleration_m_s2": -3.5},
        {"s_m": 3, "target_speed_m_s": 3, "implied_acceleration_m_s2": -2.5},
    ]
    a0_csv, a0_json = tmp_path / "a0.csv", tmp_path / "a0.json"
    a1_csv, a1_json = tmp_path / "a1.csv", tmp_path / "a1.json"
    write_csv(a0_csv, tuple(a0_rows[0]), a0_rows)
    a0_meta = {
        "centerline_sha256": "1" * 64,
        "competition_environment_sha256": "2" * 64,
        "map_metadata_sha256": "3" * 64,
        "map_sha256": "4" * 64,
        "metrics": {"centerline": {"length": 4.0}},
        "output_csv_sha256": digest(a0_csv),
        "sample_count": len(a0_rows),
        "source_track": "iccas2025",
    }
    if a0_update:
        a0_update(a0_meta)
    a0_json.write_text(json.dumps(a0_meta, sort_keys=True))
    write_csv(a1_csv, tuple(a1_rows[0]), a1_rows)
    a1_meta = {
        "output_csv_sha256": digest(a1_csv),
        "profile": {
            "offline_envelope_predicted_lap_time_s": 1.2,
            "path_length_m": 4.0,
            "sample_count": len(a1_rows),
        },
        "source_r_a0": {
            "centerline_sha256": a0_meta["centerline_sha256"],
            "csv_sha256": digest(a0_csv),
            "map_metadata_sha256": a0_meta["map_metadata_sha256"],
            "map_sha256": a0_meta["map_sha256"],
            "metadata_sha256": digest(a0_json),
        },
    }
    if a1_update:
        a1_update(a1_meta)
    a1_json.write_text(json.dumps(a1_meta, sort_keys=True))
    return a0_csv, a0_json, a1_csv, a1_json


def compose(paths):
    return compose_racing_trajectory(*paths, "5" * 64)


def test_synthetic_product_round_trip_identity_and_seam(tmp_path):
    trajectory = compose(source_products(tmp_path))
    csv_path, metadata_path, written = write_racing_trajectory(tmp_path / "output", trajectory)
    loaded = load_racing_trajectory(csv_path, metadata_path)
    assert loaded == written
    assert loaded.trajectory_id.startswith("sha256:")
    assert validate_racing_trajectory(loaded, csv_path)["product_valid"] is True
    assert loaded.sample_at_s(0.0) == loaded.sample_at_s(loaded.station_period_m)
    assert loaded.sample_at_s(-0.5) == loaded.sample_at_s(loaded.station_period_m - 0.5)
    assert abs(loaded.sample_at_s(0.5).heading_rad) > 3.0
    assert loaded.sample_at_s(0.5).target_speed_mps == pytest.approx(math.sqrt(6.5))
    assert loaded.samples[-1].target_accel_mps2 == -2.5


@pytest.mark.parametrize(
    "change,match",
    [
        (lambda paths: paths[0].write_text(paths[0].read_text() + "\n"), "R-A0 CSV hash"),
        (
            lambda paths: _change_json(paths[3], lambda data: data["source_r_a0"].update(csv_sha256="0" * 64)),
            "exact R-A0 product",
        ),
        (
            lambda paths: _change_json(paths[3], lambda data: data["source_r_a0"].update(map_sha256="0" * 64)),
            "provenance mismatch",
        ),
    ],
)
def test_source_hash_and_provenance_tampering_is_rejected(tmp_path, change, match):
    paths = source_products(tmp_path)
    change(paths)
    with pytest.raises(ValueError, match=match):
        compose(paths)


def _change_json(path, change):
    data = json.loads(path.read_text())
    change(data)
    path.write_text(json.dumps(data, sort_keys=True))


def test_track_count_station_and_numeric_mismatches_are_rejected(tmp_path):
    paths = source_products(tmp_path / "track", a0_update=lambda data: data.update(source_track="wrong"))
    with pytest.raises(ValueError, match="track identity"):
        compose(paths)

    fewer = [
        {"s_m": index, "target_speed_m_s": 2, "implied_acceleration_m_s2": 0}
        for index in range(3)
    ]
    paths = source_products(tmp_path / "count", a1_rows=fewer)
    with pytest.raises(ValueError, match="sample count"):
        compose(paths)

    stations = [
        {"s_m": value, "target_speed_m_s": 2, "implied_acceleration_m_s2": 0}
        for value in (0, 1.5, 2, 3)
    ]
    paths = source_products(tmp_path / "station", a1_rows=stations)
    with pytest.raises(ValueError, match="station mismatch"):
        compose(paths)

    paths = source_products(tmp_path / "numeric")
    rows = list(csv.DictReader(paths[0].open()))
    rows[0]["x_m"] = "nan"
    write_csv(paths[0], tuple(rows[0]), rows)
    _change_json(paths[1], lambda data: data.update(output_csv_sha256=digest(paths[0])))
    _change_json(paths[3], lambda data: data["source_r_a0"].update(
        csv_sha256=digest(paths[0]), metadata_sha256=digest(paths[1])
    ))
    with pytest.raises(ValueError, match="non-finite"):
        compose(paths)

    paths = source_products(tmp_path / "acceleration")
    rows = list(csv.DictReader(paths[2].open()))
    rows[0]["implied_acceleration_m_s2"] = "0"
    write_csv(paths[2], tuple(rows[0]), rows)
    _change_json(paths[3], lambda data: data.update(output_csv_sha256=digest(paths[2])))
    with pytest.raises(ValueError, match="acceleration/segment mismatch"):
        compose(paths)


def test_duplicate_station_and_unsupported_or_tampered_product_are_rejected(tmp_path):
    paths = source_products(tmp_path / "duplicate")
    a0_rows = list(csv.DictReader(paths[0].open()))
    a1_rows = list(csv.DictReader(paths[2].open()))
    a0_rows[2]["s_m"] = a0_rows[1]["s_m"]
    a1_rows[2]["s_m"] = a1_rows[1]["s_m"]
    write_csv(paths[0], tuple(a0_rows[0]), a0_rows)
    _change_json(paths[1], lambda data: data.update(output_csv_sha256=digest(paths[0])))
    write_csv(paths[2], tuple(a1_rows[0]), a1_rows)
    _change_json(paths[3], lambda data: data.update(
        output_csv_sha256=digest(paths[2]),
        source_r_a0={
            **data["source_r_a0"],
            "csv_sha256": digest(paths[0]),
            "metadata_sha256": digest(paths[1]),
        },
    ))
    with pytest.raises(ValueError, match="unique and increasing"):
        compose(paths)

    valid_paths = source_products(tmp_path / "product")
    csv_path, metadata_path, _ = write_racing_trajectory(tmp_path / "product/output", compose(valid_paths))
    _change_json(metadata_path, lambda data: data.update(schema_version=SCHEMA_VERSION + 1))
    with pytest.raises(ValueError, match="unsupported"):
        load_racing_trajectory(csv_path, metadata_path)

    csv_path, metadata_path, _ = write_racing_trajectory(tmp_path / "product/output2", compose(valid_paths))
    _change_json(metadata_path, lambda data: data["source"].update(map_image_sha256="0" * 64))
    with pytest.raises(ValueError, match="content identity"):
        load_racing_trajectory(csv_path, metadata_path)


def test_canonical_product_preserves_sources_and_dense_sampling_semantics():
    data = Path(get_package_share_directory("f1tenth_racing_raceline")) / "data"
    trajectory = load_racing_trajectory()
    a0 = list(csv.DictReader((data / "iccas2025_r_a0_baseline.csv").open()))
    a1 = list(csv.DictReader((data / "iccas2025_r_a1_speed_profile.csv").open()))
    output = list(csv.DictReader((data / "iccas2025_r_a2_racing_trajectory.csv").open()))
    for geometry, speed, composed in zip(a0, a1, output):
        assert composed["s_m"] == geometry["s_m"] == speed["s_m"]
        assert composed["x_m"] == geometry["x_m"]
        assert composed["y_m"] == geometry["y_m"]
        assert composed["heading_rad"] == geometry["heading_rad"]
        assert composed["curvature_per_m"] == geometry["curvature_1_m"]
        assert composed["lateral_offset_m"] == geometry["d_m"]
        assert composed["target_speed_mps"] == speed["target_speed_m_s"]
        assert composed["target_accel_mps2"] == speed["implied_acceleration_m_s2"]

    stations = tuple(sample.s_m for sample in trajectory.samples)
    vectors = tuple((sample.x_m, sample.y_m) for sample in trajectory.samples)
    headings = tuple(sample.heading_rad for sample in trajectory.samples)
    curvature = tuple(sample.curvature_per_m for sample in trajectory.samples)
    speed_squared = tuple(sample.target_speed_mps ** 2 for sample in trajectory.samples)
    period = trajectory.station_period_m
    previous_heading = None
    for index in range(6601):
        query = -period + index * 3.0 * period / 6600
        sample = trajectory.sample_at_s(query)
        assert sample == trajectory.sample_at_s(query)
        assert 0.0 <= sample.s_m < period
        assert (sample.x_m, sample.y_m) == pytest.approx(
            interpolate_periodic_vector(stations, vectors, query, period), abs=1.0e-12
        )
        assert sample.heading_rad == pytest.approx(
            interpolate_periodic_angle(stations, headings, query, period), abs=1.0e-12
        )
        assert sample.curvature_per_m == pytest.approx(
            interpolate_periodic_scalar(stations, curvature, query, period), abs=1.0e-12
        )
        assert sample.target_speed_mps == pytest.approx(
            math.sqrt(interpolate_periodic_scalar(stations, speed_squared, query, period)),
            abs=1.0e-12,
        )
        assert all(math.isfinite(value) for value in (
            sample.x_m,
            sample.y_m,
            sample.heading_rad,
            sample.curvature_per_m,
            sample.lateral_offset_m,
            sample.target_speed_mps,
            sample.target_accel_mps2,
        ))
        if previous_heading is not None:
            heading_step = (sample.heading_rad - previous_heading + math.pi) % (2.0 * math.pi) - math.pi
            assert abs(heading_step) < 0.01
        previous_heading = sample.heading_rad
