import csv
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def load(name):
    path = Path(__file__).with_name(name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_canonical_curvature_segmentation_is_reproducible():
    analyzer = load("analyze_r_b_run")
    path = ROOT / "src/driving/algorithms/f1tenth_racing_raceline/data/iccas2025_r_a2_racing_trajectory.csv"
    segments = analyzer.corner_segments(list(csv.DictReader(path.open())))
    assert [(segment["direction"], round(segment["apex_s_m"], 3))
            for segment in segments] == [("LEFT", 14.082), ("RIGHT", 22.106),
                                         ("LEFT", 29.820)]


def test_common_runner_keeps_resource_and_solo_ownership_gates():
    source = Path(__file__).with_name("qualify_solo.py").read_text()
    for contract in ("require_qualification_preflight", "solo:=true",
                     "unexpected Blue command owners", "state1"):
        assert contract in source
