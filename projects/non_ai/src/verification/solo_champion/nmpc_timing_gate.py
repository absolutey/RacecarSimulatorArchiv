#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path
import sqlite3
import statistics
import time
from types import SimpleNamespace

import numpy as np
import yaml

from f1tenth_racing_control import PursuitReference
from f1tenth_racing_sim.physics.vehicle import Dynamics, State


def percentile(values, fraction):
    values = sorted(values)
    return values[round(fraction * (len(values) - 1))]


def load_reference(path):
    rows = list(csv.DictReader(Path(path).open()))
    samples = tuple(SimpleNamespace(
        sample_index=int(row["sample_index"]), s_m=float(row["s_m"]),
        x_m=float(row["x_m"]), y_m=float(row["y_m"]),
        heading_rad=float(row["heading_rad"]), curvature_per_m=float(row["curvature_per_m"]),
        lateral_offset_m=float(row["lateral_offset_m"]),
        target_speed_mps=float(row["target_speed_mps"]),
        target_accel_mps2=float(row["target_accel_mps2"]), ds_to_next_m=float(row["ds_to_next_m"]),
    ) for row in rows)
    return PursuitReference(samples, sum(sample.ds_to_next_m for sample in samples))


def load_states(bak, count):
    database = sqlite3.connect(f"file:{bak}?mode=ro", uri=True)
    rows = database.execute(
        "SELECT blue_x_m,blue_y_m,blue_yaw_rad,actual_speed_mps,slip_angle_rad,"
        "yaw_rate_rad_s,steering_angle_rad,longitudinal_acceleration_mps2,target_progress_m "
        "FROM telemetry JOIN r_b_runtime USING(row_no) WHERE actual_speed_mps > 0.5"
    ).fetchall()
    database.close()
    step = max(1, len(rows) // count)
    return rows[::step][:count]


class ShootingGate:
    def __init__(self, dynamics, limits, reference, horizon):
        self.dynamics = Dynamics(dynamics)
        self.limits = limits
        self.reference = reference
        self.horizon = horizon
        self.dt = 0.025

    def rollout(self, sample, controls):
        px, py, yaw, speed, slip, omega, steer, accel, station = sample
        state = State(x=px, y=py, yaw=yaw, speed=speed, slip=slip, yaw_rate=omega,
                      steering=steer, requested_acceleration=accel, acceleration=accel,
                      vx=speed * math.cos(slip), vy=speed * math.sin(slip))
        cost = 0.0
        previous = np.array([steer, accel])
        for control in controls:
            rate = np.array([self.limits["steering_rate_abs_max_rad_s"],
                             self.limits["jerk_abs_max_m_s3"]]) * self.dt
            control = np.clip(control, previous - rate, previous + rate)
            control = np.clip(control, [-self.limits["steering_angle_abs_max_rad"],
                                        -self.limits["deceleration_max_m_s2"]],
                                       [self.limits["steering_angle_abs_max_rad"],
                                        self.limits["acceleration_max_m_s2"]])
            state = self.dynamics.step(state, float(control[0]), float(control[1]), self.dt)
            station = (station + max(state.speed, 0.0) * self.dt) % self.reference.station_period_m
            dx = state.x - self.reference.scalar("x_m", station)
            dy = state.y - self.reference.scalar("y_m", station)
            heading = self.reference.angle("heading_rad", station)
            normal_error = -math.sin(heading) * dx + math.cos(heading) * dy
            speed_error = state.speed - self.reference.scalar("target_speed_mps", station)
            heading_error = math.atan2(math.sin(state.yaw - heading), math.cos(state.yaw - heading))
            cost += (30 * normal_error ** 2 + 12 * heading_error ** 2 +
                     4 * speed_error ** 2 + state.slip ** 2 + 0.5 * state.yaw_rate ** 2)
            previous = control
        return cost

    def solve(self, sample, formulation):
        curvature = self.reference.scalar("curvature_per_m", sample[-1])
        target_accel = self.reference.scalar("target_accel_mps2", sample[-1])
        controls = np.tile([math.atan(0.34 * curvature), target_accel], (self.horizon, 1))
        if formulation == "COORDINATE":
            for scale in (0.10, 0.03):
                for index in range(2 * self.horizon):
                    best = controls.copy(); best_cost = self.rollout(sample, best)
                    for direction in (-1.0, 1.0):
                        trial = controls.copy(); trial.flat[index] += direction * scale
                        cost = self.rollout(sample, trial)
                        if cost < best_cost:
                            best, best_cost = trial, cost
                    controls = best
        elif formulation == "SPSA":
            for iteration in range(6):
                delta = np.where((np.arange(2 * self.horizon) + iteration) % 2, 1.0, -1.0)
                delta = delta.reshape(self.horizon, 2)
                plus = self.rollout(sample, controls + 0.05 * delta)
                minus = self.rollout(sample, controls - 0.05 * delta)
                controls -= 0.01 * (plus - minus) / 0.1 * delta
        else:
            best, best_cost = controls, self.rollout(sample, controls)
            for offset in np.linspace(-0.20, 0.20, 17):
                trial = controls.copy(); trial[:, 0] += offset
                cost = self.rollout(sample, trial)
                if cost < best_cost:
                    best, best_cost = trial, cost
            controls = best
        return self.rollout(sample, controls), controls[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bak", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--environment", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=24)
    args = parser.parse_args()
    environment = yaml.safe_load(args.environment.read_text())["dynamics"]
    reference = load_reference(args.reference)
    states = load_states(args.bak, args.samples)
    results = []
    for formulation, horizon in (("COORDINATE", 4), ("SPSA", 6), ("PATTERN", 8)):
        solver = ShootingGate(environment, environment["limits"], reference, horizon)
        wall, cpu, finite = [], [], True
        for state in states:
            wall_start, cpu_start = time.perf_counter(), time.process_time()
            objective, control = solver.solve(state, formulation)
            wall.append((time.perf_counter() - wall_start) * 1000)
            cpu.append((time.process_time() - cpu_start) * 1000)
            finite &= math.isfinite(objective) and np.all(np.isfinite(control))
        results.append({
            "formulation": formulation, "horizon": horizon, "samples": len(states),
            "finite": bool(finite), "wall_ms": {"median": statistics.median(wall),
                "p95": percentile(wall, .95), "max": max(wall)},
            "cpu_ms": {"median": statistics.median(cpu), "p95": percentile(cpu, .95),
                "max": max(cpu)},
        })
    print(json.dumps(results, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
