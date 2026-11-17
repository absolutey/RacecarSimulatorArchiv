# Deep Racing v2 — In-place architecture

This **reuses** `/home/neo/F1TENTH_DEEP_RACING_WS`, not `~/f1tenth_racing`.

| Layer | Responsibility | Progress now | Previous assets |
|---|---|---|---|
| L0 | Map normalizer and provenance | basic map catalog implemented; occupancy extraction pending | RG1/RG2 maps |
| L1 | Sensor/ego observation and OOD | typed state only | old RB Shadow |
| L2 | Neural Solo Planner line + speed | interface/contract only | V7/V8 trajectories and teacher |
| L3 | Neural Driver | NumPy MLP checkpoint inference (no checkpoint provided) | prior PPO/NN weights |
| L4 | Physics/Safety | fail closed boundary only | original simulator and Guard, **no copy of control ownership** |
| L5 | Episode and reward/training | strict zero-speed start contract only | PPO offline trials |
| L6 | ROS Ego bridge | intentionally **no actuator publisher** | old ROS2 packages in legacy |
| L7 | Opponent C/D/E | disabled until Solo passed | legacy R-C/D/E |

Old code and experiments are not automatically *labeled* training data. Legacy archives are reproducible evidence; a separately validated ETL must select lawful, useful teacher samples. Model weights are not automatically compatible with the new MLP input schema.

## Mandatory gates

1. Reproduce original maps and baselines from legacy, with consistent source SHA.
2. Map adapter: unseen map format, occupancy-only, closure and scale validity.
3. Real MLP training (start-grid episodes), checkpoint evaluation and no-zero-motion exploitation.
4. Multi-map train/val/blind holdout, compare one lap timings and crash rates.
5. Original F1TENTH Pacejka simulator physics under ≤20 m/s and independent safety receipt.
6. ROS Ego Shadow → supervised actuator admission; no Guard bypass.
7. Opponent learning/tracking/forecast and overtaking only after Solo gate.

Map assets stay at their existing pathname under `map_assets`. All other preexisting top-level workspace entries are archived by move, byte-for-byte, under `legacy/snapshots/<id>/original/` with SHA, and can be restored. Outputs of the new project live in `data`, `models`, `runs`, `configs`. No background run or ROS processes are launched by this kit.
