# PPO / deep-learning workspace preservation — 2026-11-17

This branch preserves the reviewable source subset of
`F1TENTH_DEEP_RACING_WS`: the `deep_racing_next` ROS package, PPO training and
inference source, Pacejka/original-frozen source, Shadow ROS integration,
configuration, map catalog metadata, and self-authored architecture material.
`SOURCE_MANIFEST.tsv` records SHA-256 and byte size for every copied source
file.

## Safety and validation boundary

`src/deep_racing_next/deep_racing_next/solo_v1/l6_ros.py` sets
`SHADOW_ONLY=True` and publishes a `std_msgs/String` shadow proposal; it has no
drive or actuator authority.  The V215 audit evidence records
`physical_certification: false`.  This preservation snapshot makes neither a
ROS-driving, physical-driving, map-generalization, nor performance claim.

## Checkpoints

`CHECKPOINT_INVENTORY.tsv` records all 545 discovered `.pt`, `.pth`, and
`.onnx` artifacts under the original `runs/` tree, including size, SHA-256,
duplicate count, purpose class, and publication decision.  They total
32,690,393 bytes and 440 unique hashes (105 duplicate instances beyond the
unique set).  All 545 are excluded from this public branch because their
redistribution rights and provenance were not established.  No checkpoint is
included.

## Deliberate exclusions

Virtual environments, build/install/log trees, raw run outputs, checkpoint and
model directories, map assets/normalized maps, rollouts, legacy snapshots,
archives, and external data are excluded.  The copied ROS package declares
Apache-2.0; that declaration does not establish redistribution rights for the
excluded models or assets.
