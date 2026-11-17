# Non-AI ROS workspace preservation — 2026-11-17

This is a source-preservation snapshot of `/home/neo/f1tenth_racing`, copied
without modifying that original workspace.  Its relative source layout is
preserved under `src/`; it includes ROS package metadata, launch files,
configuration, source, tests, integration tools, and self-authored research
and change records.

## Included packages

The snapshot contains 17 ROS packages: control, raceline, perception,
prediction, overtaking, baseline, simulator/environment, interfaces,
bringup, evaluation, geometry, native proof, reference, telemetry, and tools.
`SOURCE_MANIFEST.tsv` records source and archived paths, SHA-256, and byte size
for every copied file; `SOURCE_MANIFEST.sha256` is a local archive checksum
list.

## Deliberate exclusions

`build/`, `install/`, `log/`, Python caches, binary build products, bag/db
artifacts, ZIP archives, and `src/environment/f1tenth_racing_sim/maps/` are
not included.  The latter map assets were excluded because public
redistribution rights were not established.  No ROS execution, performance
test, or physical validation was performed for this preservation snapshot.

Package metadata declares Apache-2.0 for the copied ROS packages.  This record
does not independently establish rights for any excluded asset or a runtime
performance claim.
