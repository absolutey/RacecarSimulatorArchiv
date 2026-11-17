# 2026-10-19 Release evidence recursion RCA

Original failure: `UNEXPECTED_OR_MISSING_FILES:['EVIDENCE_FINAL_INTEGRATED/RELEASE_HASHES.log']` occurs before ROS build because `RUN_FINAL_INTEGRATED_ROS_VERIFY.sh` creates `EVIDENCE_FINAL_INTEGRATED/` inside the sealed release, then `tee` opens `RELEASE_HASHES.log` before `VERIFY_RELEASE.py` enumerates all files. The verifier correctly flags this new untracked file; the launcher writes it in the wrong place.

Repairs:
- Full ROS verification now uses `../EVIDENCE_FINAL_INTEGRATED/` by default.
- Explicit `EVIDENCE_ROOT` under the signed release is blocked before creating a file, even when the path is relative or includes symlinks.
- Older structural and provenance ROS launchers also default outside the release and the structural runner blocks an explicitly internal evidence root.
- RELEASE_HASHES and `VERIFY_RELEASE.py` remain **strict**. No hash checking is disabled, and actual unexpected files or hash modifications still cause fail-closed behavior.
- Five extra ROS-registered tests cover evidence root choice, early failure on in-tree overrides, old runner paths, strict hashes, and real verifier behavior with an external tee log.

The previous 17-package/587 registered test PASS is the user's Ubuntu result from the parent ZIP. This modified release requires a new Ubuntu build/test, expected 597 ROS tests (parent runtime fix expected 592 + 5). No H2H or realtime qualification is claimed.
