# Research references and preservation note

This directory preserves bibliographic references and a concise, self-authored
research note for the RacecarSimulator/F1TENTH archive.  It intentionally does
not redistribute third-party paper PDFs: no redistribution permission was
established during this preservation pass.

## Scope

| Title | Research area | Source / local evidence path | Preservation form |
| --- | --- | --- | --- |
| F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning | autonomous racing, simulation, reinforcement learning | https://doi.org/10.1109/ICRA40945.2020.9196868 ; `/home/neo/Downloads/RACECAR_CODEX_AUDIT_20261117T120544Z/00_EXECUTIVE_SUMMARY.md` | DOI/URL and BibTeX only |
| Proximal Policy Optimization Algorithms | reinforcement learning | https://arxiv.org/abs/1707.06347 ; `/home/neo/F1TENTH_DEEP_RACING_WS/src/deep_racing_next/deep_racing_next/solo_v1/l5_training.py` | arXiv URL and BibTeX only |
| Current workspace preservation boundary | non-AI ROS control, PPO research, safety boundary | `/home/neo/Downloads/RACECAR_CODEX_AUDIT_20261117T120544Z/00_EXECUTIVE_SUMMARY.md` and `05_EVIDENCE_REGISTER.tsv` | self-authored note below |

## Preservation note

The `archive/non-ai-20261117` branch preserves the current non-AI ROS source
workspace separately from this simulator baseline.  The
`archive/ppo-deep-learning-20261117` branch preserves the PPO research source
workspace separately.  Static audit evidence records that the PPO ROS module
is shadow-only and has no actuator authority; V215 records
`physical_certification: false`.  These are preservation facts, not runtime or
physical-driving claims.

No paper PDF is included.  The cited works remain available from their
publishers or preprint hosts, subject to their respective terms.
