# Log Analyzer Dynamics Analysis Checklist

Scope: add a read-only dynamics analysis tab to `log_main.py` / `modules/logAnalyzer` using the 0401-heavy logs under:

- `Logs/Scenario_2026-05-21T085327`
- `Logs/Scenario_2026-05-21T105639`

The SIM-side reference is `modules/sim/tune/log_dynamics_calibrator.py`, which produces `uav_pid_db.json` and `uav_dynamics_profile.json` proposals from 0401 logs. The logAnalyzer tab must not apply changes directly; it should explain observed behavior and produce mission-planning-friendly recommended values.

## Agent Roles

- Parent implementation: own integration, final verification, and conflict control.
- Explorer 1: inspect SIM 0401 dynamics calibration, runtime UAV profile application, and reusable metrics.
- Explorer 2: inspect logAnalyzer API/UI extension points and layout risks.

## Analysis Coverage

- [x] Read all `SBC3/0401/0401*.json` messages, not only parsed timeline data.
- [x] Analyze every aircraft, while marking UAV 4-6 as the calibration target group.
- [x] Derive time cadence, speed, acceleration, vertical rate, heading rate, yaw acceleration, turn radius, and bank/roll proxy.
- [x] Detect direct roll/bank/pitch/yaw fields when logs provide them; fall back to bank proxy when roll is absent.
- [x] Segment behavior by `flightMode`, `flying`, waypoint, target following, sensor operation, and filming state.
- [x] Correlate `0602` commands with later `0401` state transitions to estimate response latency.
- [x] Detect aggressive turns, opposite-turn reversals, speed bucket radius, and mission-phase sensitivity.
- [x] Produce mission-planner speed turn-radius rows for `turn_radius_30_m`, `turn_radius_40_m`, and `turn_radius_50_m`.
- [x] Scan non-0401 logs for command/mission/replan signal availability and roll-related fields.
- [x] Produce recommended values aligned with SIM runtime and mission planning needs:
  - `reference_turn_radius_scale`
  - `max_yaw_rate_dps`
  - `turn_bank_limit_deg`
  - `roll_limit_deg`
  - `max_roll_rate_dps`
  - `lookahead_m`
  - `freeze_yaw_dist`
  - speed-bucket turn-radius table

## UI Coverage

- [x] Add a toolbar entry separate from ICD validation.
- [x] Add a dense right-side dynamics panel with summary, recommendations, per-aircraft rows, phase rows, command latency, and event list.
- [x] Keep panel independent from the existing detail, timeline, track playback, aircraft filter, and ICD analysis panel.
- [x] Make long content vertically scrollable and keep text inside compact containers.
- [x] Add explicit report persistence without applying SIM runtime settings.
- [x] Save timestamped JSON/Markdown plus `dynamics_analysis_latest.*` under each scenario's `LogAnalyzer_Reports` folder.

## Verification

- [x] `python -m py_compile` for touched Python modules.
- [x] Analyze `Scenario_2026-05-21T085327` through the new analysis engine.
- [x] Analyze `Scenario_2026-05-21T105639` through the new API.
- [x] Run the web UI and verify the dynamics panel opens and renders real metrics.
- [x] Confirm no direct writes to SIM PID/profile files from logAnalyzer.
- [x] Save a dynamics report through the new API and verify generated files.
- [x] Save a dynamics report from the web UI.
