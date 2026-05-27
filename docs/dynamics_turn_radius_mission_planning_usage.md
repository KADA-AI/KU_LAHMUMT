# Dynamics Turn Radius Mission Planning Usage Map

Date: 2026-05-21

Purpose: document where the log-analyzed aircraft dynamics, especially observed turn radius, turn-rate limits, speed, roll/yaw response, `lookaheadM`, and `freezeYawDistanceM`, can be used across mission planning and replanning. This is intentionally broader than "set a turn radius value"; it maps the places where route-to-route and mission-to-mission transition timing such as `T`, `T'`, `T0`, and `T0'` is created or consumed.

## Source Report

The current saved analysis report is:

- `Logs/Scenario_2026-05-21T105639/LogAnalyzer_Reports/dynamics_analysis_latest.json`
- `kind`: `logAnalyzerDynamicsReport`
- `schemaVersion`: `logAnalyzer.dynamics.v1`
- `analysis.ok`: `true`
- `analysis.recommendations.quality`: `ok`

Representative values from the report:

| Field | Value | Planning meaning |
| --- | ---: | --- |
| `basis.medianSpeedMps` | `44.86` | Cohort nominal speed for timing fallback |
| `basis.medianTurnRadiusM` | `538.77` | Observed nominal radius around the actual operating speed |
| `missionPlanningHints.plannerTurnRadiusScale` | `1.0701` | Initial candidate scale for speed-table based next-collab planners |
| `missionPlanningHints.nominalTurnRadiusM` | `538.8` | Absolute Dubins/transition radius candidate |
| `missionPlanningHints.conservativeTurnRadiusM` | `540.2` | Safer feasibility/rejoin/trim radius candidate |
| `missionPlanningHints.lookaheadM` | `242.4` | Entry projection distance for live aircraft handoff |
| `missionPlanningHints.freezeYawDistanceM` | `75.4` | Minimum near-aircraft distance where yaw/turn intent should not be churned |
| `simDynamicsProfile.max_yaw_rate_dps` | `7.967` | Yaw-rate cap for ETA and turn feasibility |
| `simDynamicsProfile.max_roll_rate_dps` | `9.914` | Roll response cap for aggressive-turn/reversal feasibility |

Derived handoff:

```text
entryLeadTimeS ~= lookaheadM / medianSpeedMps
               ~= 242.4 / 44.86
               ~= 5.4 s
```

## Agent Split

The analysis was split across sub agents as follows:

| Agent | Scope | Key output |
| --- | --- | --- |
| Banach | normal planning, attack, prior/lead mission, core route generation | Found absolute Dubins radius, FOV/SEP coupling, area transition links, attack/rejoin ETA hooks |
| Epicurus | monitoring, next collaboration, current remaining, reexecute/reperform | Found monitoring turn projections, `turnRadiusScale` handoff, T/T'/T0 consumption, payload gaps |
| Franklin | shared report contract, sim/runtime/common geometry, ETA/ICD surfaces | Defined safe report JSON paths and warned about radius semantics mismatch |

## Radius Semantics

Use these as separate concepts. Do not collapse them into one global value without checking the consumer.

| Concept | Current owner | Meaning |
| --- | --- | --- |
| Common reference radius | `modules/common/turn_radius.py` | Static speed table interpolation, currently `30m/s=340m`, `40m/s=450m`, `50m/s=560m` |
| Sim tuned radius | `modules/sim/runtime/uav.py` | Common reference radius multiplied by `reference_turn_radius_scale` |
| General mission Dubins radius | `modules/mission_planning/MissionPlanner/runtime_settings.py` and `d0303.py` | Absolute radius used to build route-to-route Dubins links and some loiter defaults |
| Next-collab radius scale | `next_collab_turn_radius_scale` and `turnRadiusScale` detail fields | Scale applied to the speed table for live turn prediction, tangent selection, and T/T' generation |
| Observed per-aircraft radius | Log Analyzer report `analysis.agents.*` | Best future source for per-aircraft, per-speed, per-phase transition feasibility |

Recommended policy:

1. Validate `kind`, `schemaVersion`, `analysis.ok`, and `analysis.recommendations.quality` before consuming report fields.
2. Use `missionPlanningHints.*` for mission-planning decisions.
3. Use `simDynamicsProfile.*` only for fields that match sim runtime semantics.
4. Use cohort values only as fallback. When an aircraft id is available, prefer per-aircraft/per-phase observed values.
5. Treat `plannerTurnRadiusScale=1.0701` as a candidate for next-collab scale, not an unconditional global overwrite. It changes planner geometry, branching, and timing, so it needs route-output regression checks.

## Where Radius Actually Matters

Turn radius currently changes more than a circular arc:

- It changes whether a route-to-route link is feasible.
- It changes where an aircraft can leave the current route and where it can enter the next route.
- It changes `T`, `T'`, `T0`, and `T0'` positions.
- It changes `horizonSec`, `etaSec`, `estimatedTotalSec`, and exported waypoint `eta`.
- It changes FOV/SEP selection in prior/general mission profiles.
- It changes how much path should be trimmed or protected near the live aircraft.
- It changes attack/rejoin hold timing and whether a reconnect path is realistic.
- It changes option ranking when decision support compares time or feasibility.

## Category Map

| Category | Current code surface | Current use | Dynamics report use |
| --- | --- | --- | --- |
| Normal mission planning | `runtime_settings.py`, `export_0303_0304.py`, `d0303.py`, `dubins_turn_link.py` | Uses absolute `dubins_turn_radius_m`, default `500m`, for area Dubins entry links and transition waypoints | Seed `dubins_turn_radius_m` from `nominalTurnRadiusM` or `conservativeTurnRadiusM`; later make it speed/aircraft dependent |
| Area route-to-route transition | `d0303._append_area_transition_links_inplace`, `d0303._build_area_dubins_entry_wps`, `dubins_turn_link.compute_turn_link` | Radius controls link feasibility, link points, arc length, sampled path, heading change, `min_link_gap_m`, and ETA recompute after waypoint insertion | Use observed radius to reject too-tight links and compute more realistic ETA/ECF |
| FOV/SEP profile selection | `runtime_settings.select_fov_db_row_by_turn_radius`, `get_runtime_prior_mission_profile` | Radius selects FOV DB row where separation is compatible with turning | Learned larger/smaller radius changes sensor separation/width and prior approach geometry |
| Next collaboration | `turn_radius_monitor.py`, `next_collab_replan.py`, `next_collab_division_runner.py`, `_planner_window.py`, `next_collab_line_runner.py`, `next_collab_path_builder.py` | Uses `turnRadiusScale`, live turn projection, tangent search, T/T' and T0/T0' path rows | Feed `plannerTurnRadiusScale`, `entryLeadTimeS`, `lookaheadM`, `freezeYawDistanceM`, and per-aircraft radius into payload and runner |
| Current remaining/general replan | `current_remaining_replan.py`, `monitoring_gui.py`, `current_remaining_hybrid.py`, `general_remaining_hybrid_replan.py` | Can carry `turnRadiusScale`, but defaults often remain `1.4` | Always populate detail from report and live monitor instead of runtime default |
| Reexecute/reperform | `collab_reexecute.py`, `reexecute_first_mission_hybrid.py`, `current_remaining_hybrid_replan.py` | Some wrappers support `turn_radius_scale`, but snapshot rebuilds can pass `None` | Add `turnRadiusScale`, `entryLeadTimeS`, `entryAircraftList[].dynamics`, and report source into the reexecute detail |
| Path-deviation replan | `turn_radius_monitor.py`, `path_deviation_replan.py`, `path_deviation_replan_pipeline_impl.py` | Monitor creates alternate waypoint and stores actual/ideal radius, but pipeline mostly consumes coordinate only | Use radius/rate/ETA for alternate selection, trim/stitch feasibility, completion hold, and near-aircraft freeze |
| Prior/lead mission | `prior_mission_pipeline_impl.py`, `runtime_settings.get_runtime_prior_mission_profile` | Uses profile turn radius/FOV/loiter; later resume timing is mostly distance/speed based | Use learned radius to tune approach distance, target ETA, loiter radius/time, and release/resume ETA |
| Attack/re-attack | `mission_planning_attack_helpers.py`, `attack_plan_pipeline.py`, `target_detection_replan.py` | Standoff/attack ETA and target sequencing are largely straight-line or distance/order based | Use observed radius, yaw-rate, and lookahead for attack approach, target ordering, release/reconnect ETA |
| Post-attack rejoin | `post_attack_rejoin_pipeline.py` | `_estimate_turn_aware_eta_s` already adds turn arc length from `turn_radius_m`; default is much smaller than observed UAV radius | Use `conservativeTurnRadiusM` for UAV rejoin timing, hold duration, and rejoin feasibility; do not blindly apply UAV radius to LAH |
| Imaging/schedule replan | `imaging_schedule_replan_pipeline_impl.py`, `mission_path_trim.py` | Rewrites replacement waypoints after speed scaling; buffer trimming is time/distance based | Use speed bucket and radius to tune buffer seconds, preserve feasible sweep entries, and avoid impossible turn-in |
| Decision support | `option_processing.py`, `decision_support_gui.py` | Options are built from templates/placeholders and 0701 push context | Feed dynamics-aware `estimatedTotalSec`, route feasibility flags, and turn-prefix duration into option ranking |
| Common ETA/export | `modules/common/eta.py`, `runtime/json_io.py` | ETA is distance/speed plus loiter. Turn arcs count only if upstream inserted turn waypoints or loiter fields exist | Prefer fixing upstream route generation first; later add optional turn-prefix metadata if required |

## T, T', T0, T0' Map

These are planner-internal labels, not ICD fields. Their exported effect is visible through route points, `horizonSec`, `etaSec`, `phaseRows`, `estimatedTotalSec`, and final waypoint `eta`.

| Label | Meaning in current planners | Turn-radius dependency |
| --- | --- | --- |
| `T` | Tangent or entry point from current aircraft/route into the candidate mission path | Comes from turn-circle projection and tangent search; larger radius moves `T` away from the aircraft and can invalidate direct entry |
| `T'` | Adjusted entry point advanced from `T` toward the mission start to satisfy SEP/FOV/path constraints | Radius affects the starting geometry and `T -> T'` distance; speed/radius affect ETA from current state to `T'` |
| `T0` | Path-0 / mid-line preview tangent for current mission continuation or area takeover | Built from turn prediction and visibility/tangent candidate selection; branch choice depends on radius |
| `T0'` | Adjusted Path-0 entry point after binary search for required separation | Radius affects candidate feasibility, start ETA, and whether the path is direct or takeover |

Important code anchors:

- `modules/mission_planning/planners/next_collab_division/_planner_window.py`
  - `_turn_radius_scale_value`
  - `_turn_radius_for_speed_m`
  - `_turn_prediction_points_xy`
  - `_find_visibility_segment`
  - `_mid_line_t0_preview`
  - `_build_turn_prefix_rows`
  - `_build_assignment_path_1_row`
  - `_build_waypoint_row_from_path`
  - Path-1 `Entry T'` search
  - Path-0 `T0/T0'` search and timeline rows
- `modules/mission_planning/runtime/next_collab_division_runner.py`
  - `run_next_collab_division_plan(... turn_radius_scale=...)`
- `modules/mission_planning/runtime/next_collab_line_runner.py`
  - `_bind_scaled_turn_methods`
  - `run_next_collab_line_plan(... turn_radius_scale=...)`
- `modules/mission_planning/pipelines/next_collab_path_builder.py`
  - consumes `entryTPrimeXY`, `waypointStartXY`, `tangentXY`, `horizonSec`, `estimatedTotalSec`

## Payload Contract To Add Later

The current system already has a usable scalar handoff, `turnRadiusScale`, but it is too thin. Future replan details should preserve both report-level and per-aircraft dynamics.

Recommended detail-level block:

```json
{
  "dynamicsReport": {
    "kind": "logAnalyzerDynamicsReport",
    "schemaVersion": "logAnalyzer.dynamics.v1",
    "sourcePath": "Logs/Scenario_2026-05-21T105639/LogAnalyzer_Reports/dynamics_analysis_latest.json",
    "quality": "ok"
  },
  "turnRadiusScale": 1.0701,
  "entryLeadTimeS": 5.4,
  "lookaheadM": 242.4,
  "freezeYawDistanceM": 75.4,
  "nominalTurnRadiusM": 538.8,
  "conservativeTurnRadiusM": 540.2
}
```

Recommended per-aircraft entry extension:

```json
{
  "aircraftId": 4,
  "entryCoordinate": { "latitude": 0.0, "longitude": 0.0, "altitude": 0.0 },
  "entryEtaS": 5.4,
  "dynamics": {
    "speedMps": 44.86,
    "observedTurnRadiusM": 538.8,
    "conservativeTurnRadiusM": 540.2,
    "turnRadiusScale": 1.0701,
    "maxYawRateDps": 7.967,
    "maxRollRateDps": 9.914,
    "lookaheadM": 242.4,
    "freezeYawDistanceM": 75.4
  }
}
```

Consumer rule:

- Planners that currently accept only `turn_radius_scale` can receive `1.0701` as a first pass.
- Planners that accept an absolute radius should receive `nominalTurnRadiusM` for normal planning and `conservativeTurnRadiusM` for safety/trim/rejoin.
- Monitoring-triggered live entries should use per-aircraft live speed/radius when available; use cohort values only as fallback.

## Detailed Findings By Flow

### 1. Normal Initial Mission Plan

Current hooks:

- `modules/mission_planning/MissionPlanner/runtime_settings.py`
  - `dubins_turn_radius_m` default is the absolute route-planning radius.
  - `select_fov_db_row_by_turn_radius()` selects FOV/SEP rows based on radius.
  - `get_runtime_prior_mission_profile()` exposes turn radius, FOV, SEP, and width together.
- `modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0303_0304.py`
  - Applies runtime radius into `d0303`.
- `modules/mission_planning/MissionPlanner/data_def/d0303.py`
  - `_turn_radius_m_for_speed()` returns runtime fixed radius when set, otherwise speed-table interpolation.
  - `_build_area_dubins_entry_wps()` builds transition waypoints from radius.
  - `_append_area_transition_links_inplace()` inserts area route transition links and recomputes ETA/ECF.
- `modules/mission_planning/MissionPlanner/data_def/dubins_turn_link.py`
  - `compute_turn_link()` uses radius for path feasibility, arc geometry, total length, and heading-change result.

Where learned radius should apply:

- Replace fixed `500m` default with `nominalTurnRadiusM=538.8m` only after route-output regression checks.
- Use `conservativeTurnRadiusM=540.2m` when the purpose is feasibility or avoiding too-tight transitions.
- Keep a path for speed-dependent radius because the report contains speed buckets and phase metrics. Absolute radius alone loses detail.
- Re-run FOV/SEP selection after radius changes, because a radius change also changes selected separation/width.

### 2. Next Collaboration

Current hooks:

- `modules/monitoring/logic/turn_radius_monitor.py`
  - Computes ideal/effective radius from static table and runtime/sim scale.
  - Produces alternate waypoints and predicted next-collab entry coordinates.
- `modules/monitoring/logic/next_collab_replan.py`
  - Reads `next_collab_turn_radius_scale` and `next_collab_entry_lead_time_s`.
  - Places `turnRadiusScale`, `entryLeadTimeS`, and `entryAircraftList` into detail payload.
- `modules/mission_planning/pipelines/next_collab_replan_pipeline_impl.py`
  - Resolves effective `turn_radius_scale`.
  - Passes it into line/area replacement planners.
- `modules/mission_planning/runtime/next_collab_division_runner.py`
  - Direct runner-level injection point for `turn_radius_scale`.
- `modules/mission_planning/runtime/next_collab_line_runner.py`
  - Scales the planner turn methods for line takeover paths.

Where learned radius should apply:

- Set detail `turnRadiusScale` from report or live analysis, not default `1.4`.
- Set `entryLeadTimeS` to about `5.4s` for this report unless live speed yields a better value.
- Include the report path and per-aircraft dynamics in each entry so the mission planner can stop guessing from a scalar.
- For Path-0/Path-1 T/T' branches, run before/after geometry comparison because a smaller scale than `1.4` can materially change branch choice.

### 3. Current Remaining And General Replan

Current hooks:

- `modules/monitoring/logic/current_remaining_replan.py`
- `modules/monitoring/monitoring_gui.py`
- `modules/mission_planning/pipelines/current_remaining_hybrid.py`
- `modules/mission_planning/pipelines/current_remaining_hybrid_replan.py`
- `modules/mission_planning/pipelines/general_remaining_hybrid_replan.py`

Observed behavior:

- Several flows already support `turnRadiusScale`.
- Some payloads pass it through correctly.
- Some rebuilds pass `turn_radius_scale=None` or default to `1.4`.

Where learned radius should apply:

- Treat `turnRadiusScale` as required context for current remaining/general replan.
- When reconstructing entries from snapshots, preserve source dynamics and entry ETA.
- Use `freezeYawDistanceM` to prevent inserting a new transition point too close to the current aircraft.

### 4. Reexecute / Reperform

Current hooks:

- `modules/monitoring/logic/collab_reexecute.py`
- `modules/mission_planning/pipelines/reexecute_first_mission_hybrid.py`
- `modules/mission_planning/pipelines/current_remaining_hybrid_replan.py`

Observed behavior:

- Reexecute detail focuses on trigger/source mission context.
- Some downstream wrappers support `turn_radius_scale`.
- Dynamics are not consistently carried from monitoring into the hybrid planner.

Where learned radius should apply:

- Add report-level dynamics block to the reexecute detail.
- Add `entryAircraftList[].dynamics` to preserve per-aircraft speed/radius/ETA.
- Use `conservativeTurnRadiusM` to test whether a reexecute handoff point is too close to the live aircraft or previous path.

### 5. Path Deviation

Current hooks:

- `modules/monitoring/logic/turn_radius_monitor.py`
- `modules/monitoring/logic/path_deviation_replan.py`
- `modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py`

Observed behavior:

- Monitoring stores useful fields such as alternate coordinate, alternate ETA, spiral state, ideal radius, actual radius, and turn rate.
- Pipeline consumption is still centered around coordinates and route rebuilding.

Where learned radius should apply:

- Use `lookaheadM` to choose projected alternate/resume entry points.
- Use `freezeYawDistanceM` to avoid churn near the current aircraft.
- Use `actualRadiusM` or `conservativeTurnRadiusM` to decide whether path stitching creates an infeasible turn.
- Use `alternateWaypointEtaS` as a timing seed, not only distance/speed recompute.

### 6. Prior / Lead Mission

Current hooks:

- `modules/mission_planning/pipelines/prior_mission_pipeline_impl.py`
- `modules/mission_planning/MissionPlanner/runtime_settings.py`

Observed behavior:

- Prior mission profile couples turn radius with FOV, SEP, width, speed, and loiter.
- Resume/release timing is still often distance/speed based after path generation.

Where learned radius should apply:

- Use learned radius when choosing the prior mission FOV profile.
- Use learned radius for approach/target waypoint spacing and loiter radius.
- Replace fixed prior insert ETA assumptions with turn-aware ETA where the path involves a heading change or reconnect.
- Check collaborative resume calls that currently pass `turn_radius_scale=None`.

### 7. Attack / Re-Attack

Current hooks:

- `modules/monitoring/logic/target_detection_replan.py`
- `modules/mission_planning/pipelines/mission_planning_attack_helpers.py`
- `modules/mission_planning/pipelines/attack_plan_pipeline.py`

Observed behavior:

- Attack helper computes standoff/attack waypoints and ETA mostly from straight-line distance and speed.
- Target ordering is distance/order/load based.
- Runtime attack trim radius exists, but it is not the same as observed flight dynamics.

Where learned radius should apply:

- Increase minimum standoff or preferred approach distance when observed radius makes direct attack entry unrealistic.
- Use yaw-rate and radius to estimate time-to-turn into attack heading.
- Use turn-aware ETA for target sequencing and friendly lead prediction.
- Use `freezeYawDistanceM` to avoid re-anchoring attack paths too close to the aircraft.
- Keep LAH and UAV dynamics separate. The current saved report values should not be blindly applied to LAH attack paths unless the per-agent report confirms matching LAH behavior.

### 8. Post-Attack Rejoin

Current hooks:

- `modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py`
  - `_estimate_turn_aware_eta_s()` explicitly adds `turn_deg * turn_radius_m` arc length.

Observed behavior:

- This is one of the strongest current insertion points because the function already has explicit radius semantics.
- Default radius is smaller than the observed UAV radius from this report.

Where learned radius should apply:

- Use `conservativeTurnRadiusM=540.2m` for UAV rejoin ETA and hold-time decisions.
- Preserve aircraft class separation: use UAV values for UAV rejoin, LAH values only when report evidence exists.
- Carry rejoin timing into decision support so options do not overstate post-attack availability.

### 9. Imaging Schedule / Sweep Buffers

Current hooks:

- `modules/mission_planning/pipelines/imaging_schedule_replan_pipeline_impl.py`
- `modules/mission_planning/pipelines/mission_path_trim.py`

Observed behavior:

- Replan output is often made valid by speed scaling and retained/buffered sweep points.
- Buffer logic is time/distance based.

Where learned radius should apply:

- Use speed buckets to tune retained sweep buffers.
- Use turn radius to avoid keeping an entry waypoint that cannot be rejoined without overshoot.
- Use roll/yaw response limits when deciding if an aggressive line-to-line or area-to-line transition is feasible.

### 10. Decision Support

Current hooks:

- `modules/decision_support/core/option_processing.py`
- `modules/decision_support/decision_support_gui.py`

Observed behavior:

- Options are currently more template/placeholder driven than dynamics-aware.
- 0701 push context does not appear to consume route feasibility or turn-aware ETA metadata.

Where learned radius should apply:

- Attach planner outputs such as `estimatedTotalSec`, `turnPrefixDurationS`, `entryEtaS`, and route-feasibility flags to option metadata.
- Penalize options that require impossible or high-risk turn transitions.
- Use dynamics-aware ETA for time-contraction, re-attack, post-attack, and next-collab option ranking.

## Shared Loader Proposal

Future implementation should avoid every pipeline reading the report differently. Add one resolver that returns a normalized, typed profile.

Suggested location:

- `modules/mission_planning/runtime/dynamics_profile.py`

Suggested responsibilities:

1. Locate the latest report relative to active scenario root:
   - `active_db_root / "LogAnalyzer_Reports" / "dynamics_analysis_latest.json"`
   - or `active_db_root.parent / "LogAnalyzer_Reports" / "dynamics_analysis_latest.json"` when the active root is `SBC3`.
2. Validate report identity and quality.
3. Return normalized fields:
   - `planner_turn_radius_scale`
   - `nominal_turn_radius_m`
   - `conservative_turn_radius_m`
   - `lookahead_m`
   - `freeze_yaw_distance_m`
   - `entry_lead_time_s`
   - `max_yaw_rate_dps`
   - `max_roll_rate_dps`
   - optional per-aircraft map.
4. Preserve source metadata for logs and reproducibility.
5. Never write sim PID/profile settings from mission planning.

## Implementation Checklist

Use this checklist when wiring the report into actual planning code.

### Report Contract

- [ ] Validate `kind == logAnalyzerDynamicsReport`.
- [ ] Validate `schemaVersion == logAnalyzer.dynamics.v1`.
- [ ] Validate `analysis.ok == true`.
- [ ] Validate `analysis.recommendations.quality == ok`.
- [ ] Log source report path into every replan detail that consumes it.
- [ ] Use cohort fallback only when no per-aircraft evidence exists.

### Normal Mission Planning

- [ ] Add a shared dynamics profile resolver.
- [ ] Compare `dubins_turn_radius_m=500m` vs `nominalTurnRadiusM=538.8m` on generated 0303/0304 paths.
- [ ] Verify FOV/SEP DB row changes after radius update.
- [ ] Verify area transition links do not increase impossible gaps or redundant anchors.
- [ ] Verify exported ETA remains monotonic.

### Next Collaboration

- [ ] Populate detail `turnRadiusScale` from report/live profile instead of default `1.4`.
- [ ] Populate `entryLeadTimeS` from `lookaheadM / speedMps`.
- [ ] Add per-aircraft dynamics to `entryAircraftList`.
- [ ] Regression-check Path-1 `T/T'` geometry.
- [ ] Regression-check Path-0 `T0/T0'` geometry.
- [ ] Confirm Gantt/timeline `horizonSec`, `etaSec`, and `estimatedTotalSec` remain consistent.

### Replan / Current Remaining / General Remaining

- [ ] Preserve report-derived scale through snapshot rebuilds.
- [ ] Preserve live entry ETA through `current_remaining_hybrid`.
- [ ] Apply `freezeYawDistanceM` before inserting near-aircraft transition waypoints.
- [ ] Store radius/entry metadata in debug artifacts for later replay.

### Reexecute / Reperform

- [ ] Add dynamics block to reexecute detail.
- [ ] Preserve `turnRadiusScale` through `reexecute_first_mission_hybrid`.
- [ ] Avoid defaulting to `None` when current remaining rebuilds entries.
- [ ] Validate that repeated execution does not create a tighter-than-radius reconnect.

### Path Deviation

- [ ] Use `actualRadiusM`, `idealRadiusM`, and `turnRateDegS` already computed by monitoring.
- [ ] Use `alternateWaypointEtaS` in route timing.
- [ ] Use `lookaheadM` for projected alternate entry.
- [ ] Use `freezeYawDistanceM` for near-current-state protection.
- [ ] Reject or adjust stitching that violates `conservativeTurnRadiusM`.

### Prior / Lead Mission

- [ ] Recompute prior FOV/SEP profile from learned radius.
- [ ] Tune prior loiter radius and approach distance.
- [ ] Replace fixed inserted prior ETA assumptions where heading change is present.
- [ ] Pass dynamics profile into collaborative resume calls.

### Attack / Re-Attack / Post-Attack

- [ ] Separate LAH and UAV dynamics before applying values.
- [ ] Use turn-aware ETA for attack approach and target sequencing.
- [ ] Use `conservativeTurnRadiusM` for UAV post-attack rejoin.
- [ ] Feed rejoin hold time into decision support.
- [ ] Use `freezeYawDistanceM` to protect near-aircraft attack re-anchors.

### Decision Support

- [ ] Attach route feasibility and dynamics-aware ETA to option metadata.
- [ ] Penalize impossible/high-risk turn transitions.
- [ ] Use planner timing instead of placeholder timing for 0701 ranking fields.

### Validation

- [ ] Compare before/after generated mission plans on the same saved scenario.
- [ ] Compare route plots for `T/T'` and `T0/T0'`.
- [ ] Check waypoint count, ETA monotonicity, FOV/SEP row, and path length deltas.
- [ ] Replay in sim with 0401 logging enabled.
- [ ] Re-run Log Analyzer Dynamics and verify observed radius/roll/yaw behavior did not regress.

## Main Gaps To Fix Later

- Static defaults are inconsistent: normal planning uses `500m`, next-collab defaults to table `* 1.4`, post-attack rejoin has its own default, and monitoring can sync sim profile scale.
- `turnRadiusScale` is currently scalar. It cannot express per-aircraft, per-speed, or per-phase behavior.
- Common ETA does not model turns unless upstream inserts turn waypoints or loiter data.
- Some replan/reexecute flows preserve scale, while others default or drop it.
- Decision support cannot yet compare options using dynamics-aware feasibility.
- Current report values should be class-aware. Applying UAV-derived radius to LAH logic is unsafe unless the log evidence supports it.

