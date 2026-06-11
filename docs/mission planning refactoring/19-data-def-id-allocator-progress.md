# Data Def And ID Allocator Progress

Date: 2026-06-04, Asia/Seoul

## Completed

- Wrote the initial `MissionPlanner/data_def` public API inventory below.
- Created the first engine namespace:
  - `modules/mission_planning/engine/`
  - `modules/mission_planning/engine/mission_generation/`
  - `modules/mission_planning/engine/mission_generation/id_allocation/`
- Moved `MissionPlanner/data_def/id_allocator.py` implementation to:
  - `engine/mission_generation/id_allocation/allocator.py`
- Preserved `MissionPlanner/data_def/id_allocator.py` as a compatibility proxy.
- Kept fallback `id_tracker.json` behavior pointed at the original `MissionPlanner/data_def/id_tracker.json`.
- Updated active mission-planning internal imports for replan triggers/runtime to the canonical allocator path.
- Added smoke coverage for:
  - old wrapper function/object identity
  - old wrapper `__file__` path
  - canonical `_LEGACY_STORE` path
  - `from data_def import id_allocator` from `MissionPlanner` cwd
  - `from data_def.id_allocator import ...`
  - `from modules.mission_planning.MissionPlanner.data_def.id_allocator import ...`
  - assignment forwarding for `_volatile_counters`
  - `_state` mutation after canonical `_state` rebind
- Sub-agent review found stale private-state forwarding in the first proxy version; fixed by making wrapper attribute reads always resolve through the canonical allocator.

## Data Def Public API Inventory

| Module | Public API / Contract | Move Risk |
| --- | --- | --- |
| `coord_transform.py` | `llh_to_xy`, `xy_to_llh`, `EARTH_RADIUS` | Low, pure coordinate helper. |
| `d0301.py` | `build_mission_plan` | High, emits 0301 and allocates missionPlan/IMP IDs. |
| `d0302.py` | `build_mission_packages` | High, emits 0302 and allocates IMP/mission/path IDs. |
| `d0303.py` | `SweepConfig`, `set_flyover_options`, dense-line-search metrics, `build_flight_plans`, many runtime constants | Very high, large 0303 builder with config/runtime/settings imports. |
| `d0304.py` | `apply_uav_eta_follow_speed_plan`, `build_lah_flight_plans_from_mrpk`, `build_lah_flight_plans_fixed` | High, 0304/LAH waypoint generation and ETA postprocess. |
| `d0304_RL.py` | `build_lah_flight_plans_rl` | Medium/high, optional RL dependencies. |
| `d0304 copy.py` | backup-style copy of d0304 builders | Deletion candidate only after reachability/manual check. |
| `dubins_path.py` | `mod2pi`, `dubins_shortest_path`, `sample_dubins_path` | Medium, path geometry helper. |
| `dubins_turn_link.py` | `Point2D`, `Pose2D`, `DubinsTurnLinkResult`, turn-link helpers, CLI `main` | Medium, may be a manual tool. |
| `filming_altitude_guard.py` | altitude metrics, waypoint normalization/sanitization | Medium, used by replan triggers and path builders. |
| `id_allocator.py` | ID allocator constants/state, reserve APIs, waypoint/path usage recorders | Very high, moved behind wrapper in this pass. |
| `id_allocator_0202.py` | `next_0202_individual_mission_id`, `next_0202_path_id`, `next_0202_waypoint_id` | High, depends on `id_allocator` and has separate tracker file. |
| `lah_attack_assistance.py` | raster/visibility/attack-point analysis helpers, CLI `main` | High, subprocess/manual utility potential. |
| `mission_helpers.py` | terrain cache, terrain lookup, mission row builder, map/dialog helpers, `now_ms_since_2000` | High, mixed UI/terrain/artifact helper. |
| `route_planner_algorithms.py` | `plan_route_linear` | Low/medium. |
| `search_speed.py` | `spacing_based_search_speed` | Medium, config fallback and planner tuning. |
| `__init__.py` | lazy `build_lah_flight_plans_fixed` export | Low, compatibility surface. |

## Import Contracts Found

- Bare imports still exist and are not fully shimmed yet:
  - `import AnS`
  - `from data_def import d0302, d0303, d0304`
  - `import config`
  - `from data_def.id_allocator import ...`
- External/public callers still use the old allocator path:
  - `run.py`
  - `modules/monitoring/logic/init_replan.py`
  - `MissionPlanner/AnS/mission_pipeline.py`
  - `MissionPlanner/data_def/*`
  - `MissionPlanner/planning_enhanced/io/export_0301.py`
- These are intentionally preserved by the wrapper until the broader bare-import shim TODO is closed.

## Verification Passed

- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `python -m compileall modules\mission_planning\engine modules\mission_planning\MissionPlanner\data_def\id_allocator.py modules\mission_planning\runtime\ids\replan_reservation.py modules\mission_planning\mission_planning_gui.py modules\mission_planning\replanning\triggers\attack\pipeline.py modules\mission_planning\replanning\triggers\post_attack\pipeline.py modules\mission_planning\replanning\triggers\prior\pipeline.py modules\mission_planning\replanning\triggers\next_collab\pipeline.py "docs\mission planning refactoring\smoke_import_contract.py"`
- Confirmed no `id_tracker.json` was created under `engine/mission_generation/id_allocation/`.

## Residual Risk

- `run.py` still writes reset files through `Path(id_allocator.__file__).parent / "id_tracker.json"`, while the allocator can switch active store to `DSS_Internal/id_tracker.json` through `db_paths`. This behavior existed before the physical move, and this pass preserved the old `__file__` path rather than changing reset semantics. Treat it as a separate ID reset contract before changing launch/reset behavior.

## Progress Snapshot

- Starting point before this pass: 34 / 99 complete, 65 remaining, 34.3% complete.
- Starting Phase 5 point: 0 / 6 complete, 6 remaining, 0.0% complete.
- Overall roadmap after these checkboxes: 36 / 99 complete, 63 remaining, 36.4% complete.
- Phase 5: 2 / 6 complete, 4 remaining, 33.3% complete.

## Next Candidates

- Build a compatibility shim strategy for bare `AnS`, `data_def`, and `config` imports.
- Inventory `planning_enhanced` import graph before any physical move.
- Do not move `d0301-d0304` until artifact signature snapshots and representative ID/link checks exist.
