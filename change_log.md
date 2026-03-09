# Change Log

## 2026-03-10

### Version 1.2.0
- Dashboard 버전을 `v1.2.0`으로 상향했습니다.
- Dashboard 상단 최근 업데이트 날짜를 `26-03-10` 기준으로 갱신했습니다.

### Mission Planning Runtime Warm-Up
- 임무계획 GUI 시작 시 일반 임무계획, 선행 임무 재계획, 공격 임무 재계획 경로를 미리 import/warm-up 하도록 정리했습니다.
- `0201`, `0203` 최신 입력이 갱신되면 planner runtime cache를 다시 준비해, 초기 임무계획과 재계획이 바로 시작되도록 맞췄습니다.
- UAV 파라미터와 planner runtime 초기화가 실제 계획 시점이 아니라 background warm-up에서 선반영되도록 조정했습니다.

### Mission Planning Latency Reduction
- `0301` 전송 직후 `0305 완료`를 다음 event-loop tick에서 바로 보내도록 변경해 불필요한 완료 지연을 제거했습니다.
- GUI에 최근 계획 시간 표시를 추가해, 임무계획 체감 지연을 바로 확인할 수 있게 했습니다.
- 초기 임무계획 준비 구간에서 발생하던 import/입력 파일 materialize 비용을 사전 준비 경로로 이동했습니다.

### ID Allocation And Replan Integrity
- `id_allocator`에 연속 ID block 예약 helper를 추가하고, `0302`, 초기 임무계획, 선행 재계획, 공격 재계획에서 일괄 할당을 사용하도록 변경했습니다.
- `individualMissionPackageID`, `individualMissionID`, `pathID`를 한 건씩 파일에 기록하던 경로를 묶음 예약으로 바꿔 ID 재발급 비용을 줄였습니다.
- ID 규칙 자체는 유지해, 임무별 구분과 MissionPlan-IMP-IndividualMission-FlightPath 연결 관계가 기존 방식대로 보존되도록 맞췄습니다.

### Attack / Prior Replan Stability
- 선행 임무 재계획 pipeline에 warm-up entry를 추가해 lazy dependency 로딩을 앞당겼습니다.
- 공격 임무 재계획도 warm-up 경로를 분리해 import 준비 상태를 유지하도록 정리했습니다.
- 공격 배제 variant 생성 시 UAV resume package를 별도 MissionPlan으로 정리하고, 새 MissionPlanID를 발급해 원본 계획과 충돌하지 않도록 보강했습니다.

## 2026-03-09

### Version 1.1.0
- Dashboard 버전을 `v1.1.0`으로 상향했습니다.
- Dashboard 상단 최근 업데이트 날짜를 `26-03-09`로 갱신했습니다.

### 1) 정찰임무 쪼개기 로직 추가
- `test_mission_planning`에서 검증하던 정찰 임무 분할/경로 생성 로직을 `modules/mission_planning` 실행 경로로 이식했습니다.
- `0302` 생성 전에 영역 분할, expected path, bearing, 속도 계산 흐름을 production 파이프라인과 연결했습니다.
- 영역 임무에 대해 `Bearing_Par_Sweep Mode`, `Bearing_Ver_Sweep Mode`, `Nadir Mode`를 분리해 선택할 수 있게 구성했습니다.

### 2) 임무계획 Dashboard 수정
- `run.py` Dashboard와 각 모듈 GUI의 공통 스타일을 정리해 카드, 버튼, 입력창, 표 가독성을 개선했습니다.
- 모듈 창이 완전히 겹치지 않도록 실행 위치를 계단식으로 분산했습니다.
- 임무계획 GUI에서 불필요한 하단 로그/보조 탭을 줄여 화면 구성을 단순화했습니다.
- `main_MP`에 알고리즘 설정 진입점을 추가하고, 설정 버튼 위치를 상단으로 이동해 접근성을 높였습니다.

### 3) 임무계획 작동 시간 줄이기 (3초 이내 목표)
- `0302 -> 0303/0304` 생성 경로를 재정리해 불필요한 중복 처리와 하드코딩 분기를 줄였습니다.
- 병렬 FlightPath 생성과 경량 검증 경로를 유지하면서 초기 임무계획 시간 단축을 목표로 파이프라인을 정리했습니다.
- 임무계획 실패 시 즉시 중단만 하지 않고 원인 요약/공지까지 이어지도록 실패 처리 비용을 구조화했습니다.

### 4) 스케줄링 추가/연동
- `main_MP` 테스트 경로와 실제 임무계획 GUI가 동일한 스케줄링 설정을 사용하도록 일원화했습니다.
- `Review Area`, `0303/0304 planning`, `turn step`, `area review max segment` 등이 동일 설정 파일을 기준으로 동작하도록 맞췄습니다.
- 스케줄링 결과가 `0302/0303` 생성에 반영되는 경로를 정리해 테스트/실모듈 결과 차이를 줄였습니다.

### 5) 임무계획 입력 파일 누락/오류 대응 강화
- `0201`, `0203` 파일이 없거나 형식이 잘못된 경우, 단순 실패로 끝나지 않고 `0001` 공지 메시지로 원인을 한글 요약해 송신하도록 강화했습니다.
- 비행경로 일부 누락, IMP 생성 실패, FlightPath 생성 실패 등 핵심 실패 케이스를 임무계획 GUI에서 구분해 기록하도록 보강했습니다.
- 내부 상세 로그는 유지하고, 운용자는 코드명 없이 원인 중심 안내를 받을 수 있게 정리했습니다.

### 6) 스케줄 기반 속도 / DB 기반 임무계획 연동
- `uav_params.json`을 단일 설정 원본으로 정리해 `main_MP`와 실제 `mission_planning_gui.py`가 같은 값을 사용하도록 맞췄습니다.
- 일반 `Line/Area` 임무는 DB 기반 `FOV/SEP/VEL` 선택 흐름을 유지하고, `Custom` 모드에서는 하드코딩 수동값을 직접 사용하도록 분리했습니다.
- Nadir 임무는 UAV 고도를 기준으로 `SEP <= 고도` 조건을 만족하는 DB 행 중 최대 `FOV`를 선택하도록 변경했습니다.

### 7) 다른 임무계획 버전 추가
- 기존 로직은 `Bearing_Par_Sweep Mode`로 고정 보존했습니다.
- 새 `Bearing_Ver_Sweep Mode`를 추가해, 영역 분할/경로 생성 방향을 기존 모드와 분리했습니다.
- 새 `Nadir Mode`를 추가해:
  - bearing 기준 직접 분할,
  - `Review Area` 미사용,
  - area `patternType=3`,
  - nadir BF 경로를 실제 path로 사용하는 흐름을 구성했습니다.
- BF Nadir planner에서 빈 경로가 발생할 때의 예외를 보강하고, 필요 시 단순 nadir planner로 fallback 하도록 안전장치를 추가했습니다.
- `Nadir Mode`의 area waypoint는 `Fly-over` pass type을 사용하도록 조정했습니다.

### 임무계획 속도 최적화
- `uav_params.json` 런타임 설정 로딩에 변경 감지 기반 캐시를 추가해 반복 파일 읽기를 줄였습니다.
- `0301` 생성 후 임시 파일을 다시 읽지 않고, 메모리상의 MissionPlan 결과를 바로 재사용하도록 정리했습니다.
- `IMP` 수집/`0302` 저장/`0303·0304` 저장을 안전한 범위에서 병렬 처리하도록 바꿨습니다.
- `main_MP`의 `0303/0304` 생성도 UAV/LAH 경로를 병렬 생성하도록 정리했습니다.
- `ID allocator`의 sleep/retry는 파일 잠금 보호용이므로 유지했고, 임무계획 핵심 경로에서는 불필요한 대기 추가 없이 최적화했습니다.

## 2026-02-25

### Dashboard Patch Notes Source + Version Bump
- Dashboard version updated: `v1.0.1`.
- Replaced dashboard patch-note source from `version_notes.txt` to `change_log.md`.
- Updated dashboard footer card title to `Change Log`.

### Mission Planning Altitude Update
- Updated UAV/LAH altitude layering to avoid collision by aircraft index:
  - layer set: `610m`, `620m`, `630m` (repeated by aircraft order).
- `d0303` (UAV flight path):
  - Added per-aircraft altitude offset logic.
  - Added mission-level ground reference (median terrain from mission geometry).
  - WP altitude now uses `mission_ground_median + aircraft_layer_offset`.
  - Applied to line/area/move/entry waypoint generation.
- `d0304` (LAH flight path):
  - Added same per-aircraft altitude layer logic for manned aircraft.
  - Added mission-level ground median usage for LAH route WPs.
  - Start/RTB altitude generation now follows aircraft layer offset.

### Related Prior Updates (same working session)
- Area entry waypoint direction fixed to align with **final simplified area first-leg vector**.
- UAV parameter handling normalized to keep one authoritative settings source:
  - `modules/mission_planning/MissionPlanner/uav_params.json`
  - Added `area_nadir_fov_deg` support in GUI/manager.

### Monitoring 0402 Target Replan Filter/Parallel Update
- Hardened 0402 target candidate filtering in `modules/monitoring/logic/target_detection_replan.py`:
  - Ignore `targetID <= 0`.
  - Ignore destroyed targets (`isDestroyed=true`).
  - Ignore `isUsed != 0` / `isIgnored != 0`.
  - Ignore malformed/invalid coordinates (including `(0,0)` sentinel).
- Added actionable filtering before dedupe/replan dispatch to avoid null/noise-triggered 0902.
- Kept multi-target handling; same 0402 can generate multiple 0902 payloads (up to attack slot count).
- Added overflow policy for `>2` actionable targets:
  - Limit dispatch to current attack slots.
  - Mark overflow targets as `isUsed=1` and log overflow handling.
- Updated destroyed-state blocking:
  - If one watcher marks destroyed but another still reports alive for same target, alive state is now allowed.

## 2026-02-26

### Prior Mission(0202) Target-Tracking ICD Alignment
- Updated `modules/monitoring/logic/prior_mission_replan.py` for missionType=2 prior missions:
  - Require `targetOrientation.targetID` (invalid missionType=2 entries are skipped with log).
  - Include `targetOrientation` in outgoing `0902.priorMissionList` (ICD-aligned payload shape).
  - Keep `replanDetail.targetOrientation/targetID` for mission-planning pipeline compatibility.
- Verified `modules/mission_planning/prior_mission_pipeline_impl.py` target-tracking flow:
  - Resolves target coordinate from `DSS_Internal/targetInfo.json` by `targetID`.
  - Uses watcher UAV if available for tracking mission assignment.
  - Writes auto-tracking (`filmingProperty.autoTracking.targetID`) with loiter tracking waypoint.
### Recon Option Revert (정찰특화 경로강제 비활성화)
- Reverted special area-Nadir forcing for option_code == 4 in modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py.
- 정찰특화 now uses the same mission pattern flow as other options (no forced patternType=3).
- Added runtime log line for traceability: 정찰특화: pattern 강제 없음 (기본 계획 로직과 동일).

### Entry Offset Update (2번째 임무부터 300m)
- Updated modules/mission_planning/MissionPlanner/data_def/d0303.py:
  - Added SWEEP_ENTRY_OFFSET_FOLLOWON_M = 300.0.
  - For each aircraft, first mission keeps existing rule:
    - takeover anchor exists: SWEEP_ENTRY_OFFSET_TAKEOVER_M (500m)
    - no takeover anchor: SWEEP_ENTRY_OFFSET_M (500m)
  - From second mission onward, both line/area entry offset now fixed to 300m.
### Attack Resume Sweep Trim 안정화
- Updated modules/mission_planning/mission_path_trim.py:
  - 	rim_waypoints_by_sweep_points(..., preserve_waypoints=True)에서 lineSearch coordinateList가 1점만 남는 케이스를 방지.
  - 가능한 경우 각 waypoint의 sweep 좌표를 최소 2점 유지하도록 변경.
- 영향:
  - 공격 재계획 시 추적 비대상 UAV의 resume 임무에서 lineSearch가 단일 좌표만 남는 현상 완화.
  - 후속 ETA/경로 계산 안정성 개선.- Revised trim behavior again:
  - Resume sweep now follows the true remaining coordinate sequence after cut.
  - If a waypoint sweep is fully consumed by cut points, that waypoint is removed from resume path.
  - If only one sweep point remains, lineSearch is removed and waypoint is kept as transit point.
  - Applied commonly via mission_path_trim.py, so both attack and prior resume flows use the same rule.
### Follow-on Entry 임시 비활성화
- Updated modules/mission_planning/MissionPlanner/data_def/d0303.py.
- Added switch: ENABLE_FOLLOWON_ENTRY_WP = False.
- Behavior:
  - 첫 번째 임무: 기존 entry 생성 규칙 유지.
  - 두 번째 임무부터: entry waypoint 생성 비활성화(line/area 공통).
- Re-enable 방법:
  - ENABLE_FOLLOWON_ENTRY_WP = True로 변경하면 기존 2번째 이후 entry 생성 복구.

### Area Path Direction (Next-Mission Look-Ahead)
- Updated `modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py`.
- Area bearing now considers next mission continuity.
  - Previous behavior: based on `prevPoint -> current area center` only.
  - New behavior: blends `prev -> center` and `center -> next` vectors.
  - If vectors are opposite, next-mission continuity is prioritized.
- `split_mission_into_subareas(...)` extended with `next_pt`.
- Added `nextPoint` metadata for area sub-missions (trace/debug).

### Database Root Cleanup
- Common message push/receive fallback paths were changed from project-root `database/` to `temp/database/`.
- Updated generated message adapters under `modules/common/push/` and `modules/common/receive/`, plus `modules/common/make_message_push.py` and `modules/common/make_message_receiver.py`.
- Moved existing root `database/` folder into `temp/database/`.
- Verified with `python -m compileall` on common push/receive modules.

### GUI Refresh (2026-03-08)
- Refreshed the shared desktop QSS for the dashboard and module GUIs with a cleaner light theme, softer cards, updated tabs, table styling, and improved button/input visuals.
- Applied the shared stylesheet to `modules/mission_planning/mission_planning_gui.py`, `modules/monitoring/monitoring_gui.py`, `modules/decision_support/decision_support_gui.py`, and `modules/info_manage/info_manage.py` so child windows launched from `run.py` use the same look.
- Updated shared message tables in `modules/common/Tabs/csc_tab_base.py` to use non-uniform column sizing: narrower `Message ID`, fixed action/data columns, and wider name/status columns.
- Removed the legacy bottom log panes from the shared CSC tab layout and the dashboard module card widget (`app/widgets/module_with_log.py`) to reduce UI clutter and avoid keeping large text buffers in memory.
- Adjusted dashboard layout spacing/cards in `app/ui/main_window.py` and removed per-button inline styling so the shared QSS controls the appearance consistently.

### Mission Planning GUI Simplification (2026-03-08)
- Removed the `???? Log` and `?? ???` tabs from `modules/mission_planning/mission_planning_gui.py`.
- The mission planning GUI now shows only the main CSC tab.
- Internal pipeline logging/file generation remains intact; only the extra GUI tabs were removed.

### Info Manage 0101 Selector Layout (2026-03-08)
- Adjusted the `0101` system-mode selector cell in `modules/common/Tabs/manage_info_tab.py`.
- Widened the action column for the info-management TX table, set a taller row for `0101`, and increased the combo minimum width/height so the mode text is not clipped.
- Explicitly reset the info-management TX/RX table headers after init to keep the visible labels stable.

### Mission Planning Folder Reorganization (2026-03-08)
- Reorganized `modules/mission_planning` into three implementation groups: `pipelines/`, `runtime/`, and `ui/`.
- Kept the original top-level module filenames as thin compatibility wrappers so existing imports continue to work.
- Moved attack/prior replan logic into `pipelines/`, runtime state/log/cache helpers into `runtime/`, and GUI support helpers into `ui/`.
- Added `modules/mission_planning/README.md` to document the structure and the compatibility rule.
- Added `modules/mission_planning/_paths.py` so moved modules resolve `MissionPlanner/data_def` and project-root paths consistently from their new locations.
- Updated internal imports in the moved pipeline/runtime modules to use the new package structure directly.
- Validation: `py_compile` passed for the reorganized modules and `mission_planning_gui.py`; direct imports of the attack/prior pipeline entrypoints also succeeded.

### Mission Planning Folder Review Pass (2026-03-08)
- Documented the active runtime path in `modules/mission_planning/README.md` and added `modules/mission_planning/MissionPlanner/README.md`.
- Explicitly separated safe-to-reorganize helpers (`pipelines/`, `runtime/`, `ui/`) from high-risk core generation code (`MissionPlanner/AnS/`, `MissionPlanner/data_def/`, `mission_planning_gui.py`).
- Fixed the misleading package docstring in `modules/mission_planning/MissionPlanner/tools/__init__.py`.
- Validation: `py_compile` passed for the reorganized mission-planning entry files and helpers.

### Mission Planning Direct-Script Import Fix (2026-03-08)
- Fixed `modules/mission_planning/mission_planning_gui.py` so running it by file path adds the project root to `sys.path` before wrapper imports.
- Switched the GUI-side JSON helper import to the local compatibility wrapper (`from json_io import write_json`) to reduce package-path fragility.
- Updated the top-level compatibility wrappers in `modules/mission_planning/` to bootstrap the repo root themselves and mirror the full target module namespace, including underscore-prefixed helpers.
- Validation: `py_compile` passed for the GUI and wrapper modules; a direct-script style import simulation of `mission_planning_gui.py` also passed.

### Dashboard GUI Window Cascade (2026-03-08)
- Added `position_window_from_env(...)` to `modules/common/gui_style.py`.
- `run.py` now launches module GUIs with per-role `KU_WINDOW_OFFSET` values so mission/monitor/decision/info windows do not open on top of each other.
- Updated module GUI entrypoints to apply the offset after `show()`.
- Mirrored the same offset passing in `app/ui/main_window.py` for direct per-role launches.
- Validation: `py_compile` passed for the dashboard and affected GUI modules.

### Mission Planning Enhanced Line/Area Pipeline Migration (2026-03-08)
- Added `modules/mission_planning/MissionPlanner/planning_enhanced/` as the production home for the stronger line/area planning code migrated from `test_mission_planning`.
- Migrated and adapted the following planning stages into production:
  - split/assignment pipeline
  - expected-path generation
  - expected-velocity selection
  - deterministic mission type/pattern decision
  - enhanced 0302 export
- `MissionPlanner/AnS/mission_pipeline.py` now keeps the same `run_divide_and_pattern(...)` entrypoint but delegates to the enhanced pipeline implementation.
- `MissionPlanner/data_def/d0303.py` now consumes per-mission `FOV`/`SEP` hints produced by the enhanced 0302 export so 0303 reflects the upgraded planning results.
- Validation:
  - direct `run_divide_and_pattern(...)` smoke passed
  - 0302 → 0303 → 0304 generation smoke passed on sample `0201/0203`
  - no runtime dependency on `test_mission_planning` remains in the production planning path

### Mission Planning Algorithm Config Tab (2026-03-08)
- Added `알고리즘 설정` tab to `modules/mission_planning/mission_planning_gui.py`.
- New widget file: `modules/mission_planning/ui/algo_config_tab.py`.
- Settings are stored in `modules/mission_planning/MissionPlanner/uav_params.json` and applied live to `d0303` / mission-planning config modules.
- Exposed planning controls include:
  - route planner
  - cruise speed / turn step / altitude
  - sweep separation / FOV / nadir FOV
  - merge tolerance / interpolation / spacing thresholds
  - DB-based automatic FOV/SEP selection
  - area-review refinement
  - fly-over options

### Mission Planning Import/Planner Compatibility Fixes (2026-03-08)
- Updated `modules/mission_planning/mission_planning_gui.py` imports to support both:
  - direct script execution by file path
  - package import (`modules.mission_planning.mission_planning_gui`)
- Fixed `MissionPlanner/data_def/route_planner_algorithms.py` so DTA planner calls `UAVMissionPlanner.plan_route_only(...)` with only the parameters supported by the currently loaded implementation.
- Hardened enhanced pipeline `VehicleStatus` filtering to prefer the `VehicleStatus/status.json` adjacent to the active `0201` path before falling back to the global DB path resolver.

### main_MP Tester Migration (2026-03-08)
- Migrated the tester GUI stack from `test_mission_planning/assignment/app` into production under:
  - `modules/mission_planning/MissionPlanner/planning_enhanced/gui/`
  - `modules/mission_planning/MissionPlanner/planning_enhanced/map/`
  - `modules/mission_planning/MissionPlanner/planning_enhanced/io/`
  - `modules/mission_planning/MissionPlanner/planning_enhanced/scheduling/`
- `modules/mission_planning/MissionPlanner/main_MP.py` now launches the migrated tester window first, while keeping the legacy window as fallback.
- Added MILP scheduling support and tester-side 0303/0304 export helpers to the migrated package.
- Added a tester-side `Settings` dialog that opens the shared `MissionAlgoConfigTab`, so `uav_params.json` can be edited directly from the tester.

### 0303 Speed Hint Wiring (2026-03-08)
- Updated `modules/mission_planning/MissionPlanner/data_def/d0303.py` to consume `individualMissionInfo.SPEED` when present.
- The value is interpreted as `km/h` from enhanced 0302 export and converted to `m/s` before route/search-speed planning.
- This lets enhanced 0302 speed selection flow into actual 0303 path generation.

### main_MP Launcher Simplification / Exactness Check (2026-03-09)
- Replaced `modules/mission_planning/MissionPlanner/main_MP.py` with a small tester launcher aligned to `test_mission_planning/assignment/main.py`.
- `main_MP.py` now prepares `sys.path` and launches `planning_enhanced.gui.MainWindow` directly, instead of importing the legacy monolithic GUI file first.
- Verified core tester parity against `test_mission_planning` on sample `0201/0203`:
  - split piece count: same
  - direction/bearing fields: same
  - generated expected paths: same
- Remaining intentional difference:
  - `planning_enhanced/type_decider/logic.py` is still the productionized deterministic variant, not the original random-sampling tester variant.

### Area Sweep Bearing Update (2026-03-09)
- Updated `modules/mission_planning/MissionPlanner/data_def/d0303.py` so area sweep generation now prefers the piece move-bearing (`phaseMoveBearing_deg` / `MOVE_BEARING`) over the split-axis bearing.
- Effect: non-nadir area sweep lines are generated parallel to the mission flow direction, while the actual flight waypoint remains the offset anchor that references the sweep through `lineSearch`.
- Fallback behavior is preserved: if move-bearing metadata is missing, the previous `bearing_deg` path is still used.

- 2026-03-09
  - Adjusted area 0303 sweep ordering in `modules/mission_planning/MissionPlanner/data_def/d0303.py` so sequential split-area paths choose their first sweep using the previous mission end, not just anchor order.
  - Stored previous aircraft tail using the effective end of the last `lineSearch` instead of the last waypoint anchor, improving zig-zag continuity across adjacent area pieces.
  - When merge mode is `all`, rebuilt merged `lineSearch.coordinateList` from the re-oriented sweep items so the exported path order matches the selected WP order.

- 2026-03-09
  - Added selectable mission-planning preset support to `modules/mission_planning/ui/algo_config_tab.py`.
  - Registered the current production area/line configuration as `Bearing_Par_Sweep Mode` and persisted it via `preset_key` in `modules/mission_planning/MissionPlanner/uav_params.json`.
  - Preset selection is now available both in the mission planning GUI algorithm tab and the `main_MP.py` settings dialog.

- 2026-03-09
  - Clarified DB vs manual sweep parameter usage in `modules/mission_planning/ui/algo_config_tab.py`.
  - When DB mode is enabled, the settings UI now marks base SEP/FOV as fallback values and explains that general Line/Area missions use DB-selected SEP/FOV/VEL.
  - Base SEP/FOV edits stay available in `Custom` preset, while preset mode keeps the current set visible as fixed values.

- 2026-03-09
  - Unified mission-planning runtime settings through `modules/mission_planning/MissionPlanner/runtime_settings.py`.
  - `main_MP` and `modules/mission_planning/mission_planning_gui.py` now read the same `uav_params.json` through the shared helper.
  - `main_MP` review-area max segment and 0303 turn-step now follow the same settings file as the production mission planner.

- 2026-03-09
  - Strengthened mission-planning failure reporting in `modules/mission_planning/mission_planning_gui.py`.
  - Added user-facing failure classification for 0001 notices so missing `0201/0203`, invalid input data, IMP generation failures, flight-path failures, and attack-pipeline failures now send concise Korean notices.
  - Added internal traceback logging on unexpected pipeline exceptions while keeping the 0001 notice free of raw exception details.

- 2026-03-09
  - Added a second mission-planning preset, `Bearing_Ver_Sweep Mode`, without changing the existing `Bearing_Par_Sweep Mode`.
  - Unified the new mode through shared runtime settings so `main_MP.py`, enhanced 0302 export, area review, and production 0303 path generation all read the same `area_sweep_mode` value.
  - `Bearing_Par_Sweep Mode` keeps area sweep/path behavior aligned to bearing-parallel flow.
  - `Bearing_Ver_Sweep Mode` flips the area sweep basis to the perpendicular/split-axis side and flips review-area cut direction accordingly.

- 2026-03-09
  - Fixed `Custom` mission-planning mode so general Line/Area planning no longer uses DB-selected FOV/SEP/VEL.
  - `Custom` now forces manual/hardcoded sweep values at runtime, even if `enhanced_auto_fov_from_db` remains `true` in a stale settings file.
  - The settings UI now disables the DB toggle in `Custom` and clearly indicates that `Custom` uses direct values.

- 2026-03-09
  - Corrected `Bearing_Ver_Sweep Mode` so area sweep lines are generated perpendicular to the mission bearing.
  - `Bearing_Par_Sweep Mode` remains bearing-parallel, while `Bearing_Ver_Sweep Mode` now exports and builds 0303 area sweeps using `moveBearing + 90°`.

- 2026-03-09
  - Fixed `modules/mission_planning/mission_planning_gui.py` so applying algorithm settings no longer strips `bearing_ver_sweep` and `area_sweep_mode` back to legacy values.
  - The production GUI now preserves `preset_key`, `area_sweep_mode`, and `Custom` DB-disable behavior when it rewrites `uav_params.json`.

- 2026-03-09: `Bearing_Ver_Sweep`?? `flightpath_missing_ids`? ?? ?? ??. Review Area ? ?? area piece? ?? polygon(?? ??)?? 0302? ?? 0303 sweep? ?? pathID? ???? ??? ????. 0302 area export? review-subdivided piece? `rawCoordinateList`? ?? ????, 0303 area sweep? sweep endpoint ??/?? polygon fallback? ????. ?? ??? ?? UAV mission 33? -> 0303 path 33?, missing pathID 0? ??.
- 2026-03-09: `Bearing_Par_Sweep`?? sweep ?? ??? ? entry ?? ?? ?? ??? ? ?? ???? ??. ?? tail? ??? ?? ? ??? ??? 1/2 ?? ?? ??? ????, ? ??/?? tail ?? ???? ?? offset ??? ??.
- 2026-03-09: `Bearing_Par_Sweep`?? entry ?? ? ?? ?? WP? ?? ?? ???? ?? ????? ??. entry offset? 20% ??(?? 120m)?? ???, ???? ????(coordinateOrientation)? ?? ???? ??? ??.
- 2026-03-09: `main_MP` ??? ???? ??. `0303`? `0303 Route`? `0303 Sweep`?? ?? ?? ???? ??, route(waypoint ???)? ?? sweep(lineSearch/point-target)? ?? ? ? ?? ??.
- 2026-03-09
  - Added `Nadir Mode` preset to mission-planning settings.
  - In `Nadir Mode`, area division now skips the two-stage split and divides the area directly into UAV-count strips using the mission bearing line.
  - Area export now forces `patternType=3` so 0303 uses the nadir BF planner for area missions.
  - `main_MP` and the production mission-planning GUI now share the same `Nadir Mode` setting through `uav_params.json`.
- 2026-03-09
  - In `Nadir Mode`, Review Area is now skipped in both the enhanced pipeline and `main_MP` UI.
  - Hardened the BF nadir planner so empty CPP results no longer crash path generation.
  - Added fallback from `area_nadir_bf_planner.py` to the simpler nadir strip planner when the BF planner returns an empty path.
- 2026-03-09
  - Updated `Nadir Mode` so area nadir FOV is chosen from the DB using the UAV altitude layer as the SEP threshold (`sep <= altitude`, then max FOV).
  - `Nadir Mode` 0303 now passes the selected nadir FOV and the actual UAV altitude layer into the BF nadir planner, so the final nadir path is generated from the true nadir sweep spacing basis.

- 2026-03-09
  - ?? ???? ?? ??? ??? ?? 0303/0304? Waypoint ID? ? ?? ??? ???? ??? ?? ?? ???? ????.
  - ???? `[SWEEP]` ??? ?? ? ???? ? ?? ????? ?? ?? ????? ???.
  - DEM ???? ??? exact-coordinate ??? ????, `UAVMissionPlanner.plan_route_only()`? UTM transformer ??? zone ?? ??? ?????? ?????.
  - ?? ?? 0303/0304 ?? ?? ??? ? `514ms -> 479ms` ???? ???.

- 2026-03-09
  - `UAVMissionPlanner.plan_route_only()`? ????? ???? ?????.
  - UTM transformer? zone ?? ??? ?????, ????? ? ?? ??? ?? ?? ??? ??? ???/???? ???? ???.
  - ?? ?? `plan_route_only()` 200? ?? ??? ? `1.328s -> 1.246s`, ?? 0303/0304 ?? ??? ? `479ms -> 453ms` ???? ???.
  - ?? ??? ?? ?? waypoint ???? ??? ??? ??? ????.

- 2026-03-09
  - ?? ??? ??(2~3?) ?? ??? variant ?? precompute? ????.
  - ?? ???/???? ???? ????, ?? ?????? `run_divide_and_pattern -> 0301 -> FlightPath` ?? ??? ?? ????.
  - MissionPlan ID ?? ??? ?? ??? ???? variant ???? ?? ??? ?? ??? ????.

- 2026-03-09
  - ?? ??? 3??? ?? ??? ???? ??? ?? ????.
  - `??/?? ??`? `Bearing_Par_Sweep Mode`, `?? ??`? `Nadir Mode`, `?? ??`? `Bearing_Ver_Sweep Mode`? ????.
  - ? ??? ?? ????? ????, ??/???? ????? ???? ???.

- 2026-03-09
  - ???? ?? UI? ???(0304) ?? ?? ??? ????.
  - ??? `Normal` / `????` ?? ??? ??, 0304 ?? ??? ?? ?? ???.
