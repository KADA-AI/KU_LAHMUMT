# Deletion Candidates

이 문서는 삭제 확정 목록이 아니다. 모든 항목은 삭제 전 검증이 필요하다.

## 삭제 후보

| 후보 | 근거 | 삭제 전 검증 |
| --- | --- | --- |
| root compatibility wrappers 대부분 | root 파일 다수가 `pipelines/`, `runtime/`, `ui/`로 re-export만 수행 | 외부 import 검색, launcher/script 검색, 한 파일씩 삭제 후 smoke |
| `legacy/wrappers` | archived wrapper bucket | manual workflow 또는 외부 `python -m` 사용 확인 |
| `legacy/compat_packages` | old compatibility package bucket | 외부 import 검색 |
| `legacy/apps`, `legacy/tests`, `legacy/docs`, `legacy/static` 일부 | active runtime 밖으로 분리된 archived area | 문서/수동 테스트/데모 용도 확인 |
| `logic_test/*/output` JSON | generated/sample output처럼 보이며 active code 참조 미확인 | golden fixture 여부 결정 |
| `legacy/tests/*/output` JSON | generated/sample output처럼 보임 | fixture 전환 또는 삭제 결정 |
| `MissionPlanner/data_def/d0304 copy.py` | backup-style filename, active imports는 `d0304.py` | dynamic path loader, manual workflow 확인 |
| `MissionPlanner/AnS/Training/TensorBoard Logs*` | tracked training event files, code 참조 미확인 | 재현성/보고자료 필요 여부 확인 |
| duplicate visualizer copy | `MissionVisualizer/main_visualizer.py`와 `MissionPlanner/tools/main_visualizer.py`가 wrapper로 전환됨 | compatibility period 뒤 wrapper 제거 여부 결정 |
| duplicate `Dubins_Path.py` under UAV pattern tools, excluding active `Nadir_BF` chain | 여러 prototype folder에 동일/유사 파일 존재. 단, `MissionPlanner/tools/UAV_pattern/Nadir_BF/**`는 active `d0303.py` 경로에서 쓰일 수 있어 삭제 후보에서 제외 | relative import 및 manual tool 사용 확인 |
| typo/backup-style prototype files | `Inerval_Round_Flight_BF.py`, `testtest.py`, `main_example.py` 등 | active import와 수동 도구 사용 확인 |
| `__pycache__` if tracked | source control에 있을 필요 없음 | 실제 tracked 여부 확인 후 삭제 |

## 삭제 금지 또는 보류

| 항목 | 이유 |
| --- | --- |
| `mission_planning_gui.py` | public launcher이자 active orchestration entrypoint |
| `id_relationship_tab.py` root wrapper | `MissionVisualizer/main_visualizer.py`와 visualizer tool이 아직 root path를 import |
| `manual/MissionVisualizer/main_visualizer.py` | canonical manual visualizer implementation |
| `MissionVisualizer/main_visualizer.py` | advertised public visualizer wrapper |
| `MissionPlanner/AnS` active source/assets | core planner path. 단, `Training/TensorBoard Logs*`는 별도 삭제 후보로 검증 가능 |
| `MissionPlanner/data_def` active source | d0301/d0302/d0303/d0304 active artifact builders. 단, `d0304 copy.py`는 별도 삭제 후보로 검증 가능 |
| `MissionPlanner/planning_enhanced` | next-collab/area split/expected path active logic |
| `MissionPlanner/config.py` | planner/runtime code에서 active import |
| `MissionPlanner/runtime_settings.py` | app/monitoring/GUI 외부 import 다수 |
| `MissionPlanner/data_def/id_allocator.py` | ID band/counter/file lock 계약 |
| `MissionPlanner/tools/UAV_pattern/Nadir_BF/**` | `d0303.py` active path에서 import될 수 있음 |
| `MissionPlanner/portable_mission_bundle/portable_mission/**` | RL GUI/portable mission code에서 import |
| `MissionPlanner/portable_mission_bundle/models/model_config.json` | portable mission service model metadata |
| `MissionPlanner/corridor_planner.py`, `MissionPlanner/corridor_gui.py`, `MissionPlanner/Aisle_Sweep_CPP_shoot_plan.py` | auxiliary corridor tool과 bare import 계약. owner decision 전 삭제 보류 |
| `pipelines/attack_plan_pipeline.py` | active target detection/attack replanning |
| `pipelines/prior_mission_pipeline_impl.py` | active prior mission replanning |
| `pipelines/next_collab_replan_pipeline_impl.py` | active next collaborative mission replanning |
| `pipelines/path_deviation_replan_pipeline_impl.py` | active path deviation replanning |
| `pipelines/imaging_schedule_replan_pipeline_impl.py` | active imaging schedule replanning |
| `pipelines/post_attack_rejoin_pipeline.py` | active post-attack rejoin |
| `next_area_mode` | `runtime/next_collab_line_runner.py`에서 active import |
| `planners/next_collab_division/**` | `runtime/next_collab_division_runner.py`에서 active import |
| `portable_mission_bundle/models/latest_model.zip` | `d0304.py`와 RL GUI에서 active load |
| `logic_test/division_test/**` | active `planning_enhanced`/next-collab GUI 로직을 직접 import하므로 manual fixture 여부 확인 전 보류 |
| `MissionPlanner/tools/main_visualizer.py` | canonical manual visualizer로 위임하는 compatibility wrapper |
| `manual/lah_rl_planner_gui.py` and root `lah_rl_planner_gui.py` wrapper | `MissionPlanner/portable_mission_bundle`와 `latest_model.zip`을 직접 로드하는 수동/운영 GUI |
| `modules/common/*replan*_store.py`와 monitoring replan logic의 mission_planning imports | mission_planning runtime state를 외부 모듈이 import하므로 이동 전 shim 필요 |
| `runtime/replan_id_reservation.py`, `runtime/replan_validation.py` | 미추적 상태여도 active pipeline 다수가 직접 import |
| `runtime/source_artifact_cache.py`, `runtime/debug_artifacts.py` | 재계획 artifact cache/debug JSON write 경로 |
| `runtime/json_io.py`, `runtime/latest_input_cache.py`, `runtime/mission_plan_file_logger.py`, `runtime/mission_planning_pipeline_logging.py` | GUI/runtime I/O, latest 0201/0203 cache, mission plan file logging, pipeline logging active helper |
| `runtime/next_collab_replan_store.py`, `runtime/next_collab_replan_runtime.py`, `modules/common/next_collab_replan_store.py` | next-collab detail/event store와 monitoring/common wrapper surface |
| `runtime/attack_tracking_state.py`, `runtime/prior_tracking_state.py`, `runtime/attack_assignment_state.py`, root `attack_assignment_state.py` wrapper | monitoring/common/mission planning에서 tracking/assignment state를 직접 import |
| `pipelines/current_remaining_hybrid.py`, `pipelines/current_remaining_hybrid_replan.py`, `pipelines/general_remaining_hybrid_replan.py`, `pipelines/reexecute_first_mission_hybrid.py` | general replan 내부 hybrid/remaining flow에서 active 사용 |
| `pipelines/next_collab_path_builder.py`, `runtime/next_collab_line_runner.py`, `runtime/aircraft_parallel_0303.py`, root `next_collab_replan_pipeline.py` | next-collab replan 지원/공개 wrapper 경로 |
| `pipelines/recon_specialized_pipeline.py` | GUI와 enhanced planning pipeline 양쪽에서 import |
| `MissionPlanner/UAV_missionPlanning.py` | `d0303.py` bare import 경로와 planner engine 호환성에 필요 |
| `MissionPlanner/data_def/id_tracker.json`, `id_tracker_0202.json`, `_id_counters.json`, `MissionPlanner/AnS/_id_counters.json` | ID 연속성/초기화 계약. 생성물처럼 보여도 삭제 전 parity 확인 필요 |
| `DSS_Internal/targetInfo.json`, `VehicleStatus/status.json`, `sweep_progress.json`, `coverage_progress.json`, attack/prior/mission-area state JSON | 로그가 아니라 재계획 입력 상태로 사용될 수 있음 |
| `DSS_Internal/mission_planning_gui_*.log`, `DSS_Internal/missionPlan_*.json` | GUI/run logger 산출물 prefix/path 계약. runtime artifact manifest에 포함 전 삭제 보류 |
| `mission_planning_map.html`, `map.html`, `mission_map.html` | GUI/manual visualizer map 산출물 및 열기 경로 smoke 전 삭제 보류 |
| `attack_visualization.png` and attack assistance PNG outputs | `lah_attack_assistance.py` subprocess/visualization 산출물 계약 |
| `resource/db/fov_db.csv`, `resource/db/test_alg/**`, `settings/uav_params.json` | runtime settings, FOV, algorithm setting active resource |
| `resource/*.tif`, `MissionPlanner/AnS/DEM.jpg` | terrain/DEM/GeoTIFF resolver가 직접 참조 |
| `MissionPlanner/portable_mission_bundle/app.py`, `requirements.txt`, `run_portable.bat`, `data/inputs/**`, `data/work/**` | portable web execution/cwd/sys.path 계약 |

## 삭제 검증 절차

1. Reachability 검색

   - module path 검색
   - filename 검색
   - class/function export 검색
   - `python -m` launch string 검색
   - `.bat`, `.ps1`, docs, app launch config 검색

2. Behavior 검증

   - import smoke
   - GUI startup smoke
   - representative initial mission planning
   - representative attack/prior/next-collab/path-deviation replan
   - generated artifact validation

3. 삭제 batch 원칙

   - 같은 성격의 작은 bucket만 삭제
   - 삭제 이유와 검증 결과를 PR에 남김
   - 실패 시 batch 단위로 되돌릴 수 있게 유지
