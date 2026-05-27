# 로그와 산출물 맵

마지막 재정리: 2026-05-04

재계획 분석은 메시지 로그만으로 끝나지 않는다. active DB의 계획 파일, `DSS_Internal` 상세 저장소, module log, sidecar, 시뮬레이터 상태를 함께 봐야 한다.

## Active DB 찾기

현재 시나리오 DB 루트는 다음 경로 계층 중 하나다.

- `Logs/Scenario_<iso>/<agency>`
- legacy fallback: `temp/database`
- 환경 변수: `KU_MISSION_DB_ROOT`
- 현재 시나리오 파일: `current_scenario.json`

`run.py`는 `0101 SystemMode=1` 진입 시 `current_scenario.json`을 갱신하고 각 모듈 프로세스에 `KU_MISSION_DB_ROOT`를 넘긴다.

이 문서에서 `%ACTIVE_DB_ROOT%`는 현재 활성 시나리오 DB 루트를 뜻한다.

## 최상위 DB 산출물

| 위치 | 내용 |
| --- | --- |
| `%ACTIVE_DB_ROOT%/InputMissionPlan` | 0201 입력, 재계획 변형 입력, 새 input package |
| `%ACTIVE_DB_ROOT%/MissionPlan` | 실제 MissionPlan JSON |
| `%ACTIVE_DB_ROOT%/IndividualMissionPlan` | 개별 mission package JSON |
| `%ACTIVE_DB_ROOT%/FlightPath` | UAV/LAH waypoint 경로 JSON |
| `%ACTIVE_DB_ROOT%/MissionReferenceInfo` | mission reference JSON |
| `%ACTIVE_DB_ROOT%/MissionPlanOptionInfo` | 0305/0701 option info JSON |
| `%ACTIVE_DB_ROOT%/VehicleStatus` | 시뮬레이터 vehicle status |
| `%ACTIVE_DB_ROOT%/mission_output` | variant, 중간/호환 산출물 |
| `%ACTIVE_DB_ROOT%/cache` | 일부 최신 요청/옵션 캐시 |

`MissionPlan/<id>.json`이 실제 계획이다. `DSS_Internal/missionPlan_<id>.json`은 run log 성격이므로 혼동하지 않는다.

## DSS_Internal 공통

| 위치 | 내용 |
| --- | --- |
| `DSS_Internal/module_logs/dashboard.log` | dashboard/run.py orchestration log |
| `DSS_Internal/module_logs/mission_planning.log` | MMR process log |
| `DSS_Internal/module_logs/monitoring.log` | MSM process log |
| `DSS_Internal/module_logs/decision_support.log` | decision support log |
| `DSS_Internal/module_logs/info_manage.log` | info manage log |
| `DSS_Internal/latest_0401_agent_status.json` | 최신 0401 snapshot |
| `DSS_Internal/log_0401_agent_status_sim.jsonl` | 0401 시계열 |
| `DSS_Internal/targetInfo.json` | target detection 상태 |
| `DSS_Internal/suppress_option_request.json` | option suppression flag |
| `DSS_Internal/coverage_progress.json` | coverage progress |
| `DSS_Internal/sweep_progress.json` | sweep progress |
| `DSS_Internal/path_usage.json` | path ID 사용 상태 |
| `DSS_Internal/waypoint_usage.json` | waypoint ID 사용 상태 |
| `DSS_Internal/id_tracker.json` | ID allocator 상태 |

## Mission Planning 로그

| 패턴 | 의미 |
| --- | --- |
| `DSS_Internal/mission_planning_gui_*.log` | GUI/runtime 상세 로그 |
| `DSS_Internal/missionPlan_<missionPlanID>.json` | planning run log 또는 요약 |
| `DSS_Internal/replan_inputs/*` | 재계획용 0201 override/filtered 입력 |
| `DSS_Internal/PathDeviation_*` | 경로 이탈 파이프라인 로그 |
| `DSS_Internal/ImagingSchedule_*` | 촬영 스케줄 파이프라인 로그 |
| `DSS_Internal/QualitySpeed_*` | 품질 속도 보정 로그 |
| `DSS_Internal/NextCollab_*` | 다음 협업 파이프라인 로그 |
| `DSS_Internal/log_attack_algorithm*.json` | 공격 특화/공격 제외 로그 |
| `DSS_Internal/log_prior_algorithm.json` | 선행 임무 로그 |

파일명 패턴은 파이프라인/실행 모드에 따라 조금씩 다를 수 있다. 없으면 module log에서 실제 `log_path`를 찾는다.

## 0902 sidecar

| 위치 | 의미 |
| --- | --- |
| `DSS_Internal/replan_request_transport/replan_request_<timestamp>.json` | 0902 확장 detail 보존 |
| `DSS_Internal/replan_request_archive/*` | fallback/archive 저장 |

sidecar는 단순 로그가 아니다. nFusion 스키마에 없는 `replanDetail` 확장 필드를 보존하는 기능이다. Mission Planning 수신기는 sidecar를 병합해 파이프라인 context를 복원할 수 있다.

## 전용 재계획 저장소

| 저장소 | 위치 | 내용 |
| --- | --- | --- |
| path deviation | `DSS_Internal/path_deviation_replan` | 이탈 상세, alternate waypoint, 이벤트 |
| imaging schedule | `DSS_Internal/imaging_schedule_replan` | schedule/quality detail, 이벤트 |
| prior replan | `DSS_Internal/prior_replan` | prior mission 상세 |
| next collab | `DSS_Internal/next_collab_replan` | 다음 협업 임무 상세 |
| mission area | `DSS_Internal/mission_area_replan` | 잔여 영역 snapshot |
| prior target rediscovery | `DSS_Internal/prior_target_rediscovery` | prior target rediscovery 상태 |

Mission Planning은 0902의 detail이 부족하면 이 저장소에서 plan ID 기준으로 상세를 보완한다.

## 공격/선행 추적 상태

| 파일 | 의미 |
| --- | --- |
| `DSS_Internal/attack_assignment_state.json` | 공격 후보/할당 상태 |
| `DSS_Internal/attack_tracking_state.json` | 공격 계획 lineage, watcher/target 추적 |
| `DSS_Internal/prior_tracking_state.json` | 선행 임무 추적 상태 |

post-attack rejoin과 prior-post-rejoin은 이 상태가 맞지 않으면 계획을 만들지 않는다.

## Monitoring 진단

| 위치 | 의미 |
| --- | --- |
| `DSS_Internal/monitoring_diagnostics/msm_0501_*.json` | 0501 송신 진단 |
| `DSS_Internal/monitoring_diagnostics/latest_msm_0501_diag.json` | 최신 0501 진단 |
| Replan Queue tab snapshot | active/queued/history 상태 |
| `DSS_Internal/targetInfo.json` | target detection 판단 상태 |

GUI 메모리 상태는 파일에 전부 남지 않는다. queue tab에서 보이는 active/queued/history와 파일 로그를 함께 본다.

## Simulation/Web 로그와 캐시

| 위치/API | 의미 |
| --- | --- |
| `modules/sim/runtime/sim_service.py` log | 0401/0402, plan load, target 상태 |
| `%ACTIVE_DB_ROOT%/VehicleStatus/status.json` | 현재 vehicle status |
| `/api/mission/plan_load` | missionPlanID로 시뮬레이터 로드 |
| `/api/integration/payload?msgId=0503&type=rx` | 웹 추천 팝업 polling |
| `/api/integration/send_custom` | 웹에서 0702/0803 등 custom 송신 |
| `%ACTIVE_DB_ROOT%/cache/latest_0901.json` | 최신 0901/옵션 캐시 |

웹에서 plan이 바뀌지 않았을 때는 `0903` 수신 여부와 별도로 `/api/mission/plan_load`가 성공했는지 확인한다.

## 분석 순서

재계획 하나를 끝까지 추적하는 기본 순서:

1. 트리거 원본 메시지 확인: `0401`, `0402`, `0201`, `0202`, `0802`, `0803`
2. Monitoring module log에서 coordinator가 payload를 만들었는지 확인
3. Queue stage가 dispatch까지 갔는지 확인
4. `replan_request_transport`에 0902가 저장됐는지 확인
5. Mission Planning log에서 선택된 파이프라인 확인
6. 전용 store/log에서 skip/no-op 사유 확인
7. `MissionPlan`/`FlightPath` 산출물 확인
8. `0305`, `0903`, `0702`, `0001` 중 어떤 완료 신호가 왔는지 확인
9. Monitoring이 새 plan을 적용했는지 확인
10. Simulation/Web이 같은 plan을 로드했는지 확인

이 순서를 지키면 "트리거 자체가 없음", "큐에서 밀림", "파이프라인 스킵", "산출물 깨짐", "적용 실패"를 분리할 수 있다.
