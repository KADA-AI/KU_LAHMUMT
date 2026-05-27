# 2026-05-18 Replan Regression Deep TODO

작성 목적: 오늘 확인할 9개 이슈를 다른 AI/작업자가 바로 이어받을 수 있도록 코드 위치, 로그 근거, 현재 판단, 남은 TODO를 한곳에 정리한다.

기준 worktree: `C:\Users\LAHMUMT_2\Desktop\DSS_KU`

주의:

- 이 문서는 코드 수정 결과가 아니라, 현재 코드/로그를 읽고 만든 작업 TODO이다.
- 기존 사용자 변경이 많은 dirty worktree 상태이므로, 수정 작업자는 unrelated 변경을 되돌리지 말 것.
- raw nFusion 메시지 폴더와 DB 산출물을 분리해서 봐야 한다. `0301/0401/0501/531xx`는 raw 메시지, `MissionPlan/IndividualMissionPlan/FlightPath`는 계획 산출물이다.

## 한눈에 보는 상태

| 번호 | 이슈 | 현재 판단 |
| --- | --- | --- |
| 1 | 실제 지형보다 촬영 고도가 높게 계획되는 문제 | 보정 코드 있음. DEM miss/nodata fallback 검증 필요 |
| 2 | 재계획 실패를 0001이 아닌 재계획 완료로 보도록 수정 | 미구현/부분 구현. 실패 문구와 queue 완료 상태 분리 필요 |
| 3 | 적 공격 후 재계획 | 연결 경로는 대체로 있음. 빠른 close + new target 회귀 테스트 필요 |
| 4 | next-collab 재계획 때 FOV 미적용/계획 차이 | FOV가 완전히 빠진 것은 아님. source/resolved/template FOV 우선순위 검증 필요 |
| 5 | 편대모드에서 편대리더 촬영 제외 | 요구와 불일치 가능성 큼. formation waypoint에 filmingProperty가 들어감 |
| 6 | 모니터링 모듈 죽음 | crash 원인 미확인. 0401 처리와 visualization 의존 취약점 의심 |
| 7 | 경로 미추종 재계획 시 유인기 경로 사라짐 | DB 참조 보존은 확인. generated metadata/표시/전달 누락 가능 |
| 8 | 유인기가 임무 대기지역에서 대기하는지 | 부분 구현. 현재는 resume/current 근처 hold이며 명시 대기지역 사용은 미확인 |
| 9 | 유인기 isDone 적용 불가, 건대 계획에서 유인기 경로 미제공 필요 | 현재는 유인기 경로를 생성/제공함. 운용 모드 분기 필요 |

## 1. 촬영 고도와 실제 지형고 불일치

### 현재 확인

- 주요 코드 근거:
  - `modules/mission_planning/MissionPlanner/data_def/mission_helpers.py:353` `terrain_elev()`
  - `modules/mission_planning/MissionPlanner/data_def/filming_altitude_guard.py:10` `DEFAULT_FILMING_TARGET_CLEARANCE_M = 30.0`
  - `modules/mission_planning/MissionPlanner/data_def/filming_altitude_guard.py:33` 대상 좌표 DEM 고도 정규화
  - `modules/mission_planning/MissionPlanner/data_def/filming_altitude_guard.py:60` waypoint/촬영 target altitude guard
  - `modules/mission_planning/MissionPlanner/data_def/d0303.py:1371`, `d0303.py:1432`, `d0303.py:5401` 초기 0303 촬영 고도 보정
  - `modules/mission_planning/pipelines/next_collab_path_builder.py:300`, `next_collab_path_builder.py:2305`, `next_collab_path_builder.py:2690` next-collab 경로 보정
- DEM 샘플링은 `modules/mission_planning/MissionPlanner/data_def/mission_helpers.py`의 `terrain_elev()` 경로를 사용한다.
- 촬영 대상 좌표 고도를 DEM 지형고로 보정하고, waypoint 고도를 `촬영 대상 지형고 + clearance` 이상으로 올리는 공통 guard가 있다.
  - `modules/mission_planning/MissionPlanner/data_def/filming_altitude_guard.py`
  - 핵심 함수: `_normalize_target_coord_altitude()`, `normalize_filming_target_altitudes_in_waypoints()`, `sanitize_flight_path_payload_filming_altitudes()`
- 초기 계획 `d0303.py`, next-collab `next_collab_path_builder.py`, prior/post-attack/path-deviation 계열에서 이 guard를 호출하는 흔적이 있다.
- `filming_altitude_guard.py`는 DEM 실패 시 대상 고도를 `0`으로 두는 경로가 있다. 이 경우 실제 지형 반영 실패와 정상 저지대 케이스가 로그상 구분되지 않을 수 있다.

### 미흡점

- “실제 지형보다 촬영 고도가 높다”는 말이 다음 두 케이스 중 어느 쪽인지 분리해야 한다.
  - 촬영 대상 좌표 altitude가 실제 DEM보다 높게 들어감.
  - 항공기 waypoint altitude가 의도보다 높게 들어감.
- guard는 항공기 waypoint 고도를 최소 clearance 이상으로 올리는 목적이므로, 항공기 고도가 지형보다 높은 것 자체는 정상이다. 문제는 촬영 대상 좌표가 DEM 지형고와 불일치하거나, DEM 실패로 0/이상값이 들어가는 경우다.

### TODO

- [ ] `FlightPath/*.json`에서 모든 `filmingProperty.coordinateOrientation.coordinate`, `lineSearch.coordinateList[]`, `areaSearch.coordinateList[]`의 altitude를 DEM 샘플과 비교하는 검사 스크립트를 만든다.
- [ ] 같은 waypoint의 `coordinate.altitude - max(filming target altitude)`가 `DEFAULT_FILMING_TARGET_CLEARANCE_M=30m` 이상인지 검사한다.
- [ ] DEM tile 없음/nodata/예외 발생 시 `0` fallback이 아니라 `dem_missing`, `dem_nodata`, `dem_exception`을 debug artifact에 남긴다.
- [ ] 과거 로그와 신규 재현 로그에서 “대상 고도 불일치”와 “항공기 고도 clearance 적용”을 분리해 판정한다.

## 2. 재계획 실패를 0001이 아닌 재계획 완료로 보기

### 현재 확인

- 주요 코드 근거:
  - `modules/monitoring/logic/replan_queue_manager.py:289` 0305 status 해석
  - `modules/monitoring/logic/replan_queue_manager.py:535` 0305/0701/0702/0903 stage 전이
  - `modules/monitoring/logic/replan_queue_manager.py:1045` 0001 failure notice 처리
  - `modules/monitoring/logic/replan_queue_manager.py:1088` 일반 실패를 `dispatch_failed`로 완료
  - `modules/mission_planning/mission_planning_gui.py:6016`, `mission_planning_gui.py:6028` 실패 notice 0001 전송
  - `modules/mission_planning/mission_planning_gui.py:10390` 일부 no-op에서 0305 status=2 + 0001 처리
- `modules/monitoring/logic/replan_queue_manager.py`
  - `handle_0305()`는 `missionPlanningStatus=2`를 queue 완료가 아니라 `planning_finished` 단계로 처리한다.
  - `handle_0001()`의 일반 실패 notice는 active item을 완료시키지만 `status=dispatch_failed`, `stage=dispatch_failed`, `completion_signal=0001`로 기록한다.
- 단, `0001` 처리가 없는 것은 아니다. `no replan` 계열 0001은 이미 `completed/no_replan_needed`로 끝나며, 일반 failure 0001만 `dispatch_failed`로 끝난다.
- `modules/mission_planning/mission_planning_gui.py`
  - `_notify_failure_once()`는 일반 실패에서 `0001`만 보낸다.
  - `0305 missionPlanningStatus=2`는 성공 완료/일부 no-op 계열에서 주로 전송된다.
- 일부 “재계획 불필요” 경로는 `0305 status=2`와 `0001`을 같이 보내는 예외가 있다. 일반 실패와 정책이 다르다.

### 요구 해석

- 사용자 요구: “재계획 실패를 0001이 아닌, 재계획 완료로 보도록 수정. 문구는 실패로.”
- 즉 UI/로그/운용자에게 보이는 문구는 `임무계획 실패` 또는 실패 사유를 유지하되, queue/replan flow의 완료 신호는 실패가 아니라 “이번 재계획 요청은 종결됨”으로 처리해야 한다.

### TODO

- [ ] Mission Planning 일반 실패 시 `0305 missionPlanningStatus=2`를 반드시 전송할지 정책 결정.
- [ ] `0001` 실패 notice 수신 시 queue 상태를 `dispatch_failed`가 아니라 `completed` 또는 신규 `planning_failed_completed`로 끝내도록 변경한다.
- [ ] 단, 표시 문구는 `0001.contents`와 GUI 로그에 “실패”로 남긴다.
- [ ] `ReplanQueueTab`에서 `planning_failed_completed`를 완료 이력으로 보되 tone/text는 실패로 보이게 한다.
- [ ] 회귀 테스트:
  - Mission Planning pipeline 실패 발생.
  - `0305 status=2` 수신 여부 확인.
  - queue active가 해제되고 다음 queued item이 dispatch되는지 확인.
  - UI 문구는 실패로 남는지 확인.

## 3. 적 공격 후 재계획

### 현재 확인

- 주요 코드 근거:
  - `modules/monitoring/monitoring_gui.py:552`, `monitoring_gui.py:591` queue/coordinator 연결
  - `modules/monitoring/monitoring_gui.py:2980`, `monitoring_gui.py:3003` 0402 post-attack/general payload queue
  - `modules/monitoring/monitoring_gui.py:3009`, `monitoring_gui.py:4020` 0903/0702 뒤 deferred attack 재개
  - `modules/monitoring/logic/target_detection_replan.py:996`, `target_detection_replan.py:1140`, `target_detection_replan.py:1297` target/post-attack payload 생성
  - `modules/monitoring/logic/replan_queue_manager.py:1126` close 완료 뒤 deferred target detection 재개 관련 queue 처리
  - `modules/mission_planning/mission_planning_gui.py:3561`, `mission_planning_gui.py:5801`, `mission_planning_gui.py:10333` attack/post-attack pipeline 분기
- Monitoring 쪽 연결:
  - `modules/monitoring/logic/target_detection_replan.py`
  - `modules/monitoring/monitoring_gui.py`
  - `TargetDetectionCoordinator`가 일반 공격 payload와 `triggerType=attackClosedDestroyed` post-attack payload를 만든다.
  - post-attack close는 일반 target detection보다 우선 queue 처리되는 흐름이 있다.
- Mission Planning 쪽 연결:
  - `modules/mission_planning/mission_planning_gui.py`
  - `triggerType=attackClosedDestroyed`는 post-attack rejoin pipeline.
  - 일반 `0402` target detection은 attack pipeline.
  - 관련 pipeline: `modules/mission_planning/pipelines/attack_plan_pipeline.py`, `modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py`
- 기존 `docs/todo/2026-05-07_regression_issue_checklist.md`에도 다중 표적/후속 공격 회귀가 기록되어 있다.

### 미흡점

- attack tracking/assignment state에 의존성이 크다.
- 빠른 `0402 close + new target` 연속 수신에서 다음 항목이 막히거나 source plan lineage가 틀어질 수 있다.
- post-attack rejoin 후 `release_manned_used()`와 deferred target backlog 재평가가 실제 로그에서 재검증되어야 한다.

### TODO

- [ ] `1001/1002` 공격 중 `1001 destroyed`, 동시에 `1003~1005` 신규 표적 수신 시나리오를 재현한다.
- [ ] 기대 결과:
  - post-attack rejoin이 먼저 완료된다.
  - 공격 슬롯 release 후 deferred target detection이 재개된다.
  - 후속 공격 0902가 `followUpAttackMode=true` 또는 동등한 context를 가진다.
  - `attack_assignment_state.json`, `attack_tracking_state.json`, `targetInfo.json`이 일관된다.
- [ ] `0903`/`0702` 처리 후 `monitoring_gui._resume_deferred_attack_after_post_attack()` 경로가 실제로 호출되는지 로그로 남긴다.
- [ ] source plan rebinding 결과를 `DSS_Internal/log_attack_algorithm_*.json`과 MissionPlan lineage로 대조한다.

## 4. next-collab 재계획 FOV 미적용/계획 차이

### 현재 확인

- 주요 코드/로그 근거:
  - `modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0302.py:405` 초기 0302 FOV/SEP 산출
  - `modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0303_0304.py:81` runtime FOV/SEP 적용
  - `modules/monitoring/logic/next_collab_replan.py:333` next-collab payload 구성
  - `modules/mission_planning/runtime/next_collab_line_runner.py:1397`, `next_collab_line_runner.py:1450` line runner resolved FOV/spacing
  - `modules/mission_planning/runtime/next_collab_division_runner.py:168` area division runner 입력
  - `modules/mission_planning/pipelines/next_collab_path_builder.py:2108` `resolvedFovDeg`를 `mission_info["FOV"]`로 복사
  - `modules/mission_planning/pipelines/next_collab_path_builder.py:2235` formation path가 template/mission FOV를 사용
  - `modules/mission_planning/pipelines/next_collab_path_builder.py:2352` 최종 path FOV 선택
  - `Logs/Scenario_2026-05-14T160219/SBC3/DSS_Internal/next_collab_replan/next_collab_detail_700000002.json`
  - `Logs/Scenario_2026-05-14T160219/SBC3/DSS_Internal/NextCollab_3_832057351714.json`
- 초기 계획은 `FOV/SEP`를 `IndividualMissionPlan.individualMissionInfo.FOV`, `FlightPath.waypointList[].filmingProperty.fieldOfView`, sweep spacing에 반영한다.
  - `modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0302.py`
  - `modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0303_0304.py`
  - `modules/mission_planning/MissionPlanner/data_def/d0303.py`
- next-collab trigger payload에는 `source/current/target mission ID`, entry coordinate 중심 정보가 있고 FOV/SEP/footprint를 명시 전달하는 구조는 확인되지 않았다.
  - `modules/monitoring/logic/next_collab_replan.py`
- next-collab pipeline은 runner 내부에서 DB/runtime 기반 resolved FOV를 계산한다.
  - `modules/mission_planning/runtime/next_collab_line_runner.py`
  - `modules/mission_planning/runtime/next_collab_division_runner.py`
  - `modules/mission_planning/pipelines/next_collab_path_builder.py`
- 로그 근거:
  - `Logs/Scenario_2026-05-14T160219/SBC3/DSS_Internal/next_collab_replan/next_collab_detail_700000002.json`
  - `Logs/Scenario_2026-05-14T160219/SBC3/DSS_Internal/NextCollab_3_832057351714.json`
  - 위 로그에서 `sourceMissionPlanID=700000001`, `missionPlanID=700000002`, `targetInputMissionID=3`, `forceDirectUpdate=true`, `suppress0702Fallback=true` 등 next-collab 산출물은 확인된다.

### 결론

- next-collab에서 FOV가 완전히 빠지는 것은 아니다.
- `next_collab_path_builder.py`는 runner의 `resolvedFovDeg`를 mission info와 최종 path에 반영하는 경로가 있다.
- 남은 핵심 의심점은 “FOV 누락”보다 “source FOV/SEP/footprint, runner resolved FOV, 기존 template `filmingProperty.fieldOfView` 사이의 우선순위가 로그와 산출물에서 명확히 검증되지 않음”이다.
- 특히 formation 경로와 일부 template fallback 경로에서는 stale template `filmingProperty.fieldOfView`가 기대한 resolved/source FOV보다 먼저 쓰일 수 있는지 확인해야 한다.
- 따라서 “FOV 뿐 아니라 계획 자체가 달라진다”는 지적은 타당한 의심이다. next-collab은 기존 계획을 단순 trim/continue하는 것이 아니라 line/area runner가 새로 path를 계산한다.

### 2026-05-18 수정 메모

- `modules/mission_planning/pipelines/next_collab_path_builder.py`
  - 일반 next-collab path 생성 시 `path_row["resolvedFovDeg"]` 다음으로 `mission_info["FOV"]`를 fallback으로 사용하도록 수정했다.
  - formation path 생성 시 stale template `filmingProperty.fieldOfView`보다 `mission_info["FOV"]`를 우선하도록 수정했다.
  - 첫 LINE waypoint FOV 보정은 유지하되, 입력을 보정 후 FOV가 아니라 DB 원본 FOV(`resolvedBaseFovDeg`)로 바꾸고, `next_collab_first_line_fov_scale/max` 적용 후 `apply_runtime_camera_adjusted_fov_deg()`를 마지막에 적용하도록 수정했다.
- `modules/mission_planning/runtime/next_collab_line_runner.py`
  - DB row에서 선택한 원본 FOV를 `resolvedBaseFovDeg`로 path row에 저장하도록 수정했다.
- smoke 검증:
  - `resolvedBaseFovDeg=15.4`, `resolvedFovDeg=13.86`, 카메라 보정 `10%` 조건에서 첫 LINE waypoint FOV가 `15.4`로 되돌아가지 않고 `13.86`으로 유지됨을 확인했다.
  - template FOV가 `15.4`, `mission_info.FOV`가 `2.8`인 formation path에서 waypoint FOV가 `2.8`로 유지됨을 확인했다.
- 로그 데이터 판정:
  - `Scenario_2026-05-15T095522`: 초기 `2.8/4.2/3.7`에서 next-collab `13.2/3.1/11.0`으로 바뀌며 coordinate count, bearing, SEP도 변경된다.
  - `Scenario_2026-05-15T105017`: 초기 `2.8/4.2`에서 next-collab `11.2/13.2`로 바뀌며 `areaReview.changed=true`이고 path가 재생성된다.
  - 즉 해당 로그의 계획 차이는 FOV만의 문제가 아니라 next-collab repartition/rebuild 영향도 포함한다. 별도 정책으로 “source FOV/SEP 고정”이 필요하면 runner DB row 선택 정책까지 추가 수정해야 한다.

### TODO

- [ ] next-collab detail에 다음 값을 명시 저장한다.
  - source `individualMissionInfo.FOV`
  - source `individualMissionInfo.SEP`
  - source `FlightPath` 첫 촬영 waypoint `fieldOfView`
  - template `filmingProperty.fieldOfView`
  - runner resolved `resolvedFovDeg`, `sepCandM/dbSepM`
  - 최종 output `filmingProperty.fieldOfView`
- [ ] 초기 계획과 next-collab 계획의 sweep spacing/line count/path length/FOV를 같은 target input mission 기준으로 비교하는 artifact를 만든다.
- [ ] runtime manual FOV sync가 켜진 경우와 DB auto FOV가 켜진 경우를 분리해 회귀 케이스를 만든다.
- [ ] 0303/53112/53120에는 FOV가 비0인데 0401 `sensorInfo.fov=0.0`이면 simulator/monitoring 상태 반영 문제로 별도 분리한다.

검증 순서:

1. `IndividualMissionPlan/*.json`의 `individualMissionInfo.FOV`
2. `FlightPath/*.json`의 `filmingProperty.fieldOfView`
3. `53112`/`53120`의 fieldOfView
4. `0401 agentStateList[].unmannedInfo.sensorInfo.fov`

## 5. 편대모드에서 편대리더도 촬영 제외

### 현재 확인

- 주요 코드 근거:
  - `modules/mission_planning/MissionPlanner/data_def/d0303.py:3622` 초기 계획 formation leader/follower 구분
  - `modules/mission_planning/MissionPlanner/data_def/d0303.py:3700`, `d0303.py:4437` leader waypoint filmingProperty 적용
  - `modules/mission_planning/MissionPlanner/data_def/d0303.py:4489`, `d0303.py:5371` follower가 leader waypoint를 복사하는 경로
  - `modules/mission_planning/pipelines/next_collab_path_builder.py:2216` next-collab formation path 생성
  - `modules/mission_planning/pipelines/next_collab_path_builder.py:2253` `_make_hold_waypoint()`로 filmingProperty 생성
- 초기 계획 `d0303.py`는 편대 leader/follower를 구분하지만, formation waypoint에 `filmingProperty`를 넣는 경로가 있다.
- next-collab formation도 `build_formation_flight_path_from_template()`에서 `_make_hold_waypoint()`를 사용하고, `_make_hold_waypoint()`는 기본적으로 `filmingProperty`를 생성한다.
  - `modules/mission_planning/pipelines/next_collab_path_builder.py`
- 따라서 “편대모드일 때 편대리더도 촬영이 안 가도록 계획되는지”에 대해 현재 코드만 보면 요구와 불일치 가능성이 크다.

### TODO

- [ ] 요구를 명확히 고정한다.
  - 편대모드에서는 leader만 촬영 금지인가?
  - leader/follower 모두 촬영 금지인가?
  - 촬영 금지는 `filmingProperty` 제거인가, `operationMode` 변경인가, `sensorType`/camera command 억제인가?
- [ ] 초기 계획 `d0303.py` formation branch에서 leader/follower waypoint의 `filmingProperty` 제거를 적용한다.
- [ ] next-collab `build_formation_flight_path_from_template()`에서도 formation waypoint 생성 후 `filmingProperty`를 제거한다.
- [ ] 회귀 검사:
  - `inputMissionType=7`
  - `FlightPath.isFormationFlight=true`
  - `formationInfo.leaderAircraftID` 존재
  - 모든 `waypointList[].filmingProperty` 미존재
  - `53112` 또는 실제 전송 payload에서도 filmingProperty가 재삽입되지 않음

## 6. 모니터링 모듈 죽음

### 로그 확인

- 주요 코드/로그 근거:
  - `modules/monitoring/monitoring_gui.py:1048` faulthandler/process diagnostics 초기화
  - `modules/monitoring/monitoring_gui.py:1309` 0401 trace/watchdog loop
  - `modules/monitoring/monitoring_gui.py:4579` `_on_rx_0401()` visualization 의존 early return 의심
  - `modules/monitoring/monitoring_gui.py:5523` forced hold poller silent exception 의심
  - `modules/monitoring/logic/monitoring_logic.py:40` background logic loop 예외 처리
  - `Logs/Scenario_2026-05-14T100656_건대 모니터링 모듈 꺼짐/SBC3/DSS_Internal/module_logs/monitoring.log`
- 주요 로그:
  - `Logs/Scenario_2026-05-14T100656_건대 모니터링 모듈 꺼짐/SBC3/DSS_Internal/module_logs/monitoring.log`
  - `Logs/Scenario_2026-05-14T100656_건대 모니터링 모듈 꺼짐/SBC3/DSS_Internal/monitoring_fatal.log` 또는 fatal 계열 파일
- fatal 파일은 `faulthandler enabled` 수준만 있고 Python traceback은 확인되지 않았다.
- `monitoring.log`에는 0401 handler slow 반복 후 process-log 재attach 흔적이 있다. Python 예외로 잡힌 crash라기보다 native/Qt 종료, UI thread block, silent restart 가능성을 같이 봐야 한다.

### 코드상 취약점

- `modules/monitoring/monitoring_gui.py`
  - `_on_rx_0401()`에서 visualization tab 또는 `update_agent_status`가 없으면 early return한다.
  - 그 return 아래에 RTB/path-deviation/quality/imaging trigger가 있으면 visualization 준비 실패가 monitoring trigger 정지로 이어질 수 있다.
- forced hold poller 등 일부 background loop는 최상위 예외를 삼키거나 로그가 약하다.

### TODO

- [ ] `_on_rx_0401()`에서 UI visualization update와 replan trigger 계산을 분리한다.
- [ ] visualization tab이 없어도 `path_deviation`, `rtb`, `quality`, `imaging_schedule`, `next_collab context update`가 계속 돌도록 한다.
- [ ] silent `except Exception: pass` 경로에 최소 `emit_process_log("monitoring", ...)`를 추가한다.
- [ ] fatal/watchdog artifact에 마지막 처리 message ID, 0401 payload 크기, handler elapsed, active thread stack을 남긴다.
- [ ] 재현 테스트:
  - visualization tab 초기화 실패를 강제로 만들고 0401 수신.
  - monitoring process가 유지되는지 확인.
  - path deviation trigger가 계속 생성되는지 확인.

## 7. 경로 미추종 재계획 시 유인기 경로 사라짐

### 현재 확인

- 주요 코드/로그 근거:
  - `modules/monitoring/logic/turn_radius_monitor.py:16` tracked aircraft 4/5/6
  - `modules/monitoring/logic/path_deviation_replan.py:117` path-deviation coordinator
  - `modules/monitoring/logic/path_deviation_replan.py:288`, `path_deviation_replan.py:324` payload detail/source/current/path 정보
  - `modules/mission_planning/mission_planning_gui.py:11202` dedicated path-deviation pipeline 분기
  - `modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py:1423`, `path_deviation_replan_pipeline_impl.py:1562` source MissionPlan deepcopy 및 UAV만 교체
  - `modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py:1640`, `path_deviation_replan_pipeline_impl.py:1697` generated IDs metadata
  - `modules/monitoring/gui/tabs/monitoring_visualization_tab.py:2679` mission view UAV 중심 로딩
  - `Logs/260428_건대 송부 로그/Scenario_2026-04-28T114518_다음임무 인가 후 0501에서 보는 id 바뀜/SBC3/DSS_Internal/module_logs/monitoring.log:77` source artifact unresolved 반복
  - `Logs/260428_건대 송부 로그/Scenario_2026-04-28T114518_다음임무 인가 후 0501에서 보는 id 바뀜/SBC3/DSS_Internal/module_logs/mission_planning.log:135` other aircraft currentWP fallback
- path-deviation trigger는 UAV 4/5/6만 대상으로 한다.
  - `modules/monitoring/logic/turn_radius_monitor.py`
  - `modules/monitoring/logic/path_deviation_replan.py`
- Mission Planning dedicated path-deviation pipeline은 원본 MissionPlan을 deepcopy하고, 이탈 UAV 및 다른 UAV 4/5/6의 IMP/path만 새 ID로 교체한다.
  - `modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py`
- 유인기 1/2/3은 원본 IMP/path 참조를 유지하는 방식이다.
- 로그 근거:
  - `Logs/260428_건대 송부 로그/Scenario_2026-04-28T114518_다음임무 인가 후 0501에서 보는 id 바뀜/SBC3`
  - `700000003~700000006` MissionPlan에서 LAH 패키지 `800000007/008/009`가 유지되고 UAV 패키지만 바뀌는 흐름이 확인되었다.

### 미흡점

- monitoring dispatch/detail에는 아직 generated IDs가 없다. Mission Planning pipeline 결과/log에는 `generatedMissionPlanID`, `generatedIndividualMissionPackageIDs`, `generatedPathIDs`가 남는다.
- 이 generated set에는 새로 만든 UAV 것만 들어간다. preserved LAH path는 “원본 참조 유지”로만 남고, `preservedMannedPackageIDs/pathIDs` 같은 명시 metadata가 없다.
- visualization 쪽 mission view는 UAV 중심으로 빌드되어 유인기 경로 표시가 빠질 수 있다.
- detail store의 `sourceMissionPlanID`가 replan transport payload와 불일치한 사례가 있다.

### TODO

- [ ] path-deviation 완료 artifact에 `preservedMannedPackageIDs`, `preservedMannedPathIDs`를 명시 추가한다.
- [ ] 새 MissionPlan의 모든 `aircraftList`에 대해 `individualMissionPackageID -> IndividualMissionPlan -> pathID -> FlightPath` resolve 검사를 수행한다.
- [ ] UI/시뮬레이터가 `generatedPathIDs`만 로드하는지, 새 MissionPlan의 `aircraftList` 전체를 로드하는지 확인한다.
- [ ] detail store와 transport payload의 `sourceMissionPlanID/currentMissionPlanID` 불일치를 수정한다.
- [ ] 요구가 “새 산출물에도 유인기 path 파일을 복제”라면, 보존 참조 방식에서 복제 방식으로 바꿀지 정책 결정한다.

## 8. 유인기가 임무 대기지역에서 대기하는지

### 현재 확인

- 주요 코드 근거:
  - `modules/mission_planning/pipelines/attack_plan_pipeline.py:3430`, `attack_plan_pipeline.py:3474` active/non-active LAH 분기
  - `modules/mission_planning/pipelines/attack_plan_pipeline.py:4839` hold coordinate 계산
  - `modules/mission_planning/pipelines/attack_plan_pipeline.py:6644`, `attack_plan_pipeline.py:6771`, `attack_plan_pipeline.py:6795` LAH_HOLD_RESUME 생성
  - `modules/mission_planning/MissionPlanner/data_def/d0304.py:1547`, `d0304.py:1552` LAH 마지막 waypoint hovering 설정
- 공격 계획에서 공격에 투입되지 않는 유인기는 `LAH_HOLD_RESUME` 경로로 hold mission을 생성한다.
  - `modules/mission_planning/pipelines/attack_plan_pipeline.py`
- hold 좌표는 `_build_lah_hold_coordinate_near_resume()` 계열에서 current/resume 근처 좌표로 계산된다.
- 초기 `d0304.py`는 LAH 마지막 waypoint에 `hovering`을 넣는 경로가 있다.
- 그러나 “임무 대기지역”이라는 별도 source field를 읽어 해당 좌표로 대기시키는 구현은 확인되지 않았다.

### TODO

- [ ] `0203 FlightReferenceInfo`, `MissionReferenceInfo`, 또는 건대 입력 중 “임무 대기지역”에 해당하는 필드명을 확정한다.
- [ ] LAH hold 좌표 source priority를 정의한다.
  - 명시 임무 대기지역
  - takeover/RTB/loiter reference
  - current/resume 근처 fallback
- [ ] `LAH_HOLD_RESUME` 생성 시 어떤 source로 좌표를 정했는지 log/artifact에 남긴다.
- [ ] hold waypoint에 `hovering` 또는 `loiterProperty` 중 ICD상 올바른 표현을 적용한다.
- [ ] 재현 로그에서 LAH hold coordinate가 임무 대기지역 polygon/point 내부인지 검사한다.

## 9. 건대 임무 계획에서 유인기 경로 미제공

### 현재 확인

- 주요 코드/로그 근거:
  - `modules/mission_planning/MissionPlanner/data_def/d0304.py:1338` `build_lah_flight_plans_fixed()`
  - `modules/mission_planning/MissionPlanner/data_def/d0304.py:1353` `manned_plan_mode` capstone 여부만 확인
  - `modules/mission_planning/MissionPlanner/planning_enhanced/io/export_0303_0304.py:271`, `export_0303_0304.py:286` LAH missions -> 0304 생성
  - `modules/mission_planning/pipelines/attack_plan_pipeline.py:6095`, `attack_plan_pipeline.py:6138` LAH attack sequence path 생성
  - `modules/mission_planning/pipelines/attack_plan_pipeline.py:6224`, `attack_plan_pipeline.py:6273` LAH resume path 생성
  - `modules/mission_planning/pipelines/attack_plan_pipeline.py:6801` LAH hold/resume path 생성
- 현재 초기 계획/공격 재계획/hold-resume/post-attack 계열은 유인기 1/2/3에 대해 `IndividualMissionPlan`, `FlightPath`, `lahWaypointList`를 생성한다.
- `d0304.build_lah_flight_plans_fixed()`는 `manned_plan_mode`를 받지만, 현재 확인 가능한 mode는 `normal/capstone` 계열이며 “유인기 경로 미제공” 모드는 없다.
- 로그에서도 유인기 경로가 실제로 많이 생성된다.
  - 100656 `53111` LAH waypoint 72개
  - 0501문제 `53111` LAH waypoint 786개
  - 260428-114518 `53111` LAH waypoint 750개

### 요구 해석

- “유인기에 isDone 적용이 불가하므로, 건대 임무 계획에서 유인기 경로를 안주는 방향”은 현재 구현과 다르다.
- 이 요구는 전체 운용이 아니라 건대 연동/특정 모드에서만 적용해야 할 가능성이 높다.

### TODO

- [ ] runtime setting에 예: `manned_plan_mode = no_path` 또는 `konkuk_no_manned_path`를 추가할지 결정한다.
- [ ] 해당 모드에서 0304/LAH FlightPath 생성을 생략한다.
- [ ] MissionPlan `aircraftList`에서 LAH `individualMissionPackageID`를 어떻게 처리할지 결정한다.
  - 기존 ID 유지
  - 0/null
  - aircraft entry 자체 제외
- [ ] 0302 `IndividualMissionPlan_LAH*.json` 생성도 같이 생략할지, 0304만 생략할지 결정한다.
- [ ] monitoring/sim loader가 LAH path 없음 상태에서 죽지 않고 “유인기 경로 미제공”으로 표시하는지 확인한다.
- [ ] `isDone` 기반 trim/resume 로직이 LAH에는 적용되지 않도록 guard를 추가한다.

## 공통 검증 스크립트/TODO

- [ ] 로그 검증 스크립트 입력:
  - scenario root: `Logs/<Scenario>/SBC3`
  - DB folders: `MissionPlan`, `IndividualMissionPlan`, `FlightPath`
  - raw folders: `0301`, `0401`, `0501`, `53111`, `53112`, `53120`
- [ ] 출력:
  - MissionPlan별 aircraft -> IMP -> path resolve 표
  - UAV/LAH별 path 존재 여부
  - filmingProperty 존재/삭제 여부
  - FOV chain: IMP FOV -> FlightPath FOV -> 531 FOV -> 0401 sensorInfo.fov
  - next-collab source/resolved/output FOV 비교
  - path-deviation source/current plan ID consistency
  - monitoring fatal/watchdog summary
- [ ] 기존 로그 우선 검증 대상:
  - `Logs/Scenario_2026-05-14T100656_건대 모니터링 모듈 꺼짐/SBC3`
  - `Logs/Scenario_2026-05-14T160219/SBC3`
  - `Logs/0501문제/SBC3`
  - `Logs/260428_건대 송부 로그/Scenario_2026-04-28T114518_다음임무 인가 후 0501에서 보는 id 바뀜/SBC3`
  - `Logs/260428_건대 송부 로그/Scenario_2026-04-23T145113_사천로그_무인기 카메라/SBC3`

## 우선순위 제안

1. 편대모드 `filmingProperty` 제거 정책 확정 및 수정.
2. 실패 notice의 queue 완료 정책 수정.
3. next-collab FOV/SEP source-resolved-output logging 및 회귀 검증.
4. monitoring `_on_rx_0401()` early return 의존 제거.
5. path-deviation LAH preserved metadata 추가 및 source plan mismatch 수정.
6. 건대 모드 LAH path 미제공 정책 설계/구현.
