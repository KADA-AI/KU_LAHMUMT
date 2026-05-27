# AI/개발자 작업 체크리스트

마지막 재정리: 2026-05-04

Mission Planning/Monitoring 코드를 수정하거나 로그를 분석할 때는 기능 이름보다 메시지 ID, source tag, active DB 산출물, queue stage를 기준으로 추적한다.

## 시작 전 확인

1. 현재 활성 DB 루트 확인
2. 루트 `replan_settings.json` 확인
3. 루트 `uav_params.json` 확인
4. `current_scenario.json` 확인
5. `DSS_Internal/module_logs` 확인
6. 관련 `0902` sidecar 유무 확인
7. MissionPlan/FlightPath 실제 파일 유무 확인

PowerShell 예시:

```powershell
Get-Content -Raw -Encoding UTF8 .\current_scenario.json
Get-Content -Raw -Encoding UTF8 .\replan_settings.json
Get-Content -Raw -Encoding UTF8 .\uav_params.json
rg -n "triggerType|replanLevel|missionPlanID|0902|0903" .\Logs -g "*.json" -g "*.jsonl" -g "*.log"
```

## 코드 탐색 우선순위

기능별 첫 진입점:

| 기능 | 먼저 볼 파일 |
| --- | --- |
| Mission Planning 재계획 분기 | `modules/mission_planning/mission_planning_gui.py` |
| 공격/공격 제외 | `modules/mission_planning/pipelines/attack_plan_pipeline.py` |
| 공격 종료 후 합류 | `modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py` |
| 다음 협업 임무 | `modules/mission_planning/pipelines/next_collab_replan_pipeline_impl.py` |
| 촬영/품질 속도 | `modules/mission_planning/pipelines/imaging_schedule_replan_pipeline_impl.py` |
| 경로 이탈 | `modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py` |
| 선행 임무 | `modules/mission_planning/pipelines/prior_mission_pipeline_impl.py` |
| 일반 계획 | `modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py` |
| enhanced planning | `modules/mission_planning/MissionPlanner/planning_enhanced/pipeline.py` |
| 0302 | `modules/mission_planning/MissionPlanner/data_def/d0302.py` |
| 0303 | `modules/mission_planning/MissionPlanner/data_def/d0303.py` |
| 0304 | `modules/mission_planning/MissionPlanner/data_def/d0304.py` |
| Monitoring 오케스트레이션 | `modules/monitoring/monitoring_gui.py` |
| 재계획 큐 | `modules/monitoring/logic/replan_queue_manager.py` |
| runtime 설정 | `modules/monitoring/logic/replan_runtime_settings.py` |
| active DB 경로 | `modules/common/db_paths.py` |
| 0902 sidecar | `modules/common/replan_request_transport_store.py` |
| 시뮬레이터 plan load | `modules/sim/mission/mission_plan_loader.py` |

## 재계획 분기 확인

Mission Planning 쪽 문제를 볼 때는 현재 분기 순서를 먼저 고정한다.

1. attack
2. post-attack rejoin
3. next-collab
4. imaging/quality
5. path deviation
6. prior/prior-post-rejoin
7. general fallback

분기 조건을 확인할 때는 `reason` 문자열보다 `replanDetail.trigger`, `triggerType`, `replanLevel`, `source_tag`, current/source plan ID를 우선한다. 한글 option label은 인코딩 문제로 깨질 수 있으므로 보조 힌트로만 쓴다.

## Monitoring 트리거 확인

트리거 분석 순서:

1. 해당 toggle이 루트 `replan_settings.json`에서 켜져 있는지 확인
2. 입력 메시지가 실제 수신됐는지 확인
3. coordinator가 payload를 만들었는지 확인
4. `ReplanQueueManager`에 enqueue됐는지 확인
5. active 재계획이 있어 지연됐는지 확인
6. target detection delay/merge/suppression 대상인지 확인
7. `0902`가 sidecar에 저장됐는지 확인

`0701`만 받았는데 큐가 안 풀리는 것은 현재 설정상 정상일 수 있다. `release_on_option_info=false`이므로 `0903`, `0702`, `0001`, timeout을 봐야 한다.

## ID/파일 체크

계획 산출물이 문제라면 다음을 순서대로 본다.

1. `MissionPlan/<missionPlanID>.json` 존재
2. 내부 `missionPlanID`가 파일명과 일치
3. `aircraftList[*].individualMissionPackageID`가 실제 `IndividualMissionPlan` 파일로 이어짐
4. `individualMissionList[*].pathID`가 실제 `FlightPath` 파일로 이어짐
5. `FlightPath` 내부 waypoint ID가 중복되지 않음
6. `MissionPlanOptionInfo`가 실제 plan ID를 가리킴
7. `0301`/`0903` payload의 plan ID가 같은 파일을 가리킴

주의: `DSS_Internal/missionPlan_<id>.json`은 run log이고 실제 plan은 `MissionPlan/<id>.json`이다.

## 초기 계획 체크

현재 초기 계획은 Monitoring의 system mode `2` 전환 시 생성되는 `0902` 형식 요청에서 시작한다.

확인 항목:

- `0201`/`0203` 최신 payload가 들어왔는가
- `init_replan.py`가 input/plan ID를 수집했는가
- `0902` 초기임무재계획 요청이 생성됐는가
- Mission Planning이 `AnS.run_divide_and_pattern()`을 실행했는가
- `0305 missionPlanningStatus=2` 완료가 나갔는가
- `0901` option request 또는 `0903` direct apply가 나갔는가

## 전용 파이프라인 체크

전용 파이프라인은 보통 관련 pending/source plan ID가 정확해야 한다.

| 파이프라인 | 필수 확인 |
| --- | --- |
| post-attack rejoin | `0402 attackClosedDestroyed`, target ID, attack tracking assignment, remaining ETA/progress |
| next-collab | `0803 execute=1`, next collab detail store, formation-flight skip 여부 |
| imaging schedule | imaging schedule toggle, schedule detail store, trigger type |
| quality speed | quality monitor + quality speed toggle, `qualityMonitorSep` |
| path deviation | turn view warning, alternate waypoint, synthetic waypoint guard |
| prior | level 4, prior detail store, current/source lineage |

조건이 맞지 않으면 파이프라인이 실패가 아니라 "스킵 후 fallback" 또는 `0001 no-replan`으로 종료될 수 있다.

## 설정 변경 체크

설정 관련 코드를 바꿀 때는 다음을 지킨다.

- runtime 기준은 루트 `replan_settings.json`
- defaults는 루트 `replan_settings_defaults.json`
- 설정 UI는 normalize된 값을 저장
- 모듈 내부 설정 파일과 값이 달라도 runtime 기준으로 판단
- 설정 추가 시 defaults, normalize, UI, docs를 함께 갱신

## 로그 분석 체크

재계획 하나를 추적할 때 필요한 파일:

- `DSS_Internal/replan_request_transport/replan_request_<timestamp>.json`
- `DSS_Internal/module_logs/monitoring.log`
- `DSS_Internal/module_logs/mission_planning.log`
- `DSS_Internal/latest_0401_agent_status.json`
- 전용 store detail/event
- `MissionPlan/<id>.json`
- `MissionPlanOptionInfo/<id>.json`
- `FlightPath/<pathID>.json`

가능하면 `timestamp`, `missionPlanID`, `source_tag`, `triggerType`, `targetID` 중 하나를 기준 키로 삼아 전체를 묶는다.

## 문서 갱신 체크

코드 변경 후 문서를 갱신할 때는 다음 네 가지를 반드시 반영한다.

1. 새/변경된 메시지 ID와 trigger type
2. runtime 설정 경로와 기본값/현재값
3. 파이프라인 분기 순서 변화
4. 산출물 파일 위치와 ID 관계 변화

특히 재계획 분기 순서와 Monitoring 토글은 자주 바뀌므로 문서 상단에 검증 날짜를 남긴다.
