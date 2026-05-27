# Mission Planning 재계획 파이프라인

마지막 재정리: 2026-05-04

이 문서는 `modules/mission_planning/mission_planning_gui.py`와 `modules/mission_planning/pipelines`의 현재 코드 기준 재계획 분기와 전용 파이프라인을 정리한다.

## 전체 분기 순서

`_run_replan_pipeline_do`의 현재 실행 순서는 다음과 같다.

1. 공격 특화/공격 제외 후보 판단
2. 공격 종료 후 합류
3. 다음 협업 임무
4. 촬영 스케줄 또는 품질 속도 보정
5. 경로 이탈
6. 선행 임무 또는 선행 종료 후 합류
7. 일반 재계획 fallback

중요한 예외는 두 가지다.

- `0402`의 `triggerType=attackClosedDestroyed`는 공격 특화 후보에서 제외된 뒤 post-attack rejoin 분기로 간다.
- `triggerType=priorClosedResume`은 독립 최상위 분기가 아니라 prior mission 분기 안에서 `replan_level == 4`일 때 처리된다.

## 공격 특화/공격 제외

대표 파일:

- `modules/mission_planning/pipelines/attack_plan_pipeline.py`
- `modules/mission_planning/runtime/attack_assignment_state.py`
- `modules/mission_planning/runtime/attack_tracking_state.py`

공격 특화 분기는 다음 조건 중 하나로 선택된다.

- reason에 공격 특화/공격추천 성격의 문자열이 포함됨
- `replanDetail.trigger == "0402"`이고 post-attack rejoin 상세가 아님
- option name에 공격 특화/공격 제외 계열 라벨이 포함됨

주요 함수는 `run_attack_plan_pipeline`과 `run_attack_exclusion_pipeline`이다. target bundle, manned aircraft 2/3, watcher UAV 4/5/6, target priority, attack assignment/tracking state를 사용한다.

공격 특화 후보와 공격 제외 후보가 각각 하나씩 있을 때 `REPLAN_ATTACK_EXCLUSION_PARALLEL`이 false가 아니면 병렬 생성 경로를 탈 수 있다. 공격 제외 후보는 일반 재계획 후보와 섞이지 않도록 별도 필터링된다.

## 공격 종료 후 합류

대표 파일:

- `modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py`
- `modules/mission_planning/runtime/attack_tracking_state.py`
- `modules/common/mission_area_replan_store.py`

분기 조건:

- `replanDetail.trigger == "0402"`
- `replanDetail.triggerType == "attackClosedDestroyed"`
- post-attack rejoin 토글이 켜져 있음
- current/source plan 및 target ID가 유효함
- attack tracking assignment와 현재 임무 lineage가 맞음

현재 루트 `replan_settings.json`에서는 `post_attack_rejoin=true`다. 모듈 내부 `modules/monitoring/replan_settings.json`은 값이 다를 수 있으므로 실행 판단 기준으로 쓰지 않는다.

주요 설정:

| 항목 | 값 |
| --- | --- |
| `closure_cooldown_ms` | 30000 |
| `min_remaining_eta_s` | 120 |
| `rejoin_margin_s` | 45 |
| `turn_radius_m` | 180 |
| `default_cruise_speed_mps` | 35 |
| `active_progress_skip_percent` | 70 |

파이프라인은 공격 후 감시/추적 항공기를 원래 임무의 잔여 구간으로 재결합시키는 계획을 만든다. 남은 작업량이 너무 작거나 진행률 조건을 넘으면 새 계획을 만들지 않고 `0001` no-replan notice로 끝날 수 있다.

전달 정책은 `0903` 직접 갱신이며 `0702` fallback은 억제된다.

## 다음 협업 임무

대표 파일:

- `modules/mission_planning/pipelines/next_collab_replan_pipeline.py`
- `modules/mission_planning/pipelines/next_collab_replan_pipeline_impl.py`
- `modules/mission_planning/runtime/next_collab_replan_store.py`
- `modules/mission_planning/runtime/next_collab_line_runner.py`
- `modules/mission_planning/runtime/next_collab_division_runner.py`
- `modules/mission_planning/planners/next_collab_division/*`

분기 조건:

- `triggerType == "nextCollaborativeMission"`
- Monitoring에서 `0803 execute=1` 또는 관련 UI 흐름으로 다음 협업 임무 요청 생성
- formation-flight 성격의 입력 임무는 제외

주요 result dataclass는 `NextCollabPipelineResult`이며, plan IDs, option names, plan meta map, 생성된 input package/IMP/path IDs, log path를 담는다.

파이프라인은 line/area 협업 임무를 현재 0401/turn view 기반 entry coordinate에서 다시 시작할 수 있도록 새 InputMissionPlan package, IndividualMissionPlan, FlightPath, MissionPlan을 만든다. 상세가 0902에 없으면 `next_collab_replan_store`에서 plan ID 기준으로 보완한다.

짧은 line이나 유효 row가 없는 경우에는 short-line skip fallback으로 `0001` 안내를 낼 수 있다.

전달 정책은 `0903` 직접 갱신이며 `0702` fallback은 억제된다.

## 촬영 스케줄/품질 속도 보정

대표 파일:

- `modules/mission_planning/pipelines/imaging_schedule_replan_pipeline.py`
- `modules/mission_planning/pipelines/imaging_schedule_replan_pipeline_impl.py`
- `modules/common/imaging_schedule_replan_store.py`

지원 trigger type:

- `imagingScheduleDeviation`
- `qualityMonitorSep`

`imagingScheduleDeviation`은 촬영 일정 또는 센서 운용 상태가 기대와 어긋났을 때 사용된다. `qualityMonitorSep`는 품질 모니터가 촬영 속도 보정을 요청할 때 사용된다.

주요 result에는 다음 정보가 들어간다.

- 새/대체 waypoint ID
- 제거/anchor waypoint ID
- search speed scale/direction
- trimmed sweep points
- trigger type
- log path

품질 속도 보정은 옵션 후보를 비우고 직접 갱신 성격으로 처리된다. 전달 정책은 둘 다 `0903` 직접 갱신이지만, `qualityMonitorSep`는 `0702` fallback도 억제한다. 일반 촬영 스케줄 보정은 fallback이 억제되지 않을 수 있다.

## 경로 이탈

대표 파일:

- `modules/mission_planning/pipelines/path_deviation_replan_pipeline.py`
- `modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py`
- `modules/common/path_deviation_replan_store.py`

분기 조건:

- `triggerType == "pathDeviation"`
- Monitoring의 turn radius/path deviation 판정이 warning/hold 조건을 만족
- 현재 plan과 대상 aircraft/waypoint가 유효
- synthetic alternate waypoint 또는 entry coordinate가 준비됨

파이프라인은 현재 위치와 대체 waypoint를 기준으로 기존 경로 일부를 제거하고 새 waypoint를 삽입한다. result에는 생성된 plan/path/IMP ID와 함께 `removed_waypoint_id`, `inserted_waypoint_id`가 포함된다.

상세가 0902에 없으면 `path_deviation_replan_store`에서 plan ID 기준으로 불러온다. synthetic alternate waypoint ID는 Monitoring 쪽에서 자기 재계획 체인을 만들지 않도록 guard된다.

전달 정책은 `0903` 직접 갱신이지만 `0702` fallback은 기본적으로 억제되지 않는다.

## 선행 임무와 선행 종료 후 합류

대표 파일:

- `modules/mission_planning/pipelines/prior_mission_pipeline_impl.py`
- `modules/common/prior_replan_store.py`
- `modules/common/prior_target_rediscovery_store.py`
- `modules/mission_planning/runtime/prior_tracking_state.py`

선행 임무 재계획은 `replanLevel == 4`가 핵심이다. `0202` 선행 임무 목록, `0401` 기반 선행 종료/복귀 상태, prior target rediscovery 상태를 사용한다.

선행 종료 후 합류는 다음 조건일 때 prior 분기 안에서 먼저 시도된다.

- `replan_level == 4`
- `replanDetail.trigger == "0401"`
- `replanDetail.triggerType == "priorClosedResume"`

일반 prior와 prior-post-rejoin 모두 current/source plan lineage와 현재 aircraft 상태가 중요하다. 코드에는 collaborative resume, mission trim, prior tracking state, attack tracking lineage helper가 함께 들어 있다.

전달 정책은 다음과 같다.

- 일반 prior: `0903` 직접 갱신, `0702` fallback 가능
- prior-post-rejoin: `0903` 직접 갱신, `0702` fallback 억제

## 일반 재계획 fallback

전용 파이프라인이 처리하지 않은 요청은 일반 divide-and-pattern 기반 재계획으로 넘어간다.

주요 구성 요소:

- `modules/mission_planning/pipelines/current_remaining_hybrid.py`
- `modules/mission_planning/pipelines/current_remaining_hybrid_replan.py`
- `modules/mission_planning/pipelines/general_remaining_hybrid_replan.py`
- `modules/mission_planning/pipelines/recon_specialized_pipeline.py`
- `modules/mission_planning/runtime/aircraft_parallel_0303.py`
- `modules/mission_planning/runtime/source_artifact_cache.py`

일반 fallback은 다음 작업을 수행한다.

1. mission area replan store의 잔여 영역 snapshot 적용
2. 완료 임무, 단일 좌표, `isDone`, whitelist 외 임무 필터링
3. `inputMissionType` 누락/0 타입 추론
4. line width 및 타입별 파라미터 보정
5. current remaining hybrid 요청 생성
6. 정찰 특화 후보(option code 4) 생성
7. variant parallel 조건에 따라 후보 병렬 실행
8. 새 MissionPlan/OptionInfo/FlightPath 저장

`current_remaining_hybrid`는 현재 수행 중인 협업 임무의 남은 구간을 live aircraft entry coordinate에서 다시 만들기 위해 next-collab planner를 재사용한다. `general_remaining_hybrid_replan`은 `0401`/`0802` 계열에서 현재 첫 미완료 input mission의 line/area geometry를 이어받는다.

정찰 특화 후보는 option code `4`, split width `600m`, fixed FOV `15deg`, sweep separation scale `0.50`을 기본으로 사용한다.

## 파이프라인별 전달 요약

| 파이프라인 | 직접 0903 | 0702 fallback 억제 | no-op 가능 |
| --- | --- | --- | --- |
| 공격 특화/공격 제외 | 상황별 | 상황별 | 예 |
| 공격 종료 후 합류 | 예 | 예 | 예 |
| 다음 협업 임무 | 예 | 예 | 예 |
| 촬영 스케줄 | 예 | 아니오 | 예 |
| 품질 속도 보정 | 예 | 예 | 예 |
| 경로 이탈 | 예 | 아니오 | 예 |
| 선행 임무 | 예 | 아니오 | 예 |
| 선행 종료 후 합류 | 예 | 예 | 예 |
| 일반 fallback | 상황별 | 아니오 | 예 |

분석할 때는 "파이프라인이 선택됐는가", "result가 empty/no-op인가", "0903이 나갔는가", "0702 fallback이 의도된 것인가"를 분리해서 본다.
