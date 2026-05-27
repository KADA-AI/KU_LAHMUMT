# Mission Planning / Monitoring AI 문서 인덱스

마지막 재정리: 2026-05-04  
검증 기준 코드: `modules/mission_planning`, `modules/monitoring`, `modules/common`, `modules/sim`

이 문서 묶음은 Mission Planning GUI, Monitoring GUI, 시뮬레이터, 공통 DB/메시지 계층이 재계획 AI 흐름을 어떻게 구성하는지 코드 기준으로 정리한 것이다. 이전 문서의 설명과 현재 코드가 다른 부분, 특히 재계획 분기 순서, 모니터링 토글, 경로이탈 설정값, 직접 갱신 여부를 현재 구현에 맞춰 갱신했다.

## 빠른 결론

현재 시스템은 Monitoring이 이벤트를 수집해 `0902` 재계획 요청을 만들고, Mission Planning이 `0902`의 `replanDetail`, 선택 후보, 현재/원본 계획 ID를 해석해 전용 파이프라인 또는 일반 분할 계획 파이프라인을 실행한다. 산출된 계획은 옵션 정보(`0305`/`0701`)와 계획 갱신(`0903`), 필요 시 선택 확인(`0702`)을 통해 다시 Monitoring/시뮬레이터 쪽으로 적용된다.

현재 Mission Planning의 재계획 파이프라인 우선순위는 다음 순서다.

1. 공격 특화 및 공격 제외 병렬 후보
2. 공격 종료 후 합류(`0402`, `attackClosedDestroyed`)
3. 다음 협업 임무(`nextCollaborativeMission`)
4. 촬영 스케줄/품질 속도 보정(`imagingScheduleDeviation`, `qualityMonitorSep`)
5. 경로 이탈(`pathDeviation`)
6. 선행 임무(`0401`/`0802`) 및 선행 종료 후 합류(`0401`, `priorClosedResume`)
7. 일반 재계획 및 현재 잔여 임무 하이브리드/정찰 특화 후보

주의할 점은 공격 종료 후 합류와 선행 종료 후 합류가 단순히 "공격" 또는 "선행" 설명에 묶여 있지 않다는 점이다. `post_attack_rejoin`은 공격 특화보다 먼저 실행되는 것이 아니라 공격 특화 조건에서 제외된 뒤 별도 분기에서 처리된다. `prior-post-rejoin`은 선행 임무 분기 안에서 `replan_level == 4`이고 `trigger=0401`, `triggerType=priorClosedResume`일 때만 실행된다.

## 문서 구성

- `01_runtime_boundaries_and_data_roots.md`  
  실행 경계, 활성 DB 루트, 공통 저장소, 시뮬레이터/GUI/메시지 경계.

- `02_mission_planning_gui_and_delivery.md`  
  Mission Planning GUI의 런타임 구성, 입력 수신, 재계획 컨텍스트 구성, 계획 전달 방식.

- `03_mission_planning_ids_and_schema.md`  
  MissionPlan/InputMissionPlan/IndividualMissionPlan/FlightPath ID 규칙과 JSON 관계.

- `04_initial_planning_pipeline.md`  
  최초 계획 생성 흐름, 0201/0203 입력 처리, 옵션 생성, 0301~0305 출력.

- `05_mission_planning_replanning_pipelines.md`  
  현재 코드 기준 재계획 분기 순서와 전용 파이프라인별 기능 정리.

- `06_monitoring_runtime_and_state.md`  
  Monitoring GUI의 리스너, 폴러, 탭, 상태 저장, 0401/0402/0501 계열 처리.

- `07_monitoring_replan_triggers_and_queue.md`  
  재계획 트리거, 레벨, 토글, 큐 매니저, 중복 제거, 선점/지연 규칙.

- `08_message_and_file_interfaces.md`  
  메시지 ID별 역할, 0902/0903/0702 인터페이스, 활성 DB 파일 산출물.

- `09_ai_work_checklist.md`  
  AI/개발자가 코드를 수정하거나 로그를 분석할 때 확인해야 할 체크리스트.

- `10_logs_artifact_map.md`  
  `DSS_Internal`, 계획 산출물, 재계획 저장소, 진단 로그의 위치와 해석 순서.

- `11_logs_analysis_guide_and_prompts.md`  
  장애/품질/재계획 로그를 AI에게 분석시킬 때 쓰는 절차와 프롬프트 템플릿.

- `12_rules_and_gotchas.md`  
  현재 구현에서 자주 틀리는 규칙, 설정값, 분기 조건, 운영상 주의점.

## 코드 진입점

Mission Planning 쪽 핵심 파일은 다음과 같다.

- `modules/mission_planning/mission_planning_gui.py`
- `modules/mission_planning/pipelines/*.py`
- `modules/mission_planning/runtime/*.py`
- `modules/mission_planning/planners/next_collab_division/*`
- `modules/mission_planning/MissionPlanner/data_def/id_allocator.py`
- `modules/mission_planning/MissionPlanner/data_def/id_allocator_0202.py`

Monitoring 쪽 핵심 파일은 다음과 같다.

- `modules/monitoring/monitoring_gui.py`
- `modules/monitoring/logic/replan_queue_manager.py`
- `modules/monitoring/logic/*_replan.py`
- `modules/monitoring/gui/tabs/*.py`
- `replan_settings.json`
- `replan_settings_defaults.json`

공통/시뮬레이터 경계는 다음 파일을 먼저 본다.

- `modules/common/db_paths.py`
- `modules/common/agent_status_snapshot.py`
- `modules/common/*_replan_store.py`
- `modules/common/replan_request_transport_store.py`
- `modules/sim/integration/integration_service.py`
- `modules/sim/runtime/sim_service.py`
- `modules/sim/mission/mission_plan_loader.py`
- `modules/sim/server/http_server.py`

## 현재 모니터링 토글 요약

현재 실행 기준은 프로젝트 루트의 `replan_settings.json`이다. `modules/monitoring/replan_settings.json`도 존재하지만 runtime loader가 우선 읽는 파일은 아니다.

| 토글 | 현재값 | 의미 |
| --- | --- | --- |
| `input_refresh` | true | 0201 재입력/재실행 기반 재계획 |
| `prior_mission` | true | 선행 임무 기반 재계획 |
| `dl_risk` | false | 데이터링크 위험 재계획 |
| `target_detection` | true | 0402 표적 탐지/공격 추천 |
| `post_attack_rejoin` | true | 공격 종료 후 합류 |
| `forced_command` | true | 0802 강제 명령 |
| `rtb` | true | RTB/비정상 상태 |
| `path_deviation` | true | 경로 이탈 |
| `quality_monitor` | true | 품질 모니터 표시/판정 |
| `quality_speed` | false | 품질 기반 속도 재계획 |
| `imaging_schedule` | false | 촬영 스케줄 미준수 재계획 |
| `next_collab` | true | 다음 협업 임무 |
| `fuel_threshold` | false | 연료 임계값 |

기본값 파일(`replan_settings_defaults.json`)과 현재 실행 설정이 다를 수 있다. 코드 분석이나 실험 결과를 설명할 때는 항상 실제 실행 파일인 루트 `replan_settings.json`을 먼저 확인한다.

## 현재 경로 이탈 주요 설정

현재 로컬 설정의 핵심 값은 다음과 같다.

| 항목 | 값 |
| --- | --- |
| `turn_rate_threshold_dps` | 2.0 |
| `turn_window_s` | 8.0 |
| `stale_timeout_s` | 5.0 |
| `watch_angle_deg` | 60 |
| `warning_angle_deg` | 90 |
| `hold_s` | 2.0 |
| `release_min_distance_m` | 240 |
| `release_factor` | 2.3 |
| `alt_waypoint_trigger_s` | 2.0 |
| `alt_waypoint_lead_time_s` | 10.0 |
| `next_mission_entry_lead_time_s` | 10.0 |
| `turn_radius_30_m` | 340 |
| `turn_radius_40_m` | 450 |
| `turn_radius_50_m` | 560 |

모듈 내부 설정 파일에는 `45/70` 값이 남아 있을 수 있지만, 현재 runtime loader 기준은 루트 설정의 `60/90`이다.

## 문서 사용 방법

기능을 빠르게 파악하려면 `05`와 `07`을 먼저 읽는다. 장애 로그를 해석하려면 `10`에서 파일 위치를 찾고 `11`의 순서대로 이벤트를 재구성한다. 코드 수정 전에는 `09`와 `12`를 같이 확인해 ID 충돌, 직접 갱신 여부, 재계획 큐 중복 제거, 토글 상태를 점검한다.
