# 런타임 경계와 데이터 루트

마지막 재정리: 2026-05-04

이 시스템은 하나의 코드베이스 안에 Mission Planning GUI, Monitoring GUI, Simulation/Integration Service, 공통 DB 경로/저장소가 함께 있다. 기능을 분석할 때는 "어느 프로세스가 파일을 쓰고, 어느 메시지를 보내며, 어느 쪽이 최종 계획을 적용하는가"를 먼저 나눠야 한다.

## 실행 단위

| 영역 | 대표 파일 | 역할 |
| --- | --- | --- |
| Mission Planning GUI | `modules/mission_planning/mission_planning_gui.py` | 0201/0203/0902 입력을 받아 계획/재계획 산출물을 생성하고 0301/0305/0903/0702 계열 메시지를 송신 |
| Mission Planning 파이프라인 | `modules/mission_planning/pipelines/*.py` | 공격, 합류, 협업, 촬영, 경로이탈, 선행, 일반 재계획 로직 |
| Mission Planning 런타임 | `modules/mission_planning/runtime/*.py` | 캐시, 로그, 병렬 0303 생성, 재계획 런타임 상태, 전용 저장소 접근 |
| Monitoring GUI | `modules/monitoring/monitoring_gui.py` | 0401/0402/0802/0803/0201 등을 감시해 재계획 요청을 만들고 현재 계획 적용 상태를 관리 |
| Monitoring 로직 | `modules/monitoring/logic/*.py` | 재계획 트리거, 품질/경로/연료/RTB/선행/공격/협업 조건 판단 |
| Simulation Runtime | `modules/sim/runtime/sim_service.py` | 임무 실행, 에이전트 상태, 0401/0402 이벤트, 표적/위협/경로 진행 상태 생성 |
| Integration Service | `modules/sim/integration/integration_service.py` | nFusion 메시지 송수신, 주기 송신, 사용자 지정 페이로드 처리 |
| HTTP/Web | `modules/sim/server/http_server.py`, `modules/sim/web/js/*` | 시뮬레이터 웹 UI, 계획 로딩, 표적 조작, 모니터링 스냅샷 API |
| Common | `modules/common/*.py` | 활성 DB 경로, 스냅샷, 재계획 상세 저장소, 0902 사이드카 저장소 |

## 활성 DB 루트

활성 DB 루트는 `modules/common/db_paths.py`가 관리한다. Mission Planning, Monitoring, Simulation은 모두 이 모듈을 통해 현재 시나리오의 파일 위치를 맞춘다.

주요 하위 폴더는 다음과 같다.

| 폴더 | 역할 |
| --- | --- |
| `DSS_Internal` | 내부 상태, 스냅샷, 재계획 상세, 진단 로그, 큐/요청 보조 파일 |
| `FlightPath` | `0303` 비행 경로 JSON |
| `IndividualMissionPlan` | `0302` 개별 임무 계획 JSON |
| `InputMissionPlan` | `0201` 입력 임무 묶음 및 파이프라인용 변형 입력 |
| `MissionPlan` | `0301`/`0903` 전체 임무 계획 JSON |
| `MissionPlanOptionInfo` | `0305`/`0701` 옵션 안내 JSON |
| `MissionReferenceInfo` | `0304` 참조 정보 JSON |
| `VehicleStatus` | 시뮬레이터/모니터링 상태 스냅샷 |
| `mission_output` | 일부 계획 실행 산출물 및 호환 출력 |

`db_paths.activate_scenario`, `point_to_scenario`, `set_manual_db_root`, `ensure_db_payload`, `get_db_subpath`가 경로 제어의 중심이다. 특정 기능이 "파일을 못 찾는다"면 코드보다 먼저 활성 DB 루트와 시나리오 선택이 맞는지 확인한다.

## 공통 저장소

재계획 상세 정보는 대부분 `DSS_Internal` 아래의 전용 저장소에 남는다. Monitoring이 상세를 저장하고 0902에는 ID/요약만 싣는 경우가 있으므로, Mission Planning은 0902 내용과 저장소 내용을 함께 본다.

| 저장소 | 대표 파일 패턴 | 역할 |
| --- | --- | --- |
| mission area | `DSS_Internal/mission_area_replan/mission_area_snapshot_<missionPlanID>.json` | 현재 임무 영역/잔여 영역 스냅샷 |
| path deviation | `DSS_Internal/path_deviation_replan/path_deviation_detail_<missionPlanID>.json` | 경로 이탈 상세, 대체 웨이포인트, 이벤트 |
| imaging schedule | `DSS_Internal/imaging_schedule_replan/imaging_schedule_detail_<missionPlanID>.json` | 촬영 스케줄/품질 속도 보정 상세 |
| prior replan | `DSS_Internal/prior_replan/prior_detail_<missionPlanID>.json` | 선행 임무 폐쇄/재개 상세 |
| next collaboration | `DSS_Internal/next_collab_replan/*` | 다음 협업 임무 상세와 후보 입력 |
| replan transport | `DSS_Internal/replan_request_transport/replan_request_<timestamp>.json` | 0902 보조 저장/중복 제거 |
| latest 0401 | `DSS_Internal/latest_0401_agent_status.json` | 최신 에이전트 상태 스냅샷 |
| 0401 log | `DSS_Internal/log_0401_agent_status_sim.jsonl` | 0401 시계열 로그 |

`modules/common/agent_status_snapshot.py`는 최신 0401 저장뿐 아니라 마지막 유효 웨이포인트, 공격/선행 추적 상태 갱신에도 관여한다. 재계획 파이프라인이 "현재 위치"를 요구할 때 이 파일을 간접적으로 쓰는 경우가 많다.

## 메시지 경계

시스템에서 가장 중요한 경계는 다음 세 가지다.

1. Monitoring -> Mission Planning: `0902` 재계획 요청
2. Mission Planning -> Monitoring/Simulation: `0903` 계획 갱신
3. Monitoring/Simulation -> 상태 루프: `0401`, `0402`

`0902`에는 `replanLevel`, `reason`, `optionList`, `pendingOptionList`, `missionPlanIDList`, `inputMissionIDList`, `replanDetail` 등이 실린다. 다만 상세가 항상 모두 들어오는 것은 아니므로, Mission Planning은 `replanDetail.missionPlanID`, 현재 계획 ID, 저장소 파일을 조합해 컨텍스트를 복원한다.

`0903`은 직접 계획 갱신에 쓰인다. 일부 파이프라인은 옵션 선택 과정 없이 바로 `0903`을 보내며, 일부는 `0702` fallback 또는 사용자의 선택 응답을 기다린다. 직접 갱신 여부는 파이프라인마다 다르므로 `02`와 `05`의 전달 매트릭스를 확인한다.

## Integration/Simulation 경계

`modules/sim/integration/integration_service.py`는 수신/송신 메시지와 주기 송신을 묶는다. `0501`은 주기 송신 대상에 포함되어 있으며, `0902`/`0903`/`0803` 등은 시뮬레이터 상태 전환과 연결된다.

`modules/sim/runtime/sim_service.py`는 현재 계획을 로드하고, 에이전트 이동/촬영/공격/표적 탐지 상태를 갱신하며, 0401/0402 이벤트를 만든다. 웹 UI는 `modules/sim/server/http_server.py`의 `/api/mission/plan_load`, `/api/monitoring/*`, `/api/sim/*` 계열 API로 이 런타임을 조작한다.

## 설정 파일

Monitoring 재계획 설정은 프로젝트 루트의 두 파일을 구분해서 봐야 한다.

- `replan_settings_defaults.json`: 기본값
- `replan_settings.json`: 현재 실행 설정

`modules/monitoring/replan_settings.json` 파일도 존재하지만 `modules/monitoring/logic/replan_runtime_settings.py`의 `settings_path()`는 프로젝트 루트 파일을 반환한다. 현재 실행 설정은 기본값과 다를 수 있으므로 루트 파일을 우선 확인한다.

Mission Planning 쪽은 환경 변수도 일부 영향을 준다.

Mission Planning 항공기/계획 파라미터의 canonical 설정은 프로젝트 루트 `uav_params.json`이며, legacy fallback은 `modules/mission_planning/MissionPlanner/uav_params.json`이다. ID 상태는 active DB의 `DSS_Internal/id_tracker.json`, `waypoint_usage.json`, `path_usage.json`에 저장된다.

| 환경 변수 | 의미 |
| --- | --- |
| `REPLAN_0303_AIRCRAFT_PARALLEL` | 항공기별 0303 병렬 생성 |
| `REPLAN_VARIANT_PARALLEL` | 일반 재계획 옵션 후보 병렬 실행 사용 여부 |
| `REPLAN_VARIANT_WORKERS` | 일반 재계획 후보 워커 수 |
| `REPLAN_VARIANT_WAYPOINT_BLOCK_SIZE` | 병렬 0303 웨이포인트 블록 크기 |
| `REPLAN_RECON_WORKER_CAP` | 정찰 특화 후보 워커 상한 |
| `REPLAN_ATTACK_EXCLUSION_PARALLEL` | 공격 특화와 공격 제외 후보 병렬 생성 여부 |
| `REPLAN_RUNTIME_ARTIFACT_MODE` | 런타임 산출물/로그 보존 방식 |
| `REPLAN_0902_SIDECAR_MODE` | 0902 보조 저장 방식 |

환경 변수 기본값은 코드에서 해석되므로, 재현 로그를 작성할 때는 실행 시점의 환경과 설정 JSON을 같이 보존하는 것이 좋다.

## 분석 순서

1. 활성 DB 루트가 어디인지 확인한다.
2. `DSS_Internal/latest_0401_agent_status.json`과 관련 재계획 저장소 상세를 확인한다.
3. Monitoring의 큐/트리거 로그에서 `0902`가 만들어졌는지 본다.
4. Mission Planning의 재계획 run log와 생성된 `MissionPlan`, `MissionPlanOptionInfo`를 확인한다.
5. `0903` 또는 `0702` 적용 후 시뮬레이터가 어떤 계획을 로드했는지 확인한다.

이 순서를 지키면 "트리거 미발생", "0902 생성 후 큐 대기", "Mission Planning 파이프라인 스킵", "계획 산출 후 적용 실패"를 분리해서 볼 수 있다.
