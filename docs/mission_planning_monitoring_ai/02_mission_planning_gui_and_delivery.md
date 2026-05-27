# Mission Planning GUI와 계획 전달

마지막 재정리: 2026-05-04

Mission Planning GUI는 단순한 화면이 아니라 계획 파이프라인 런타임이다. `modules/mission_planning/mission_planning_gui.py`가 입력 수신, 파이프라인 선택, 산출물 저장, 옵션 안내, 직접 계획 갱신까지 대부분의 흐름을 조정한다.

## 핵심 역할

Mission Planning GUI는 다음 일을 한다.

1. `0201`, `0203`, `0902` 계열 입력을 수신하거나 파일에서 읽는다.
2. 현재 활성 DB와 입력 캐시를 기준으로 계획 런타임을 구성한다.
3. 최초 계획 또는 재계획 파이프라인을 실행한다.
4. `MissionPlan`, `InputMissionPlan`, `IndividualMissionPlan`, `FlightPath`, `MissionReferenceInfo`, `MissionPlanOptionInfo`를 생성한다.
5. `0301`/`0305`/`0903`/`0702` 계열 메시지를 전송한다.
6. 파이프라인별 로그와 내부 상세를 `DSS_Internal`에 남긴다.

## 런타임 준비

`_refresh_live_planning_helpers()`는 개발 중 변경된 파이프라인/도우미 모듈을 다시 불러오는 hot reload 성격의 함수다. 현재 대상에는 일반 잔여 임무 하이브리드, 정찰 특화, 다음 협업 경로 생성, 선행/촬영/경로이탈/공격 후 합류 관련 모듈이 포함된다.

`_build_planner_runtime()`은 실제 파이프라인 실행에 필요한 함수/모듈 참조를 묶는다. 여기에는 다음 기능이 포함된다.

- 선행 임무 재계획
- 선행 종료 후 합류
- 촬영 스케줄/품질 속도 재계획
- 경로 이탈 재계획
- 다음 협업 임무 재계획
- 공격 종료 후 합류
- 일반 분할 계획/정찰 특화/현재 잔여 하이브리드

이 구조 때문에 특정 파이프라인의 코드를 고친 뒤 GUI를 완전히 재시작하지 않아도 반영되는 경로가 있다. 다만 모든 상태가 초기화되는 것은 아니므로, 재현 실험에서는 재시작 여부를 명확히 기록한다.

## 0902 수신과 컨텍스트 구성

`_handle_replan_received`는 `0902`를 받아 재계획 컨텍스트를 만든다. 컨텍스트에는 다음 정보가 들어간다.

| 항목 | 출처 |
| --- | --- |
| 후보 계획 ID | `optionList`, `pendingOptionList`, `missionPlanIDList`, `replanDetail.missionPlanID` |
| 입력 임무 ID | `inputMissionIDList`, `replanDetail.inputMissionIDList`, 저장소 상세 |
| 사유/레벨 | `reason`, `replanLevel` |
| 상세 | `replanDetail`, 공통 저장소 상세 |
| 현재/원본 계획 | `currentMissionPlanID`, `sourceMissionPlanID`, `replanDetail.currentPlanID` |
| 옵션명 | `optionList[].optionName`, 내부 파이프라인 결과 |
| 전달 제어 | `force_direct_update`, `suppress_0702_fallback` |

특히 `0402`는 공격 추천, 표적 탐지, 공격 종료 후 합류가 같은 메시지 ID를 공유한다. Mission Planning은 `triggerType`, target bundle, 현재 계획 ID, 추적 상태를 함께 보고 분기를 고른다.

## 재계획 스케줄링

`_schedule_replan_pipeline`은 수신된 컨텍스트를 GUI 이벤트 루프에 올리고, 실제 실행은 `_run_replan_pipeline_do`가 담당한다. 실행 결과는 파이프라인별 result dataclass 또는 일반 계획 결과로 정리된다.

실행 중에는 다음 런타임 보조 기능이 사용된다.

- `SourceArtifactCache`: 기존 계획/입력/경로 JSON 재사용과 참조
- `PipelinePhaseTimer`: 단계별 소요 시간 기록
- `write_json(skip_if_unchanged=True)`: 불필요한 파일 갱신 감소
- `mission_plan_file_logger`: 계획 산출물 로그
- `debug_artifacts`: 실패/중간 산출물 디버그 저장
- `aircraft_parallel_0303.py`: 항공기별 0303 생성 병렬화

## 계획 전달 방식

Mission Planning은 산출물 종류와 파이프라인 결과에 따라 여러 메시지를 보낸다.

| 메시지 | 의미 |
| --- | --- |
| `0301` | 최초 또는 일반 MissionPlan 전달 |
| `0305` | 옵션 정보/추천 정보 전달 |
| `0701` | 옵션 요청/옵션 안내 계열 |
| `0702` | 옵션 선택/적용 확인 계열 |
| `0901` | 임무 계획 요청 계열 |
| `0903` | 재계획 결과 직접 갱신 |
| `0001` | 재계획 불필요, 실패, 억제 등 안내 |

`_schedule_plan_delivery`가 전달 정책을 결정한다. 재계획 전용 파이프라인은 일반 옵션 선택 흐름을 우회하고 `0903` 직접 갱신을 쓰는 경우가 많다.

## 직접 갱신과 0702 fallback

현재 코드 기준 파이프라인별 전달 성격은 다음과 같다.

| 파이프라인 | `0903` 직접 갱신 | `0702` fallback 억제 | 비고 |
| --- | --- | --- | --- |
| 공격 특화 | 상황별 | 상황별 | 공격/공격 제외 후보를 옵션으로 제공할 수 있음 |
| 공격 종료 후 합류 | 예 | 예 | `0402`, `attackClosedDestroyed` |
| 다음 협업 임무 | 예 | 예 | `nextCollaborativeMission` |
| 촬영 스케줄 | 예 | 아니오 | 기본 촬영 스케줄 보정 |
| 품질 속도 보정 | 예 | 예 | `qualityMonitorSep` |
| 경로 이탈 | 예 | 아니오 | 대체 웨이포인트/현재 위치 기반 보정 |
| 선행 임무 | 예 | 아니오 | 0903 뒤 0702 fallback이 붙을 수 있음 |
| 선행 종료 후 합류 | 예 | 예 | 선행 분기 안의 특수 케이스 |
| 일반 재계획 | 보통 옵션/일반 전달 | 아니오 | 설정/결과에 따라 0305/0702 흐름 |

이 표는 실제 운영 분석에서 중요하다. `0903`을 받았는데 `0702`가 없다는 것이 항상 오류는 아니다. 반대로 `0702` fallback이 나오는 파이프라인에서 Monitoring이 이를 무시하거나 적용하지 못하면 계획 적용이 멈춘 것처럼 보일 수 있다.

## 0402 후속 공격 상세 보강

`_prepare_follow_up_attack_detail`은 현재 계획이 이미 공격 계획일 때 후속 `0402` 표적 탐지 상세를 공격 재계획에 맞게 보강한다. 대상 bundle은 최대 3개까지 구성될 수 있고, current/source plan ID가 함께 주입된다.

공격 종료 후 합류는 `_is_post_attack_rejoin_detail`로 구분된다.

- `trigger == "0402"`
- `triggerType == "attackClosedDestroyed"`

이 조건이면 `_should_use_attack_pipeline`은 false가 되어 일반 공격 특화 파이프라인이 아니라 post-attack rejoin 분기로 넘어간다.

## 입력 필터링과 일반 재계획 준비

일반 재계획 fallback은 단순히 기존 `0201`을 다시 쓰지 않는다. 현재 코드에는 다음 정리가 포함된다.

- 완료된 `0201` 입력 임무 제외
- `isDone` 임무 제외
- 단일 좌표만 있는 coordinate-only 입력 제외
- `inputMissionType` 누락/0일 때 geometry 기반 타입 추론
- 타입 1/7 라인 폭 보정
- mission whitelist 적용
- 임무 진행/영역 관리 스냅샷을 이용한 잔여 영역 override
- 현재 임무 잔여 구간 하이브리드 입력 생성
- `_filtered` payload와 `DSS_Internal/replan_inputs/0201_override_source...` 저장

이 단계에서 입력이 사라지면 파이프라인은 정상적으로 "재계획 불필요" 또는 실패 안내를 낼 수 있다. 재계획 품질을 볼 때는 최종 `MissionPlan`만 보지 말고 필터링된 입력 파일도 같이 봐야 한다.

## 실패와 no-op

전용 파이프라인은 조건이 맞지 않으면 조용히 일반 재계획으로 넘어가거나, `0001` no-replan notice를 보낼 수 있다. 예를 들어 공격 종료 후 합류는 남은 임무량이 너무 작거나 추적 assignment가 없으면 계획을 만들지 않는다.

따라서 "트리거가 왔는데 계획이 안 바뀜"은 다음 네 가지로 나눠 확인한다.

1. Monitoring 토글이 꺼져 있어 0902가 만들어지지 않음
2. 0902는 만들어졌지만 큐에서 지연/중복 제거/선점됨
3. Mission Planning 전용 파이프라인 조건이 맞지 않아 스킵됨
4. 계획은 만들어졌지만 0903/0702 적용 또는 시뮬레이터 로드가 실패함
