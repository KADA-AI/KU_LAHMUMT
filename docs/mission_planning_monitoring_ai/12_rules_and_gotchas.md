# 규칙과 주의사항

마지막 재정리: 2026-05-04

현재 코드에서 자주 헷갈리는 규칙만 모은 문서다.

## 재계획 분기 순서

Mission Planning 재계획 우선순위:

1. attack
2. post-attack rejoin
3. next-collab
4. imaging/quality
5. path deviation
6. prior/prior-post-rejoin
7. general fallback

`post-attack rejoin`은 `0402 attackClosedDestroyed`라서 공격 메시지를 쓰지만 일반 attack pipeline에서 제외된 뒤 별도 분기로 간다.

`prior-post-rejoin`은 독립 최상위 분기가 아니다. prior 분기 안에서 `replanLevel == 4`, `trigger=0401`, `triggerType=priorClosedResume`일 때만 실행된다.

## 설정 경로

Monitoring runtime 기준:

- 현재값: 루트 `replan_settings.json`
- 기본값: 루트 `replan_settings_defaults.json`

`modules/monitoring/replan_settings.json`은 runtime 기준이 아니다. 값이 다르면 루트 파일을 우선한다.

Mission Planning 항공기/계획 파라미터 기준:

- 현재값: 루트 `uav_params.json`
- legacy fallback: `modules/mission_planning/MissionPlanner/uav_params.json`

현재 주요 runtime 값:

| 항목 | 값 |
| --- | --- |
| `post_attack_rejoin` | true |
| `next_collab` | true |
| `quality_speed` | false |
| `imaging_schedule` | false |
| `path_deviation.watch_angle_deg` | 60 |
| `path_deviation.warning_angle_deg` | 90 |
| `replan_queue.release_on_option_info` | false |

## 0301은 전체 계획이 아니다

현재 `0301` payload는 보통 전체 계획 JSON이 아니라 `{timestamp, source, missionPlanID}` 성격의 참조다. downstream은 active DB에서 `MissionPlan/<id>.json`과 하위 파일을 읽어야 한다.

`0305 missionPlanningStatus`는 현재 MMR 송신 기준 `1=수행 중`, `2=완료` 흐름이다. 오래된 주석이나 UI 코드가 다른 관성을 가질 수 있으므로 실제 payload 값을 확인한다.

## 실제 plan 파일과 run log를 구분한다

| 파일 | 의미 |
| --- | --- |
| `MissionPlan/<id>.json` | 실제 계획 |
| `DSS_Internal/missionPlan_<id>.json` | run log/요약 |

이름이 비슷하지만 역할이 다르다. 시뮬레이터 plan load는 `MissionPlan/<id>.json`을 기준으로 하위 파일을 읽는다.

## 0902 sidecar는 필수 경계다

`0902`에는 nFusion 스키마에 없는 확장 `replanDetail`이 포함될 수 있다. 손실을 막기 위해 `DSS_Internal/replan_request_transport` sidecar가 저장된다.

Mission Planning 수신기는 sidecar를 병합할 수 있으므로, 실제 파이프라인 입력을 보려면 raw message와 sidecar를 함께 확인한다.

## 0701만으로 큐가 풀리지 않는다

현재 `replan_queue.release_on_option_info=false`다. 따라서 `0701` 옵션 정보가 왔다고 active 재계획이 자동 완료되지 않는다.

큐 완료 신호로 봐야 하는 것은 보통 다음이다.

- `0903`
- `0702`
- `0001`
- timeout
- 일부 `0305` 상태 변화

## 직접 갱신과 옵션 흐름을 구분한다

다음 파이프라인은 옵션 팝업 없이 `0903` 직접 갱신으로 끝날 수 있다.

- post-attack rejoin
- next-collab
- quality speed
- prior-post-rejoin

path deviation, imaging schedule, prior mission은 직접 갱신을 하면서도 `0702` fallback이 붙을 수 있다. 옵션 UI가 안 뜨거나 `0702`가 안 보인다고 무조건 오류로 보지 않는다.

## 초기 계획도 0902 흐름을 탄다

초기 계획은 단순 버튼 실행만이 아니다. Monitoring이 system mode `2` 전환 시 `0902` 형식의 초기임무재계획 요청을 만들어 Mission Planning에 넣는다.

초기 계획 문제를 볼 때는 `0201/0203` 입력뿐 아니라 `0902`, `0305 status=2`, `0901` 또는 `0903`까지 확인한다.

## dedicated pipeline은 조건이 좁다

전용 파이프라인은 보통 다음 조건이 필요하다.

- 관련 source/current plan ID가 정확함
- detail store가 존재하거나 0902 detail이 충분함
- 대상 aircraft/target/input mission이 현재 plan에 연결됨
- pending plan 후보가 너무 많지 않음
- tracking state가 현재 plan lineage와 맞음

조건이 안 맞으면 전용 파이프라인이 실패하지 않고 fallback 또는 no-op으로 끝날 수 있다.

## 경로 이탈은 turn view에 의존한다

path deviation은 단순히 현재 좌표가 경로에서 떨어졌는지만 보지 않는다. `TurnRadiusMonitorTab`의 선회 반경, spiral, alternate waypoint, predicted entry coordinate가 중요하다.

synthetic alternate waypoint는 자기 자신을 다시 path deviation으로 만들지 않도록 guard된다. 이 guard를 제거하면 반복 재계획이 생길 수 있다.

## current remaining은 독립 트리거가 아니다

`current_remaining`은 source tag라기보다 RTB/forced/path/general 재계획에 붙는 context다. `currentRemainingCollaborativeReplan` 상세에 entry coordinate와 aircraft list가 들어간다.

이 정보가 없으면 Mission Planning은 현재 잔여 임무 하이브리드를 만들지 못하고 일반 fallback으로 갈 수 있다.

## target detection은 큐에서 특별 취급된다

target detection은 `target_dispatch_delay_ms=800` 동안 묶이거나 merge될 수 있다. post-attack rejoin이 들어오면 active target detection이 option-suppressed로 선점될 수 있다.

`targetInfo.json`의 `isUsed`, `isIgnored`, `isDestroyed`와 attack slot 상태가 재계획 생성 여부에 직접 영향을 준다.

## ID tracker를 수동으로 되돌리지 않는다

ID allocator는 기존 DB 파일과 다음 내부 상태를 함께 본다.

- `DSS_Internal/id_tracker.json`
- `DSS_Internal/path_usage.json`
- `DSS_Internal/waypoint_usage.json`
- 기존 `MissionPlan`/`FlightPath`

파일을 수동 삭제하거나 tracker를 과거로 돌리면 ID jump 또는 충돌이 생길 수 있다.

## JSON 저장 후 내용이 바뀔 수 있다

`json_io.write_json()`은 FlightPath 저장 시 `Source`를 `source`로 정규화하고 UAV speed weight를 적용할 수 있다. 메모리의 raw dict와 디스크 결과가 완전히 같다고 가정하지 않는다.

## 한글 라벨은 보조 정보다

일부 코드/설정 파일에는 한글 option label이 인코딩 깨짐처럼 보일 수 있다. 분기 판단은 다음 값을 우선한다.

- message ID
- `trigger`
- `triggerType`
- `replanLevel`
- `source_tag`
- mission/input/aircraft/target ID

한글 라벨은 UI 표시와 사람이 읽는 설명용으로만 확인한다.

## active DB가 다르면 모든 것이 어긋난다

Mission Planning, Monitoring, Simulation이 같은 active DB root를 보고 있어야 한다. 한 프로세스만 다른 DB를 보면 `0903`은 성공했는데 웹 지도는 예전 plan을 보여주는 식의 문제가 생긴다.

확인 대상:

- `current_scenario.json`
- `KU_MISSION_DB_ROOT`
- `%ACTIVE_DB_ROOT%/MissionPlan`
- `%ACTIVE_DB_ROOT%/DSS_Internal/module_logs`
- `/api/mission/plan_load` 대상 plan ID
