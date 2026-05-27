# 2026-05-07 재계획 오류 점검 체크리스트

점검 범위:

- `Logs\target여러개`: 다중 표적 발견 이후 공격/복귀/후속 공격 재계획 흐름
- `Logs\0501문제`: 0501 `currentInputMissionID` 회귀와 현재임무 재수행 대상 오염 가능성
- 관련 코드: `modules/monitoring`, `modules/mission_planning`, `modules/common`

## 이번 코드 반영 요약

- [x] 슬롯 부족으로 0402 공격 재계획을 못 보낼 때 표적 후보를 버리지 않고 `attack_assignment_state.json`의 `deferred_attack_targets_by_input_package`에 보존하도록 수정.
- [x] 슬롯 부족 분기는 실제 0902가 나간 것이 아니므로 `_target_trigger_history` cooldown을 찍지 않도록 수정.
- [x] 0903/0702 처리 뒤 deferred 표적이 있으면 기존 target detection queue 경로로 재평가해 후속 공격 0902를 다시 생성하도록 수정.
- [x] post-attack rejoin plan 적용 뒤에도 `postAttackRejoinContext.source_plan_id`를 따라 후속 공격 source lineage를 찾도록 수정.
- [x] 0501 outbound current는 첫 미완료 input fallback보다 progress/active/effective current를 우선하고, 같은 input package 안에서 `7 -> 3`처럼 앞쪽 input으로 회귀하면 guard 로그를 남기고 직전 current를 유지하도록 수정.
- [x] `collabReexecuteInputRefresh`의 `replanDetail.currentInputMissionID`는 `inputMissionIDList[0]` 대신 모니터링 active current resolver 결과를 우선 사용하도록 수정.
- [x] 0501 heartbeat cache는 cached `currentMissionPlanID`가 현재 plan과 다르면 timestamp만 바꿔 재송신하지 않고 폐기하도록 수정.
- [ ] 실로그 재시험: `Logs\target여러개`와 동일하게 1001~1006 순차 발견/파괴 흐름에서 deferred 후속 공격 0902가 이어지는지 확인.
- [ ] 실로그 재시험: `Logs\0501문제`와 동일하게 700000009 적용 직후 0501 currentInputMissionID가 3으로 회귀하지 않는지 확인.

## 1. 다중 표적 발견 후 후속 공격 재계획

### 확인한 로그 흐름

- 최초 `0402`에서 `1001`, `1002`가 묶여 들어왔고, Monitoring은 `targets=[1001, 1002]` bundle로 공격 재계획을 생성했다.
- 이때 `0902`는 `planIds=700000003, 700000004`로 전송됐다. `700000003`은 공격 특화, `700000004`는 공격 배제 옵션이다.
- `0702`로 공격 옵션 `700000003`이 적용됐고, 공격 임무 확정 로그가 남았다.
- 이후 `1003`, `1004`, `1005`, `1006`까지 들어왔지만 Monitoring 로그에는 `replan skipped: attack slots exhausted`가 반복된다.
- `1001` 파괴 후 `700000005`는 생성됐지만, 내용은 `1001` 격파 확인 후 복귀/재합류 재계획이다. 남은 `1002~1005`를 대상으로 한 공격 재계획은 아니다.
- 이후 새 `MissionPlan`은 `700000006`이 확인되지만, 이것은 `0401` 경로이탈 재계획이다.

### 최종 상태 증거

- `Logs\target여러개\SBC3\DSS_Internal\targetInfo.json`
  - `1001`: `isDestroyed=true`, `isUsed=1`
  - `1002`: 생존, `isUsed=1`
  - `1003~1006`: 생존, `isUsed=0`
- `Logs\target여러개\SBC3\DSS_Internal\attack_tracking_state.json`
  - UAV 5의 `1001` tracking은 `active=false`로 clear됨
  - UAV 4의 `1002` tracking은 `active=true`로 남음
- `Logs\target여러개\SBC3\DSS_Internal\attack_assignment_state.json`
  - `used_manned_by_input_package.100 = [2, 3]`
  - 공격 가능 유인기 슬롯 2대가 모두 사용 중으로 남아 있음

### 현재 코드상 해결된 부분

- `target_detection_replan.py`는 `0402` destroyed target을 감지해 `triggerType=attackClosedDestroyed` 복귀 재계획 payload를 만든다.
- `post_attack_rejoin_pipeline.py`는 복귀 재계획에서 tracking assignment를 clear하고, 조건이 맞으면 `release_manned_used()`를 통해 유인기 공격 슬롯을 해제한다.
- `replan_queue_manager.py`는 `0701`에서 바로 queue를 완료하지 않고, 기본적으로 `0903` 또는 `0702`를 기다린다. 옵션 선택 전 다음 재계획이 섞이는 문제는 완화되어 있다.
- `mission_planning_gui.py`에는 `followUpAttackMode=True`로 남은 표적 bundle을 다시 구성하는 보강 코드가 있다.

### 아직 남은 위험

현재 로그와 코드 기준으로는 **완전 해결로 판단하기 어렵다.**

핵심 이유는 `target_detection_replan.py`의 슬롯 검사 순서다. 현재 구조는 공격 슬롯이 0이면 후보 표적을 queue/backlog에 보존하지 않고 바로 종료한다.

관련 코드:

- `modules\monitoring\logic\target_detection_replan.py`
  - `get_used_manned_ids(package_id)`로 사용 슬롯 확인
  - `available_slots <= 0`이면 `attack slots exhausted` 로그 후 return
- `modules\mission_planning\pipelines\post_attack_rejoin_pipeline.py`
  - `_release_attack_slots_if_tracking_closed()`는 같은 plan lineage에 active tracking이 남아 있으면 release를 지연한다.

따라서 `1001`이 파괴되어도 `1002` tracking이 active이면 유인기 슬롯 해제가 지연될 수 있고, `1003~1005`는 이미 슬롯 소진 시점에 큐에 보류되지 않았기 때문에 후속 공격 재계획으로 이어지지 않는다.

### 수정 방향

- `attack slots exhausted`일 때도 표적 후보를 버리지 말고 deferred target backlog로 보존한다.
- post-attack rejoin 완료 또는 공격 슬롯 해제 시점에 backlog를 다시 평가해 `followUpAttackMode=True` 공격 재계획을 생성한다.
- `isUsed=1`의 의미를 분리한다.
  - `reserved_in_bundle`: 최초 bundle에 포함됐지만 아직 독립 공격이 끝나지 않은 상태
  - `attack_assigned`: 실제 유인기 공격 대상으로 확정된 상태
  - `destroyed/closed`: 공격 완료 후 닫힌 상태
- 유인기 슬롯 해제 정책을 명확히 한다.
  - 현재는 active tracking이 하나라도 남으면 전체 release가 지연된다.
  - 순차 후속 공격이 목적이면 파괴 완료된 target에 대응되는 유인기 슬롯만 부분 release하거나, active tracking이 남아도 새 공격 가능한 슬롯을 계산할 수 있어야 한다.
- `post_attack_rejoin`은 복귀/재합류만 담당하고, 후속 공격은 별도 normal target detection queue item 또는 follow-up attack queue item으로 분리한다.

### 체크리스트

- [ ] `1001/1002` 최초 bundle 이후 `1003~1005`가 들어왔을 때 `attack slots exhausted`만 남지 않고 deferred backlog가 생성되는가?
- [ ] `1001` 파괴 후 `700000005` 같은 post-attack rejoin이 끝나면 backlog 재평가가 실행되는가?
- [ ] 후속 공격 `0902`에 `followUpAttackMode=true`가 들어가는가?
- [ ] 후속 공격 `0902`의 `attackTargetList`에 생존 표적 `1002~1005` 중 정책상 공격해야 할 표적이 들어가는가?
- [ ] `1001` 파괴 후에도 `1002` tracking이 살아 있는 경우, `1003~1005` 재계획이 막히는지 별도 회귀 테스트가 있는가?
- [ ] `targetInfo.isUsed=1`인 생존 표적이 후속 공격 후보에서 의도대로 포함/제외되는가?
- [ ] `attack_assignment_state.json`에서 닫힌 공격 슬롯이 적절히 해제되는가?
- [ ] 생성된 후속 공격 MissionPlan이 `post_attack_rejoin`이 아니라 공격 옵션 MissionPlan으로 분류되는가?

## 2. 0501 currentInputMissionID 회귀 문제

### 확인한 로그 흐름

`Logs\0501문제\SBC3\0501\0501.json`에서 아래 변화가 실제로 확인된다.

| timestamp | currentMissionPlanID | currentInputMissionID |
| --- | ---: | ---: |
| 831373815579 | 700000008 | 7 |
| 831373815795 | 700000008 | 7 |
| 831373815829 | 700000008 | 7 |
| 831373816014 | 700000008 | 7 |
| 831373816083 | 700000009 | 3 |
| 831373816472 | 700000009 | 3 |

`831373816014`에서 `831373816083`까지는 69ms 차이다. 실제 임무가 정상적으로 `7 -> 3`으로 되돌아갔다고 보기 어렵고, MissionPlan/InputMissionPlan 소스가 바뀌면서 current 계산 기준이 바뀐 것으로 보는 것이 타당하다.

### 직접 원인 후보

`700000009`는 `inputMissionPackageID=2`를 사용한다. `Logs\0501문제\SBC3\InputMissionPlan\2.json` 기준 미완료 input은 다음과 같다.

- `3`
- `8`
- `9`
- `10`
- `11`
- `12`

현재 코드에는 “첫 미완료 input”을 current로 선택하는 경로가 있다.

- `modules\monitoring\logic\mission_update.py`
  - `_select_next_pending_id()`가 `isDone=false`인 첫 항목을 반환한다.
  - `build_uav_mission_view()`가 이 값을 `current_input_mission_id`로 넣는다.
- `modules\monitoring\gui\tabs\monitoring_visualization_tab.py`
  - 0501 payload 생성 시 mission view와 progress snapshot을 사용한다.
  - active input을 확정하지 못하면 첫 미완료 fallback이 current로 들어갈 수 있다.

결과적으로 새 package 2가 적용되면서 0501 current가 실제 수행 중인 `7`이 아니라 package 2의 첫 미완료인 `3`으로 회귀한 것으로 보인다.

### 재수행 문제와 연결되는 지점

현재임무 재수행 흐름도 같은 위험을 가진다.

- `modules\monitoring\logic\collab_reexecute.py`
  - 0201의 미완료 input ID를 정렬해서 `inputMissionIDList`로 만든다.
- `modules\monitoring\monitoring_gui.py`
  - `collabReexecuteInputRefresh` context 부착 시 `inputMissionIDList`의 첫 번째 값을 `currentInputMissionID`로 사용한다.

따라서 0501 또는 0201 package가 이미 `3`을 current처럼 보이게 만든 상태에서는, “현재임무 재수행”도 실제 active input이 아니라 첫 미완료 input을 기준으로 잡을 위험이 있다.

### heartbeat/cache 관련 판단

0501 heartbeat는 주원인이라기보다 증상을 헷갈리게 만드는 요소다.

- `monitoring_gui.py`는 cached 0501 payload를 heartbeat로 재전송하면서 timestamp만 새로 찍을 수 있다.
- 따라서 plan 전환 직후 old payload와 new payload가 매우 가까운 timestamp로 섞여 보일 수 있다.
- 다만 이번 `7 -> 3` 회귀는 `currentMissionPlanID`도 `700000008 -> 700000009`로 같이 바뀌므로, 단순 timestamp 지연만이 아니라 새 MissionPlan/current 계산 기준 변경 문제로 보는 것이 맞다.

### 수정 방향

- 운용 중 current input의 단일 source of truth를 정한다.
  - 우선순위: 실제 progress tracker active input, 0401/mission progress 기반 active individual mission의 related input, 명시적 current snapshot
  - “첫 미완료 input”은 초기 로딩 fallback으로만 사용한다.
- 0501 currentInputMissionID에 회귀 guard를 둔다.
  - 같은 시나리오/연속 운용 중 `7 -> 3`처럼 과거 input으로 내려가는 전환은 explicit reset, 새 package 재시작, 또는 `7` 완료 근거가 없으면 막고 경고 로그를 남긴다.
- `0201 inputRefresh` 재계획 대상에서 현재 active input보다 과거인 미완료 항목을 그대로 current 후보로 쓰지 않는다.
- `collabReexecuteInputRefresh` payload에 `replanDetail.currentInputMissionID`를 명시한다.
  - 이 값은 `inputMissionIDList[0]`이 아니라 trigger 시점의 active current id를 사용한다.
- 0501 cache에는 plan generation guard를 둔다.
  - cached payload의 `currentMissionPlanID/inputMissionPackageID`가 현재 mission view와 다르면 heartbeat 재송신 전에 폐기한다.

### 체크리스트

- [ ] `831373816014 -> 831373816083` 같은 100ms 이내 current rollback을 잡는 진단 로그가 있는가?
- [ ] `700000009` 적용 직후에도 active input이 `7`이면 0501이 `3`으로 회귀하지 않는가?
- [ ] `InputMissionPlan package=2`에 `3`이 미완료로 남아 있어도 current 계산이 첫 미완료 fallback으로 떨어지지 않는가?
- [ ] `collabReexecuteInputRefresh` 0902의 `replanDetail.currentInputMissionID`가 active current id로 들어가는가?
- [ ] `monitoring_gui.py`의 reexecute context 부착이 `inputMissionIDList[0]` 대신 active current resolver를 쓰는가?
- [ ] heartbeat 0501과 GUI tick 0501이 서로 다른 plan generation payload를 섞어 보내지 않는가?
- [ ] current rollback이 발생하면 ICD 송신은 기존처럼 유지하되, GUI/로그에 알람이 남는가?

## 공통 회귀 테스트 제안

- 다중 표적 시나리오
  - `1001~1005`를 순차/동시로 넣는다.
  - 유인기 2대가 모두 할당된 뒤 `1001`만 destroyed 처리한다.
  - 기대 결과: `1001` post-attack rejoin 이후 생존 표적 후속 공격 판단이 실행된다.
- 0501 current 회귀 시나리오
  - current input이 `7`인 상태에서 새 0201 package가 `3,8,9,10,11,12` 미완료를 포함하도록 한다.
  - 기대 결과: 0501 current는 active 근거 없이 `3`으로 내려가지 않는다.
- 현재임무 재수행 시나리오
  - 위 상태에서 execute=2를 넣는다.
  - 기대 결과: reexecute 0902의 current context는 `inputMissionIDList[0]`이 아니라 실제 active current id를 사용한다.

## 이번 점검에서 실행한 확인

- `Logs\target여러개` 실제 `0402`, `0902`, `0903`, `0702`, `targetInfo`, `attack_tracking_state`, `attack_assignment_state` 흐름 확인
- `Logs\0501문제` 실제 0501 timestamp/currentMission/currentInput 변화 확인
- `InputMissionPlan\2.json`, `MissionPlan\700000009.json`, `replan_request_831373799533.json`, `replan_request_831373827791.json` 확인
- 관련 Python 파일 syntax compile 확인 완료

## 결론

- 1번 다중 표적 문제는 “공격 완료 후 복귀/clear” 쪽은 개선되어 있으나, 후속 공격 후보를 슬롯 소진 상태에서 보존/재개하는 구조가 아직 약하다. 동일 조건에서는 다시 누락될 가능성이 있다.
- 2번 0501 문제는 새 0201/InputMissionPlan package 적용 후 current를 첫 미완료 input으로 다시 계산하는 구조가 핵심 원인으로 보인다. current source of truth와 rollback guard를 넣는 방향으로 수정해야 한다.
