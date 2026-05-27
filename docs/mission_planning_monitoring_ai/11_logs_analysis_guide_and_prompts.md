# 로그 분석 가이드와 AI 프롬프트

마지막 재정리: 2026-05-04

이 문서는 재계획/모니터링 장애를 AI나 개발자가 빠르게 분석할 때 쓰는 절차와 질문 템플릿이다.

## 공통 분석 절차

1. 문제 시각과 대상 aircraft/missionPlanID/targetID를 정한다.
2. active DB root를 확인한다.
3. 루트 `replan_settings.json`의 toggle을 확인한다.
4. 원본 이벤트 메시지를 찾는다.
5. Monitoring queue에 들어갔는지 본다.
6. `0902` sidecar를 확인한다.
7. Mission Planning이 어떤 파이프라인을 선택했는지 본다.
8. 산출된 MissionPlan/FlightPath 관계를 검증한다.
9. `0903`/`0702`/`0001` 중 어떤 종료 신호가 있었는지 본다.
10. 시뮬레이터가 새 plan을 로드했는지 본다.

## 문제 유형별 판단

### 트리거가 안 생김

확인할 것:

- 해당 toggle이 루트 설정에서 켜져 있는가
- system mode가 조건에 맞는가
- 입력 메시지가 실제 들어왔는가
- coordinator guard가 막았는가
- path deviation/quality처럼 tab 계산 상태가 필요한가

AI에게 줄 정보:

```text
다음 파일을 기준으로 왜 재계획 트리거가 생성되지 않았는지 분석해줘.
- replan_settings.json
- DSS_Internal/module_logs/monitoring.log
- DSS_Internal/latest_0401_agent_status.json
- 관련 0401/0402/0802/0803 payload
관심 source tag는 <source_tag>이고 대상 aircraft/target/plan은 <id>야.
```

### 0902는 있는데 계획이 안 나옴

확인할 것:

- sidecar에 `replanDetail`이 제대로 있는가
- Mission Planning이 받은 payload에 current/source plan ID가 있는가
- 전용 파이프라인 조건이 맞는가
- dedicated pipeline이 skip 후 fallback으로 갔는가
- fallback 입력 필터링 후 남은 mission이 있는가
- `0001` no-replan notice가 있었는가

AI 프롬프트:

```text
이 0902 재계획 요청이 Mission Planning에서 어떤 파이프라인을 탔는지 추적해줘.
다음 키를 기준으로 봐줘: trigger=<trigger>, triggerType=<triggerType>, replanLevel=<level>, missionPlanID=<id>.
sidecar, mission_planning.log, 전용 store detail, MissionPlan 산출물을 비교해서
스킵/실패/no-op/fallback 중 무엇인지 결론을 내줘.
```

### 계획은 있는데 적용이 안 됨

확인할 것:

- `MissionPlan/<id>.json`이 실제 존재하는가
- `0301`/`0903` payload의 plan ID가 그 파일과 같은가
- `0702 ignore=2`가 있었는가
- 직접 갱신 파이프라인이라 옵션 없이 끝나는 것이 정상인가
- `/api/mission/plan_load`가 성공했는가
- DB 파일 관계가 깨지지 않았는가

AI 프롬프트:

```text
MissionPlan <id>가 생성됐지만 UI/시뮬레이터에 적용되지 않았어.
MissionPlan, MissionPlanOptionInfo, 0903/0702 payload, sim plan_load 로그를 비교해서
메시지 전달 문제인지 DB 파일 관계 문제인지 구분해줘.
```

### target detection이 밀리거나 사라짐

확인할 것:

- target detection toggle
- `targetInfo.json`의 `isUsed`, `isIgnored`, `isDestroyed`
- queue의 target dispatch delay/merge
- post-attack rejoin 선점 여부
- active target detection option suppression 여부
- attack slot/manned aircraft 상태

AI 프롬프트:

```text
0402 target detection 재계획이 큐에서 왜 merge/suppress/지연됐는지 봐줘.
targetInfo.json, replan_queue_manager 로그, 0902 sidecar, 0001 notice를 기준으로
target bundle과 queue signature 변화를 정리해줘.
```

### path deviation이 반복되거나 안 나옴

확인할 것:

- `path_deviation` toggle
- `TurnRadiusMonitorTab` view
- `watch_angle_deg=60`, `warning_angle_deg=90`
- alternate waypoint 생성 여부
- synthetic alternate waypoint guard
- 0702 적용 직후 guard
- active queue option 대기 상태

AI 프롬프트:

```text
path deviation 재계획이 발생하지 않거나 반복 발생해.
0401 시계열, turn radius monitor 상태, path_deviation_replan detail,
replan_queue 상태를 기준으로 guard 조건과 alternate waypoint 상태를 분석해줘.
```

### post-attack rejoin이 안 나옴

확인할 것:

- 루트 `post_attack_rejoin` toggle
- `0402 triggerType=attackClosedDestroyed`
- target destroyed/closed 상태
- `attack_tracking_state.json`
- `attack_assignment_state.json`
- remaining ETA와 active progress skip percent
- post-attack이 target detection을 선점했는지

AI 프롬프트:

```text
공격 종료 후 합류 재계획이 기대와 다르게 동작했어.
0402 payload, attack tracking/assignment state, mission_area_replan snapshot,
post_attack_rejoin pipeline log를 비교해서 조건 미충족인지 no-op인지 산출 실패인지 알려줘.
```

### next collab이 안 나옴

확인할 것:

- `next_collab` toggle
- `0803 execute=1`
- mission recommendation popup에서 보낸 payload
- current remaining/turn view entry coordinate
- formation-flight input type skip 여부
- `next_collab_replan` store detail

AI 프롬프트:

```text
0803 execute=1 이후 next collaborative mission 재계획이 생성되지 않았어.
0803 payload, monitoring log, next_collab_replan detail, turn radius/current remaining context를 보고
트리거 생성 실패인지 Mission Planning pipeline skip인지 구분해줘.
```

## 로그 수집 템플릿

분석 요청 전 최소 정보:

```text
시간대:
active DB root:
관심 missionPlanID:
관심 aircraftID:
관심 targetID:
source tag:
trigger/triggerType:
기대 동작:
실제 동작:
첨부/확인 파일:
- replan_settings.json
- current_scenario.json
- DSS_Internal/module_logs/monitoring.log
- DSS_Internal/module_logs/mission_planning.log
- DSS_Internal/replan_request_transport/*
- DSS_Internal/latest_0401_agent_status.json
- MissionPlan/<id>.json
- MissionPlanOptionInfo/<id>.json
```

## AI에게 맡길 때 주의할 점

AI에게는 "왜 안 됐어?"보다 "어느 단계에서 멈췄는지 분류해줘"라고 요청하는 것이 좋다.

좋은 분류:

- trigger not created
- queued but not dispatched
- dispatched but pipeline skipped
- pipeline failed
- no-replan/no-op
- plan created but not delivered
- delivered but not applied
- applied but sim load failed

이 분류를 먼저 만들면 로그가 많아도 원인을 좁힐 수 있다.

## 한 번에 물어볼 수 있는 종합 프롬프트

```text
이 재계획 케이스를 end-to-end로 분석해줘.

기준:
- active DB root: <path>
- 시간대: <time range>
- trigger/type: <trigger>/<triggerType>
- source tag: <source>
- missionPlanID/current/source: <ids>
- aircraft/target: <ids>

확인할 파일:
- replan_settings.json
- current_scenario.json
- DSS_Internal/module_logs/monitoring.log
- DSS_Internal/module_logs/mission_planning.log
- DSS_Internal/replan_request_transport/*
- DSS_Internal/latest_0401_agent_status.json
- 관련 전용 store
- MissionPlan/MissionPlanOptionInfo/FlightPath

원하는 출력:
1. 실제 이벤트 타임라인
2. 큐 stage 변화
3. Mission Planning 분기 선택
4. 산출물 생성 여부
5. 0903/0702/0001 결과
6. 원인 분류
7. 코드 수정이 필요하면 수정 후보 파일과 이유
```

## 결론 작성 형식

분석 결과는 다음 형식으로 정리한다.

```text
결론: <한 줄 요약>

단계별 판단:
1. Trigger: 생성/미생성, 근거
2. Queue: dispatch/대기/merge/suppress, 근거
3. Planning: 선택 파이프라인, skip/fail/success 근거
4. Delivery: 0305/0903/0702/0001 상태
5. Apply: Monitoring/Simulation 적용 여부

원인:
- <주 원인>

수정/운영 조치:
- <필요한 조치>
```
