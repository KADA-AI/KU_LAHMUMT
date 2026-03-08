# 임무계획/재계획 최종 체크리스트 (2026-02-26)

## 점검 범위
- 모듈: `modules/monitoring`, `modules/mission_planning`
- 방식: 코드 경로 추적 + 문법 점검(`py_compile`)
- 참고: 본 문서는 코드 구현 점검 기준이며, 실기동 E2E 시뮬레이션 결과와는 구분됨

## 체크리스트
| 항목 | 상태 | 확인 내용 | 근거 |
|---|---|---|---|
| 협업임무 추가 | 구현됨 | `0201` 신규/변경 수신 시 실행 모드(3/4)에서 `0902` 재계획 요청 생성 | `modules/monitoring/monitoring_gui.py:1660`, `modules/monitoring/logic/input_refresh_replan.py:104` |
| 협업임무 순서변경 | 구현됨(조건부) | `0201` payload fingerprint가 바뀌면 재계획 트리거됨. 순서변경이 payload에 반영되어 들어와야 감지됨 | `modules/monitoring/logic/input_refresh_replan.py:68`, `modules/monitoring/logic/input_refresh_replan.py:120` |
| 협업기저임무 재수행 | 구현됨 | `0803 execute=2` 수신 시 다음 `0201` 도착을 대기 후 `0902` 발송 | `modules/monitoring/logic/collab_reexecute.py:118`, `modules/monitoring/logic/collab_reexecute.py:154`, `modules/monitoring/monitoring_gui.py:1895` |
| 좌표지향 선행임무 | 구현됨 | `0202 missionType=1` 처리 후 level-4 `0902` 생성, 전용 prior 파이프라인으로 임무 삽입 | `modules/monitoring/logic/prior_mission_replan.py:139`, `modules/mission_planning/mission_planning_gui.py:4224` |
| 표적추적 선행임무 | 구현됨 | `0202 missionType=2 + targetOrientation.targetID`를 기반으로 targetInfo 좌표/감시기 조회 후 autoTracking 임무 생성 | `modules/monitoring/logic/prior_mission_replan.py:174`, `modules/mission_planning/prior_mission_pipeline_impl.py:405`, `modules/mission_planning/prior_mission_pipeline_impl.py:855` |
| 강제명령(임무대기) | 구현됨 | `0802 mandatoryType=1` 수신 시 30초 유예 후 조건 충족 시 지연 재계획 발행 | `modules/monitoring/logic/forced_command_replan.py:188`, `modules/monitoring/logic/forced_command_replan.py:232` |
| 강제명령(임무복귀) | 구현됨 | `0802 mandatoryType=3`은 hold 해제/복귀 처리(즉시 재계획 없음) | `modules/monitoring/logic/forced_command_replan.py:204`, `modules/monitoring/logic/forced_command_replan.py:236` |
| 표적 발견 후 재계획(공격 특화/공격 배제) | 구현됨 | `0402`에서 `공격 특화`, `공격 배제` 2옵션 생성. MP에서 `공격 특화`는 공격 전용 파이프라인, 나머지(공격 배제)는 일반 파이프라인으로 분리 처리 | `modules/monitoring/logic/target_detection_replan.py:41`, `modules/monitoring/logic/target_detection_replan.py:531`, `modules/mission_planning/mission_planning_gui.py:2382`, `modules/mission_planning/mission_planning_gui.py:2434` |
| 무인기 연료부족 | 부분 구현 | `0401` 기반 연료 경고(`0504`) 전송은 구현됨. 연료부족 자체를 재계획(`0902`)으로 자동 승격하는 로직은 없음 | `modules/monitoring/logic/fuel_warning.py:89`, `modules/monitoring/monitoring_gui.py:1745`, `modules/monitoring/monitoring_gui.py:1754` |
| 무인기 소실 | 조건부 구현 | 소실 자체(`0401.health`)를 직접 재계획 트리거로 쓰진 않음. 다만 소실 후 `0201.availableAircraftList`가 갱신되어 들어오면 비가용 반영 + `0201` 재입력 트리거로 재계획이 실행됨 | `modules/monitoring/logic/mission_update.py:536`, `modules/monitoring/monitoring_gui.py:1658`, `modules/monitoring/logic/input_refresh_replan.py:104` |
| 무인기 임무장비 고장 | 구현됨(RTB 기반) | UAV `flightMode=5(RTB)` 감지 시 level-1 `0902` 재계획 요청 생성 | `modules/monitoring/logic/rtb_replan.py:63`, `modules/monitoring/logic/rtb_replan.py:93`, `modules/monitoring/logic/rtb_replan.py:175` |

## 문법 점검 결과
- `py_compile` 성공(11개 핵심 파일):  
  - `modules/monitoring/monitoring_gui.py`  
  - `modules/monitoring/logic/input_refresh_replan.py`  
  - `modules/monitoring/logic/collab_reexecute.py`  
  - `modules/monitoring/logic/prior_mission_replan.py`  
  - `modules/monitoring/logic/forced_command_replan.py`  
  - `modules/monitoring/logic/target_detection_replan.py`  
  - `modules/monitoring/logic/fuel_warning.py`  
  - `modules/monitoring/logic/rtb_replan.py`  
  - `modules/mission_planning/mission_planning_gui.py`  
  - `modules/mission_planning/prior_mission_pipeline_impl.py`  
  - `modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py`

## 최종 판단
- 요청 항목 기준으로 **핵심 재계획 흐름은 대부분 구현 완료**.
- 다만 재계획 관점에서 보완 필요:
  - `무인기 연료부족`: 현재는 경고 전송 중심
  - `무인기 소실`: 전용 자동 재계획 트리거 미구현
