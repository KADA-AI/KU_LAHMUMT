# Monitoring 런타임과 상태

마지막 재정리: 2026-05-04

Monitoring GUI는 MSM 런타임이다. `modules/monitoring/monitoring_gui.py`의 `MainWindow`가 메시지 수신, 탭 갱신, 재계획 트리거, 0902 큐, 현재 plan 적용 상태를 조정한다.

## MainWindow 역할

`MainWindow`는 초기화 시 다음 리스너/폴러를 구성한다.

| 메시지 | 주요 처리 |
| --- | --- |
| `0101` | 시스템 모드/시나리오 상태 |
| `0201` | 입력 임무 갱신, reexecute/input refresh 재계획 |
| `0202` | 선행 임무 재계획 |
| `0305` | 계획 옵션/진행 상태 수신 |
| `0401` | agent 상태, progress, coverage, RTB, path deviation, quality, schedule |
| `0402` | target detection, target destroyed, post-attack rejoin |
| `0701` | option info/option requested |
| `0702` | option 선택/적용 결과 |
| `0802` | 강제 명령 |
| `0803` | 다음 협업/재수행 사용자 실행 |
| `0903` | 새 mission plan 직접 적용 |
| `0001` | 실패/no-op/suppression notice |

대부분의 폴러는 250ms 수준이고, `0501` 송신은 시스템 모드 3에서 200ms 주기로 동작한다. watchdog은 1000ms 주기로 송신 상태를 확인한다.

## 주요 탭

| 탭/파일 | 역할 |
| --- | --- |
| `monitoring_visualization_tab.py` | 실시간 임무 표시, progress tracker, 0501/0503, current remaining context |
| `mission_schedule_tab.py` | 임무 일정, waypoint ETA, imaging schedule, 관련 toggle |
| `mission_progress_area_management_tab.py` | 영역/선형 coverage, area cut/restore, replan snapshot |
| `replan_management_tab.py` | `replan_settings.json` 편집 UI |
| `replan_queue_tab.py` | active/queued/history 재계획 큐 상태 표시 |
| `turn_radius_monitor_tab.py` | 선회 반경, spiral, alternate waypoint, entry coordinate |
| `quality_monitor_tab.py` | 공간 해상도 품질 모니터와 quality speed 설정 |

탭은 단순 표시용이 아니다. path deviation, next collab, current remaining, quality speed 같은 기능은 탭 내부 계산 결과를 재계획 상세에 사용한다.

## 0401 처리 루프

`_on_rx_0401`은 Monitoring의 중심 루프다. 한 번의 0401에서 다음 처리를 수행한다.

1. agent state 정규화
2. latest 0401 snapshot 저장
3. mission progress/coverage 갱신
4. visualization/schedule/area/quality/turn-radius 탭 갱신
5. fault/availability/RTB 상태 갱신
6. fuel warning 및 0504 판단
7. prior mission close/resume 판단
8. path deviation 판단
9. quality speed 판단
10. imaging schedule 판단
11. forced/RTB availability override 반영
12. mission recommend 조건이면 0503/0502 송신

`modules/common/agent_status_snapshot.py`는 최신 0401을 `DSS_Internal/latest_0401_agent_status.json`에 저장하고 `DSS_Internal/log_0401_agent_status_sim.jsonl`에 누적한다.

## 0201/0202 처리

`_on_rx_0201`은 입력 임무 갱신을 받아 availability, reexecute coordinate, input refresh를 처리한다. system mode와 toggle이 맞으면 `InputRefreshReplanCoordinator`가 level 3 0902 payload를 만든다.

`_on_rx_0202`는 선행 임무 재계획 흐름이다. `PriorMissionReplanCoordinator`가 level 4 payload를 만들고, prior 관련 상세는 common store에 저장될 수 있다.

## 0402 처리

`_on_rx_0402`는 target detection과 post-attack rejoin을 함께 다룬다.

- 신규/actionable target이면 target detection level 2 재계획
- destroyed/closed target이면 post-attack rejoin 후보
- target info store 갱신
- attack assignment/tracking state 갱신
- post-attack rejoin을 일반 target detection보다 먼저 큐에 넣을 수 있음

`_queue_0402_replan_payloads`는 post-attack, target detection, 기타 payload를 분리해 큐에 넣는다. 큐 안에서는 post-attack이 active target detection을 option-suppressed로 밀어낼 수 있다.

## 0802/0803 처리

`0802`는 강제 명령이다.

| `mandatoryType` | 의미 |
| --- | --- |
| `1` | forced wait, availability false 및 hold |
| `2` | forced return, level 1 재계획 및 RTB suppression 가능 |
| `3` | forced mission resume, hold 해제 또는 재결합 재계획 |

`0803`은 사용자 실행 명령이다. 웹 추천 팝업의 "다음"은 `execute=1`로 다음 협업 임무 흐름을 만들고, "재수행"은 `execute=2`로 reexecute/collab 흐름을 만든다.

## 0903/0702 plan 적용

`_on_rx_0903`은 Mission Planning이 직접 갱신한 새 plan을 적용한다. `0702 ignore=2`도 plan 적용을 의미한다. DB 파일이 아직 준비되지 않은 경우 pending으로 보류하고 poller가 재시도할 수 있다.

`0702 ignore=1`은 현재 plan 유지다. 이 경우 적용 이벤트가 아니므로 재계획 큐 상태와 UI 표시를 따로 확인해야 한다.

plan 적용 후에는 visualization, schedule, quality, area/progress 탭이 새 plan 기준으로 갱신된다.

## 0501/0502/0503/0504

`0501`은 MSM 상태 주기 송신이다. system mode 3에서 `MonitoringVisualizationTab.build_0501_payload(timestamp_ms, source="MSM")`로 payload를 만들고 200ms 주기로 push한다.

진단 파일은 다음 위치에 남는다.

- `DSS_Internal/monitoring_diagnostics/msm_0501_*.json`
- `DSS_Internal/monitoring_diagnostics/latest_msm_0501_diag.json`

`0503`은 mission recommendation이다. 협업기저임무 완료 또는 다음 임무 가능 상태를 확인해 `systemRecommend`를 보낸다. 특정 recommend 값에서는 `0502` mission end request도 함께 보낸다.

`0504`는 연료/상태 계열 경고 송신에 연결된다.

## Runtime 설정

실제 Monitoring runtime 설정 경로는 프로젝트 루트의 다음 파일이다.

- `replan_settings.json`
- `replan_settings_defaults.json`

`modules/monitoring/logic/replan_runtime_settings.py`의 `settings_path()`는 프로젝트 루트를 반환한다. `modules/monitoring/replan_settings.json` 파일도 존재하지만, 현재 runtime 판단의 기준으로 보지 않는다.

설정은 defaults와 현재 파일을 병합한 뒤 normalize된다. GUI의 재계획 관리 탭은 이 runtime 설정을 읽고 저장한다.

## 상태 저장 위치

| 상태 | 위치 |
| --- | --- |
| latest 0401 | `DSS_Internal/latest_0401_agent_status.json` |
| 0401 로그 | `DSS_Internal/log_0401_agent_status_sim.jsonl` |
| target info | `DSS_Internal/targetInfo.json` |
| option suppression | `DSS_Internal/suppress_option_request.json` |
| coverage | `DSS_Internal/coverage_progress.json` |
| sweep progress | `DSS_Internal/sweep_progress.json` |
| attack tracking | `DSS_Internal/attack_tracking_state.json` |
| attack assignment | `DSS_Internal/attack_assignment_state.json` |
| path deviation detail | `DSS_Internal/path_deviation_replan/*` |
| mission area replan | `DSS_Internal/mission_area_replan/*` |

메모리 상태는 `ReceiveStorage`, `LogicStorage`, `PushStorage`, `ReplanQueueManager`, 각 tab tracker/cache에 나뉘어 있으므로 파일만으로 모든 상태가 복원되지는 않는다.
