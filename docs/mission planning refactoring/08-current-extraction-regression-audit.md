# Current Extraction Regression Audit

요청: 이미 추출한 범위를 기준으로 한 번 더 검토하고, 빠진 항목과 기능 변화 가능성을 확인한다.

범위: `app/bootstrap.py`, `mission_control/planner_runtime.py`, `app/message_handlers/system_mode.py`, `input_packages.py`, `replan_requests.py`, 그리고 `mission_planning_gui.py`의 wrapper 연결부. 코드 이동/삭제를 추가로 진행하지 않고, 현재 추출본의 동작 동일성만 확인했다.

## 결론

현재 추출 범위에서 compile/import 수준의 회귀는 발견되지 않았다. `mission_planning_gui.py`의 기존 private 함수명은 wrapper로 유지되어 있고, helper 호출 뒤에도 0101/0102, 0201/0203, 0902의 대표 payload smoke가 통과했다.

단, 한 가지 작은 차이를 발견해 수정했다. `0201/0203` payload의 `Source/source` 로그 표시는 기존 코드가 원문 값을 그대로 사용했는데, helper가 strip/문자열화하면서 로그 표현이 바뀔 수 있었다. `extract_payload_source()`를 기존과 같이 truthy 원문 값을 그대로 반환하도록 되돌렸다.

## 직접 확인한 항목

- `configure_mission_role()`은 `KU_ROLE=mission`을 설정한다.
- `configure_mission_process_console()`은 기존과 같이 `ensure_console()`과 `install_process_file_logging("mission_planning")`을 호출한다.
- `modules/mission_planning` package `__init__`들은 role-sensitive side effect가 없다.
- planner runtime watch list는 18개, reload order는 12개로 유지된다.
- `_refresh_live_planning_helpers()`는 여전히 `mission_planning_gui.py`의 `globals()`를 넘겨 runtime function/class binding을 갱신한다.
- 0101 JSON/free-text payload에서 `systemMode`, `mode`, `modeCode`, `state` alias 추출이 유지된다.
- 0102 body template은 `Timestamp/timestamp` 제거, `source=MMR` fallback, `status=1` fallback, 새 `timestamp` 삽입을 유지한다.
- 0201/0203 latest input materialization은 non-empty core list가 있어야 JSON 파일을 준비한다.
- 0203 저장 디렉터리는 `MissionReferenceInfo`로 유지된다.
- 0902 plan id 우선순위는 `optionList/pendingOptionList` -> `missionPlanIDList` -> `replanDetail.missionPlanID` 순서로 유지된다.
- 0902 delay는 `collabReexecuteInputRefresh` short delay, `0402`/`attackClosedDestroyed` immediate, 일부 `0401` RTB류 55초 delay를 유지한다.

## 기능 변화 금지선

다음 항목은 지금 당장 수정할 대상이 아니라, 이후 리팩토링에서 “정리”하면 기능이 바뀔 수 있는 보존 계약이다.

1. `KU_ROLE=mission`은 가능한 한 wrapper 최상단 계약으로 유지한다. 지금은 helper import 뒤에 호출되지만, 그 사이 import는 side effect가 없다. 이후 `app/*` helper import에 role-sensitive side effect를 넣으면 안 된다.
2. console/file logging 설치 전 import 구간에 로그 emit, thread start, lifecycle event를 추가하면 `mission_planning` 로그 파일 계약에서 빠질 수 있다.
3. `_bootstrap_paths()`의 `sys.path` 삽입과 `os.chdir(root)`는 아직 `app/bootstrap.py`로 대체되지 않았다. `bootstrap.py`를 직접 entrypoint처럼 쓰면 bare import 계약이 깨진다.
4. `recon_specialized_pipeline.py`는 watch list에는 있으나 reload order/global rebinding 대상에는 없다. 현행 동작을 보존하려면 그대로 두고, hot-reload 보강은 별도 기능 변경으로 다뤄야 한다.
5. `_refresh_live_planning_helpers(namespace)`는 return value가 아니라 전달된 namespace를 직접 갱신하는 계약이다.
6. 0101 raw fallback은 lowercase `"systemMode": number`에 맞춰져 있다. case-insensitive, quoted number, nested key 허용은 호환성 확대처럼 보여도 동작 범위를 바꾸는 변경이다.
7. CTRL text mode에서 `"on"`/`"poweron"`은 mode `0`으로 해석되고, 이후 power-on side effect가 실행된다. 이를 대기모드나 현재 모드 유지로 바꾸면 흐름이 바뀐다.
8. 0202는 latest input cache/banner 대상이 아니다. 0202는 prior mission 경로에서 0902로 변환되는 별도 흐름으로 취급한다.
9. 0201/0203 payload는 package id만 있어서는 파일 materialize 대상이 아니며, non-empty core list가 필요하다.
10. 0902 `optionList`가 있으면 `pendingOptionList`는 보지 않는다. malformed `optionList`와 valid `pendingOptionList`를 병합하거나 fallback하면 후보 순서가 바뀐다.
11. 0902 `inputMissionIDList`는 dict item의 `inputMissionID`만 처리한다. raw int list 허용은 context 보존 범위를 바꾼다.
12. 0902 trigger/delay는 exact string match 기반이다. case/alias normalization 확대는 dispatch timing 변경이다.
13. init planning 중 0902는 terrain warmup/timing marker가 큐잉 전 먼저 실행된다. warmup을 deferred 이후로 미루면 준비 타이밍이 달라진다.
14. nFusion import, config 보정, MessageLibrary load, receive import, tab import, `MMR_ReceiveNode` 초기화 순서는 wrapper에서 유지한다.

## 보강해야 할 TODO

- 0101 parsing allow-list fixture: top-level alias, raw fallback, bool mapping, text mode `"on" -> 0`.
- 0202 exclusion fixture: 0202가 latest input cache/banner에 들어가지 않는지 확인.
- 0201/0203 core-list materialization fixture: empty list는 저장하지 않고 non-empty list만 저장.
- 0203 directory fixture: `MissionReferenceInfo` 경로 유지.
- 0201/0203 `Source/source` log fixture: truthy 원문 값 보존.
- 0902 malformed `optionList` fixture: `pendingOptionList` fallback이 일어나지 않는 현행 동작 고정.
- 0902 `inputMissionIDList` dict-only fixture.
- init-plan-running 0902 fixture: terrain warmup/timing marker가 deferred queue보다 먼저 예약되는 순서 고정.
- planner hot-reload fixture: `globals()` rebinding 방식과 watch-only `recon_specialized_pipeline.py` 현행 계약 기록.

## 실행한 검증

```text
python -m compileall modules\mission_planning\app modules\mission_planning\mission_control modules\mission_planning\mission_planning_gui.py
mission_planning extracted-helper smoke ok
```
