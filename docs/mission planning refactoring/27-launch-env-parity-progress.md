# 27. Launch Env Parity Progress

## Scope

이번 수정은 Phase 0의 `run.py` 경유 launch와 `app/ui/main_window.py` 경유 launch env parity smoke 작성 항목이다.

## Added Script

- `docs/mission planning refactoring/smoke_launch_env_parity.py`

## Contract

스크립트는 실제 dashboard GUI를 띄우지 않고 `subprocess.Popen`, stale process cleanup, console flag 계산을 monkeypatch해서 mission launch 계약을 캡처한다.

확인하는 공통 계약은 다음과 같다.

- 두 경유 모두 `modules/mission_planning/mission_planning_gui.py`를 실행 대상으로 선택한다.
- 두 경유 모두 repo root를 child process `cwd`로 사용한다.
- 두 경유 모두 `KU_CTRL_PORT=45981`, `KU_START_HIDDEN=1`, `KU_HIDE_ON_CLOSE=1`, `KU_VIEWER_ONLY=0`, `KU_WINDOW_OFFSET=40,40`, `KU_CONSOLE_TITLE=KU Mission Planning Console`, `PYTHONUNBUFFERED=1`을 유지한다.
- 캡처한 각 launcher env에 `MISSION_PLANNING_GUI_SMOKE_LAUNCH=1`과 `QT_QPA_PLATFORM=offscreen`을 추가했을 때 `mission_planning_gui.py` smoke launch가 성공한다.

## Allowed Difference

현재 `run.py`는 child env에 `KU_LAUNCHED_BY_DASHBOARD=1`과 `KU_MISSION_DB_ROOT`를 직접 설정한다. `app/ui/main_window.py` 경유는 이 값을 직접 설정하지 않고 inherited env/db_paths 동기화에 의존한다. 이번 항목에서는 기능 변화를 피하기 위해 이 차이를 수정하지 않고 기록된 허용 차이로 남겼다.

`mission_planning_gui.py` 자체는 `configure_mission_role()`로 child process의 `KU_ROLE`을 `mission`으로 덮어쓰므로 parent process의 `KU_ROLE=decision` 상속 여부는 SW code baseline의 최종 조건을 깨지 않는다.

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python -m py_compile "docs\mission planning refactoring\smoke_launch_env_parity.py"`
- `python "docs\mission planning refactoring\smoke_launch_env_parity.py" --timeout-s 30`

## Pause Point

사용자 요청에 따라 이번 수정만 완료하고 여기서 일시 중단한다. 다음 미완료 TODO는 `run.py` cold start 전체 GUI launch/control-port smoke 작성이다.
