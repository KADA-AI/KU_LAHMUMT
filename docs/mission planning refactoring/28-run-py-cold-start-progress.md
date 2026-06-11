# 28. Run.py Cold Start Progress

## Scope

이번 수정은 Phase 0의 `run.py` cold start 전체 GUI launch/control-port smoke 작성 항목이다.

## Added Script

- `docs/mission planning refactoring/smoke_run_py_cold_start.py`

## Contract

스크립트는 실제 child GUI를 띄우지 않고 `run.py`의 `DashboardOrchestrator._launch_all_guis()`를 cold-start 경로로 실행한다. `subprocess.Popen`, stale process cleanup, console creation flags, `QTimer.singleShot`, UDP socket을 monkeypatch해서 launch/control 계약만 캡처한다.

확인하는 계약은 다음과 같다.

- cold start가 mission, monitor, decision, info 네 역할을 모두 launch한다.
- 실행 대상은 각각 `modules/mission_planning/mission_planning_gui.py`, `modules/monitoring/monitoring_gui.py`, `modules/decision_support/decision_support_gui.py`, `modules/info_manage/info_manage.py`이다.
- 모든 child process의 `cwd`는 repo root이고, `start_new_session=True`, `new_process_group=True` creation flag 계약을 유지한다.
- 모든 child env에 `KU_LAUNCHED_BY_DASHBOARD=1`, `KU_MISSION_DB_ROOT`, `PYTHONUNBUFFERED=1`, 역할별 `KU_CTRL_PORT`, `KU_WINDOW_OFFSET`, `KU_CONSOLE_TITLE`, `KU_START_HIDDEN=1`, `KU_HIDE_ON_CLOSE=1`, `KU_VIEWER_ONLY=0`이 들어간다.
- role process registry가 mission/monitor/decision/info key로 child process를 보관한다.
- cold start 후 초기화 모드/status refresh timer가 `1000`, `1000`, `1200` ms 순서로 예약된다.
- `_send_role_ctrl()`는 `127.0.0.1`의 역할별 port로 `show_window` payload를 보낸다.
- `_send_role_ctrl()`는 override port를 사용한 실제 localhost UDP 수신에서도 `{"cmd":"show_window","role":"mission"}` payload를 보낸다.
- 각 GUI entrypoint는 `start_ctrl_listener`, 역할별 `env_ctrl_port(default)`, `handle_window_control(... role=...)` 계약을 유지한다.
- `gui_process_control.handle_window_control()`은 역할별 alias target을 허용하고, 잘못된 target을 무시하며, `show_window`와 `hide_window` 명령을 fake window에 적용한다.

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python -m py_compile "docs\mission planning refactoring\smoke_run_py_cold_start.py"`
- `python "docs\mission planning refactoring\smoke_run_py_cold_start.py"`
- `python -m py_compile "docs\mission planning refactoring\smoke_run_py_cold_start.py" "docs\mission planning refactoring\smoke_import_contract.py"`
- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `git diff --check -- "docs/mission planning refactoring"`

## Next TODO

다음 미완료 TODO는 주요 pipeline import smoke 정의이다.
