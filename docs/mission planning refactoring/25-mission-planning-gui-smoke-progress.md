# 25. Mission Planning GUI Smoke Progress

## Scope

이번 수정은 Phase 0의 `mission_planning_gui.py` import/launch smoke 정의 항목이다. 전체 GUI cold start와 control-port 검증은 별도 TODO로 남아 있으므로, 이번 smoke는 안전한 import와 env opt-in headless launch만 확인한다.

## Changes

- `docs/mission planning refactoring/smoke_mission_planning_gui.py`를 추가했다.
- `modules/mission_planning/mission_planning_gui.py`에 `MISSION_PLANNING_GUI_SMOKE_LAUNCH=1` 전용 launch path를 추가했다.
- smoke launch path는 `MainWindow`를 생성하지 않는다. `QApplication`, shared stylesheet, `MainWindow` class contract, planner runtime source signature, event-loop tick만 확인하고 종료한다.
- 일반 `python modules/mission_planning/mission_planning_gui.py` 실행 경로는 기존처럼 `MainWindow()`를 생성하고 `app.exec_()`에 진입한다.

## Smoke Contract

`smoke_mission_planning_gui.py`는 다음을 확인한다.

- import smoke:
  - `modules.mission_planning.mission_planning_gui` import 가능
  - `KU_ROLE=mission`
  - `PROJECT_ROOT`가 repo root와 일치
  - `MainWindow`가 `QMainWindow` subclass
  - planner runtime source signature가 비어 있지 않음
- launch smoke:
  - `MISSION_PLANNING_GUI_SMOKE_LAUNCH=1`
  - `QT_QPA_PLATFORM=offscreen`
  - 직접 파일 경로 `python modules/mission_planning/mission_planning_gui.py` 실행
  - headless Qt event loop가 즉시 정상 종료

## Verification

이번 항목을 완료 처리하기 전에 아래 명령을 실행했다.

- `python "docs\mission planning refactoring\smoke_mission_planning_gui.py"`
- `python -m py_compile "modules\mission_planning\mission_planning_gui.py" "docs\mission planning refactoring\smoke_mission_planning_gui.py" "docs\mission planning refactoring\smoke_import_contract.py"`
- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `git diff --check -- "modules/mission_planning/mission_planning_gui.py" "docs/mission planning refactoring"`
