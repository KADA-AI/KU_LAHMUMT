# Review Log

총 3회 검토를 수행했다. 1회는 로컬 정적 분석, 2회는 서브 에이전트 교차 분석, 3회는 리팩토링 계획의 안전성 재검토다.

## Review 1. 로컬 구조/참조 분석

수행한 확인:

- `modules/mission_planning` 전체 파일 inventory
- top-level folder별 Python 파일 수
- 주요 대형 파일 line count
- 외부 `modules.mission_planning` import 검색
- `pipelines/`, `runtime/`, `MissionPlanner` public function 검색
- 테스트/pytest/unittest 존재 여부 검색
- root wrapper 패턴 확인

결론:

- active runtime은 이미 `pipelines`, `runtime`, `ui`로 일부 정리되어 있지만, `mission_planning_gui.py`가 너무 많은 application responsibility를 가진다.
- 외부 호출자는 `run.py`, `app`, `monitoring`, `common`, `sim`에 존재한다.
- formal regression test는 거의 없으므로 안전망 구축이 Phase 0이어야 한다.

## Review 2. 서브 에이전트 교차 분석

세 명의 서브 에이전트에게 별도 관점의 read-only 분석을 맡겼다.

### 기능 경계 분석

확인된 경계:

- GUI/orchestration: `mission_planning_gui.py`
- core mission generation: `MissionPlanner/AnS`, `MissionPlanner/data_def/d0301-d0304`
- enhanced optimization/search: `MissionPlanner/planning_enhanced`
- replanning/reactive behavior: `pipelines/*`
- runtime state/validation/adapters: `runtime/*`
- UI/API surfaces: `ui/*`, `planners/next_collab_division/*`
- config: `MissionPlanner/runtime_settings.py`, `MissionPlanner/config.py`

반영:

- 목표 구조에 `app`, `mission_control`, `replanning`, `engine`, `runtime`, `interfaces`, `ui`, `config`, `compat`를 제안했다.

### 의존성/진입점 분석

확인된 migration constraint:

- `mission_planning_gui.py` path/name 유지
- `MainWindow` 유지
- top-level shim 유지
- bare import `MissionPlanner`, `AnS`, `data_def` compatibility 유지
- `run_divide_and_pattern`, `d0301-d0304` builder contract 유지
- pipeline result field 유지
- 0902 payload key, replan-level, trigger string 유지
- 0301 -> 0305 -> 0901/0903 delivery ordering 유지
- generated artifact directories와 `DSS_Internal` state 위치 유지
- ID allocator band/counter/file-lock 유지

반영:

- `01-current-architecture.md`의 유지 계약과 `03-refactoring-roadmap.md`의 Phase 0/1에 반영했다.

### 삭제 후보/중복 분석

확인된 후보:

- root compatibility wrappers 일부
- `legacy/wrappers`, `legacy/compat_packages`, archived app/test/doc/static buckets
- generated output JSON
- `d0304 copy.py`
- TensorBoard event logs
- duplicate visualizer
- duplicate prototype `Dubins_Path.py`
- 반복되는 `_to_int`, `_to_float`, `_normalize_coordinate`류 helper
- 중복 flight-path payload loader

반영:

- `04-deletion-candidates.md`에 후보와 검증 조건을 분리했다.

## Review 3. 안전성/실행 순서 재검토

점검 기준:

- 사용자의 요구인 기능/로직별 묶음이 반영되었는가
- 재계획 관련 코드가 별도 축으로 정리되었는가
- 삭제 후보가 보수적으로 표현되었는가
- folder rename/move가 바로 실행 가능한 단계로 쪼개졌는가
- high-risk 계약이 문서에 남아 있는가
- 실제 코드 변경 전에 필요한 TODO가 명확한가

결론:

- 문서 기준으로는 바로 리팩토링 PR 계획을 시작할 수 있다.
- 단, 코드 이동 전 Phase 0 safety smoke 없이는 대규모 move/delete를 시작하지 않는 것이 맞다.
- 첫 implementation PR은 삭제가 아니라 `mission_planning_gui.py` 주변 safety smoke와 bootstrap/runtime extraction이 적절하다.

추가 확인:

- `docs/mission planning refactoring` 아래 6개 문서가 생성되었음을 확인했다.
- 문서의 핵심 키워드인 `mission_control`, `replanning`, `Phase 0`, `MainWindow`, `mission_planning_gui.py`, `Review 1/2/3` 포함 여부를 확인했다.
- 문서에서 언급한 핵심 경로인 `mission_planning_gui.py`, `MissionPlanner/AnS/mission_pipeline.py`, `d0301.py`, `d0302.py`, `d0303.py`, `d0304.py`, 주요 pipeline, `runtime/replan_validation.py`가 실제 존재함을 확인했다.
- 삭제 관련 표현은 `삭제 확정`이 아니라 `삭제 후보`와 `검증 후 삭제`로 유지되어 있음을 확인했다.

## 남은 리스크

- nFusion/.NET runtime과 PyQt GUI는 단순 import smoke만으로 충분하지 않다.
- 현재 워크트리에 로그/리소스 변경이 매우 많아 refactor PR은 반드시 코드 변경 scope를 작게 유지해야 한다.
- `modules copy/`가 존재하므로 검색/검증 시 active repo와 copy를 구분해야 한다.
- 일부 문자열 encoding이 깨져 보이는 파일이 있어, payload reason text 비교는 byte/normalized behavior를 실제로 검증해야 한다.
- manual operator workflow가 문서화되어 있지 않으면 삭제 후보 검증에서 누락될 수 있다.

## Additional Review. 기능 변화 위험 재검토

사용자 요청에 따라 코드 이동/삭제 없이 기능 변화 가능성만 다시 봤다.

추가로 보강해야 할 계약:

- `run.py`와 `app/ui/main_window.py`가 `mission_planning_gui.py` 파일명과 mission role/control port를 직접 사용하므로 launcher shim 전에는 경로/파일명을 바꾸지 않는다.
- `mission_planning_gui.py`의 0902 처리는 power gate, deferred queue, replay capture, delay timer, terrain warmup이 얽혀 있어 `replan_requests.py` 추출 전에 fixture가 필요하다.
- delivery는 단순 `0301 -> 0305 -> 0901/0903`가 아니라 quality-speed 직접전송, 0702 fallback suppression, attack suppress flag, post-delivery waypoint/snapshot carry-forward가 포함된다.
- `modules/common`, `modules/monitoring`, `app/ui/main_window.py`, `run.py`가 mission_planning runtime/config/ID allocator를 외부에서 import하므로 폴더 이동 전 wrapper가 필요하다.
- `next_area_mode`, `planners/next_collab_division`, `MissionVisualizer`, `manual/lah_rl_planner_gui.py`, root `lah_rl_planner_gui.py` wrapper, `logic_test/division_test`는 active runtime 또는 manual workflow와 연결될 수 있으므로 삭제/rename은 owner decision 뒤로 미룬다.

반영:

- `03-refactoring-roadmap.md` Phase 0 TODO에 direct delivery, suppress flag, post-delivery 후처리, 외부 import contract, manual workflow owner decision을 추가했다.
- `04-deletion-candidates.md`의 보류 목록에 manual/tool/runtime 외부 import 관련 항목을 추가했다.
- 상세 근거는 `07-third-review-risk-audit.md`에 남겼다.

### Sub-agent 추가 검토 반영

세 sub-agent의 launcher, replan state-machine, 삭제/rename 관점 검토에서 다음 보강이 추가로 필요하다고 확인했다.

- launcher: `KU_ROLE=mission` 설정이 `d0301/d0302/d0304` SW code에 영향을 주므로, wrapper 분리 시 가장 먼저 고정해야 한다.
- launcher: `run.py`와 `app/ui/main_window.py`는 env/cwd/control-port 설정이 달라 두 경로를 각각 smoke 해야 한다.
- bootstrap: nFusion 설정이 없을 때 dashboard는 warn 후 계속하지만 mission GUI bootstrap은 실패할 수 있으므로 실패 정책을 합치면 기능 변화가 난다.
- replan: 0902 ID extraction 우선순위, trigger별 delay/deferred queue, dispatcher `handled=True` 의미를 별도 fixture로 고정해야 한다.
- delivery: 0301 실패 뒤 0305/0901/0903가 나가지 않는 drop semantics, mode-ready/completion-ready/grace/timeout 조건을 보존해야 한다.
- monitoring: queue priority, source/current plan rebound, suppress flag가 mission dispatcher와 함께 동작하므로 monitoring logic도 regression scope에 포함한다.
- deletion: 미추적 runtime 파일, runtime DB state JSON, DEM/GeoTIFF, FOV/resource DB, portable bundle root 파일은 삭제 후보가 아니라 보류 항목이다.

## Current Extraction 재검토

요청에 따라 현재 추출본을 다시 검토했다. 범위는 `app/bootstrap.py`, `mission_control/planner_runtime.py`, `app/message_handlers/system_mode.py`, `input_packages.py`, `replan_requests.py`, 그리고 `mission_planning_gui.py` wrapper 연결부다.

확인 결과:

- compile과 extracted-helper smoke가 통과했다.
- 기존 private 함수명 wrapper는 유지되어 있다.
- `0201/0203` payload의 `Source/source` 로그 표시가 helper에서 strip될 수 있던 작은 차이를 발견했고, 기존처럼 truthy 원문 값을 그대로 쓰도록 수정했다.
- `KU_ROLE`/console logging import 순서, 0101 parsing allow-list, 0202 latest-input cache 제외, 0201/0203 non-empty core list 저장 조건, 0902 option/delay exact-match 정책은 이후 리팩토링에서 변경하면 안 되는 금지선으로 기록했다.
- 세부 내용은 `08-current-extraction-regression-audit.md`에 정리했다.
