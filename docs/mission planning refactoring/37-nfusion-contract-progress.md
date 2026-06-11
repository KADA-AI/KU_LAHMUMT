# nFusion Config and MMR_ReceiveNode Contract

## 목적

`modules/mission_planning` 리팩토링 중 nFusion 설정 파일 처리와 mission GUI receiver channel이 바뀌지 않도록 현재 실패 정책과 초기화 순서를 smoke로 고정한다.

## 추가 파일

- `smoke_nfusion_contract.py`

## Config 정책

### `run.py`

`run.py::_ensure_fusion_configs()` 후보 순서:

1. `PROJECT_ROOT/settings/nFusionSettings.json`
2. `PROJECT_ROOT/nFusionSettings.json` legacy fallback
3. `DS_DIR/nFusionSettings.json`
4. `COMMON_DIR/nFusionSettings.json`
5. `PROJECT_ROOT/settings/FusionSettings.json`
6. `PROJECT_ROOT/FusionSettings.json`
7. `DS_DIR/FusionSettings.json`
8. `COMMON_DIR/FusionSettings.json`
9. `PROJECT_ROOT/nFusion/FusionSettings.json`

정책:

- 설정 파일이 없으면 stderr에 warning을 쓰고 `None`을 반환한다.
- 이 경우 dashboard는 bus 없이 계속 실행할 수 있다.
- 설정 파일이 있으면 `PROJECT_ROOT/settings/nFusionSettings.json`으로 텍스트 복사하고 그 경로를 반환한다.
- license 후보가 있으면 `PROJECT_ROOT/settings/nFusionLicense.lic`으로 bytes 복사한다.

### Mission GUI

`mission_planning_gui_env._ensure_fusion_configs(project_root, common_dir)` 후보 순서:

1. `project_root/settings/nFusionSettings.json`
2. `project_root/nFusionSettings.json` legacy fallback
3. `common_dir/nFusionSettings.json`
4. `project_root/settings/FusionSettings.json`
5. `project_root/FusionSettings.json`
6. `common_dir/FusionSettings.json`
7. `project_root/nFusion/FusionSettings.json`

정책:

- 설정 파일이 없으면 `FileNotFoundError`를 발생시킨다.
- 설정 파일이 있으면 `project_root/settings/nFusionSettings.json`으로 복사하고 그 경로를 반환한다.
- license 후보가 있으면 `project_root/settings/nFusionLicense.lic`으로 복사한다.
- 복사는 `modules.common.settings_paths.ensure_fusion_settings_file()`와 `ensure_fusion_license_file()`을 통해 settings folder를 canonical 대상으로 사용한다.

주의: `run.py`와 mission GUI의 missing-config 정책은 서로 다르다. 이 차이는 현재 동작이며, 리팩토링 중 임의로 합치면 기능 변화가 된다.

## MessageLibrary 로드 계약

`run.py::_load_msglib_and_deps()`와 `mission_planning_gui_env._load_msglib_and_deps(common_dir)`는 모두:

- `dll_files.nFusionImports.clr`를 우선 사용한다.
- `MessageLibrary` stem reference를 먼저 시도한다.
- 실패하면 `MessageLibrary.dll` reference로 fallback한다.
- 있으면 `K4586Model`, `K4586Model.Assist`, `MiscUtil`도 reference한다.

## Mission GUI nFusion bootstrap 순서

`modules/mission_planning/mission_planning_gui.py` top-level 순서:

1. `from dll_files.nFusionImports import *`
2. `_ensure_fusion_configs(PROJECT_ROOT, COMMON_DIR)`
3. `_load_msglib_and_deps(COMMON_DIR)`
4. `from receive import *`
5. `from Tabs.assignment_planning_tab import AssignmentPlanningTab`

이 순서는 receive registration과 tab 초기화 전에 nFusion import/config/message library bootstrap이 끝나야 한다는 현재 계약이다.

## Receiver channel 계약

Mission GUI `MainWindow._rx_setup()` 순서:

1. `FusionNodeIoc.Configure()`
2. `NodeMessenger.Initialize("MMR_ReceiveNode")`
3. `NodeMessenger.RegistAllConsumerFromFusionNodeIoc()`
4. `NodeMessenger.InitAllSubscriberFromAssembly()`
5. `NodeMessenger.RegistAllProviderFromFusionNodeIoc()`
6. 성공 시 `_bus_ready=True`
7. 실패 시 `_bus_ready=False`와 `[BUS ERR]` log

Dashboard `run.py` bus monitor는 `NodeMessenger.Initialize("CommonChannel")`을 사용한다. Mission GUI channel과 dashboard channel을 섞지 않는다.

## Smoke 범위

`smoke_nfusion_contract.py`는 실제 nFusion middleware를 띄우지 않는다.

- temp directory에서 `run.py` config 있음/없음 정책을 검증한다.
- temp directory에서 mission GUI config 있음/없음 정책을 검증한다.
- fake `dll_files.nFusionImports.clr`로 MessageLibrary fallback과 optional dependency reference를 검증한다.
- `mission_planning_gui.py` source AST로 `MMR_ReceiveNode` channel과 `_bus_ready` success/failure assignment를 검증한다.
- `run.py` source로 dashboard `CommonChannel` 계약을 검증한다.

## 검증 결과

명령:

```powershell
python "docs\mission planning refactoring\smoke_nfusion_contract.py"
```

결과:

```text
nFusion config/MMR_ReceiveNode contract smoke ok
```

## Refactor Guardrails

- missing nFusion config에서 `run.py`는 warning + continue, mission GUI는 fail-fast인 현재 차이를 유지한다.
- mission GUI receiver channel은 `MMR_ReceiveNode`로 유지한다.
- dashboard bus monitor channel은 `CommonChannel`로 유지한다.
- MessageLibrary stem -> `.dll` fallback을 제거하지 않는다.
- `receive` import와 `AssignmentPlanningTab` import를 nFusion bootstrap보다 앞으로 당기지 않는다.

## 다음 지점

다음 미완료 TODO는 `planner hot-reload watch list와 reload order snapshot 작성`이다.
