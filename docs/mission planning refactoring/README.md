# Mission Planning Refactoring

이 폴더는 `modules/mission_planning` 대규모 리팩토링을 시작하기 전 분석, 목표 구조, TODO, 삭제 후보, 검토 결과를 모아 두는 작업 문서다.

현재 단계에서는 코드 이동/삭제를 하지 않았다. 운영 중인 import 경로, nFusion 메시지 흐름, ID 할당, 산출물 경로가 강하게 결합되어 있으므로 먼저 안전망과 migration 순서를 확정해야 한다.

## 문서 구성

- `01-current-architecture.md`: 현재 구조, 기능 경계, 외부 호출자, 고위험 계약
- `02-target-architecture.md`: 목표 폴더 구조와 현재 파일의 이동 방향
- `03-refactoring-roadmap.md`: 단계별 실행 계획과 TODO
- `04-deletion-candidates.md`: 삭제/아카이브 후보와 삭제 전 검증 조건
- `05-review-log.md`: 3회 검토 결과와 남은 리스크
- `06-second-review-risk-audit.md`: 추가 재검토에서 발견한 기능 변화 위험과 누락 계약
- `07-third-review-risk-audit.md`: 기능 변화 가능성 중심 3차 재검토 보강
- `08-current-extraction-regression-audit.md`: 현재 추출본 기준 기능 동일성 재검토와 보존 계약
- `09-import-order-pause-note.md`: top-level `app` import 순서 회귀 수정과 중단 지점 기록
- `10-wrapper-support-matrix.md`: trigger pipeline wrapper 지원 경로와 export 정책
- `11-trigger-move-progress.md`: prior/next-collab 구현 이동과 검증 결과
- `12-remaining-hybrid-progress.md`: current/general remaining hybrid 구현 이동과 검증 결과
- `13-recon-reexecute-progress.md`: recon/reexecute helper 이동과 검증 결과
- `14-phase3-wrapper-completion.md`: Phase 3 wrapper 유지 항목 완료와 support helper allow-list
- `15-runtime-state-progress.md`: runtime state 파일 이동과 기존 wrapper 검증 결과
- `16-runtime-cache-progress.md`: runtime cache 파일 이동과 기존 wrapper 검증 결과
- `17-runtime-logging-progress.md`: runtime logging 파일 이동과 기존 wrapper 검증 결과
- `18-runtime-validation-ids-progress.md`: runtime validation/ID reservation 이동과 Phase 4 wrapper 검증 결과
- `19-data-def-id-allocator-progress.md`: `data_def` public API inventory와 ID allocator engine wrapper 이동 결과
- `20-planning-enhanced-import-graph.md`: `planning_enhanced` import graph와 이동 전 shim 조건
- `21-bare-import-shims.md`: project-root `AnS`/`data_def`/`config.py` shim 부재와 `MissionPlanner` path bootstrap bare import 검증 결과
- `22-artifact-builders-progress.md`: 0301/0302/0303/0304 artifact builder engine package 이동과 compatibility wrapper 검증 결과
- `23-ans-bootstrap-reload-progress.md`: `AnS/mission_pipeline.py` 이동 전 path bootstrap과 dynamic reload 계약 검증 결과
- `24-active-import-smoke-progress.md`: `modules/mission_planning` active import smoke script 작성과 검증 결과
- `25-mission-planning-gui-smoke-progress.md`: `mission_planning_gui.py` import/launch smoke 정의와 검증 결과
- `26-sw-code-baseline-progress.md`: `KU_ROLE=mission`과 0301/0302/0303/0304 SW code baseline smoke 작성 결과
- `27-launch-env-parity-progress.md`: `run.py`와 `app/ui/main_window.py` mission GUI launch env parity smoke 작성 결과
- `28-run-py-cold-start-progress.md`: `run.py` cold start 전체 GUI launch/control-port smoke 작성 결과
- `29-pipeline-import-smoke-progress.md`: 주요 pipeline entrypoint import smoke 작성 결과
- `30-pipeline-signature-snapshot-progress.md`: 주요 `run_*` pipeline signature snapshot 작성 결과
- `31-artifact-builder-signature-snapshot-progress.md`: `d0301/d0302/d0303/d0304` public builder signature snapshot 작성 결과
- `32-id-allocator-baseline-progress.md`: ID allocator counter 파일과 reserve API baseline 기록 결과
- `33-sample-payload-fixtures-progress.md`: sample 0201/0203/0902 payload fixture 확보 결과
- `34-generated-artifact-link-validation-progress.md`: generated 0301/0302/0303/0304 artifact link validation smoke 작성 결과
- `35-launcher-control-contract-inventory.md`: launcher/control contract inventory와 검증 smoke 근거
- `36-cwd-syspath-bare-import-matrix.md`: cwd/sys.path/bare import matrix와 검증 smoke 근거
- `37-nfusion-contract-progress.md`: nFusion config 실패 정책과 `MMR_ReceiveNode` channel smoke 작성 결과
- `38-planner-hot-reload-snapshot-progress.md`: planner hot-reload watch list/reload order snapshot 작성 결과
- `39-planner-hot-reload-rebinding-fixture-progress.md`: planner hot-reload `globals()` rebinding fixture 작성 결과
- `40-recon-specialized-reload-policy-progress.md`: `recon_specialized_pipeline.py` watch/reload 정책 smoke 작성 결과
- `41-bootstrap-import-order-contract-progress.md`: mission bootstrap import-order contract smoke 작성 결과
- `42-0902-normalization-fixture-progress.md`: 0902 normalization fixture smoke 작성 결과
- `43-0902-id-extraction-priority-progress.md`: 0902 ID extraction priority smoke 작성 결과
- `44-0902-malformed-option-fallback-progress.md`: 0902 malformed option fallback smoke 작성 결과
- `45-0902-input-mission-ids-progress.md`: 0902 inputMissionIDList extraction smoke 작성 결과
- `46-0902-trigger-delay-exact-match-progress.md`: 0902 trigger/delay exact-match smoke 작성 결과
- `47-0902-trigger-deferred-queue-progress.md`: trigger별 0902 delay/deferred queue smoke 작성 결과
- `48-0902-predefer-order-progress.md`: init planning 중 0902 timing/terrain before deferred queue smoke 작성 결과
- `49-0902-replay-store-detail-progress.md`: captured 0902 replay/store-backed detail smoke 작성 결과
- `50-replan-dispatcher-semantics-progress.md`: replan dispatcher priority/handled semantics smoke 작성 결과
- `51-monitoring-queue-contract-progress.md`: monitoring queue priority/source-plan rebound/suppress semantics smoke 작성 결과
- `52-delivery-order-matrix-progress.md`: delivery order matrix와 fake push_message smoke 작성 결과
- `53-quality-direct-delivery-suppression-progress.md`: quality-speed/direct delivery 0901/0702 suppression smoke 작성 결과
- `54-attack-delivery-suppress-flag-progress.md`: attack delivery suppress flag smoke 작성 결과
- `55-post-delivery-carry-forward-progress.md`: post-delivery waypoint mark/snapshot carry-forward smoke 작성 결과
- `56-pipeline-result-shapes-progress.md`: pipeline result dataclass/dict shape snapshot smoke 결과
- `57-id-allocator-cold-concurrency-progress.md`: ID allocator cold-reset/concurrent-reserve parity smoke 결과
- `58-runtime-artifact-resource-paths-progress.md`: runtime artifact/resource path manifest smoke 결과
- `59-runtime-io-cache-log-helpers-progress.md`: runtime I/O/cache/log helper smoke 결과
- `60-0101-parsing-allow-list-progress.md`: 0101 parsing allow-list smoke 결과
- `61-0201-0203-latest-input-fixture-progress.md`: 0201/0203 latest input fixture smoke result
- `62-id-state-json-artifact-manifest-progress.md`: ID/state JSON artifact manifest smoke result
- `63-runtime-db-state-artifact-manifest-progress.md`: runtime DB state artifact manifest smoke result
- `64-html-png-output-artifact-manifest-progress.md`: HTML/PNG output artifact manifest smoke result
- `65-manual-operator-entrypoint-inventory-progress.md`: manual/operator entrypoint inventory smoke result
- `66-external-import-contract-inventory-progress.md`: monitoring/common/app external import contract smoke result
- `67-manual-workflow-owner-decisions-progress.md`: logic/tool/portable manual workflow owner decision smoke result
- `68-lah-attack-assistance-subprocess-progress.md`: LAH attack assistance subprocess smoke result
- `69-portable-bundle-launch-progress.md`: portable bundle launch smoke result
- `70-manual-planner-flow-mode-progress.md`: next-collab/next-area manual planner flow-mode smoke result
- `71-current-remaining-hybrid-fallback-pathid-progress.md`: current-remaining hybrid failure fallback/pathID smoke result
- `72-wrapper-template-consolidation-progress.md`: wrapper template contract smoke result
- `73-compat-root-strategy-decision.md`: root compatibility path strategy decision and smoke result
- `74-deprecated-import-policy-decision.md`: deprecated import logging/documentation policy decision
- `75-mission-planning-gui-public-launcher-handoff-progress.md`: public launcher handoff preparation result
- `76-deletion-candidate-reachability-progress.md`: deletion/archive candidate reachability smoke result
- `77-deletion-owner-manual-workflow-progress.md`: deletion/archive owner and manual workflow decision result
- `78-generated-output-fixture-policy-progress.md`: generated output fixture/delete policy decision result
- `79-root-wrapper-deprecation-period-decision.md`: root wrapper deprecation period decision
- `80-legacy-bucket-archive-strategy-decision.md`: legacy bucket archive/delete strategy decision
- `81-backup-style-file-deletion-decision.md`: backup-style file deletion policy decision
- `smoke_active_imports.py`: active mission planning module import smoke 스크립트
- `smoke_mission_planning_gui.py`: `mission_planning_gui.py` import/launch smoke 스크립트
- `smoke_sw_code_baseline.py`: mission role SW code baseline smoke 스크립트
- `smoke_launch_env_parity.py`: dashboard launch 경유별 mission GUI env parity smoke 스크립트
- `smoke_run_py_cold_start.py`: `run.py` cold start 전체 GUI launch/control-port smoke 스크립트
- `smoke_pipeline_imports.py`: 주요 pipeline entrypoint import/export/wrapper identity smoke 스크립트
- `smoke_pipeline_signatures.py`: 주요 `run_*` pipeline signature snapshot smoke 스크립트
- `smoke_artifact_builder_signatures.py`: `d0301/d0302/d0303/d0304` public builder signature snapshot smoke 스크립트
- `smoke_id_allocator_baseline.py`: ID allocator counter 파일과 reserve API baseline smoke 스크립트
- `smoke_sample_payload_fixtures.py`: sample 0201/0203/0902 payload fixture smoke 스크립트
- `smoke_generated_artifact_links.py`: generated 0301/0302/0303/0304 artifact link validation smoke 스크립트
- `smoke_cwd_import_matrix.py`: cwd/sys.path/bare import matrix smoke 스크립트
- `smoke_nfusion_contract.py`: nFusion config/MMR_ReceiveNode contract smoke 스크립트
- `smoke_planner_hot_reload_snapshot.py`: planner hot-reload watch/reload snapshot smoke 스크립트
- `smoke_planner_rebinding_fixture.py`: planner hot-reload `globals()` rebinding fixture smoke 스크립트
- `smoke_recon_specialized_reload_policy.py`: recon specialized watch/reload policy smoke 스크립트
- `smoke_bootstrap_import_order_contract.py`: mission bootstrap import-order contract smoke 스크립트
- `smoke_0902_normalization_fixture.py`: 0902 normalization fixture smoke 스크립트
- `smoke_0902_id_extraction_priority.py`: 0902 ID extraction priority smoke 스크립트
- `smoke_0902_malformed_option_fallback.py`: 0902 malformed option fallback smoke 스크립트
- `smoke_0902_input_mission_ids.py`: 0902 inputMissionIDList extraction smoke 스크립트
- `smoke_0902_trigger_delay_exact_match.py`: 0902 trigger/delay exact-match smoke 스크립트
- `smoke_0902_trigger_deferred_queue.py`: trigger별 0902 delay/deferred queue smoke 스크립트
- `smoke_0902_predefer_order.py`: init planning 중 0902 timing/terrain before deferred queue smoke 스크립트
- `smoke_0902_replay_store_detail.py`: captured 0902 replay/store-backed detail smoke 스크립트
- `smoke_replan_dispatcher_semantics.py`: replan dispatcher priority/handled semantics smoke 스크립트
- `smoke_monitoring_queue_contract.py`: monitoring queue priority/source-plan rebound/suppress semantics smoke 스크립트
- `smoke_delivery_order_matrix.py`: delivery order matrix와 fake push_message smoke 스크립트
- `smoke_quality_direct_delivery_suppression.py`: quality-speed/direct delivery 0901/0702 suppression smoke 스크립트
- `smoke_attack_delivery_suppress_flag.py`: attack delivery suppress flag smoke 스크립트
- `smoke_post_delivery_carry_forward.py`: post-delivery waypoint mark/snapshot carry-forward smoke 스크립트
- `smoke_pipeline_result_shapes.py`: pipeline result dataclass/dict shape snapshot smoke 스크립트
- `smoke_id_allocator_cold_concurrency.py`: ID allocator cold-reset/concurrent-reserve parity smoke 스크립트
- `smoke_runtime_artifact_paths.py`: runtime artifact/resource path manifest smoke 스크립트
- `smoke_runtime_io_cache_log_helpers.py`: runtime I/O/cache/log helper smoke 스크립트
- `smoke_0101_parsing_allow_list.py`: 0101 parsing allow-list smoke 스크립트
- `smoke_0201_0203_latest_input_fixture.py`: 0201/0203 latest input fixture smoke script
- `smoke_id_state_json_artifacts.py`: ID/state JSON artifact manifest smoke script
- `smoke_runtime_db_state_artifacts.py`: runtime DB state artifact manifest smoke script
- `smoke_html_png_output_artifacts.py`: HTML/PNG output artifact manifest smoke script
- `smoke_manual_operator_entrypoints.py`: manual/operator entrypoint inventory smoke script
- `smoke_external_import_contract.py`: monitoring/common/app external import contract smoke script
- `smoke_manual_workflow_owner_decisions.py`: logic/tool/portable manual workflow owner decision smoke script
- `smoke_lah_attack_assistance_subprocess.py`: LAH attack assistance subprocess smoke script
- `smoke_portable_bundle_launch.py`: portable bundle launch smoke script
- `smoke_manual_planner_flow_modes.py`: next-collab/next-area manual planner flow-mode smoke script
- `smoke_current_remaining_hybrid_fallback_pathids.py`: current-remaining hybrid failure fallback/pathID smoke script
- `smoke_wrapper_template_contract.py`: compatibility wrapper template contract smoke script
- `smoke_compat_root_strategy_contract.py`: root-vs-compat compatibility strategy smoke script
- `smoke_deprecated_import_policy_contract.py`: deprecated import logging/documentation policy smoke script
- `smoke_mission_planning_gui_public_launcher_handoff.py`: mission GUI public launcher handoff smoke script
- `smoke_deletion_candidate_reachability.py`: deletion/archive candidate reachability smoke script
- `smoke_deletion_owner_manual_workflow.py`: deletion/archive owner and manual workflow smoke script
- `smoke_generated_output_fixture_policy.py`: generated output fixture/delete policy smoke script
- `smoke_root_wrapper_deprecation_period.py`: root wrapper deprecation period smoke script
- `smoke_legacy_bucket_archive_strategy.py`: legacy bucket archive/delete strategy smoke script
- `smoke_backup_style_file_policy.py`: backup-style file deletion policy smoke script
- `smoke_root_surface_inventory.py`: mission_planning root surface classification smoke script
- `fixtures/current_remaining_hybrid/`: current-remaining hybrid failure/pathID fixture JSON
- `fixtures/payloads/`: 리팩터링 회귀 확인용 sample 0201/0203/0902 payload JSON
- `smoke_import_contract.py`: wrapper/import-order/stale-import 실행 검증 스크립트

## 핵심 결론

1. `mission_planning_gui.py`는 GUI가 아니라 전체 mission planning application shell이다. 메시지 수신, 초기 계획, 재계획 dispatch, plan delivery, warmup, 시각화까지 모두 포함한다.
2. 실제 재계획 로직은 `pipelines/`와 `runtime/`에 어느 정도 모여 있으나, `MainWindow`가 직접 호출하고 결과 필드를 읽는다.
3. 핵심 계획 엔진은 `MissionPlanner/AnS`, `MissionPlanner/data_def`, `MissionPlanner/planning_enhanced`에 남아 있다. 이 영역은 rename/move 위험이 가장 크다.
4. 삭제는 가능하지만 보수적으로 해야 한다. `legacy`, root wrapper, generated output, backup-style 파일은 후보일 뿐이며, 실제 삭제 전 reachability 검증과 smoke test가 필요하다.
5. 추가 재검토 결과, hot reload, 0902 normalization, delivery state machine, ID reset, resource path, manual entrypoint는 별도 계약으로 고정해야 한다.
6. 3차 재검토 결과, quality-speed 직접전송, attack delivery suppress flag, post-delivery waypoint/snapshot 후처리, monitoring/common 외부 import, manual logic test/tool entrypoint도 기능 보존 계약으로 추가해야 한다.
7. 추가 sub-agent 검토 결과, `KU_ROLE`/SW code, nFusion 실패 정책, attack assistance subprocess, trigger별 replan delay, dispatcher `handled` 의미, store-backed detail/replay capture, runtime state/resource artifacts도 Phase 0에 고정해야 한다.
8. 현재 추출본 재검토 결과, 0201/0203 `Source/source` 로그 원문 보존을 수정했고, `KU_ROLE`/console logging import 순서, 0101 parsing allow-list, 0202 cache 제외, 0902 exact-match 정책은 이후 리팩토링 금지선으로 문서화했다.

## 리팩토링 원칙

- 공개 실행 파일 `modules/mission_planning/mission_planning_gui.py`는 당분간 유지한다.
- `MainWindow`, `run_*_pipeline`, `warm_*_pipeline`, `d0301/d0302/d0303/d0304` builder 계약을 먼저 유지한다.
- root-level compatibility wrapper는 한 번에 지우지 않는다.
- `MissionPlanner`, `AnS`, `data_def` bare import 호환성을 깨지 않는다.
- 0301, 0305, 0702, 0901, 0902, 0903 payload key와 전송 순서를 유지한다.
- `DSS_Internal`, DB artifact directory, ID band, allocator counter 파일은 migration 전후 동일하게 동작해야 한다.
