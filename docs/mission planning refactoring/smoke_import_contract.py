from __future__ import annotations

import argparse
import importlib
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


REQUIRED_NEW_FILES = (
    "modules/mission_planning/app/bootstrap.py",
    "modules/mission_planning/app/message_handlers/system_mode.py",
    "modules/mission_planning/app/message_handlers/input_packages.py",
    "modules/mission_planning/app/message_handlers/replan_requests.py",
    "modules/mission_planning/app/gui_entrypoint.py",
    "modules/mission_planning/app/delivery/mission_plan_delivery.py",
    "modules/mission_planning/app/visualization/mission_visualization_tab.py",
    "modules/mission_planning/mission_control/planner_runtime.py",
    "modules/mission_planning/mission_control/plan_metrics.py",
    "modules/mission_planning/replanning/dispatcher.py",
    "modules/mission_planning/replanning/triggers/attack/pipeline.py",
    "modules/mission_planning/replanning/triggers/imaging_schedule/pipeline.py",
    "modules/mission_planning/replanning/triggers/path_deviation/pipeline.py",
    "modules/mission_planning/replanning/triggers/post_attack/pipeline.py",
    "modules/mission_planning/replanning/triggers/prior/pipeline.py",
    "modules/mission_planning/replanning/triggers/next_collab/pipeline.py",
    "modules/mission_planning/replanning/triggers/remaining_hybrid/current.py",
    "modules/mission_planning/replanning/triggers/remaining_hybrid/current_replan.py",
    "modules/mission_planning/replanning/triggers/remaining_hybrid/general.py",
    "modules/mission_planning/replanning/triggers/remaining_hybrid/reexecute_first.py",
    "modules/mission_planning/replanning/triggers/recon_specialized/pipeline.py",
    "modules/mission_planning/runtime/state/attack_assignment.py",
    "modules/mission_planning/runtime/state/attack_tracking.py",
    "modules/mission_planning/runtime/state/prior_tracking.py",
    "modules/mission_planning/runtime/cache/source_artifacts.py",
    "modules/mission_planning/runtime/cache/latest_input.py",
    "modules/mission_planning/runtime/logging/pipeline_events.py",
    "modules/mission_planning/runtime/logging/plan_file_logger.py",
    "modules/mission_planning/runtime/validation/replan_payloads.py",
    "modules/mission_planning/runtime/ids/replan_reservation.py",
    "modules/mission_planning/runtime/_compat_import.py",
    "modules/mission_planning/engine/__init__.py",
    "modules/mission_planning/engine/mission_generation/__init__.py",
    "modules/mission_planning/engine/mission_generation/id_allocation/__init__.py",
    "modules/mission_planning/engine/mission_generation/id_allocation/allocator.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/__init__.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0301.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0302.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py",
    "docs/mission planning refactoring/smoke_active_imports.py",
    "docs/mission planning refactoring/smoke_mission_planning_gui.py",
    "docs/mission planning refactoring/smoke_sw_code_baseline.py",
    "docs/mission planning refactoring/smoke_launch_env_parity.py",
    "docs/mission planning refactoring/smoke_run_py_cold_start.py",
    "docs/mission planning refactoring/smoke_pipeline_imports.py",
    "docs/mission planning refactoring/smoke_pipeline_signatures.py",
    "docs/mission planning refactoring/smoke_artifact_builder_signatures.py",
    "docs/mission planning refactoring/smoke_id_allocator_baseline.py",
    "docs/mission planning refactoring/smoke_sample_payload_fixtures.py",
    "docs/mission planning refactoring/smoke_generated_artifact_links.py",
    "docs/mission planning refactoring/smoke_cwd_import_matrix.py",
    "docs/mission planning refactoring/smoke_nfusion_contract.py",
    "docs/mission planning refactoring/smoke_planner_hot_reload_snapshot.py",
    "docs/mission planning refactoring/smoke_planner_rebinding_fixture.py",
    "docs/mission planning refactoring/smoke_recon_specialized_reload_policy.py",
    "docs/mission planning refactoring/smoke_bootstrap_import_order_contract.py",
    "docs/mission planning refactoring/smoke_0902_normalization_fixture.py",
    "docs/mission planning refactoring/smoke_0902_id_extraction_priority.py",
    "docs/mission planning refactoring/smoke_0902_malformed_option_fallback.py",
    "docs/mission planning refactoring/smoke_0902_input_mission_ids.py",
    "docs/mission planning refactoring/smoke_0902_trigger_delay_exact_match.py",
    "docs/mission planning refactoring/smoke_0902_trigger_deferred_queue.py",
    "docs/mission planning refactoring/smoke_0902_predefer_order.py",
    "docs/mission planning refactoring/smoke_0902_replay_store_detail.py",
    "docs/mission planning refactoring/smoke_replan_dispatcher_semantics.py",
    "docs/mission planning refactoring/smoke_monitoring_queue_contract.py",
    "docs/mission planning refactoring/smoke_delivery_order_matrix.py",
    "docs/mission planning refactoring/smoke_quality_direct_delivery_suppression.py",
    "docs/mission planning refactoring/smoke_attack_delivery_suppress_flag.py",
    "docs/mission planning refactoring/smoke_post_delivery_carry_forward.py",
    "docs/mission planning refactoring/smoke_pipeline_result_shapes.py",
    "docs/mission planning refactoring/smoke_id_allocator_cold_concurrency.py",
    "docs/mission planning refactoring/smoke_runtime_artifact_paths.py",
    "docs/mission planning refactoring/smoke_runtime_io_cache_log_helpers.py",
    "docs/mission planning refactoring/smoke_0101_parsing_allow_list.py",
    "docs/mission planning refactoring/smoke_0201_0203_latest_input_fixture.py",
    "docs/mission planning refactoring/smoke_id_state_json_artifacts.py",
    "docs/mission planning refactoring/smoke_runtime_db_state_artifacts.py",
    "docs/mission planning refactoring/smoke_html_png_output_artifacts.py",
    "docs/mission planning refactoring/smoke_manual_operator_entrypoints.py",
    "docs/mission planning refactoring/smoke_external_import_contract.py",
    "docs/mission planning refactoring/smoke_manual_workflow_owner_decisions.py",
    "docs/mission planning refactoring/smoke_lah_attack_assistance_subprocess.py",
    "docs/mission planning refactoring/smoke_portable_bundle_launch.py",
    "docs/mission planning refactoring/smoke_manual_planner_flow_modes.py",
    "docs/mission planning refactoring/smoke_current_remaining_hybrid_fallback_pathids.py",
    "docs/mission planning refactoring/smoke_wrapper_template_contract.py",
    "docs/mission planning refactoring/smoke_compat_root_strategy_contract.py",
    "docs/mission planning refactoring/smoke_deprecated_import_policy_contract.py",
    "docs/mission planning refactoring/smoke_mission_planning_gui_public_launcher_handoff.py",
    "docs/mission planning refactoring/smoke_deletion_candidate_reachability.py",
    "docs/mission planning refactoring/smoke_deletion_owner_manual_workflow.py",
    "docs/mission planning refactoring/smoke_generated_output_fixture_policy.py",
    "docs/mission planning refactoring/smoke_root_wrapper_deprecation_period.py",
    "docs/mission planning refactoring/smoke_legacy_bucket_archive_strategy.py",
    "docs/mission planning refactoring/smoke_backup_style_file_policy.py",
    "docs/mission planning refactoring/smoke_root_surface_inventory.py",
    "docs/mission planning refactoring/fixtures/payloads/sample_0201.json",
    "docs/mission planning refactoring/fixtures/payloads/sample_0203.json",
    "docs/mission planning refactoring/fixtures/payloads/sample_0902.json",
    "docs/mission planning refactoring/fixtures/current_remaining_hybrid/failure_fallback_pathid_mapping.json",
    "modules/mission_planning/MissionPlanner/data_def/d0301.py",
    "modules/mission_planning/MissionPlanner/data_def/d0302.py",
    "modules/mission_planning/MissionPlanner/data_def/d0303.py",
    "modules/mission_planning/MissionPlanner/data_def/d0304.py",
)

FORBIDDEN_PROJECT_ROOT_BARE_SHIMS = (
    "AnS",
    "data_def",
    "config.py",
)

FORBIDDEN_MOVED_IMPL_IMPORTS = (
    "modules.mission_planning.pipelines.attack_plan_pipeline",
    "modules.mission_planning.pipelines.prior_mission_pipeline_impl",
    "modules.mission_planning.pipelines.next_collab_replan_pipeline_impl",
    "modules.mission_planning.pipelines.imaging_schedule_replan_pipeline_impl",
    "modules.mission_planning.pipelines.path_deviation_replan_pipeline_impl",
    "modules.mission_planning.pipelines.post_attack_rejoin_pipeline",
    "modules.mission_planning.pipelines.current_remaining_hybrid",
    "modules.mission_planning.pipelines.current_remaining_hybrid_replan",
    "modules.mission_planning.pipelines.general_remaining_hybrid_replan",
    "modules.mission_planning.pipelines.reexecute_first_mission_hybrid",
    "modules.mission_planning.pipelines.recon_specialized_pipeline",
)

MOVED_PIPELINE_WRAPPER_TARGETS = {
    "modules/mission_planning/pipelines/attack_plan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.attack.pipeline"
    ),
    "modules/mission_planning/pipelines/prior_mission_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.prior.pipeline"
    ),
    "modules/mission_planning/pipelines/next_collab_replan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline"
    ),
    "modules/mission_planning/pipelines/next_collab_replan_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline"
    ),
    "modules/mission_planning/pipelines/current_remaining_hybrid.py": (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current"
    ),
    "modules/mission_planning/pipelines/current_remaining_hybrid_replan.py": (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan"
    ),
    "modules/mission_planning/pipelines/general_remaining_hybrid_replan.py": (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.general"
    ),
    "modules/mission_planning/pipelines/reexecute_first_mission_hybrid.py": (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first"
    ),
    "modules/mission_planning/pipelines/recon_specialized_pipeline.py": (
        "modules.mission_planning.replanning.triggers.recon_specialized.pipeline"
    ),
    "modules/mission_planning/pipelines/imaging_schedule_replan_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline"
    ),
    "modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline"
    ),
    "modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py": (
        "modules.mission_planning.replanning.triggers.post_attack.pipeline"
    ),
}

WRAPPER_IDENTITY_CHECKS = (
    (
        "modules.mission_planning.replanning.triggers.attack.pipeline",
        (
            "modules.mission_planning.attack_plan_pipeline",
            "modules.mission_planning.legacy.wrappers.attack_plan_pipeline",
            "modules.mission_planning.pipelines.attack_plan_pipeline",
        ),
        ("run_attack_plan_pipeline", "warm_attack_plan_pipeline"),
    ),
    (
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        (
            "modules.mission_planning.imaging_schedule_replan_pipeline",
            "modules.mission_planning.legacy.wrappers.imaging_schedule_replan_pipeline",
            "modules.mission_planning.pipelines.imaging_schedule_replan_pipeline_impl",
        ),
        ("run_imaging_schedule_replan_pipeline", "warm_imaging_schedule_replan_pipeline"),
    ),
    (
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        (
            "modules.mission_planning.path_deviation_replan_pipeline",
            "modules.mission_planning.legacy.wrappers.path_deviation_replan_pipeline",
            "modules.mission_planning.pipelines.path_deviation_replan_pipeline_impl",
        ),
        ("run_path_deviation_replan_pipeline", "warm_path_deviation_replan_pipeline"),
    ),
    (
        "modules.mission_planning.replanning.triggers.post_attack.pipeline",
        ("modules.mission_planning.pipelines.post_attack_rejoin_pipeline",),
        ("run_post_attack_rejoin_pipeline", "warm_post_attack_rejoin_pipeline"),
    ),
    (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        (
            "modules.mission_planning.prior_mission_pipeline",
            "modules.mission_planning.legacy.wrappers.prior_mission_pipeline",
        ),
        ("run_prior_mission_pipeline", "warm_prior_mission_pipeline"),
    ),
    (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        (
            "modules.mission_planning.prior_mission_pipeline_impl",
            "modules.mission_planning.legacy.wrappers.prior_mission_pipeline_impl",
            "modules.mission_planning.pipelines.prior_mission_pipeline_impl",
        ),
        (
            "run_prior_mission_pipeline",
            "warm_prior_mission_pipeline",
            "run_prior_post_rejoin_pipeline",
            "warm_prior_post_rejoin_pipeline",
            "_to_int",
            "_merge_line_remaining_detail",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        (
            "modules.mission_planning.next_collab_replan_pipeline",
            "modules.mission_planning.legacy.wrappers.next_collab_replan_pipeline",
            "modules.mission_planning.pipelines.next_collab_replan_pipeline",
            "modules.mission_planning.pipelines.next_collab_replan_pipeline_impl",
        ),
        ("run_next_collab_replan_pipeline", "warm_next_collab_replan_pipeline"),
    ),
    (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        (
            "modules.mission_planning.pipelines.next_collab_replan_pipeline",
            "modules.mission_planning.pipelines.next_collab_replan_pipeline_impl",
        ),
        ("NextCollabPipelineResult",),
    ),
    (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        ("modules.mission_planning.pipelines.next_collab_replan_pipeline_impl",),
        ("prepare_next_collab_input_replacements",),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        ("modules.mission_planning.pipelines.current_remaining_hybrid",),
        (
            "CurrentRemainingHybridRequest",
            "build_current_remaining_hybrid",
            "merge_current_remaining_hybrid",
            "validate_current_remaining_hybrid_request",
            "validate_current_remaining_hybrid_paths",
            "filter_generic_flightpath_missions_for_hybrid",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan",
        ("modules.mission_planning.pipelines.current_remaining_hybrid_replan",),
        ("CurrentRemainingHybridResult", "prepare_current_remaining_hybrid_replacements"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.general",
        ("modules.mission_planning.pipelines.general_remaining_hybrid_replan",),
        ("RemainingHybridResult", "apply_remaining_hybrid_replan", "validate_remaining_hybrid_source_geometry"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first",
        ("modules.mission_planning.pipelines.reexecute_first_mission_hybrid",),
        (
            "prepare_reexecute_first_mission_replacements",
            "reexecute_first_mission_generic_skip_policy",
            "validate_reexecute_first_mission_inputs",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.recon_specialized.pipeline",
        ("modules.mission_planning.pipelines.recon_specialized_pipeline",),
        (
            "build_recon_specialized_runtime_payload",
            "is_recon_specialized_option",
            "summarize_recon_area_review_guard",
        ),
    ),
    (
        "modules.mission_planning.runtime.state.attack_assignment",
        (
            "modules.mission_planning.runtime.attack_assignment_state",
            "modules.mission_planning.attack_assignment_state",
            "modules.mission_planning.legacy.wrappers.attack_assignment_state",
        ),
        (
            "_STATE_FILENAME",
            "get_last_assigned_manned_id",
            "set_last_assigned_manned_id",
            "mark_manned_used",
            "release_manned_used",
        ),
    ),
    (
        "modules.mission_planning.runtime.state.attack_tracking",
        ("modules.mission_planning.runtime.attack_tracking_state",),
        (
            "_STATE_FILENAME",
            "resolve_plan_lineage_ids",
            "update_from_agent_states",
            "register_tracking_assignment",
        ),
    ),
    (
        "modules.mission_planning.runtime.state.prior_tracking",
        ("modules.mission_planning.runtime.prior_tracking_state",),
        (
            "_STATE_FILENAME",
            "list_active_prior_assignments",
            "register_prior_assignment",
            "update_from_agent_states",
        ),
    ),
    (
        "modules.mission_planning.runtime.cache.source_artifacts",
        ("modules.mission_planning.runtime.source_artifact_cache",),
        (
            "SourceArtifactCache",
            "call_with_source_artifact_cache",
            "read_json_cached",
            "use_source_artifact_cache",
        ),
    ),
    (
        "modules.mission_planning.runtime.cache.latest_input",
        (
            "modules.mission_planning.runtime.latest_input_cache",
            "modules.mission_planning.latest_input_cache",
            "modules.mission_planning.legacy.wrappers.latest_input_cache",
        ),
        (
            "reset_latest_inputs",
            "update_from_payload",
            "get_latest_package_id",
            "get_latest_snapshot",
            "resolve_path_from_cache",
        ),
    ),
    (
        "modules.mission_planning.runtime.logging.pipeline_events",
        (
            "modules.mission_planning.runtime.mission_planning_pipeline_logging",
            "modules.mission_planning.mission_planning_pipeline_logging",
            "modules.mission_planning.legacy.wrappers.mission_planning_pipeline_logging",
        ),
        (
            "PipelineLogManager",
            "PipelinePhaseTimer",
            "emit_replan_checkpoint",
            "new_replan_transaction_id",
        ),
    ),
    (
        "modules.mission_planning.runtime.logging.plan_file_logger",
        (
            "modules.mission_planning.runtime.mission_plan_file_logger",
            "modules.mission_planning.mission_plan_file_logger",
            "modules.mission_planning.legacy.wrappers.mission_plan_file_logger",
        ),
        ("MissionPlanFileLogger", "MissionPlanRunLog"),
    ),
    (
        "modules.mission_planning.runtime.validation.replan_payloads",
        ("modules.mission_planning.runtime.replan_validation",),
        (
            "ReplanValidationError",
            "validate_replan_payloads",
            "validate_generated_artifact_payloads",
            "sync_flight_plan_individual_mission_ids",
            "collect_missing_flight_path_repairs",
        ),
    ),
    (
        "modules.mission_planning.runtime.ids.replan_reservation",
        ("modules.mission_planning.runtime.replan_id_reservation",),
        ("ReservedIdBlock", "ReplanIdReservation", "summarize_used_reserved_ids"),
    ),
    (
        "modules.mission_planning.engine.mission_generation.id_allocation.allocator",
        ("modules.mission_planning.MissionPlanner.data_def.id_allocator",),
        (
            "BASE",
            "_state",
            "reserve_mission_plan_ids",
            "reserve_imp_ids",
            "reserve_individual_mission_ids",
            "reserve_path_ids",
            "reserve_replan_id_bundle",
            "mark_waypoint_files_written",
        ),
    ),
    (
        "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0301",
        (
            "modules.mission_planning.MissionPlanner.data_def.d0301",
            "data_def.d0301",
        ),
        ("build_mission_plan",),
    ),
    (
        "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0302",
        (
            "modules.mission_planning.MissionPlanner.data_def.d0302",
            "data_def.d0302",
        ),
        ("build_mission_packages",),
    ),
    (
        "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0303",
        (
            "modules.mission_planning.MissionPlanner.data_def.d0303",
            "data_def.d0303",
        ),
        (
            "build_flight_plans",
            "set_flyover_options",
            "reset_dense_linesearch_metrics",
            "get_dense_linesearch_metrics",
            "_WPAllocator",
            "SweepConfig",
        ),
    ),
    (
        "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0304",
        (
            "modules.mission_planning.MissionPlanner.data_def.d0304",
            "data_def.d0304",
        ),
        (
            "build_lah_flight_plans_fixed",
            "build_lah_flight_plans_from_mrpk",
            "apply_uav_eta_follow_speed_plan",
        ),
    ),
)

MOVED_RUNTIME_WRAPPER_TARGETS = {
    "modules/mission_planning/runtime/attack_assignment_state.py": (
        "modules.mission_planning.runtime.state.attack_assignment"
    ),
    "modules/mission_planning/runtime/attack_tracking_state.py": (
        "modules.mission_planning.runtime.state.attack_tracking"
    ),
    "modules/mission_planning/runtime/prior_tracking_state.py": (
        "modules.mission_planning.runtime.state.prior_tracking"
    ),
    "modules/mission_planning/runtime/source_artifact_cache.py": (
        "modules.mission_planning.runtime.cache.source_artifacts"
    ),
    "modules/mission_planning/runtime/latest_input_cache.py": (
        "modules.mission_planning.runtime.cache.latest_input"
    ),
    "modules/mission_planning/runtime/mission_planning_pipeline_logging.py": (
        "modules.mission_planning.runtime.logging.pipeline_events"
    ),
    "modules/mission_planning/runtime/mission_plan_file_logger.py": (
        "modules.mission_planning.runtime.logging.plan_file_logger"
    ),
    "modules/mission_planning/runtime/replan_validation.py": (
        "modules.mission_planning.runtime.validation.replan_payloads"
    ),
    "modules/mission_planning/runtime/replan_id_reservation.py": (
        "modules.mission_planning.runtime.ids.replan_reservation"
    ),
}

ARTIFACT_BUILDER_PACKAGE = (
    "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304"
)

ARTIFACT_BUILDER_CONTRACTS = {
    "d0301": ("build_mission_plan",),
    "d0302": ("build_mission_packages",),
    "d0303": (
        "build_flight_plans",
        "set_flyover_options",
        "reset_dense_linesearch_metrics",
        "get_dense_linesearch_metrics",
        "_WPAllocator",
        "SweepConfig",
    ),
    "d0304": (
        "build_lah_flight_plans_fixed",
        "build_lah_flight_plans_from_mrpk",
        "apply_uav_eta_follow_speed_plan",
    ),
}

MOVED_ARTIFACT_WRAPPER_TARGETS = {
    f"modules/mission_planning/MissionPlanner/data_def/{module_name}.py": (
        f"{ARTIFACT_BUILDER_PACKAGE}.{module_name}"
    )
    for module_name in ARTIFACT_BUILDER_CONTRACTS
}


def fail(message: str) -> None:
    raise AssertionError(message)


def check_required_files(require_git_tracked: bool) -> None:
    missing = [rel for rel in REQUIRED_NEW_FILES if not (PROJECT_ROOT / rel).exists()]
    if missing:
        fail("missing required refactor files: " + ", ".join(missing))

    forbidden_root_shims = [rel for rel in FORBIDDEN_PROJECT_ROOT_BARE_SHIMS if (PROJECT_ROOT / rel).exists()]
    if forbidden_root_shims:
        fail("project-root bare import shims must stay absent: " + ", ".join(forbidden_root_shims))

    app_init = PROJECT_ROOT / "modules/mission_planning/app/__init__.py"
    if app_init.exists():
        fail("modules/mission_planning/app/__init__.py must stay absent to avoid top-level app shadowing")

    if not require_git_tracked:
        return

    untracked = []
    for rel in REQUIRED_NEW_FILES:
        result = subprocess.run(
            ["git", "ls-files", "--error-unmatch", "--", rel],
            cwd=PROJECT_ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if result.returncode != 0:
            untracked.append(rel)
    if untracked:
        fail("required refactor files are not tracked by git: " + ", ".join(untracked))


def check_import_order() -> None:
    cases = (
        ("modules.mission_planning.mission_planning_gui", "app.ui.main_window"),
        ("app.ui.main_window", "modules.mission_planning.mission_planning_gui"),
        ("run", "modules.mission_planning.mission_planning_gui", "app.ui.main_window"),
    )
    for names in cases:
        code = "import importlib\nfor name in {!r}:\n    importlib.import_module(name)\n".format(names)
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            fail(
                "import order failed for {}:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                    " -> ".join(names),
                    result.stdout,
                    result.stderr,
                )
            )


def check_wrapper_identity() -> None:
    for canonical_name, wrapper_names, attr_names in WRAPPER_IDENTITY_CHECKS:
        canonical = importlib.import_module(canonical_name)
        for wrapper_name in wrapper_names:
            wrapper = importlib.import_module(wrapper_name)
            for attr_name in attr_names:
                if not hasattr(canonical, attr_name):
                    fail(f"{canonical_name} missing {attr_name}")
                if not hasattr(wrapper, attr_name):
                    fail(f"{wrapper_name} missing {attr_name}")
                if getattr(wrapper, attr_name) is not getattr(canonical, attr_name):
                    fail(f"{wrapper_name}.{attr_name} is not identical to {canonical_name}.{attr_name}")


def check_runtime_bare_import_compat() -> None:
    grouped_cases = (
        (
            "modules/mission_planning/runtime",
            True,
            (
                (
                    "replan_id_reservation",
                    "modules.mission_planning.runtime.ids.replan_reservation",
                    ("ReservedIdBlock", "ReplanIdReservation"),
                ),
                (
                    "replan_validation",
                    "modules.mission_planning.runtime.validation.replan_payloads",
                    ("ReplanValidationError", "validate_replan_payloads"),
                ),
                (
                    "attack_assignment_state",
                    "modules.mission_planning.runtime.state.attack_assignment",
                    ("get_last_assigned_manned_id", "set_last_assigned_manned_id"),
                ),
                (
                    "attack_tracking_state",
                    "modules.mission_planning.runtime.state.attack_tracking",
                    ("resolve_plan_lineage_ids", "register_tracking_assignment"),
                ),
                (
                    "prior_tracking_state",
                    "modules.mission_planning.runtime.state.prior_tracking",
                    ("list_active_prior_assignments", "register_prior_assignment"),
                ),
                (
                    "source_artifact_cache",
                    "modules.mission_planning.runtime.cache.source_artifacts",
                    ("SourceArtifactCache", "read_json_cached"),
                ),
                (
                    "latest_input_cache",
                    "modules.mission_planning.runtime.cache.latest_input",
                    ("reset_latest_inputs", "update_from_payload"),
                ),
                (
                    "mission_planning_pipeline_logging",
                    "modules.mission_planning.runtime.logging.pipeline_events",
                    ("PipelineLogManager", "PipelinePhaseTimer"),
                ),
                (
                    "mission_plan_file_logger",
                    "modules.mission_planning.runtime.logging.plan_file_logger",
                    ("MissionPlanFileLogger", "MissionPlanRunLog"),
                ),
            ),
        ),
        (
            "modules/mission_planning/legacy/wrappers",
            False,
            (
                (
                    "mission_planning_pipeline_logging",
                    "modules.mission_planning.runtime.logging.pipeline_events",
                    ("PipelineLogManager", "PipelinePhaseTimer"),
                ),
                (
                    "mission_plan_file_logger",
                    "modules.mission_planning.runtime.logging.plan_file_logger",
                    ("MissionPlanFileLogger", "MissionPlanRunLog"),
                ),
            ),
        ),
    )
    for cwd_rel, shadow_logging_first, cases in grouped_cases:
        code = (
            "import importlib, sys\n"
            f"cases = {cases!r}\n"
            f"shadow_logging_first = {shadow_logging_first!r}\n"
            "if shadow_logging_first:\n"
            "    importlib.import_module('logging')\n"
            "for bare_name, canonical_name, attr_names in cases:\n"
            "    bare = importlib.import_module(bare_name)\n"
            "    canonical = importlib.import_module(canonical_name)\n"
            "    for attr in attr_names:\n"
            "        if getattr(bare, attr) is not getattr(canonical, attr):\n"
            "            raise SystemExit(f'{bare_name}.{attr} identity mismatch')\n"
            "logging_mod = importlib.import_module('logging')\n"
            "if not hasattr(logging_mod, 'getLogger'):\n"
            "    raise SystemExit('stdlib logging shadow was not repaired')\n"
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=PROJECT_ROOT / cwd_rel,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            fail(
                "runtime bare import failed from {}:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                    cwd_rel,
                    result.stdout,
                    result.stderr,
                )
            )


def check_id_allocator_wrapper_contract() -> None:
    canonical = importlib.import_module(
        "modules.mission_planning.engine.mission_generation.id_allocation.allocator"
    )
    wrapper = importlib.import_module("modules.mission_planning.MissionPlanner.data_def.id_allocator")
    legacy_store = Path(getattr(canonical, "_LEGACY_STORE", ""))
    expected_store = PROJECT_ROOT / "modules/mission_planning/MissionPlanner/data_def/id_tracker.json"
    if legacy_store != expected_store:
        fail(f"id allocator legacy store changed: {legacy_store} != {expected_store}")
    wrapper_file = Path(getattr(wrapper, "__file__", ""))
    expected_wrapper = PROJECT_ROOT / "modules/mission_planning/MissionPlanner/data_def/id_allocator.py"
    if wrapper_file != expected_wrapper:
        fail(f"id allocator wrapper __file__ changed: {wrapper_file} != {expected_wrapper}")

    original = getattr(canonical, "_volatile_counters")
    sentinel = {"waypoint": -123456789}
    try:
        wrapper._volatile_counters = sentinel
        if getattr(canonical, "_volatile_counters") is not sentinel:
            fail("id allocator wrapper assignment did not update canonical _volatile_counters")
    finally:
        wrapper._volatile_counters = original

    original_state = getattr(canonical, "_state")
    rebound_state: dict = {}
    try:
        canonical._state = rebound_state
        wrapper._state["missionPlanID"] = 123
        if rebound_state.get("missionPlanID") != 123:
            fail("id allocator wrapper did not follow canonical _state rebind")
    finally:
        canonical._state = original_state

    code = (
        "from data_def import id_allocator\n"
        "from data_def.id_allocator import (\n"
        "    reserve_mission_plan_ids as bare_reserve_mission_plan_ids,\n"
        "    reserve_imp_ids as bare_reserve_imp_ids,\n"
        "    reserve_individual_mission_ids as bare_reserve_individual_mission_ids,\n"
        "    reserve_path_ids as bare_reserve_path_ids,\n"
        "    reserve_replan_id_bundle as bare_reserve_replan_id_bundle,\n"
        "    mark_waypoint_files_written as bare_mark_waypoint_files_written,\n"
        ")\n"
        "from modules.mission_planning.MissionPlanner.data_def.id_allocator import (\n"
        "    reserve_mission_plan_ids as absolute_reserve_mission_plan_ids,\n"
        "    reserve_imp_ids as absolute_reserve_imp_ids,\n"
        "    reserve_individual_mission_ids as absolute_reserve_individual_mission_ids,\n"
        "    reserve_path_ids as absolute_reserve_path_ids,\n"
        "    reserve_replan_id_bundle as absolute_reserve_replan_id_bundle,\n"
        "    mark_waypoint_files_written as absolute_mark_waypoint_files_written,\n"
        ")\n"
        "import importlib\n"
        "canonical = importlib.import_module('modules.mission_planning.engine.mission_generation.id_allocation.allocator')\n"
        "direct_imports = {\n"
        "    'reserve_mission_plan_ids': (bare_reserve_mission_plan_ids, absolute_reserve_mission_plan_ids),\n"
        "    'reserve_imp_ids': (bare_reserve_imp_ids, absolute_reserve_imp_ids),\n"
        "    'reserve_individual_mission_ids': (bare_reserve_individual_mission_ids, absolute_reserve_individual_mission_ids),\n"
        "    'reserve_path_ids': (bare_reserve_path_ids, absolute_reserve_path_ids),\n"
        "    'reserve_replan_id_bundle': (bare_reserve_replan_id_bundle, absolute_reserve_replan_id_bundle),\n"
        "    'mark_waypoint_files_written': (bare_mark_waypoint_files_written, absolute_mark_waypoint_files_written),\n"
        "}\n"
        "for attr, values in direct_imports.items():\n"
        "    for value in values:\n"
        "        if value is not getattr(canonical, attr):\n"
        "            raise SystemExit(f'{attr} direct import identity mismatch')\n"
        "for attr in ('BASE', 'reserve_mission_plan_ids', 'reserve_replan_id_bundle'):\n"
        "    if getattr(id_allocator, attr) is not getattr(canonical, attr):\n"
        "        raise SystemExit(f'{attr} identity mismatch')\n"
        "original = canonical._volatile_counters\n"
        "original_state = canonical._state\n"
        "sentinel = {'waypoint': -987654321}\n"
        "rebound_state = {}\n"
        "try:\n"
        "    id_allocator._volatile_counters = sentinel\n"
        "    if canonical._volatile_counters is not sentinel:\n"
        "        raise SystemExit('bare assignment forwarding failed')\n"
        "    canonical._state = rebound_state\n"
        "    id_allocator._state['missionPlanID'] = 456\n"
        "    if rebound_state.get('missionPlanID') != 456:\n"
        "        raise SystemExit('bare _state rebind forwarding failed')\n"
        "finally:\n"
        "    id_allocator._volatile_counters = original\n"
        "    canonical._state = original_state\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=PROJECT_ROOT / "modules/mission_planning/MissionPlanner",
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        fail(
            "id allocator bare data_def wrapper failed:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                result.stdout,
                result.stderr,
            )
        )


def check_planning_enhanced_import_graph_contract() -> None:
    contracts = (
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced",
            ("run_enhanced_divide_and_pattern",),
        ),
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced.pipeline",
            ("run_enhanced_divide_and_pattern",),
        ),
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced.algo",
            ("run_split_pipeline", "review_assigned_areas_local", "review_overflow_areas"),
        ),
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner",
            ("run_split_pipeline", "assign_split_result_by_takeover_distance"),
        ),
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced.io",
            (
                "build_0301_from_0302_packages",
                "build_0302_packages_from_split_with_lah",
                "build_0303_0304_from_0302_packages",
            ),
        ),
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced.pathing",
            ("generate_expected_paths", "calculate_expected_velocity"),
        ),
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced.scheduling",
            ("run_milp_scheduling", "schedule_by_parent_order"),
        ),
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced.type_decider",
            ("apply_logic_type_decider", "PROFILE_DEFAULT", "PROFILE_RECON", "PROFILE_MIN_TIME"),
        ),
        (
            "modules.mission_planning.MissionPlanner.planning_enhanced.models",
            ("MissionRecord", "SplitPiece", "DirectionDebug", "SplitRunResult"),
        ),
        (
            "modules.mission_planning.runtime.next_collab_line_runner",
            ("run_next_collab_line_plan",),
        ),
    )
    for module_name, attr_names in contracts:
        module = importlib.import_module(module_name)
        for attr_name in attr_names:
            if not hasattr(module, attr_name):
                fail(f"{module_name} missing planning_enhanced contract attr {attr_name}")

    code = (
        "from planning_enhanced import run_enhanced_divide_and_pattern\n"
        "from planning_enhanced.io import build_0302_packages_from_split_with_lah\n"
        "from planning_enhanced.pathing import generate_expected_paths, calculate_expected_velocity\n"
        "from planning_enhanced.type_decider import apply_logic_type_decider\n"
        "if not callable(run_enhanced_divide_and_pattern):\n"
        "    raise SystemExit('run_enhanced_divide_and_pattern is not callable')\n"
        "if not callable(build_0302_packages_from_split_with_lah):\n"
        "    raise SystemExit('build_0302_packages_from_split_with_lah is not callable')\n"
        "if not callable(generate_expected_paths) or not callable(calculate_expected_velocity):\n"
        "    raise SystemExit('pathing exports are not callable')\n"
        "if not callable(apply_logic_type_decider):\n"
        "    raise SystemExit('type_decider export is not callable')\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=PROJECT_ROOT / "modules/mission_planning/MissionPlanner",
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        fail(
            "planning_enhanced bare import contract failed:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                result.stdout,
                result.stderr,
            )
        )


def check_mission_planner_bare_import_shims() -> None:
    bootstrap = (
        "import importlib, sys\n"
        "from pathlib import Path\n"
        f"project_root = Path({str(PROJECT_ROOT)!r})\n"
        "if str(project_root) not in sys.path:\n"
        "    sys.path.insert(0, str(project_root))\n"
        "runtime = importlib.import_module('modules.mission_planning.mission_control.planner_runtime')\n"
        "runtime.ensure_mission_planner_import_paths(project_root)\n"
    )
    snippets = (
        (
            "ans_first",
            bootstrap
            + (
                "ans = importlib.import_module('AnS')\n"
                "ans_pipeline = importlib.import_module('AnS.mission_pipeline')\n"
                "data_def = importlib.import_module('data_def')\n"
                "d0303 = importlib.import_module('data_def.d0303')\n"
                "config = importlib.import_module('config')\n"
                "canonical_ans = importlib.import_module('modules.mission_planning.MissionPlanner.AnS')\n"
                "canonical_ans_pipeline = importlib.import_module('modules.mission_planning.MissionPlanner.AnS.mission_pipeline')\n"
                "canonical_config = importlib.import_module('modules.mission_planning.MissionPlanner.config')\n"
                "if ans is not canonical_ans:\n"
                "    raise SystemExit('AnS package identity split')\n"
                "if ans_pipeline is not canonical_ans_pipeline:\n"
                "    raise SystemExit('AnS.mission_pipeline identity split')\n"
                "if not callable(ans.run_divide_and_pattern):\n"
                "    raise SystemExit('AnS.run_divide_and_pattern missing')\n"
                "if ans_pipeline.run_divide_and_pattern is not ans.run_divide_and_pattern:\n"
                "    raise SystemExit('AnS mission_pipeline alias mismatch')\n"
                "if not callable(data_def.build_lah_flight_plans_fixed):\n"
                "    raise SystemExit('data_def build_lah_flight_plans_fixed missing')\n"
                "if not callable(d0303.build_flight_plans):\n"
                "    raise SystemExit('data_def.d0303 build_flight_plans missing')\n"
                "if config.DEFAULT_SWEEP_SEPARATION_M != canonical_config.DEFAULT_SWEEP_SEPARATION_M:\n"
                "    raise SystemExit('config constant mismatch')\n"
            ),
        ),
        (
            "data_def_first",
            bootstrap
            + (
                "d0302 = importlib.import_module('data_def.d0302')\n"
                "d0303 = importlib.import_module('data_def.d0303')\n"
                "d0304 = importlib.import_module('data_def.d0304')\n"
                "id_allocator = importlib.import_module('data_def.id_allocator')\n"
                "mission_helpers = importlib.import_module('data_def.mission_helpers')\n"
                "config = importlib.import_module('config')\n"
                "ans = importlib.import_module('AnS')\n"
                "canonical_ans = importlib.import_module('modules.mission_planning.MissionPlanner.AnS')\n"
                "absolute_d0303 = importlib.import_module('modules.mission_planning.MissionPlanner.data_def.d0303')\n"
                "absolute_mission_helpers = importlib.import_module('modules.mission_planning.MissionPlanner.data_def.mission_helpers')\n"
                "canonical_config = importlib.import_module('modules.mission_planning.MissionPlanner.config')\n"
                "if ans is not canonical_ans:\n"
                "    raise SystemExit('AnS package identity split')\n"
                "if d0303 is not absolute_d0303:\n"
                "    raise SystemExit('data_def.d0303 module identity split')\n"
                "if mission_helpers is not absolute_mission_helpers:\n"
                "    raise SystemExit('data_def.mission_helpers module identity split')\n"
                "if not callable(d0302.build_mission_packages):\n"
                "    raise SystemExit('data_def.d0302 build_mission_packages missing')\n"
                "if not callable(d0303.build_flight_plans):\n"
                "    raise SystemExit('data_def.d0303 build_flight_plans missing')\n"
                "if not callable(d0304.build_lah_flight_plans_fixed):\n"
                "    raise SystemExit('data_def.d0304 build_lah_flight_plans_fixed missing')\n"
                "if not callable(id_allocator.reserve_mission_plan_ids):\n"
                "    raise SystemExit('data_def.id_allocator reserve_mission_plan_ids missing')\n"
                "if not callable(ans.build_mission_plan_0301):\n"
                "    raise SystemExit('AnS build_mission_plan_0301 missing')\n"
                "original = canonical_config.SEARCH_SPEED_WEIGHT\n"
                "try:\n"
                "    config.SEARCH_SPEED_WEIGHT = 7.5\n"
                "    if canonical_config.SEARCH_SPEED_WEIGHT != 7.5:\n"
                "        raise SystemExit('data_def -> config assignment forwarding failed')\n"
                "finally:\n"
                "    config.SEARCH_SPEED_WEIGHT = original\n"
            ),
        ),
        (
            "config_first",
            bootstrap
            + (
                "config = importlib.import_module('config')\n"
                "search_speed = importlib.import_module('data_def.search_speed')\n"
                "canonical_config = importlib.import_module('modules.mission_planning.MissionPlanner.config')\n"
                "if config is not canonical_config:\n"
                "    raise SystemExit('config module identity split')\n"
                "original = canonical_config.SEARCH_SPEED_WEIGHT\n"
                "try:\n"
                "    config.SEARCH_SPEED_WEIGHT = 12.5\n"
                "    if canonical_config.SEARCH_SPEED_WEIGHT != 12.5:\n"
                "        raise SystemExit('config assignment forwarding failed')\n"
                "finally:\n"
                "    config.SEARCH_SPEED_WEIGHT = original\n"
                "if not callable(search_speed.spacing_based_search_speed):\n"
                "    raise SystemExit('data_def.search_speed spacing_based_search_speed missing')\n"
            ),
        ),
    )
    for cwd_rel in (".", "modules/mission_planning/MissionPlanner"):
        for label, code in snippets:
            result = subprocess.run(
                [sys.executable, "-c", code],
                cwd=PROJECT_ROOT / cwd_rel,
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                fail(
                    "mission planner bare import shim failed for {} from {}:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                        label,
                        cwd_rel,
                        result.stdout,
                        result.stderr,
                    )
                )


def check_unsupported_paths() -> None:
    unsupported = ("modules.mission_planning.post_attack_rejoin_pipeline",)
    for module_name in unsupported:
        try:
            importlib.import_module(module_name)
        except ModuleNotFoundError:
            continue
        fail(f"unsupported compatibility path unexpectedly imports: {module_name}")


def check_planner_runtime_contract() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    import_paths = {str(path).replace("\\", "/") for path in runtime.MISSION_PLANNER_IMPORT_RELATIVE_PATHS}
    watch = {str(path).replace("\\", "/") for path in runtime.PLANNER_RUNTIME_WATCH_RELATIVE_PATHS}
    reload_order = set(runtime.PLANNER_RUNTIME_RELOAD_ORDER)
    bindings = runtime.PIPELINE_RELOAD_BINDINGS

    required_import_paths = (
        ".",
        "modules",
        "modules/mission_planning",
        "modules/mission_planning/MissionPlanner",
    )
    for rel in required_import_paths:
        if rel not in import_paths:
            fail(f"mission planner import bootstrap paths missing {rel}")

    required_modules = (
        "modules.mission_planning.MissionPlanner.AnS.coord_transform",
        "modules.mission_planning.MissionPlanner.AnS.task_patterns_ver2",
        "modules.mission_planning.MissionPlanner.AnS.mission_effectiveness_ver2",
        "modules.mission_planning.MissionPlanner.AnS.env_patternselection",
        "modules.mission_planning.MissionPlanner.AnS.mission_pipeline",
        "modules.mission_planning.MissionPlanner.AnS",
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan",
        "modules.mission_planning.replanning.triggers.remaining_hybrid.general",
        "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first",
        "modules.mission_planning.replanning.triggers.recon_specialized.pipeline",
        "modules.mission_planning.replanning.triggers.attack.pipeline",
        "modules.mission_planning.replanning.triggers.post_attack.pipeline",
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
    )
    required_paths = (
        "modules/mission_planning/MissionPlanner/AnS/__init__.py",
        "modules/mission_planning/MissionPlanner/AnS/coord_transform.py",
        "modules/mission_planning/MissionPlanner/AnS/task_patterns_ver2.py",
        "modules/mission_planning/MissionPlanner/AnS/mission_effectiveness_ver2.py",
        "modules/mission_planning/MissionPlanner/AnS/env_patternselection.py",
        "modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py",
        "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0301.py",
        "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0302.py",
        "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py",
        "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py",
        "modules/mission_planning/replanning/triggers/prior/pipeline.py",
        "modules/mission_planning/replanning/triggers/next_collab/pipeline.py",
        "modules/mission_planning/replanning/triggers/remaining_hybrid/current.py",
        "modules/mission_planning/replanning/triggers/remaining_hybrid/current_replan.py",
        "modules/mission_planning/replanning/triggers/remaining_hybrid/general.py",
        "modules/mission_planning/replanning/triggers/remaining_hybrid/reexecute_first.py",
        "modules/mission_planning/replanning/triggers/recon_specialized/pipeline.py",
        "modules/mission_planning/replanning/triggers/attack/pipeline.py",
        "modules/mission_planning/replanning/triggers/post_attack/pipeline.py",
        "modules/mission_planning/replanning/triggers/imaging_schedule/pipeline.py",
        "modules/mission_planning/replanning/triggers/path_deviation/pipeline.py",
    )
    for module_name in required_modules:
        if module_name not in reload_order:
            fail(f"planner reload order missing {module_name}")
        if module_name in (
            "modules.mission_planning.MissionPlanner.AnS.coord_transform",
            "modules.mission_planning.MissionPlanner.AnS.task_patterns_ver2",
            "modules.mission_planning.MissionPlanner.AnS.mission_effectiveness_ver2",
            "modules.mission_planning.MissionPlanner.AnS.env_patternselection",
            "modules.mission_planning.MissionPlanner.AnS.mission_pipeline",
            "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
            "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan",
            "modules.mission_planning.replanning.triggers.remaining_hybrid.general",
            "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first",
            "modules.mission_planning.replanning.triggers.recon_specialized.pipeline",
        ):
            continue
        if module_name not in bindings:
            fail(f"planner reload bindings missing {module_name}")
    ans_bindings = set(bindings.get("modules.mission_planning.MissionPlanner.AnS", ()))
    for attr_name in (
        "run_divide_and_pattern",
        "run_pulp_scheduling",
        "build_mission_plan_0301",
        "get_last_divide_and_pattern_metrics",
    ):
        if attr_name not in ans_bindings:
            fail(f"planner AnS reload bindings missing {attr_name}")
    for rel in required_paths:
        if rel not in watch:
            fail(f"planner watch paths missing {rel}")


def check_mission_planner_path_bootstrap_contract() -> None:
    code = (
        "import importlib, sys\n"
        "from pathlib import Path\n"
        f"project_root = Path({str(PROJECT_ROOT)!r})\n"
        "sys.path[:] = [\n"
        "    path for path in sys.path\n"
        "    if path and str(project_root) not in str(Path(path).resolve())\n"
        "]\n"
        "sys.path.insert(0, str(project_root))\n"
        "runtime = importlib.import_module('modules.mission_planning.mission_control.planner_runtime')\n"
        "sys.path[:] = [\n"
        "    path for path in sys.path\n"
        "    if path and str(project_root) not in str(Path(path).resolve())\n"
        "]\n"
        "runtime.ensure_mission_planner_import_paths(project_root)\n"
        "expected = [\n"
        "    str(project_root),\n"
        "    str(project_root / 'modules'),\n"
        "    str(project_root / 'modules' / 'mission_planning'),\n"
        "    str(project_root / 'modules' / 'mission_planning' / 'MissionPlanner'),\n"
        "]\n"
        "if sys.path[:4] != expected:\n"
        "    raise SystemExit(f'import path bootstrap order changed: {sys.path[:4]!r}')\n"
        "ans = importlib.import_module('AnS')\n"
        "ans_pipeline = importlib.import_module('AnS.mission_pipeline')\n"
        "canonical_ans = importlib.import_module('modules.mission_planning.MissionPlanner.AnS')\n"
        "canonical_pipeline = importlib.import_module('modules.mission_planning.MissionPlanner.AnS.mission_pipeline')\n"
        "if ans is not canonical_ans:\n"
        "    raise SystemExit('bootstrap AnS package identity split')\n"
        "if ans_pipeline is not canonical_pipeline:\n"
        "    raise SystemExit('bootstrap AnS.mission_pipeline identity split')\n"
        "expected_dem = str(project_root / 'modules' / 'mission_planning' / 'MissionPlanner' / 'AnS' / 'DEM.jpg')\n"
        "expected_counter = str(project_root / 'modules' / 'mission_planning' / 'MissionPlanner' / 'AnS' / '_id_counters.json')\n"
        "if str(Path(canonical_pipeline.DEM_PATH)) != expected_dem:\n"
        "    raise SystemExit(f'AnS DEM_PATH changed: {canonical_pipeline.DEM_PATH!r}')\n"
        "if str(Path(canonical_pipeline._ID_COUNTER_FILE)) != expected_counter:\n"
        "    raise SystemExit(f'AnS _ID_COUNTER_FILE changed: {canonical_pipeline._ID_COUNTER_FILE!r}')\n"
        "pipeline = importlib.reload(canonical_pipeline)\n"
        "package = importlib.reload(canonical_ans)\n"
        "if importlib.import_module('AnS') is not package:\n"
        "    raise SystemExit('bootstrap AnS reload package identity split')\n"
        "if importlib.import_module('AnS.mission_pipeline') is not pipeline:\n"
        "    raise SystemExit('bootstrap AnS reload pipeline identity split')\n"
        "if package.run_divide_and_pattern is not pipeline.run_divide_and_pattern:\n"
        "    raise SystemExit('bootstrap AnS reload export stale')\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=PROJECT_ROOT / "docs",
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        fail(
            "mission planner path bootstrap contract failed:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                result.stdout,
                result.stderr,
            )
        )

    order_cases = (
        "AnS",
        "modules.mission_planning.MissionPlanner.AnS",
        "AnS.mission_pipeline",
        "modules.mission_planning.MissionPlanner.AnS.mission_pipeline",
    )
    for cwd_rel in (".", "modules/mission_planning/MissionPlanner"):
        for first_name in order_cases:
            code = (
                "import importlib, sys\n"
                "from pathlib import Path\n"
                f"project_root = Path({str(PROJECT_ROOT)!r})\n"
                "if str(project_root) not in sys.path:\n"
                "    sys.path.insert(0, str(project_root))\n"
                "runtime = importlib.import_module('modules.mission_planning.mission_control.planner_runtime')\n"
                "runtime.ensure_mission_planner_import_paths(project_root)\n"
                f"first_name = {first_name!r}\n"
                "importlib.import_module(first_name)\n"
                "ans = importlib.import_module('AnS')\n"
                "ans_pipeline = importlib.import_module('AnS.mission_pipeline')\n"
                "canonical_ans = importlib.import_module('modules.mission_planning.MissionPlanner.AnS')\n"
                "canonical_pipeline = importlib.import_module('modules.mission_planning.MissionPlanner.AnS.mission_pipeline')\n"
                "if ans is not canonical_ans:\n"
                "    raise SystemExit(f'{first_name}: AnS package identity split')\n"
                "if ans_pipeline is not canonical_pipeline:\n"
                "    raise SystemExit(f'{first_name}: AnS.mission_pipeline identity split')\n"
                "pipeline = importlib.reload(canonical_pipeline)\n"
                "package = importlib.reload(canonical_ans)\n"
                "if package.run_divide_and_pattern is not pipeline.run_divide_and_pattern:\n"
                "    raise SystemExit(f'{first_name}: package export stale after reload')\n"
            )
            result = subprocess.run(
                [sys.executable, "-c", code],
                cwd=PROJECT_ROOT / cwd_rel,
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                fail(
                    "mission planner AnS import-order contract failed for {} from {}:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                        first_name,
                        cwd_rel,
                        result.stdout,
                        result.stderr,
                    )
                )


def check_forbidden_old_impl_imports() -> None:
    offenders: list[str] = []
    for path in (PROJECT_ROOT / "modules/mission_planning").rglob("*.py"):
        if "__pycache__" in path.parts:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for forbidden in FORBIDDEN_MOVED_IMPL_IMPORTS:
            if forbidden in text:
                offenders.append(f"{path.relative_to(PROJECT_ROOT)} contains {forbidden}")
    if offenders:
        fail("forbidden moved implementation imports found:\n" + "\n".join(offenders))


def check_moved_pipeline_wrapper_shape() -> None:
    import ast

    offenders: list[str] = []
    for rel_path, canonical_name in MOVED_PIPELINE_WRAPPER_TARGETS.items():
        path = PROJECT_ROOT / rel_path
        if not path.exists():
            offenders.append(f"{rel_path} is missing")
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if canonical_name not in text:
            offenders.append(f"{rel_path} does not reference {canonical_name}")
            continue
        try:
            tree = ast.parse(text)
        except SyntaxError as exc:
            offenders.append(f"{rel_path} is not parseable: {exc}")
            continue
        definitions = [
            node.name
            for node in tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        ]
        if definitions:
            offenders.append(f"{rel_path} still defines implementation symbols: {', '.join(definitions)}")
    if offenders:
        fail("moved pipeline wrapper shape check failed:\n" + "\n".join(offenders))


def check_moved_runtime_wrapper_shape() -> None:
    import ast

    offenders: list[str] = []
    for rel_path, canonical_name in MOVED_RUNTIME_WRAPPER_TARGETS.items():
        path = PROJECT_ROOT / rel_path
        if not path.exists():
            offenders.append(f"{rel_path} is missing")
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if canonical_name not in text:
            offenders.append(f"{rel_path} does not reference {canonical_name}")
            continue
        try:
            tree = ast.parse(text)
        except SyntaxError as exc:
            offenders.append(f"{rel_path} is not parseable: {exc}")
            continue
        definitions = [
            node.name
            for node in tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        ]
        if definitions:
            offenders.append(f"{rel_path} still defines implementation symbols: {', '.join(definitions)}")
    if offenders:
        fail("moved runtime wrapper shape check failed:\n" + "\n".join(offenders))


def check_moved_artifact_wrapper_shape() -> None:
    import ast

    offenders: list[str] = []
    for rel_path, canonical_name in MOVED_ARTIFACT_WRAPPER_TARGETS.items():
        path = PROJECT_ROOT / rel_path
        if not path.exists():
            offenders.append(f"{rel_path} is missing")
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if canonical_name not in text:
            offenders.append(f"{rel_path} does not reference {canonical_name}")
            continue
        try:
            tree = ast.parse(text)
        except SyntaxError as exc:
            offenders.append(f"{rel_path} is not parseable: {exc}")
            continue
        definitions = [
            node.name
            for node in tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        ]
        if definitions:
            offenders.append(f"{rel_path} still defines implementation symbols: {', '.join(definitions)}")
    if offenders:
        fail("moved artifact builder wrapper shape check failed:\n" + "\n".join(offenders))


def check_artifact_builder_contract() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    runtime.ensure_mission_planner_import_paths(PROJECT_ROOT)

    package_name = ARTIFACT_BUILDER_PACKAGE
    for module_name, attr_names in ARTIFACT_BUILDER_CONTRACTS.items():
        canonical_name = f"{package_name}.{module_name}"
        old_name = f"modules.mission_planning.MissionPlanner.data_def.{module_name}"
        bare_name = f"data_def.{module_name}"
        canonical = importlib.import_module(canonical_name)
        old = importlib.import_module(old_name)
        bare = importlib.import_module(bare_name)
        if old is not canonical:
            fail(f"{old_name} module identity split from {canonical_name}")
        if bare is not canonical:
            fail(f"{bare_name} module identity split from {canonical_name}")
        for attr_name in attr_names:
            if getattr(old, attr_name) is not getattr(canonical, attr_name):
                fail(f"{old_name}.{attr_name} identity split from canonical")
            if getattr(bare, attr_name) is not getattr(canonical, attr_name):
                fail(f"{bare_name}.{attr_name} identity split from canonical")

    d0301 = importlib.import_module(f"{package_name}.d0301")
    expected_counter = PROJECT_ROOT / "modules/mission_planning/MissionPlanner/data_def/_id_counters.json"
    if Path(getattr(d0301, "_COUNTER_FILE", "")) != expected_counter:
        fail(f"d0301 counter file path changed: {getattr(d0301, '_COUNTER_FILE', None)}")

    d0303 = importlib.import_module(f"{package_name}.d0303")
    expected_fov_db = PROJECT_ROOT / "resource/db/fov_db.csv"
    if Path(getattr(d0303, "_FOV_DB_PATH", "")) != expected_fov_db:
        fail(f"d0303 FOV DB path changed: {getattr(d0303, '_FOV_DB_PATH', None)}")

    data_def = importlib.import_module("data_def")
    if not callable(data_def.build_lah_flight_plans_fixed):
        fail("data_def package public build_lah_flight_plans_fixed missing")

    snippets = []
    import_names = (
        *(f"{package_name}.{module_name}" for module_name in ARTIFACT_BUILDER_CONTRACTS),
        *(f"modules.mission_planning.MissionPlanner.data_def.{module_name}" for module_name in ARTIFACT_BUILDER_CONTRACTS),
        *(f"data_def.{module_name}" for module_name in ARTIFACT_BUILDER_CONTRACTS),
    )
    for first_name in import_names:
        snippets.append(
            (
                first_name,
                (
                    "import importlib\n"
                    "import sys\n"
                    "from pathlib import Path\n"
                    f"package_name = {package_name!r}\n"
                    f"first_name = {first_name!r}\n"
                    f"contracts = {ARTIFACT_BUILDER_CONTRACTS!r}\n"
                    f"project_root = Path({str(PROJECT_ROOT)!r})\n"
                    "if str(project_root) not in sys.path:\n"
                    "    sys.path.insert(0, str(project_root))\n"
                    "runtime = importlib.import_module('modules.mission_planning.mission_control.planner_runtime')\n"
                    "runtime.ensure_mission_planner_import_paths(project_root)\n"
                    "importlib.import_module(first_name)\n"
                    "for module_name, attr_names in contracts.items():\n"
                    "    canonical_name = f'{package_name}.{module_name}'\n"
                    "    old_name = f'modules.mission_planning.MissionPlanner.data_def.{module_name}'\n"
                    "    bare_name = f'data_def.{module_name}'\n"
                    "    canonical = importlib.import_module(canonical_name)\n"
                    "    old = importlib.import_module(old_name)\n"
                    "    bare = importlib.import_module(bare_name)\n"
                    "    if old is not canonical or bare is not canonical:\n"
                    "        raise SystemExit(f'{first_name}: module identity split for {module_name}')\n"
                    "    for attr in attr_names:\n"
                    "        if getattr(old, attr) is not getattr(canonical, attr):\n"
                    "            raise SystemExit(f'{first_name}: old {module_name}.{attr} identity split')\n"
                    "        if getattr(bare, attr) is not getattr(canonical, attr):\n"
                    "            raise SystemExit(f'{first_name}: bare {module_name}.{attr} identity split')\n"
                    "for module_name in ('d0303', 'd0304'):\n"
                    "    canonical = importlib.import_module(f'{package_name}.{module_name}')\n"
                    "    reloaded = importlib.reload(canonical)\n"
                    "    if reloaded is not canonical:\n"
                    "        raise SystemExit(f'{first_name}: reload returned different module for {module_name}')\n"
                    "    old = importlib.import_module(f'modules.mission_planning.MissionPlanner.data_def.{module_name}')\n"
                    "    bare = importlib.import_module(f'data_def.{module_name}')\n"
                    "    if old is not canonical or bare is not canonical:\n"
                    "        raise SystemExit(f'{first_name}: reload identity split for {module_name}')\n"
                ),
            )
        )

    for label, code in snippets:
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            fail(
                "artifact builder import contract failed for {}:\nSTDOUT:\n{}\nSTDERR:\n{}".format(
                    label,
                    result.stdout,
                    result.stderr,
                )
            )


def check_runtime_state_contract() -> None:
    expected_filenames = {
        "modules.mission_planning.runtime.state.attack_assignment": "attack_assignment_state.json",
        "modules.mission_planning.runtime.state.attack_tracking": "attack_tracking_state.json",
        "modules.mission_planning.runtime.state.prior_tracking": "prior_tracking_state.json",
    }
    for module_name, expected_filename in expected_filenames.items():
        module = importlib.import_module(module_name)
        if getattr(module, "_STATE_FILENAME", None) != expected_filename:
            fail(f"{module_name} changed state filename")


def main() -> int:
    parser = argparse.ArgumentParser(description="Mission planning refactor import-contract smoke.")
    parser.add_argument(
        "--require-git-tracked",
        action="store_true",
        help="also require newly extracted refactor files to be tracked by git",
    )
    args = parser.parse_args()

    check_required_files(require_git_tracked=args.require_git_tracked)
    check_import_order()
    check_wrapper_identity()
    check_runtime_bare_import_compat()
    check_id_allocator_wrapper_contract()
    check_planning_enhanced_import_graph_contract()
    check_mission_planner_bare_import_shims()
    check_unsupported_paths()
    check_planner_runtime_contract()
    check_forbidden_old_impl_imports()
    check_moved_pipeline_wrapper_shape()
    check_moved_runtime_wrapper_shape()
    check_moved_artifact_wrapper_shape()
    check_artifact_builder_contract()
    check_mission_planner_path_bootstrap_contract()
    check_runtime_state_contract()
    print("mission planning refactor import-contract smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
