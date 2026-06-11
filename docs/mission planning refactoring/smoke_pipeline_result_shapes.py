from __future__ import annotations

import argparse
import importlib
import inspect
import os
import sys
from dataclasses import MISSING, fields, is_dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("KU_ROLE", "mission")
    desired = (
        project_root,
        project_root / "modules",
        project_root / "modules" / "mission_planning",
        project_root / "modules" / "mission_planning" / "MissionPlanner",
    )
    for path in reversed(desired):
        path_str = str(path)
        if not path.exists():
            continue
        while path_str in sys.path:
            sys.path.remove(path_str)
        sys.path.insert(0, path_str)


def _field_default_marker(field: Any) -> str:
    if field.default is not MISSING:
        return f"default={field.default!r}"
    if field.default_factory is not MISSING:
        return f"factory={getattr(field.default_factory, '__name__', repr(field.default_factory))}"
    return "required"


RESULT_SHAPES: dict[tuple[str, str], tuple[tuple[str, str], ...]] = {
    (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        "PriorMissionPipelineResult",
    ): (
        ("plan_ids", "required"),
        ("option_names", "required"),
        ("plan_meta_map", "required"),
        ("generated_imp_ids", "required"),
        ("generated_path_ids", "required"),
        ("new_imp_id", "required"),
        ("new_path_id", "required"),
        ("new_individual_id", "required"),
        ("resume_path_id", "required"),
        ("resume_individual_id", "required"),
        ("log_path", "required"),
        ("removed_waypoint_id", "required"),
        ("inserted_waypoint_id", "required"),
        ("approach_waypoint_id", "required"),
        ("target_waypoint_id", "required"),
    ),
    (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        "PriorPostRejoinPipelineResult",
    ): (
        ("plan_ids", "required"),
        ("option_names", "required"),
        ("plan_meta_map", "required"),
        ("generated_imp_ids", "required"),
        ("generated_path_ids", "required"),
        ("log_path", "required"),
        ("status", "required"),
        ("summary", "required"),
    ),
    (
        "modules.mission_planning.replanning.triggers.post_attack.pipeline",
        "PostAttackRejoinPipelineResult",
    ): (
        ("plan_ids", "required"),
        ("option_names", "required"),
        ("plan_meta_map", "required"),
        ("generated_imp_ids", "required"),
        ("generated_path_ids", "required"),
        ("log_path", "required"),
        ("status", "required"),
        ("summary", "required"),
    ),
    (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        "NextCollabPipelineResult",
    ): (
        ("plan_ids", "required"),
        ("option_names", "required"),
        ("plan_meta_map", "required"),
        ("generated_imp_ids", "required"),
        ("generated_path_ids", "required"),
        ("new_input_package_id", "required"),
        ("log_path", "required"),
    ),
    (
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        "ImagingSchedulePipelineResult",
    ): (
        ("plan_ids", "required"),
        ("option_names", "required"),
        ("plan_meta_map", "required"),
        ("generated_imp_ids", "required"),
        ("generated_path_ids", "required"),
        ("new_imp_id", "required"),
        ("new_path_id", "required"),
        ("new_individual_id", "required"),
        ("replaced_waypoint_id", "required"),
        ("new_waypoint_id", "required"),
        ("log_path", "required"),
        ("trigger_type", "default='imagingScheduleDeviation'"),
        ("removed_waypoint_id", "default=None"),
        ("anchor_waypoint_id", "default=None"),
        ("search_speed_scale", "default=None"),
        ("speed_adjustment_direction", "default=None"),
        ("trimmed_sweep_points", "default=0"),
    ),
    (
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        "PathDeviationPipelineResult",
    ): (
        ("plan_ids", "required"),
        ("option_names", "required"),
        ("plan_meta_map", "required"),
        ("generated_imp_ids", "required"),
        ("generated_path_ids", "required"),
        ("preserved_manned_imp_ids", "required"),
        ("preserved_manned_path_ids", "required"),
        ("new_imp_id", "required"),
        ("new_path_id", "required"),
        ("new_individual_id", "required"),
        ("removed_waypoint_id", "required"),
        ("inserted_waypoint_id", "required"),
        ("log_path", "required"),
        ("other_updates", "required"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan",
        "CurrentRemainingHybridResult",
    ): (
        ("prepared", "required"),
        ("current_input_id", "required"),
        ("target_aircraft_ids", "required"),
        ("entry_coord_map", "required"),
        ("heading_map", "required"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.general",
        "RemainingHybridResult",
    ): (
        ("applied", "required"),
        ("mode", "default=None"),
        ("input_mission_id", "default=None"),
        ("aircraft_ids", "default=None"),
        ("planner_workflow", "default=None"),
        ("reason", "default=None"),
        ("validation", "default=None"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        "CurrentRemainingHybridRequest",
    ): (
        ("source_plan_id", "required"),
        ("current_input_id", "required"),
        ("current_input_mission", "required"),
        ("next_input_mission", "required"),
        ("entry_coord_map", "required"),
        ("heading_map", "required"),
        ("representative_entry", "required"),
        ("turn_radius_scale", "required"),
        ("apply_option_ordinals", "default=None"),
        ("planner_mode", "default='current_remaining'"),
        ("source_template_input_id", "default=None"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        "CurrentRemainingHybridGeometry",
    ): (
        ("current_input_id", "required"),
        ("source_plan_id", "required"),
        ("planner_mode", "required"),
        ("aircraft_ids", "required"),
        ("generated_path_ids", "required"),
        ("path_aircraft_by_id", "factory=dict"),
        ("source_template_input_id", "default=None"),
        ("mission_mode", "default=''"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        "CurrentRemainingHybridRuntimeResult",
    ): (
        ("planner_workflow", "required"),
        ("planner_result_text", "required"),
        ("prepare_timing_ms", "factory=dict"),
        ("id_reservation", "factory=dict"),
        ("uav_work_summary", "factory=dict"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        "CurrentRemainingHybridResult",
    ): (
        ("current_input_id", "required"),
        ("missions", "required"),
        ("flight_plans_0303", "required"),
        ("generated_path_ids", "required"),
        ("aircraft_ids", "required"),
        ("planner_workflow", "required"),
        ("planner_result_text", "required"),
        ("flight_plans_0304", "factory=list"),
        ("prepare_timing_ms", "factory=dict"),
        ("geometry", "default=None"),
        ("runtime_result", "default=None"),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        "GenericFlightPathSkipResult",
    ): (
        ("missions", "required"),
        ("skipped_path_ids", "required"),
        ("skipped_aircraft_ids", "required"),
        ("skipped_count", "required"),
        ("skip_policy", "factory=dict"),
    ),
}


WRAPPER_CLASS_IDENTITY: tuple[tuple[str, str, tuple[str, ...]], ...] = (
    (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        "PriorMissionPipelineResult",
        (
            "modules.mission_planning.prior_mission_pipeline_impl",
            "modules.mission_planning.legacy.wrappers.prior_mission_pipeline_impl",
            "modules.mission_planning.pipelines.prior_mission_pipeline_impl",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        "PriorPostRejoinPipelineResult",
        (
            "modules.mission_planning.prior_mission_pipeline_impl",
            "modules.mission_planning.legacy.wrappers.prior_mission_pipeline_impl",
            "modules.mission_planning.pipelines.prior_mission_pipeline_impl",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        "NextCollabPipelineResult",
        (
            "modules.mission_planning.pipelines.next_collab_replan_pipeline",
            "modules.mission_planning.pipelines.next_collab_replan_pipeline_impl",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        "ImagingSchedulePipelineResult",
        (
            "modules.mission_planning.pipelines.imaging_schedule_replan_pipeline_impl",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        "PathDeviationPipelineResult",
        (
            "modules.mission_planning.pipelines.path_deviation_replan_pipeline_impl",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.post_attack.pipeline",
        "PostAttackRejoinPipelineResult",
        (
            "modules.mission_planning.pipelines.post_attack_rejoin_pipeline",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan",
        "CurrentRemainingHybridResult",
        (
            "modules.mission_planning.pipelines.current_remaining_hybrid_replan",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.general",
        "RemainingHybridResult",
        (
            "modules.mission_planning.replanning.triggers.remaining_hybrid",
            "modules.mission_planning.pipelines.general_remaining_hybrid_replan",
        ),
    ),
    (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        "CurrentRemainingHybridRequest",
        (
            "modules.mission_planning.replanning.triggers.remaining_hybrid",
            "modules.mission_planning.pipelines.current_remaining_hybrid",
        ),
    ),
)


def _load_class(module_name: str, class_name: str) -> type:
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name, None)
    if not isinstance(cls, type):
        raise RuntimeError(f"{module_name}.{class_name} is not a class")
    if not is_dataclass(cls):
        raise RuntimeError(f"{module_name}.{class_name} is not a dataclass")
    return cls


def check_dataclass_shapes() -> None:
    for (module_name, class_name), expected in RESULT_SHAPES.items():
        cls = _load_class(module_name, class_name)
        actual = tuple((field.name, _field_default_marker(field)) for field in fields(cls))
        if actual != expected:
            raise RuntimeError(
                f"{module_name}.{class_name} result shape changed:\n"
                f"actual={actual!r}\nexpected={expected!r}"
            )


def check_wrapper_class_identity() -> None:
    for canonical_module, class_name, wrapper_modules in WRAPPER_CLASS_IDENTITY:
        canonical = importlib.import_module(canonical_module)
        canonical_cls = getattr(canonical, class_name)
        for wrapper_module in wrapper_modules:
            wrapper = importlib.import_module(wrapper_module)
            wrapper_cls = getattr(wrapper, class_name, None)
            if wrapper_cls is not canonical_cls:
                raise RuntimeError(
                    f"{wrapper_module}.{class_name} identity split from "
                    f"{canonical_module}.{class_name}"
                )


def _assert_iterable_ints(values: Any, label: str) -> None:
    try:
        {int(value) for value in values if value is not None}
    except Exception as exc:
        raise RuntimeError(f"{label} no longer supports GUI int-set consumption: {exc}") from exc


def check_gui_consumption_fixtures() -> None:
    common_result = {
        "plan_ids": [101],
        "option_names": ["option"],
        "plan_meta_map": {101: {"kind": "fixture"}},
        "generated_imp_ids": {201},
        "generated_path_ids": {301},
        "log_path": PROJECT_ROOT / "fixture.log",
    }
    list(common_result["plan_ids"])
    list(common_result["option_names"] or [])
    dict(common_result["plan_meta_map"] or {})
    str(common_result["log_path"])
    _assert_iterable_ints(common_result["generated_imp_ids"], "generated_imp_ids")
    _assert_iterable_ints(common_result["generated_path_ids"], "generated_path_ids")

    int({"new_input_package_id": 401}["new_input_package_id"])
    int({"replaced_waypoint_id": 501, "new_waypoint_id": 502}["replaced_waypoint_id"])
    int({"replaced_waypoint_id": 501, "new_waypoint_id": 502}["new_waypoint_id"])
    int({"removed_waypoint_id": 601, "inserted_waypoint_id": 602}["removed_waypoint_id"])
    int({"removed_waypoint_id": 601, "inserted_waypoint_id": 602}["inserted_waypoint_id"])

    rejoin_result = {"summary": {"status": "skipped"}, "status": "skipped", "plan_ids": []}
    summary = dict(rejoin_result["summary"] or {})
    summary.setdefault("status", str(rejoin_result.get("status") or "skipped"))

    attack_result = {
        "log_path": "attack.log",
        "result": {
            "missionUpdates": {
                "mission_plan_id": 701,
                "aircraft": [
                    {
                        "individualMissionPackageID": 801,
                        "missions": [{"pathID": 901}],
                        "tracking": {"pathID": 902},
                        "resume": {"pathID": 903},
                        "flightPaths": {"main": "904.json"},
                        "pathID": 905,
                    }
                ],
            },
            "failure_notice": "",
            "primary_target": {"target_id": 1},
            "attack_targets": [{"target_id": 1}],
        },
    }
    attack_body = (attack_result or {}).get("result") or {}
    attack_updates = attack_body.get("missionUpdates")
    if not isinstance(attack_updates, dict):
        raise RuntimeError("attack result missing result.missionUpdates dict")
    int(attack_updates.get("mission_plan_id"))
    str(attack_result.get("log_path") or "")
    str(attack_body.get("failure_notice") or "").strip()
    for entry in attack_updates.get("aircraft") or []:
        int(entry.get("individualMissionPackageID"))
        for mission in entry.get("missions") or []:
            int(mission.get("pathID"))
        for block_name in ("tracking", "resume"):
            int((entry.get(block_name) or {}).get("pathID"))
        for value in (entry.get("flightPaths") or {}).values():
            text = str(value)
            if not (text.isdigit() or Path(text).stem.isdigit()):
                raise RuntimeError(f"attack flightPaths value is not GUI-parseable: {value!r}")
        int(entry.get("pathID"))

    attack_failure = {
        "log_path": "attack.log",
        "result": {
            "failure_code": "missing_target_coordinate",
            "failure_notice": "notice",
            "primary_target": None,
            "attack_targets": [],
        },
    }
    failure_body = (attack_failure or {}).get("result") or {}
    if not str(failure_body.get("failure_notice") or "").strip():
        raise RuntimeError("attack failure fixture missing failure_notice")
    str(failure_body.get("failure_code") or "").strip()

    attack_exclusion_success = {
        "logMessages": ["ok"],
        "timingMs": {"total": 1},
        "result": {
            "sourcePlanID": 1001,
            "missionPlanID": 1002,
            "planPath": "1002.json",
            "missionUpdates": {"mode": "attack_exclusion"},
        },
    }
    for message in attack_exclusion_success.get("logMessages") or []:
        str(message)
    int((attack_exclusion_success.get("result") or {}).get("missionPlanID"))

    attack_exclusion_failure = {
        "logMessages": ["failed"],
        "result": {
            "error": "no_updates",
            "sourcePlanID": 1001,
        },
    }
    if (attack_exclusion_failure.get("result") or {}).get("error") != "no_updates":
        raise RuntimeError("attack exclusion failure fixture missing result.error")


def _require_source_markers(source: str, markers: tuple[str, ...], label: str) -> None:
    missing = [marker for marker in markers if marker not in source]
    if missing:
        raise RuntimeError(f"{label} source markers missing: {', '.join(missing)}")


def check_attack_dict_source_shape() -> None:
    attack = importlib.import_module("modules.mission_planning.replanning.triggers.attack.pipeline")
    run_source = inspect.getsource(attack.run_attack_plan_pipeline)
    persist_source = inspect.getsource(attack._persist_attack_log)
    override_source = inspect.getsource(attack._apply_attack_plan_overrides)
    exclusion_source = inspect.getsource(attack.run_attack_exclusion_pipeline)

    _require_source_markers(
        run_source,
        (
            '"timestamp"',
            '"replanTransactionId"',
            '"context"',
            '"steps"',
            '"timingMs"',
            '"logMessages"',
            '"log_text"',
            '"result"',
            '"missionUpdates"',
            '"failure_code"',
            '"failure_notice"',
            '"primary_target"',
            '"attack_targets"',
            '"selected_aircraft"',
            '"selected_manned_aircraft"',
            '"attack_point"',
            '"weapon_choice"',
        ),
        "run_attack_plan_pipeline",
    )
    _require_source_markers(
        persist_source,
        (
            'payload["log_path"]',
            '"log_artifact_written"',
        ),
        "_persist_attack_log",
    )
    _require_source_markers(
        override_source,
        (
            '"mission_plan_id"',
            '"plan_path"',
            '"attack_targets"',
            '"aircraft"',
            '"collaborativeRemainingReplan"',
            '"timingMs"',
        ),
        "_apply_attack_plan_overrides",
    )
    _require_source_markers(
        exclusion_source,
        (
            '"logMessages"',
            '"timingMs"',
            '"result"',
            '"missionPlanID"',
            '"planPath"',
            '"missionUpdates"',
            '"error"',
            '"no_updates"',
            '"validationErrors"',
        ),
        "run_attack_exclusion_pipeline",
    )


def print_current_shapes() -> None:
    for (module_name, class_name), _expected in RESULT_SHAPES.items():
        cls = _load_class(module_name, class_name)
        actual = tuple((field.name, _field_default_marker(field)) for field in fields(cls))
        print(f"{module_name}.{class_name}")
        for name, marker in actual:
            print(f"  {name}\t{marker}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke snapshot pipeline result object shapes.")
    parser.add_argument("--print-current", action="store_true")
    args = parser.parse_args()

    try:
        configure_import_paths()
        if args.print_current:
            print_current_shapes()
            return 0
        check_dataclass_shapes()
        check_wrapper_class_identity()
        check_gui_consumption_fixtures()
        check_attack_dict_source_shape()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print(f"pipeline result shape smoke ok ({len(RESULT_SHAPES)} dataclasses + attack dicts)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
