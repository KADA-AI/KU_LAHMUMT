"""Planner runtime reload and import-path helpers.

This module keeps the current hot-reload contract separate from the GUI shell
while preserving the existing global rebinding behavior through an explicit
namespace argument.
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path
from typing import Any, Mapping, MutableMapping


MISSION_PLANNER_IMPORT_RELATIVE_PATHS = (
    Path("."),
    Path("modules"),
    Path("modules/mission_planning"),
    Path("modules/mission_planning/MissionPlanner"),
)

PLANNER_RUNTIME_WATCH_RELATIVE_PATHS = (
    Path("modules/mission_planning/MissionPlanner/AnS/__init__.py"),
    Path("modules/mission_planning/MissionPlanner/AnS/coord_transform.py"),
    Path("modules/mission_planning/MissionPlanner/AnS/task_patterns_ver2.py"),
    Path("modules/mission_planning/MissionPlanner/AnS/mission_effectiveness_ver2.py"),
    Path("modules/mission_planning/MissionPlanner/AnS/env_patternselection.py"),
    Path("modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py"),
    Path("modules/mission_planning/MissionPlanner/data_def/d0302.py"),
    Path("modules/mission_planning/MissionPlanner/data_def/d0303.py"),
    Path("modules/mission_planning/MissionPlanner/data_def/d0304.py"),
    Path("modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0301.py"),
    Path("modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0302.py"),
    Path("modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py"),
    Path("modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py"),
    Path("modules/mission_planning/replanning/triggers/remaining_hybrid/current.py"),
    Path("modules/mission_planning/replanning/triggers/remaining_hybrid/current_replan.py"),
    Path("modules/mission_planning/replanning/triggers/remaining_hybrid/general.py"),
    Path("modules/mission_planning/replanning/triggers/remaining_hybrid/reexecute_first.py"),
    Path("modules/mission_planning/replanning/triggers/recon_specialized/pipeline.py"),
    Path("modules/mission_planning/pipelines/next_collab_path_builder.py"),
    Path("modules/mission_planning/replanning/triggers/next_collab/pipeline.py"),
    Path("modules/mission_planning/runtime/next_collab_line_runner.py"),
    Path("modules/mission_planning/runtime/aircraft_parallel_0303.py"),
    Path("modules/mission_planning/replanning/triggers/prior/pipeline.py"),
    Path("modules/mission_planning/replanning/triggers/imaging_schedule/pipeline.py"),
    Path("modules/mission_planning/replanning/triggers/path_deviation/pipeline.py"),
    Path("modules/mission_planning/replanning/triggers/attack/pipeline.py"),
    Path("modules/mission_planning/replanning/triggers/post_attack/pipeline.py"),
)

PLANNER_RUNTIME_RELOAD_ORDER = (
    "modules.mission_planning.MissionPlanner.AnS.coord_transform",
    "modules.mission_planning.MissionPlanner.AnS.task_patterns_ver2",
    "modules.mission_planning.MissionPlanner.AnS.mission_effectiveness_ver2",
    "modules.mission_planning.MissionPlanner.AnS.env_patternselection",
    "modules.mission_planning.MissionPlanner.AnS.mission_pipeline",
    "modules.mission_planning.MissionPlanner.AnS",
    "modules.mission_planning.pipelines.next_collab_path_builder",
    "modules.mission_planning.runtime.next_collab_line_runner",
    "modules.mission_planning.replanning.triggers.next_collab.pipeline",
    "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first",
    "modules.mission_planning.replanning.triggers.recon_specialized.pipeline",
    "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan",
    "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
    "modules.mission_planning.replanning.triggers.remaining_hybrid.general",
    "modules.mission_planning.runtime.aircraft_parallel_0303",
    "modules.mission_planning.replanning.triggers.prior.pipeline",
    "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
    "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
    "modules.mission_planning.replanning.triggers.attack.pipeline",
    "modules.mission_planning.replanning.triggers.post_attack.pipeline",
)

PIPELINE_RELOAD_BINDINGS = {
    "modules.mission_planning.MissionPlanner.AnS": (
        "run_divide_and_pattern",
        "run_pulp_scheduling",
        "build_mission_plan_0301",
        "get_last_divide_and_pattern_metrics",
    ),
    "modules.mission_planning.replanning.triggers.prior.pipeline": (
        "run_prior_mission_pipeline",
        "warm_prior_mission_pipeline",
        "run_prior_post_rejoin_pipeline",
        "warm_prior_post_rejoin_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline": (
        "run_imaging_schedule_replan_pipeline",
        "warm_imaging_schedule_replan_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.path_deviation.pipeline": (
        "run_path_deviation_replan_pipeline",
        "warm_path_deviation_replan_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.next_collab.pipeline": (
        "run_next_collab_replan_pipeline",
        "warm_next_collab_replan_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.attack.pipeline": (
        "run_attack_exclusion_pipeline",
        "run_attack_plan_pipeline",
        "warm_attack_plan_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.post_attack.pipeline": (
        "run_post_attack_rejoin_pipeline",
        "warm_post_attack_rejoin_pipeline",
    ),
}


def current_remaining_hybrid_global_lock_enabled(
    environ: Mapping[str, str] | None = None,
) -> bool:
    env = os.environ if environ is None else environ
    raw = env.get("REPLAN_CURRENT_REMAINING_HYBRID_GLOBAL_LOCK", "0")
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def file_signature(path: Path) -> tuple[int, int] | None:
    try:
        stat = path.stat()
    except Exception:
        return None
    return int(stat.st_mtime_ns), int(stat.st_size)


def planner_runtime_source_signature(
    project_root: Path,
    watch_paths: tuple[Path, ...] = PLANNER_RUNTIME_WATCH_RELATIVE_PATHS,
) -> tuple[tuple[str, tuple[int, int] | None], ...]:
    root = Path(project_root)
    entries: list[tuple[str, tuple[int, int] | None]] = []
    for rel_path in watch_paths:
        path = root / rel_path
        rel_key = str(rel_path).replace("\\", "/")
        if not path.exists():
            entries.append((rel_key, None))
            continue
        entries.append((rel_key, file_signature(path)))
    return tuple(entries)


def reload_planning_module(module_name: str):
    try:
        module = importlib.import_module(module_name)
    except Exception:
        return None
    try:
        return importlib.reload(module)
    except Exception:
        return module


def refresh_live_planning_helpers(namespace: MutableMapping[str, Any]) -> None:
    importlib.invalidate_caches()
    modules: dict[str, Any] = {}
    for module_name in PLANNER_RUNTIME_RELOAD_ORDER:
        module = reload_planning_module(module_name)
        if module is not None:
            modules[module_name] = module

    current_remaining = modules.get("modules.mission_planning.replanning.triggers.remaining_hybrid.current")
    if current_remaining is not None:
        namespace["CurrentRemainingHybridRequest"] = getattr(
            current_remaining,
            "CurrentRemainingHybridRequest",
            namespace.get("CurrentRemainingHybridRequest"),
        )
        namespace["build_current_remaining_hybrid"] = getattr(
            current_remaining,
            "build_current_remaining_hybrid",
            namespace.get("build_current_remaining_hybrid"),
        )
        namespace["merge_current_remaining_hybrid"] = getattr(
            current_remaining,
            "merge_current_remaining_hybrid",
            namespace.get("merge_current_remaining_hybrid"),
        )
        namespace["validate_current_remaining_hybrid_request"] = getattr(
            current_remaining,
            "validate_current_remaining_hybrid_request",
            namespace.get("validate_current_remaining_hybrid_request"),
        )
        namespace["validate_current_remaining_hybrid_paths"] = getattr(
            current_remaining,
            "validate_current_remaining_hybrid_paths",
            namespace.get("validate_current_remaining_hybrid_paths"),
        )
        namespace["filter_generic_flightpath_missions_for_hybrid"] = getattr(
            current_remaining,
            "filter_generic_flightpath_missions_for_hybrid",
            namespace.get("filter_generic_flightpath_missions_for_hybrid"),
        )

    aircraft_parallel_0303 = modules.get("modules.mission_planning.runtime.aircraft_parallel_0303")
    if aircraft_parallel_0303 is not None:
        namespace["build_0303_flight_plans_aircraft_parallel"] = getattr(
            aircraft_parallel_0303,
            "build_0303_flight_plans_aircraft_parallel",
            namespace.get("build_0303_flight_plans_aircraft_parallel"),
        )

    for module_name, attr_names in PIPELINE_RELOAD_BINDINGS.items():
        module = modules.get(module_name)
        if module is None:
            continue
        for attr_name in attr_names:
            namespace[attr_name] = getattr(module, attr_name, namespace.get(attr_name))


def ensure_mission_planner_import_paths(project_root: Path) -> None:
    root = Path(project_root)
    desired_paths = tuple(root / rel_path for rel_path in MISSION_PLANNER_IMPORT_RELATIVE_PATHS)
    for path in reversed(desired_paths):
        path_str = str(path)
        if not path.exists():
            continue
        while path_str in sys.path:
            sys.path.remove(path_str)
        sys.path.insert(0, path_str)
