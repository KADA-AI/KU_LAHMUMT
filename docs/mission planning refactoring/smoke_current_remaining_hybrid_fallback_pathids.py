from __future__ import annotations

import argparse
import json
import os
import sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FIXTURE_PATH = (
    PROJECT_ROOT
    / "docs"
    / "mission planning refactoring"
    / "fixtures"
    / "current_remaining_hybrid"
    / "failure_fallback_pathid_mapping.json"
)


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
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


def fail(message: str) -> None:
    raise AssertionError(message)


def expect_equal(label: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{label} changed: expected {expected!r}, got {actual!r}")


def expect_true(label: str, condition: bool) -> None:
    if not condition:
        fail(f"{label} expected true")


def load_fixture() -> dict[str, Any]:
    return json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))


def _int_key_dict(raw: dict[str, Any]) -> dict[int, Any]:
    return {int(key): value for key, value in dict(raw or {}).items()}


def _path_ids(payloads: list[dict[str, Any]]) -> list[int]:
    return [int(item["pathID"]) for item in payloads if isinstance(item, dict)]


def make_request(current: Any, fixture: dict[str, Any]) -> Any:
    raw = fixture["request"]
    return current.CurrentRemainingHybridRequest(
        source_plan_id=int(raw["source_plan_id"]),
        current_input_id=int(raw["current_input_id"]),
        current_input_mission=deepcopy(raw["current_input_mission"]),
        next_input_mission=deepcopy(raw["next_input_mission"]),
        entry_coord_map={
            int(aid): dict(coord)
            for aid, coord in dict(raw["entry_coord_map"]).items()
        },
        heading_map={
            int(aid): float(value)
            for aid, value in dict(raw["heading_map"]).items()
        },
        representative_entry=deepcopy(raw["representative_entry"]),
        turn_radius_scale=float(raw["turn_radius_scale"]),
    )


def make_prepared(fixture: dict[str, Any]) -> Any:
    raw = fixture["hybrid_prepared"]
    return SimpleNamespace(
        replacement_by_aircraft={
            int(aid): deepcopy(missions)
            for aid, missions in dict(raw["replacement_by_aircraft"]).items()
        },
        generated_fp_by_path={
            int(path_id): deepcopy(payload)
            for path_id, payload in dict(raw["generated_fp_by_path"]).items()
        },
        generated_path_ids={int(path_id) for path_id in raw["generated_path_ids"]},
        planner_workflow=str(raw["planner_workflow"]),
        planner_result_text=str(raw["planner_result_text"]),
        timing_ms=dict(raw["timing_ms"]),
        id_reservation=dict(raw["id_reservation"]),
        uav_work_summary=_int_key_dict(raw["uav_work_summary"]),
        mission_mode=str(raw["mission_mode"]),
    )


def check_materialize_and_path_aircraft_mapping(current: Any, fixture: dict[str, Any]) -> Any:
    request = make_request(current, fixture)
    prepared = make_prepared(fixture)
    hybrid = current.materialize_current_remaining_hybrid_result(request, prepared)
    expect_true("hybrid materialize result", hybrid is not None)

    expect_equal("hybrid current input ID", hybrid.current_input_id, 7001)
    expect_equal("hybrid aircraft IDs", hybrid.aircraft_ids, {5})
    expect_equal("hybrid generated path IDs", hybrid.generated_path_ids, {500000001})
    expect_equal("hybrid 0303 count", len(hybrid.flight_plans_0303), 1)
    expect_equal("hybrid 0304 count", len(hybrid.flight_plans_0304), 0)
    expect_equal("hybrid 0303 pathID", hybrid.flight_plans_0303[0]["pathID"], 500000001)
    expect_equal(
        "hybrid 0303 aircraftID inferred from mission pathID mapping",
        hybrid.flight_plans_0303[0]["aircraftID"],
        5,
    )
    expect_equal(
        "geometry path-aircraft map",
        hybrid.geometry.path_aircraft_by_id,
        {500000001: 5},
    )
    expect_equal("runtime id reservation copied", hybrid.runtime_result.id_reservation, {"path_ids": [500000001]})
    expect_equal("runtime UAV summary copied", hybrid.runtime_result.uav_work_summary, {5: 1})
    return hybrid


def check_materialize_empty_results_return_none(current: Any, fixture: dict[str, Any]) -> None:
    request = make_request(current, fixture)
    prepared = make_prepared(fixture)

    empty_missions = SimpleNamespace(
        **{
            **prepared.__dict__,
            "replacement_by_aircraft": {},
        }
    )
    expect_equal(
        "materialize empty missions fallback",
        current.materialize_current_remaining_hybrid_result(request, empty_missions),
        None,
    )

    empty_flight_paths = SimpleNamespace(
        **{
            **prepared.__dict__,
            "generated_fp_by_path": {},
            "generated_path_ids": set(),
        }
    )
    expect_equal(
        "materialize empty flight paths fallback",
        current.materialize_current_remaining_hybrid_result(request, empty_flight_paths),
        None,
    )


def check_failure_fallback_call_site_contract() -> None:
    source = (PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py").read_text(
        encoding="utf-8",
        errors="ignore",
    )
    markers = (
        "current remaining collaborative hybrid failed before generic skip",
        "current_remaining_hybrid_result = None",
        "if current_remaining_hybrid_result is not None:",
        "skip_result = filter_generic_flightpath_missions_for_hybrid(",
        "current remaining collaborative hybrid unavailable; keep generic output",
        "return missions, flight_plans_0303, flight_plans_0304, set()",
        "current remaining hybrid temporary pathID remapped",
        "pathID mapping delayed until current remaining hybrid merge",
        "pathID mapping done after current remaining hybrid merge",
        "current_remaining_parallel_store",
        "remap_candidates.setdefault(old_pid, set()).add(new_pid)",
        "desired = pid_map.get((aid, int(mid)))",
        "if desired is None and old_path_id is not None:",
        '"flightpath_missing_ids"',
    )
    missing = [marker for marker in markers if marker not in source]
    if missing:
        fail(f"mission_planning_gui.py missing current remaining hybrid fallback markers: {missing!r}")


def check_success_skip_and_merge_pathid_remap(current: Any, fixture: dict[str, Any], hybrid: Any) -> None:
    request = make_request(current, fixture)
    generic = fixture["generic"]
    expected = fixture["expected"]

    skip_result = current.filter_generic_flightpath_missions_for_hybrid(
        deepcopy(generic["missions"]),
        request=request,
        hybrid=hybrid,
    )
    expect_equal("success skip count", skip_result.skipped_count, 1)
    expect_equal("success skipped aircraft IDs", skip_result.skipped_aircraft_ids, {5})
    expect_equal(
        "success skipped path IDs",
        sorted(skip_result.skipped_path_ids),
        expected["skipped_path_ids_after_success"],
    )
    expect_equal(
        "success keeps unrelated generic mission",
        _path_ids(skip_result.missions),
        [500000001, 400000001, 100000001],
    )

    merged = current.merge_current_remaining_hybrid(
        missions=deepcopy(generic["missions"]),
        flight_plans_0303=deepcopy(generic["flight_plans_0303"]),
        flight_plans_0304=deepcopy(generic["flight_plans_0304"]),
        hybrid=hybrid,
    )
    expect_equal("removed current generic path IDs", merged["removed_path_ids"], expected["removed_path_ids_after_merge"])
    expect_equal(
        "temporary pathID remap",
        merged["temporaryPathIdRemap"],
        {int(k): int(v) for k, v in expected["temporary_path_id_remap"].items()},
    )
    expect_equal("merged generated path IDs", sorted(merged["generated_path_ids"]), expected["merged_generated_path_ids"])
    expect_equal("merged 0303 path IDs", _path_ids(merged["flight_plans_0303"]), expected["merged_0303_path_ids"])
    expect_equal("merged 0304 path IDs", _path_ids(merged["flight_plans_0304"]), expected["merged_0304_path_ids"])
    expect_equal("path validation valid", merged["pathValidation"]["valid"], True)
    expect_equal("path validation overlap", merged["pathValidation"]["overlapPathIDs"], [])

    merged_mission_path_ids = _path_ids(merged["missions"])
    expect_true("removed generic mission absent", 500000010 not in merged_mission_path_ids)
    expect_true("hybrid mission pathID rewritten", 500000002 in merged_mission_path_ids)
    expect_true("unrelated collision baseline preserved", 500000001 in merged_mission_path_ids)


def check_path_validation_overlap(current: Any) -> None:
    overlap = current.validate_current_remaining_hybrid_paths(
        generic_path_ids=[{"pathID": 500000001}, {"pathID": "400000001"}],
        hybrid_path_ids={500000001, "600000001"},
    )
    expect_equal("overlap validation valid", overlap["valid"], False)
    expect_equal("overlap validation generic IDs", overlap["genericPathIDs"], [400000001, 500000001])
    expect_equal("overlap validation hybrid IDs", overlap["hybridPathIDs"], [500000001, 600000001])
    expect_equal("overlap validation overlap IDs", overlap["overlapPathIDs"], [500000001])


def _patch_attr(obj: Any, name: str, value: Any) -> Callable[[], None]:
    original = getattr(obj, name)
    setattr(obj, name, value)

    def restore() -> None:
        setattr(obj, name, original)

    return restore


def check_current_replan_failure_fallbacks(current_replan: Any) -> None:
    logs: list[str] = []
    result = current_replan.prepare_current_remaining_hybrid_replacements(
        source_plan_id=1,
        current_input_mission={"inputMissionID": "bad"},
        log=logs.append,
    )
    expect_equal("invalid input result", result, None)
    expect_true("invalid input log", any("current input mission ID is invalid" in msg for msg in logs))

    restores: list[Callable[[], None]] = []
    try:
        logs.clear()
        restores.append(_patch_attr(current_replan, "_load_vehicle_status_available", lambda: set()))
        result = current_replan.prepare_current_remaining_hybrid_replacements(
            source_plan_id=1,
            current_input_mission={"inputMissionID": 7001},
            log=logs.append,
        )
        expect_equal("no available UAV result", result, None)
        expect_true("no available UAV log", any("no available UAVs" in msg for msg in logs))
    finally:
        while restores:
            restores.pop()()

    try:
        logs.clear()
        restores.append(_patch_attr(current_replan, "_load_vehicle_status_available", lambda: {4}))
        restores.append(
            _patch_attr(
                current_replan.agent_status_snapshot,
                "load_agent_status_snapshot",
                lambda: {"agent_states": []},
            )
        )
        result = current_replan.prepare_current_remaining_hybrid_replacements(
            source_plan_id=1,
            current_input_mission={"inputMissionID": 7001},
            log=logs.append,
        )
        expect_equal("no live coordinate result", result, None)
        expect_true("no live coordinate log", any("no live UAV coordinates" in msg for msg in logs))
    finally:
        while restores:
            restores.pop()()

    try:
        logs.clear()
        restores.append(_patch_attr(current_replan, "_load_vehicle_status_available", lambda: {5}))
        restores.append(
            _patch_attr(
                current_replan.agent_status_snapshot,
                "load_agent_status_snapshot",
                lambda: {
                    "agent_states": [
                        {
                            "aircraftID": 5,
                            "isUnmanned": True,
                            "coordinate": {
                                "latitude": 36.125,
                                "longitude": 127.125,
                                "altitude": 900,
                            },
                            "velocity": {
                                "heading": 92.5,
                            },
                        }
                    ]
                },
            )
        )
        restores.append(
            _patch_attr(
                current_replan,
                "prepare_next_collab_input_replacements",
                lambda **_kwargs: None,
            )
        )
        result = current_replan.prepare_current_remaining_hybrid_replacements(
            source_plan_id=1,
            current_input_mission={"inputMissionID": 7001},
            log=logs.append,
        )
        expect_equal("helper none result", result, None)
    finally:
        while restores:
            restores.pop()()

    try:
        logs.clear()
        restores.append(_patch_attr(current_replan, "_load_vehicle_status_available", lambda: {5}))
        restores.append(
            _patch_attr(
                current_replan.agent_status_snapshot,
                "load_agent_status_snapshot",
                lambda: {
                    "agent_states": [
                        {
                            "aircraftID": 5,
                            "isUnmanned": True,
                            "coordinate": {
                                "latitude": 36.125,
                                "longitude": 127.125,
                                "altitude": 900,
                            },
                            "velocity": {
                                "heading": 92.5,
                            },
                        }
                    ]
                },
            )
        )
        restores.append(
            _patch_attr(
                current_replan,
                "prepare_next_collab_input_replacements",
                lambda **_kwargs: SimpleNamespace(replacement_by_aircraft={}),
            )
        )
        result = current_replan.prepare_current_remaining_hybrid_replacements(
            source_plan_id=1,
            current_input_mission={"inputMissionID": 7001},
            log=logs.append,
        )
        expect_equal("empty replacement result", result, None)
        expect_true("empty replacement log", any("returned no target aircraft replacements" in msg for msg in logs))
    finally:
        while restores:
            restores.pop()()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Smoke current-remaining hybrid failure fallback and pathID mapping."
    )
    parser.parse_args()
    configure_import_paths()

    try:
        from modules.mission_planning.replanning.triggers.remaining_hybrid import current
        from modules.mission_planning.replanning.triggers.remaining_hybrid import current_replan

        fixture = load_fixture()
        hybrid = check_materialize_and_path_aircraft_mapping(current, fixture)
        check_materialize_empty_results_return_none(current, fixture)
        check_failure_fallback_call_site_contract()
        check_success_skip_and_merge_pathid_remap(current, fixture, hybrid)
        check_path_validation_overlap(current)
        check_current_replan_failure_fallbacks(current_replan)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("current remaining hybrid fallback/pathID smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
