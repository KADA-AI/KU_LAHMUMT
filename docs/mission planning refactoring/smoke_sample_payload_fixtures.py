from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures" / "payloads"


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


def load_fixture(name: str) -> dict:
    path = FIXTURE_DIR / name
    if not path.exists():
        raise RuntimeError(f"payload fixture missing: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"payload fixture is not an object: {path}")
    return payload


def check_0201_0203() -> None:
    from modules.mission_planning.app.message_handlers.input_packages import (
        extract_payload_source,
        payload_has_core_data,
        prepare_cached_payload_for_file,
    )

    p0201 = load_fixture("sample_0201.json")
    if not payload_has_core_data("0201", p0201):
        raise RuntimeError("0201 fixture has no core input data")
    prepared = prepare_cached_payload_for_file("0201", p0201.get("inputMissionPackageID"), p0201)
    if prepared is None:
        raise RuntimeError("0201 fixture was not accepted for cached payload preparation")
    directory, package_id, cached_payload = prepared
    if (directory, package_id) != ("InputMissionPlan", 1):
        raise RuntimeError(f"0201 fixture cache target changed: {(directory, package_id)!r}")
    if not cached_payload.get("inputMissionList") or not cached_payload.get("availableAircraftList"):
        raise RuntimeError("0201 fixture cached payload lost core lists")
    if extract_payload_source(p0201) != "DS_SAMPLE_0201":
        raise RuntimeError("0201 fixture Source extraction changed")

    p0203 = load_fixture("sample_0203.json")
    if not payload_has_core_data("0203", p0203):
        raise RuntimeError("0203 fixture has no core reference data")
    prepared = prepare_cached_payload_for_file("0203", p0203.get("missionReferencePackageID"), p0203)
    if prepared is None:
        raise RuntimeError("0203 fixture was not accepted for cached payload preparation")
    directory, package_id, cached_payload = prepared
    if (directory, package_id) != ("MissionReferenceInfo", 1):
        raise RuntimeError(f"0203 fixture cache target changed: {(directory, package_id)!r}")
    if not cached_payload.get("takeOverInfoList") or not cached_payload.get("flightAreaList"):
        raise RuntimeError("0203 fixture cached payload lost core lists")
    if extract_payload_source(p0203) != "DS_SAMPLE_0203":
        raise RuntimeError("0203 fixture source extraction changed")


def check_0902() -> None:
    from modules.mission_planning.app.message_handlers.replan_requests import (
        extract_replan_request_selection,
        parse_replan_payload,
        replan_delay_policy,
    )

    p0902 = load_fixture("sample_0902.json")
    parsed = parse_replan_payload(("prefix " + json.dumps(p0902) + " suffix").encode("utf-8"))
    if parsed != p0902:
        raise RuntimeError("0902 fixture raw parse changed")

    selection = extract_replan_request_selection(p0902)
    if selection.plan_ids != [700000001]:
        raise RuntimeError(f"0902 fixture selected plan IDs changed: {selection.plan_ids!r}")
    if selection.option_names != ["baseline-option"]:
        raise RuntimeError(f"0902 fixture option names changed: {selection.option_names!r}")
    if selection.mission_ids != [1, 2]:
        raise RuntimeError(f"0902 fixture mission IDs changed: {selection.mission_ids!r}")
    if selection.detail_trigger_type != "communicationLossRTB":
        raise RuntimeError(f"0902 fixture detail trigger type changed: {selection.detail_trigger_type!r}")

    policy = replan_delay_policy(p0902)
    if (policy.runtime_setting_key, policy.default_delay_ms) != (None, 55_000):
        raise RuntimeError(f"0902 fixture delay policy changed: {policy!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke sample 0201/0203/0902 payload fixtures.")
    parser.parse_args()

    try:
        configure_import_paths()
        check_0201_0203()
        check_0902()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("sample 0201/0203/0902 payload fixture smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
