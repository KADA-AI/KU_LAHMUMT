from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CURRENT_SCENARIO_INFO = PROJECT_ROOT / "settings" / "current_scenario.json"
UINT32_MAX = 2**32 - 1


IMP_ID_KEYS = (
    "individualMissionPackageID",
    "individualMissionPlanPackageID",
    "individualMissionPackageId",
)
WAYPOINT_KEYS = ("waypointList", "uavWaypointList", "lahWaypointList")
AIRCRAFT_IDS = {1, 2, 3, 4, 5, 6}


@dataclass
class PlanSummary:
    mission_plans: int = 0
    aircraft: int = 0
    individual_mission_packages: int = 0
    individual_missions: int = 0
    flight_paths: int = 0
    waypoints: int = 0

    def add(self, other: "PlanSummary") -> None:
        self.mission_plans += other.mission_plans
        self.aircraft += other.aircraft
        self.individual_mission_packages += other.individual_mission_packages
        self.individual_missions += other.individual_missions
        self.flight_paths += other.flight_paths
        self.waypoints += other.waypoints


def load_json_object(path: Path, errors: list[str]) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        errors.append(f"{path}: JSON load failed: {exc}")
        return None
    if not isinstance(payload, dict):
        errors.append(f"{path}: top-level payload is not an object")
        return None
    return payload


def to_int(value: Any) -> int | None:
    try:
        if value is None or value == "":
            return None
        return int(value)
    except Exception:
        return None


def validate_uint32(
    value: Any,
    field: str,
    errors: list[str],
    *,
    context: str,
    positive: bool = False,
    required: bool = True,
) -> int | None:
    value_int = to_int(value)
    if value_int is None:
        if required:
            errors.append(f"{context}: {field} is not an int: {value!r}")
        return None
    if value_int < 0 or value_int > UINT32_MAX:
        errors.append(f"{context}: {field} out of uint32 range: {value_int}")
    if positive and value_int <= 0:
        errors.append(f"{context}: {field} is not positive: {value_int}")
    return int(value_int)


def is_aircraft_path_id(aircraft_id: int, path_id: int) -> bool:
    start = int(aircraft_id) * 100_000_000
    end = start + 99_999_999
    return start <= int(path_id) <= end


def payload_id(payload: dict[str, Any], *keys: str) -> int | None:
    for key in keys:
        value = to_int(payload.get(key))
        if value is not None:
            return int(value)
    return None


def default_db_root(errors: list[str]) -> Path:
    info = load_json_object(CURRENT_SCENARIO_INFO, errors)
    if info is None:
        return PROJECT_ROOT / "Logs"
    db_root = info.get("db_root")
    if not db_root:
        errors.append(f"{CURRENT_SCENARIO_INFO}: db_root is missing")
        return PROJECT_ROOT / "Logs"
    return Path(str(db_root)).resolve()


def validate_waypoints(payload: dict[str, Any], path_context: str, errors: list[str]) -> int:
    for key in WAYPOINT_KEYS:
        rows = payload.get(key)
        if rows is None:
            continue
        if not isinstance(rows, list):
            errors.append(f"{path_context}: {key} is not a list")
            return 0
        waypoint_ids: set[int] = set()
        next_ids: list[tuple[int, int]] = []
        valid_rows = 0
        for waypoint_index, row in enumerate(rows):
            waypoint_context = f"{path_context}: {key}[{waypoint_index}]"
            if not isinstance(row, dict):
                errors.append(f"{waypoint_context}: waypoint entry is not an object")
                continue
            valid_rows += 1
            waypoint_id = validate_uint32(
                row.get("waypointID"),
                "waypointID",
                errors,
                context=waypoint_context,
                positive=True,
            )
            next_id = validate_uint32(
                row.get("nextWaypointID"),
                "nextWaypointID",
                errors,
                context=waypoint_context,
                required=False,
            )
            if waypoint_id is not None and waypoint_id > 0:
                if int(waypoint_id) in waypoint_ids:
                    errors.append(f"{waypoint_context}: duplicate waypointID={waypoint_id}")
                waypoint_ids.add(int(waypoint_id))
            if next_id is not None and int(next_id) != 0:
                next_ids.append((waypoint_index, int(next_id)))
        for waypoint_index, next_id in next_ids:
            if next_id not in waypoint_ids:
                errors.append(f"{path_context}: {key}[{waypoint_index}] invalid nextWaypointID={next_id}")
        return valid_rows
    return 0


def mission_plan_files(db_root: Path, mission_plan_ids: list[int] | None, errors: list[str]) -> list[Path]:
    plan_dir = db_root / "MissionPlan"
    if not plan_dir.exists():
        errors.append(f"{plan_dir}: MissionPlan directory missing")
        return []
    if mission_plan_ids:
        result = []
        for plan_id in mission_plan_ids:
            path = plan_dir / f"{int(plan_id)}.json"
            if not path.exists():
                errors.append(f"{path}: selected MissionPlan missing")
                continue
            result.append(path)
        return result
    return sorted(plan_dir.glob("*.json"), key=lambda path: (to_int(path.stem) is None, to_int(path.stem) or path.stem))


def validate_plan(
    *,
    db_root: Path,
    plan_path: Path,
    errors: list[str],
    require_waypoints: bool,
    check_path_band: bool,
) -> PlanSummary:
    summary = PlanSummary(mission_plans=1)
    mission_plan = load_json_object(plan_path, errors)
    if mission_plan is None:
        return summary

    scope = f"MissionPlan {plan_path.name}"
    file_plan_id = validate_uint32(plan_path.stem, "filename stem", errors, context=scope, positive=True)
    body_plan_id = validate_uint32(
        mission_plan.get("missionPlanID"),
        "missionPlanID",
        errors,
        context=scope,
        positive=True,
    )
    if body_plan_id is not None and file_plan_id is not None and int(body_plan_id) != int(file_plan_id):
        errors.append(f"{scope}: missionPlanID mismatch file={file_plan_id} body={body_plan_id}")

    aircraft_list = mission_plan.get("aircraftList")
    if not isinstance(aircraft_list, list):
        errors.append(f"{scope}: aircraftList is not a list")
        return summary
    if not aircraft_list:
        errors.append(f"{scope}: aircraftList is empty")
        return summary

    seen_imp_ids: set[int] = set()
    seen_mission_ids: set[int] = set()
    seen_path_ids: set[int] = set()
    imp_dir = db_root / "IndividualMissionPlan"
    fp_dir = db_root / "FlightPath"

    for aircraft_index, aircraft in enumerate(aircraft_list):
        aircraft_scope = f"{scope}: aircraftIndex={aircraft_index}"
        if not isinstance(aircraft, dict):
            errors.append(f"{aircraft_scope}: aircraft entry is not an object")
            continue
        aircraft_id = validate_uint32(
            aircraft.get("aircraftID"),
            "aircraftID",
            errors,
            context=aircraft_scope,
            positive=True,
        )
        if aircraft_id is None:
            continue
        if int(aircraft_id) not in AIRCRAFT_IDS:
            errors.append(f"{aircraft_scope}: aircraftID out of supported range 1..6: {aircraft_id}")
        imp_id = payload_id(aircraft, *IMP_ID_KEYS)
        if imp_id is None or int(imp_id) <= 0:
            errors.append(f"{aircraft_scope}: missing individualMissionPackageID")
            continue
        validate_uint32(imp_id, "individualMissionPackageID", errors, context=aircraft_scope, positive=True)

        summary.aircraft += 1
        summary.individual_mission_packages += 1
        if int(imp_id) in seen_imp_ids:
            errors.append(f"{aircraft_scope}: duplicate IMP reference imp={imp_id}")
        seen_imp_ids.add(int(imp_id))

        imp_path = imp_dir / f"{int(imp_id)}.json"
        if not imp_path.exists():
            errors.append(f"{aircraft_scope}: IMP file missing imp={imp_id}")
            continue
        imp_payload = load_json_object(imp_path, errors)
        if imp_payload is None:
            continue

        body_imp_id = payload_id(imp_payload, *IMP_ID_KEYS)
        if body_imp_id is None:
            errors.append(f"{aircraft_scope}: IMP {imp_path.name} body package ID is missing")
        elif int(body_imp_id) != int(imp_id):
            errors.append(f"{aircraft_scope}: IMP ID mismatch file/ref={imp_id} body={body_imp_id}")
        imp_aircraft_id = validate_uint32(
            imp_payload.get("aircraftID"),
            "IMP.aircraftID",
            errors,
            context=aircraft_scope,
            positive=True,
        )
        if imp_aircraft_id is None:
            pass
        elif int(imp_aircraft_id) != int(aircraft_id):
            errors.append(
                f"{aircraft_scope}: IMP aircraft mismatch planAircraft={aircraft_id} impAircraft={imp_aircraft_id}"
            )

        mission_list = imp_payload.get("individualMissionList")
        if not isinstance(mission_list, list):
            errors.append(f"{aircraft_scope}: IMP {imp_path.name} individualMissionList is not a list")
            continue
        if not mission_list:
            errors.append(f"{aircraft_scope}: IMP {imp_path.name} individualMissionList is empty")
            continue

        for mission_index, mission in enumerate(mission_list):
            mission_scope = f"{aircraft_scope}: imp={imp_id} missionIndex={mission_index}"
            if not isinstance(mission, dict):
                errors.append(f"{mission_scope}: mission entry is not an object")
                continue
            mission_id = validate_uint32(
                mission.get("individualMissionID"),
                "individualMissionID",
                errors,
                context=mission_scope,
                positive=True,
            )
            path_id = validate_uint32(mission.get("pathID"), "pathID", errors, context=mission_scope, positive=True)
            if mission_id is None:
                continue
            if path_id is None:
                continue
            if int(mission_id) in seen_mission_ids:
                errors.append(f"{mission_scope}: duplicate individualMissionID={mission_id}")
            seen_mission_ids.add(int(mission_id))
            if int(path_id) in seen_path_ids:
                errors.append(f"{mission_scope}: duplicate pathID={path_id}")
            seen_path_ids.add(int(path_id))
            if check_path_band and not is_aircraft_path_id(int(aircraft_id), int(path_id)):
                errors.append(f"{mission_scope}: pathID band mismatch aircraftID={aircraft_id} pathID={path_id}")

            fp_path = fp_dir / f"{int(path_id)}.json"
            if not fp_path.exists():
                errors.append(f"{mission_scope}: FlightPath file missing pathID={path_id}")
                continue
            fp_payload = load_json_object(fp_path, errors)
            if fp_payload is None:
                continue

            fp_path_id = validate_uint32(
                fp_payload.get("pathID"),
                "FlightPath.pathID",
                errors,
                context=mission_scope,
                positive=True,
            )
            if fp_path_id is None:
                pass
            elif int(fp_path_id) != int(path_id):
                errors.append(f"{mission_scope}: FlightPath pathID mismatch ref={path_id} body={fp_path_id}")
            fp_aircraft_id = validate_uint32(
                fp_payload.get("aircraftID"),
                "FlightPath.aircraftID",
                errors,
                context=mission_scope,
                positive=True,
            )
            if fp_aircraft_id is None:
                pass
            elif int(fp_aircraft_id) != int(aircraft_id):
                errors.append(
                    f"{mission_scope}: FlightPath aircraft mismatch ref={aircraft_id} body={fp_aircraft_id}"
                )
            fp_mission_id = validate_uint32(
                fp_payload.get("individualMissionID"),
                "FlightPath.individualMissionID",
                errors,
                context=mission_scope,
                positive=True,
                required="individualMissionID" in fp_payload,
            )
            if fp_mission_id is not None and int(fp_mission_id) > 0 and int(fp_mission_id) != int(mission_id):
                errors.append(
                    f"{mission_scope}: FlightPath mission mismatch ref={mission_id} body={fp_mission_id}"
                )

            waypoints = validate_waypoints(fp_payload, f"{mission_scope}: FlightPath {fp_path.name}", errors)
            if require_waypoints and waypoints <= 0:
                errors.append(f"{mission_scope}: FlightPath {fp_path.name} has no waypoint rows")
            summary.individual_missions += 1
            summary.flight_paths += 1
            summary.waypoints += waypoints

    return summary


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate generated 0301/0302/0303/0304 DB artifact links without rewriting artifacts.",
    )
    parser.add_argument(
        "--db-root",
        type=Path,
        default=None,
        help="DB root to inspect. Defaults to current_scenario.json db_root, read-only.",
    )
    parser.add_argument(
        "--mission-plan-id",
        "--plan-id",
        action="append",
        dest="mission_plan_ids",
        type=int,
        help="MissionPlan ID to validate. Repeat to validate multiple plans; defaults to all MissionPlan JSON files.",
    )
    parser.add_argument(
        "--require-waypoints",
        action="store_true",
        help="Fail when a linked FlightPath has no waypoint rows.",
    )
    parser.add_argument(
        "--skip-path-band",
        action="store_true",
        help="Skip the aircraftID-derived pathID band check.",
    )
    parser.add_argument("--max-errors", type=int, default=20, help="Maximum number of errors to print.")
    args = parser.parse_args()

    errors: list[str] = []
    db_root = Path(args.db_root).resolve() if args.db_root else default_db_root(errors)
    if not db_root.exists():
        errors.append(f"{db_root}: DB root missing")
    total = PlanSummary()
    for plan_path in mission_plan_files(db_root, args.mission_plan_ids, errors):
        total.add(
            validate_plan(
                db_root=db_root,
                plan_path=plan_path,
                errors=errors,
                require_waypoints=bool(args.require_waypoints),
                check_path_band=not bool(args.skip_path_band),
            )
        )

    if errors:
        print(f"generated artifact link smoke failed: db_root={db_root} errors={len(errors)}", file=sys.stderr)
        for error in errors[: max(1, int(args.max_errors))]:
            print(f"- {error}", file=sys.stderr)
        if len(errors) > int(args.max_errors):
            print(f"- ... {len(errors) - int(args.max_errors)} more", file=sys.stderr)
        return 1

    print(
        "generated artifact link smoke ok: "
        f"db_root={db_root} "
        f"missionPlans={total.mission_plans} "
        f"aircraft={total.aircraft} "
        f"individualMissionPackages={total.individual_mission_packages} "
        f"individualMissions={total.individual_missions} "
        f"flightPaths={total.flight_paths} "
        f"waypoints={total.waypoints}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
