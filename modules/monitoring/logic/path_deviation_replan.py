# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from modules.common import path_deviation_replan_store
from modules.monitoring.logic.init_replan import allocate_mission_plan_ids, collect_input_mission_ids
from modules.monitoring.logic.mission_update import load_db_json
from modules.monitoring.logic.source_artifact_index import SourceArtifactIndex
from modules.monitoring.logic.turn_radius_monitor import (
    ALT_WAYPOINT_ID_BASE,
    AircraftTurnView,
    TRACKED_AIRCRAFT_IDS,
    is_path_deviation_turn_warning_view,
)


REPLAN_LEVEL = 3
REPLAN_REASON_TEMPLATE = "UAV{uav_index} \uacbd\ub85c\ubbf8\ucd94\uc885 \uc7ac\uacc4\ud68d"
OPTION_NAME = "\ube44\ud589/\ucd2c\uc601"
TRIGGER_TYPE = "pathDeviation"


def _coerce_int(value: object | None) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _coerce_float(value: object | None) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _extract_waypoint_id(value: object | None) -> int | None:
    if isinstance(value, dict):
        return _coerce_int(
            value.get("waypointID")
            or value.get("WaypointID")
            or value.get("waypointId")
        )
    return _coerce_int(value)


def _format_path_deviation_reason(aircraft_id: int | None) -> str:
    aid = _coerce_int(aircraft_id)
    if aid is not None and 4 <= aid <= 6:
        return REPLAN_REASON_TEMPLATE.format(uav_index=aid - 3)
    if aid is not None and aid > 0:
        return f"\ud56d\uacf5\uae30 {aid}\ubc88 \uacbd\ub85c \ubbf8\ucd94\uc885\uc73c\ub85c \uc778\ud55c \uc7ac\uacc4\ud68d"
    return "\ubbf8\uc0c1 \ubb34\uc778\uae30 \uacbd\ub85c \ubbf8\ucd94\uc885\uc73c\ub85c \uc778\ud55c \uc7ac\uacc4\ud68d"


@dataclass(frozen=True)
class SourceMissionArtifacts:
    input_mission_package_id: int | None
    input_mission_ids: list[int]
    individual_mission_package_id: int
    individual_mission_id: int
    path_id: int
    resolved_current_waypoint_id: int


@dataclass
class PathDeviationReplanState:
    last_trigger_key_by_aircraft: dict[int, tuple[int, int, int]] = field(default_factory=dict)


def _waypoint_has_filming_property(waypoint: dict[str, Any] | None) -> bool:
    if not isinstance(waypoint, dict):
        return False
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return False
    line_search = filming.get("lineSearch")
    if isinstance(line_search, dict) and len(line_search.get("coordinateList") or []) >= 2:
        return True
    if isinstance(filming.get("coordinateOrientation"), dict):
        return True
    operation_mode = _coerce_int(filming.get("operationMode"))
    return operation_mode is not None and int(operation_mode) > 0


def _mission_has_filming_geometry(mission: dict[str, Any] | None, waypoints: list[dict[str, Any]] | None) -> bool:
    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        info = {}
    mission_type = _coerce_int(info.get("individualMissionType"))
    if mission_type in {3, 6}:
        return True
    if isinstance(info.get("lineList"), list) and info.get("lineList"):
        return True
    if isinstance(info.get("areaList"), list) and info.get("areaList"):
        return True
    return any(_waypoint_has_filming_property(wp) for wp in (waypoints or []) if isinstance(wp, dict))


def _path_deviation_filming_block_reason(artifacts: SourceMissionArtifacts) -> str | None:
    imp_data = load_db_json("IndividualMissionPlan", int(artifacts.individual_mission_package_id))
    fp_data = load_db_json("FlightPath", int(artifacts.path_id))
    mission = None
    for item in imp_data.get("individualMissionList") or []:
        if not isinstance(item, dict):
            continue
        if _coerce_int(item.get("individualMissionID")) == int(artifacts.individual_mission_id):
            mission = item
            break
    waypoints = [
        item
        for item in (fp_data.get("waypointList") or fp_data.get("lahWaypointList") or [])
        if isinstance(item, dict)
    ]
    current_waypoint = next(
        (
            item
            for item in waypoints
            if _extract_waypoint_id(item.get("waypointID")) == int(artifacts.resolved_current_waypoint_id)
        ),
        None,
    )
    if _waypoint_has_filming_property(current_waypoint):
        return "current waypoint is filming/sweep"
    if _mission_has_filming_geometry(mission, waypoints):
        return "source mission is filming/sweep"
    return None


def _resolve_input_mission_ids(
    *,
    mission: dict[str, Any],
    input_mission_package_id: int | None,
    source_plan_id: int,
    aircraft_id: int,
    logger: Callable[[str], None] | None,
) -> list[int]:
    related = mission.get("relatedMission") or {}
    input_mission_ids: list[int] = []
    related_input_mission_id = _coerce_int(
        related.get("inputMissionID")
        if isinstance(related, dict)
        else None
    )
    if related_input_mission_id is not None and related_input_mission_id > 0:
        input_mission_ids = [int(related_input_mission_id)]
    if not input_mission_ids and input_mission_package_id is not None:
        input_plan = load_db_json("InputMissionPlan", input_mission_package_id)
        seen_ids: set[int] = set()
        for item in input_plan.get("inputMissionList") or []:
            if not isinstance(item, dict):
                continue
            mission_id = _coerce_int(item.get("inputMissionID"))
            if mission_id is None or mission_id <= 0 or mission_id in seen_ids:
                continue
            if bool(item.get("isDone")):
                continue
            seen_ids.add(int(mission_id))
            input_mission_ids.append(int(mission_id))
    if not input_mission_ids:
        input_mission_ids = [int(value) for value in collect_input_mission_ids() if _coerce_int(value)]
    if not input_mission_ids:
        if logger:
            logger(
                f"[0401] inputMissionID unresolved for plan={source_plan_id}, "
                f"aircraft={aircraft_id}; using sentinel [0]"
            )
        input_mission_ids = [0]
    return input_mission_ids


class PathDeviationReplanCoordinator:
    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._state = PathDeviationReplanState()

    def on_turn_monitor_views(
        self,
        views: dict[int, AircraftTurnView] | None,
        *,
        enabled: bool = True,
        system_mode: int | None,
        current_mission_plan_id: int | None,
        current_input_id: int | None = None,
        aircraft_filter: Callable[[int], bool] | None = None,
        dispatch_context: Any | None = None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        logs: list[str] = []
        if not enabled:
            return [], logs
        if system_mode not in (3, 4):
            return [], logs
        if current_mission_plan_id is None:
            return [], logs
        if not isinstance(views, dict):
            return [], logs

        payloads: list[dict[str, Any]] = []
        for aircraft_id in TRACKED_AIRCRAFT_IDS:
            aircraft_id_int = int(aircraft_id)
            view = views.get(aircraft_id_int)
            if view is None:
                self._state.last_trigger_key_by_aircraft.pop(aircraft_id_int, None)
                continue
            if _coerce_int(getattr(view, "flying", None)) == 2:
                self._state.last_trigger_key_by_aircraft.pop(aircraft_id_int, None)
                continue

            current_waypoint_id = _coerce_int(view.current_waypoint_id)
            if current_waypoint_id is None or current_waypoint_id <= 0:
                self._state.last_trigger_key_by_aircraft.pop(aircraft_id_int, None)
                continue
            if current_waypoint_id >= int(ALT_WAYPOINT_ID_BASE):
                # Path-deviation alternate WP(900000+) is already a recovery waypoint.
                # Re-triggering on that synthetic WP causes chained self-replans.
                self._state.last_trigger_key_by_aircraft.pop(aircraft_id_int, None)
                continue

            if not self._is_trigger_view(view):
                self._state.last_trigger_key_by_aircraft.pop(aircraft_id_int, None)
                continue

            if aircraft_filter is not None:
                try:
                    if not bool(aircraft_filter(aircraft_id_int)):
                        self._state.last_trigger_key_by_aircraft.pop(aircraft_id_int, None)
                        continue
                except Exception as exc:
                    logs.append(
                        f"[0401] aircraft_filter raised exception for aircraftID={aircraft_id_int}: {exc}"
                    )
                    self._state.last_trigger_key_by_aircraft.pop(aircraft_id_int, None)
                    continue

            artifacts = self._resolve_source_artifacts(
                source_plan_id=int(current_mission_plan_id),
                aircraft_id=aircraft_id_int,
                current_waypoint_id=int(current_waypoint_id),
                current_input_id=current_input_id,
            )
            if artifacts is None:
                logs.append(
                    f"[0401] path-deviation replan skipped: source artifacts unresolved "
                    f"(aircraftID={aircraft_id_int}, currentWP={current_waypoint_id}, "
                    f"currentInput={_coerce_int(current_input_id)})"
                )
                continue
            resolved_current_waypoint_id = _coerce_int(artifacts.resolved_current_waypoint_id)
            if resolved_current_waypoint_id is None or resolved_current_waypoint_id <= 0:
                resolved_current_waypoint_id = int(current_waypoint_id)

            filming_block_reason = _path_deviation_filming_block_reason(artifacts)
            if filming_block_reason:
                logs.append(
                    f"[0401] path-deviation replan skipped: filming mission protected "
                    f"(aircraftID={aircraft_id_int}, currentWP={resolved_current_waypoint_id}, "
                    f"reason={filming_block_reason})"
                )
                self._state.last_trigger_key_by_aircraft.pop(aircraft_id_int, None)
                continue

            alt_waypoint_id = _coerce_int(view.alternate_waypoint_id)
            alt_coordinate = dict(view.alternate_waypoint_coordinate or {})
            alt_lat = _coerce_float(alt_coordinate.get("latitude"))
            alt_lon = _coerce_float(alt_coordinate.get("longitude"))
            if alt_waypoint_id is None or alt_waypoint_id <= 0 or alt_lat is None or alt_lon is None:
                logs.append(
                    f"[0401] path-deviation replan skipped: alternate waypoint data incomplete "
                    f"(aircraftID={aircraft_id_int}, altWP={alt_waypoint_id}, "
                    f"lat={alt_lat}, lon={alt_lon})"
                )
                continue

            trigger_key = (
                int(current_mission_plan_id),
                aircraft_id_int,
                int(resolved_current_waypoint_id),
            )
            if self._state.last_trigger_key_by_aircraft.get(aircraft_id_int) == trigger_key:
                continue

            mission_plan_ids = allocate_mission_plan_ids(1)
            if not mission_plan_ids:
                logs.append(
                    f"[0401] path-deviation replan skipped: MissionPlanID allocation failed "
                    f"(aircraftID={aircraft_id_int})"
                )
                continue

            mission_plan_id = int(mission_plan_ids[0])
            now_ts = int(self._now_ms())
            replan_reason = _format_path_deviation_reason(int(aircraft_id))
            payload = self._build_payload(
                timestamp_ms=now_ts,
                mission_plan_id=mission_plan_id,
                source_plan_id=int(current_mission_plan_id),
                aircraft_id=aircraft_id_int,
                current_waypoint_id=int(resolved_current_waypoint_id),
                alt_waypoint_id=int(alt_waypoint_id),
                alt_latitude=alt_lat,
                alt_longitude=alt_lon,
                view=view,
                artifacts=artifacts,
                replan_reason=replan_reason,
            )
            detail_payload = dict(payload.get("replanDetail") or {})
            self._persist_detail(int(mission_plan_id), detail_payload)
            payloads.append(payload)
            self._state.last_trigger_key_by_aircraft[aircraft_id_int] = trigger_key
            logs.append(
                f"[0401] path-deviation replan prepared "
                f"(aircraftID={aircraft_id_int}, currentWP={resolved_current_waypoint_id}, altWP={alt_waypoint_id}, "
                f"missionPlanID={mission_plan_id}, reason={replan_reason})"
            )

        return payloads, logs

    def _build_payload(
        self,
        *,
        timestamp_ms: int,
        mission_plan_id: int,
        source_plan_id: int,
        aircraft_id: int,
        current_waypoint_id: int,
        alt_waypoint_id: int,
        alt_latitude: float,
        alt_longitude: float,
        view: AircraftTurnView,
        artifacts: SourceMissionArtifacts,
        replan_reason: str,
    ) -> dict[str, Any]:
        input_ids = list(artifacts.input_mission_ids or [])
        if not input_ids:
            input_ids = [0]
        current_input_id = None
        for mission_id in input_ids:
            parsed_input_id = _coerce_int(mission_id)
            if parsed_input_id is not None and parsed_input_id > 0:
                current_input_id = int(parsed_input_id)
                break
        alternate_coordinate = {
            "latitude": float(alt_latitude),
            "longitude": float(alt_longitude),
        }
        alternate_altitude = _coerce_float((view.alternate_waypoint_coordinate or {}).get("altitude"))
        if alternate_altitude is None:
            alternate_altitude = _coerce_float((view.position_coordinate or {}).get("altitude"))
        if alternate_altitude is not None:
            alternate_coordinate["altitude"] = float(alternate_altitude)

        detail_payload = {
            "trigger": "0401",
            "triggerType": TRIGGER_TYPE,
            "sourceMissionPlanID": int(source_plan_id),
            "currentMissionPlanID": int(source_plan_id),
            "aircraftID": int(aircraft_id),
            "inputMissionPackageID": int(artifacts.input_mission_package_id)
            if artifacts.input_mission_package_id is not None
            else None,
            "currentInputMissionID": current_input_id,
            "targetInputMissionID": current_input_id,
            "individualMissionPackageID": int(artifacts.individual_mission_package_id),
            "individualMissionID": int(artifacts.individual_mission_id),
            "pathID": int(artifacts.path_id),
            "currentWaypointID": int(current_waypoint_id),
            "alternateWaypointID": int(alt_waypoint_id),
            "alternateWaypointCoordinate": alternate_coordinate,
            "alternateWaypointEtaS": _coerce_float(view.alternate_waypoint_eta_s),
            "alternateWaypointPredictionModel": getattr(view, "alternate_prediction_model", None),
            "triggerCoordinate": dict(view.position_coordinate or {}),
            "triggerVelocity": {
                # This field echoes the telemetry the trigger saw, so it stays
                # the reported value even when the monitor has rejected it as
                # inconsistent with the track; a reader comparing this against
                # 0401 must see 0401.  The speed the estimates actually used is
                # reported alongside rather than substituted in.
                "speed": _coerce_float(
                    getattr(view, "reported_speed_mps", None)
                    if getattr(view, "reported_speed_mps", None) is not None
                    else view.speed_mps
                ),
                "heading": _coerce_float(view.raw_heading_deg),
                "resolvedSpeedMps": _coerce_float(view.speed_mps),
                "speedSource": getattr(view, "speed_source", None),
                "trackSpeedMps": _coerce_float(getattr(view, "track_speed_mps", None)),
            },
            "triggerAttitude": {
                "roll": _coerce_float(getattr(view, "roll_deg", None)),
                "pitch": _coerce_float(getattr(view, "pitch_deg", None)),
                "yaw": _coerce_float(getattr(view, "yaw_deg", None)),
            },
            "triggerTurnDynamics": {
                "rollRateDegS": _coerce_float(getattr(view, "roll_rate_dps", None)),
                "targetRollDeg": _coerce_float(getattr(view, "target_roll_deg", None)),
                "yawTurnRateNavDegS": _coerce_float(
                    getattr(view, "yaw_turn_rate_nav_dps", None)
                ),
                "attitudeTurnRateNavDegS": _coerce_float(
                    getattr(view, "attitude_turn_rate_nav_dps", None)
                ),
                "attitudeRateScale": _coerce_float(
                    getattr(view, "attitude_rate_scale", None)
                ),
                "attitudeRateSampleCount": _coerce_int(
                    getattr(view, "attitude_rate_sample_count", None)
                )
                or 0,
                "performance": dict(getattr(view, "turn_performance", None) or {}),
            },
            "triggerFlying": _coerce_int(getattr(view, "flying", None)),
            "spiralState": str(view.spiral_state or "none"),
            "spiralAngleDeg": _coerce_float(view.spiral_angle_deg),
            "idealRadiusM": _coerce_float(view.ideal_radius_m),
            "actualRadiusM": _coerce_float(view.actual_radius_m),
            "turnRateDegS": _coerce_float(view.turn_rate_dps),
            "attitudeTurnRateNavDegS": _coerce_float(
                getattr(view, "attitude_turn_rate_nav_dps", None)
            ),
            "courseYawErrorDeg": _coerce_float(getattr(view, "course_yaw_error_deg", None)),
            "turnRateSource": str(getattr(view, "turn_rate_source", "position") or "position"),
            "selectedReplanReason": str(replan_reason),
            "timestamp": int(timestamp_ms),
        }
        detail_payload["adaptivePathDeviation"] = {
            "radiusScale": _coerce_float(getattr(view, "adaptive_radius_scale", None)),
            "thresholdScale": _coerce_float(getattr(view, "adaptive_threshold_scale", None)),
            "expectedRollDeg": _coerce_float(getattr(view, "adaptive_expected_roll_deg", None)),
            "actualRollDeg": _coerce_float(getattr(view, "roll_deg", None)),
            "actualPitchDeg": _coerce_float(getattr(view, "pitch_deg", None)),
            "actualYawDeg": _coerce_float(getattr(view, "yaw_deg", None)),
            "rollRateDegS": _coerce_float(getattr(view, "roll_rate_dps", None)),
            "targetRollDeg": _coerce_float(getattr(view, "target_roll_deg", None)),
            "attitudeRateScale": _coerce_float(getattr(view, "attitude_rate_scale", None)),
            "attitudeRateSampleCount": _coerce_int(
                getattr(view, "attitude_rate_sample_count", None)
            )
            or 0,
            "sampleCount": _coerce_int(getattr(view, "adaptive_sample_count", None)) or 0,
            "lastActualRadiusM": _coerce_float(getattr(view, "adaptive_last_actual_radius_m", None)),
            "lastIdealRadiusM": _coerce_float(getattr(view, "adaptive_last_ideal_radius_m", None)),
            "lastSpeedMps": _coerce_float(getattr(view, "adaptive_last_speed_mps", None)),
        }
        option_block = [
            {
                "optionID": 1,
                "optionName": OPTION_NAME,
                "missionPlanID": int(mission_plan_id),
            }
        ]
        return {
            "timestamp": int(timestamp_ms),
            "source": "MSM",
            "inputMissionPackageID": int(artifacts.input_mission_package_id)
            if artifacts.input_mission_package_id is not None
            else 0,
            "sourceMissionPlanID": int(source_plan_id),
            "currentMissionPlanID": int(source_plan_id),
            "replanRequestTime": {"replanRequestTimestamp": int(timestamp_ms)},
            "replanLevel": int(REPLAN_LEVEL),
            "replanRequest": str(replan_reason),
            "replanReason": str(replan_reason),
            "inputMissionIDList": [{"inputMissionID": int(mission_id)} for mission_id in input_ids],
            "individualMissionIDList": [
                {"individualMissionID": int(artifacts.individual_mission_id)}
            ],
            "pendingOptionList": option_block,
            "replanDetail": detail_payload,
        }

    def _is_trigger_view(self, view: AircraftTurnView) -> bool:
        return is_path_deviation_turn_warning_view(view)

    def _resolve_source_artifacts(
        self,
        *,
        source_plan_id: int,
        aircraft_id: int,
        current_waypoint_id: int,
        current_input_id: int | None = None,
    ) -> SourceMissionArtifacts | None:
        artifact_index = SourceArtifactIndex.from_source_plan(int(source_plan_id))
        if artifact_index is None:
            return None

        input_mission_package_id = artifact_index.input_mission_package_id
        imp_id = artifact_index.individual_package_id_for_aircraft(int(aircraft_id))
        if imp_id is None or imp_id <= 0:
            return None

        for mission in artifact_index.individual_missions(int(imp_id)):
            if not isinstance(mission, dict):
                continue
            related = mission.get("relatedMission") or {}
            mission_input_id = _coerce_int(
                related.get("inputMissionID")
                if isinstance(related, dict)
                else None
            )
            current_input_id_int = _coerce_int(current_input_id)
            if (
                current_input_id_int is not None
                and current_input_id_int > 0
                and mission_input_id is not None
                and mission_input_id > 0
                and int(mission_input_id) != int(current_input_id_int)
            ):
                continue
            path_id = _coerce_int(mission.get("pathID"))
            individual_mission_id = _coerce_int(mission.get("individualMissionID"))
            if path_id is None or path_id <= 0 or individual_mission_id is None or individual_mission_id <= 0:
                continue
            if int(current_waypoint_id) not in artifact_index.waypoint_ids(int(path_id)):
                continue

            return SourceMissionArtifacts(
                input_mission_package_id=input_mission_package_id,
                input_mission_ids=_resolve_input_mission_ids(
                    mission=mission,
                    input_mission_package_id=input_mission_package_id,
                    source_plan_id=source_plan_id,
                    aircraft_id=aircraft_id,
                    logger=self._log,
                ),
                individual_mission_package_id=int(imp_id),
                individual_mission_id=int(individual_mission_id),
                path_id=int(path_id),
                resolved_current_waypoint_id=int(current_waypoint_id),
            )
        return None

    def _persist_detail(self, mission_plan_id: int, detail_payload: dict[str, Any]) -> None:
        try:
            detail_path = path_deviation_replan_store.save_detail(int(mission_plan_id), detail_payload)
        except Exception as exc:
            detail_path = None
            if self._log:
                self._log(
                    f"[0401] _persist_detail: save_detail failed "
                    f"(missionPlanID={mission_plan_id}): {exc}"
                )
        try:
            path_deviation_replan_store.save_event(
                "monitor_dispatch",
                {
                    "missionPlanID": int(mission_plan_id),
                    "detailPath": str(detail_path) if detail_path is not None else None,
                    "detail": dict(detail_payload or {}),
                },
            )
        except Exception as exc:
            if self._log:
                self._log(
                    f"[0401] _persist_detail: save_event failed "
                    f"(missionPlanID={mission_plan_id}): {exc}"
                )
