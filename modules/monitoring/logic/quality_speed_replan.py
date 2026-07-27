# -*- coding: utf-8 -*-
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable

from modules.common import imaging_schedule_replan_store
from modules.monitoring.logic.init_replan import allocate_mission_plan_ids, collect_input_mission_ids
from modules.monitoring.logic.mission_update import (
    build_uav_mission_view,
    compute_filming_quality_threshold_m,
    load_db_json,
    lookup_fov_db_max_width_m,
)
from modules.monitoring.logic.replan_runtime_settings import get_quality_speed_settings
from modules.monitoring.logic.source_artifact_index import SourceArtifactIndex


REPLAN_LEVEL = 3
TRIGGER_TYPE = "qualityMonitorSep"
TRACKED_UAV_IDS = (4, 5, 6)


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


def _normalize_coordinate(value: object | None) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None
    lat = _coerce_float(value.get("latitude") or value.get("Latitude"))
    lon = _coerce_float(value.get("longitude") or value.get("Longitude"))
    alt = _coerce_float(value.get("altitude") or value.get("Altitude"))
    if lat is None or lon is None:
        return None
    out: dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        out["altitude"] = float(alt)
    return out


def _ground_distance_m(left: dict[str, float] | None, right: dict[str, float] | None) -> float | None:
    if not left or not right:
        return None
    import math

    lat1 = math.radians(float(left["latitude"]))
    lon1 = math.radians(float(left["longitude"]))
    lat2 = math.radians(float(right["latitude"]))
    lon2 = math.radians(float(right["longitude"]))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return 6_371_000.0 * c


@dataclass(frozen=True)
class SourceMissionArtifacts:
    input_mission_package_id: int | None
    input_mission_ids: list[int]
    individual_mission_package_id: int
    individual_mission_id: int
    path_id: int


@dataclass
class _SampleWindow:
    context_key: tuple[int, int] | None = None
    samples: deque[float] = field(default_factory=deque)


class QualitySpeedReplanCoordinator:
    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._mission_plan_id: int | None = None
        self._thresholds_by_aircraft: dict[int, dict[int, dict[str, Any]]] = {
            aircraft_id: {} for aircraft_id in TRACKED_UAV_IDS
        }
        self._sample_windows: dict[int, _SampleWindow] = {
            aircraft_id: _SampleWindow() for aircraft_id in TRACKED_UAV_IDS
        }
        self._last_trigger_key_by_aircraft: dict[int, tuple[int, int]] = {}
        self._startup_grace_started_ms: int | None = None

    @staticmethod
    def _settings() -> dict[str, Any]:
        return get_quality_speed_settings()

    def apply_mission_plan_decision(self, mission_plan_id: int | None) -> None:
        self._mission_plan_id = _coerce_int(mission_plan_id)
        self._rebuild_thresholds(self._mission_plan_id)
        for aircraft_id in TRACKED_UAV_IDS:
            self._sample_windows[int(aircraft_id)] = _SampleWindow()
        settings = self._settings()
        startup_grace_ms = max(int(float(settings.get("startup_grace_sec", 10.0)) * 1000.0), 0)
        if self._mission_plan_id is not None and startup_grace_ms > 0:
            now_ms = int(self._now_ms())
            self._startup_grace_started_ms = now_ms
            if self._log is not None:
                self._log(
                    f"[QUALITY] startup grace armed "
                    f"(missionPlanID={self._mission_plan_id}, duration={startup_grace_ms / 1000.0:.1f}s)"
                )
        else:
            self._startup_grace_started_ms = None

    def on_agent_states(
        self,
        states: list[dict[str, Any]] | None,
        *,
        enabled: bool = True,
        system_mode: int | None,
        current_mission_plan_id: int | None,
        aircraft_filter: Callable[[int], bool] | None = None,
        suppressed_aircraft: set[int] | None = None,
        dispatch_context: Any | None = None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        logs: list[str] = []
        if not enabled:
            return [], logs
        if system_mode not in (3, 4):
            return [], logs
        if current_mission_plan_id is None:
            return [], logs
        if not isinstance(states, list):
            return [], logs
        settings = self._settings()
        min_sample_count = max(1, int(settings.get("min_sample_count", 5)))
        max_sample_count = max(min_sample_count, int(settings.get("max_sample_count", 30)))
        lower_band_ratio = float(settings.get("lower_band_ratio", 0.90))
        search_speed_up_scale = float(settings.get("search_speed_up_scale", 1.10))
        search_speed_down_scale = float(settings.get("search_speed_down_scale", 0.90))
        startup_grace_ms = max(int(float(settings.get("startup_grace_sec", 10.0)) * 1000.0), 0)
        disabled_flight_mode = int(settings.get("disabled_flight_mode", 8))

        mission_plan_id = int(current_mission_plan_id)
        if self._mission_plan_id != mission_plan_id:
            self.apply_mission_plan_decision(mission_plan_id)

        grace_started_ms = self._startup_grace_started_ms
        if grace_started_ms is not None and startup_grace_ms > 0:
            now_ms = int(self._now_ms())
            if now_ms < int(grace_started_ms + startup_grace_ms):
                for aircraft_id in TRACKED_UAV_IDS:
                    self._sample_windows[int(aircraft_id)] = _SampleWindow()
                return [], logs

        suppressed = {int(value) for value in (suppressed_aircraft or set())}
        payloads: list[dict[str, Any]] = []
        for state in states:
            if not isinstance(state, dict):
                continue
            aircraft_id = _coerce_int(state.get("aircraft_id") or state.get("aircraftID"))
            if aircraft_id not in TRACKED_UAV_IDS:
                continue
            if aircraft_id in suppressed:
                continue
            if aircraft_filter is not None:
                try:
                    if not bool(aircraft_filter(int(aircraft_id))):
                        continue
                except Exception:
                    continue

            flight_mode = _coerce_int(state.get("flight_mode") or state.get("flightMode"))
            current_waypoint_id = _coerce_int(
                state.get("current_waypoint_id") or state.get("currentWaypointID")
            )
            if flight_mode == disabled_flight_mode or current_waypoint_id is None or current_waypoint_id <= 0:
                self._reset_samples(int(aircraft_id))
                continue

            threshold_meta = self._thresholds_by_aircraft.get(int(aircraft_id), {}).get(int(current_waypoint_id))
            base_sep_m = _coerce_float((threshold_meta or {}).get("sep_m"))
            if base_sep_m is None or base_sep_m <= 0.0:
                self._reset_samples(int(aircraft_id))
                continue
            sensor_fov_deg = _coerce_float(state.get("sensor_fov_deg"))
            effective_width_m = lookup_fov_db_max_width_m(sensor_fov_deg)
            if effective_width_m is None:
                effective_width_m = _coerce_float((threshold_meta or {}).get("width_m"))
            trigger_threshold_m = compute_filming_quality_threshold_m(base_sep_m, effective_width_m)
            if trigger_threshold_m is None or trigger_threshold_m <= 0.0:
                self._reset_samples(int(aircraft_id))
                continue

            actual_distance_m = _ground_distance_m(
                _normalize_coordinate(state.get("coordinate")),
                _normalize_coordinate(state.get("sensor_center_coordinate")),
            )
            if actual_distance_m is None or actual_distance_m < 0.0:
                self._reset_samples(int(aircraft_id))
                continue

            sample_window = self._sample_windows.setdefault(int(aircraft_id), _SampleWindow())
            context_key = (int(mission_plan_id), int(current_waypoint_id))
            if sample_window.context_key != context_key:
                sample_window = _SampleWindow(context_key=context_key)
                self._sample_windows[int(aircraft_id)] = sample_window
            sample_window.samples.append(float(actual_distance_m))
            while len(sample_window.samples) > max_sample_count:
                sample_window.samples.popleft()
            if len(sample_window.samples) < min_sample_count:
                continue

            average_distance_m = sum(sample_window.samples) / float(len(sample_window.samples))
            lower_band_threshold_m = float(trigger_threshold_m) * lower_band_ratio
            speed_scale = None
            direction = None
            if float(average_distance_m) < float(lower_band_threshold_m):
                speed_scale = search_speed_up_scale
                direction = "increase"
            elif float(average_distance_m) > float(trigger_threshold_m):
                speed_scale = search_speed_down_scale
                direction = "decrease"
            if speed_scale is None or direction is None:
                continue

            trigger_key = (int(mission_plan_id), int(current_waypoint_id))
            if self._last_trigger_key_by_aircraft.get(int(aircraft_id)) == trigger_key:
                continue

            artifacts = self._resolve_source_artifacts(
                source_plan_id=int(mission_plan_id),
                aircraft_id=int(aircraft_id),
                current_waypoint_id=int(current_waypoint_id),
            )
            if artifacts is None:
                logs.append(
                    f"[QUALITY] replan skipped: source artifacts unresolved "
                    f"(aircraftID={aircraft_id}, currentWP={current_waypoint_id})"
                )
                continue

            mission_plan_ids = allocate_mission_plan_ids(1)
            if not mission_plan_ids:
                logs.append(
                    f"[QUALITY] replan skipped: MissionPlanID allocation failed "
                    f"(aircraftID={aircraft_id}, currentWP={current_waypoint_id})"
                )
                continue

            mission_plan_id_out = int(mission_plan_ids[0])
            now_ts = int(self._now_ms())
            payload = self._build_payload(
                timestamp_ms=now_ts,
                mission_plan_id=mission_plan_id_out,
                source_plan_id=int(mission_plan_id),
                aircraft_id=int(aircraft_id),
                current_waypoint_id=int(current_waypoint_id),
                flight_mode=flight_mode,
                base_sep_m=base_sep_m,
                effective_width_m=effective_width_m,
                sensor_fov_deg=sensor_fov_deg,
                trigger_threshold_m=float(trigger_threshold_m),
                lower_band_threshold_m=float(lower_band_threshold_m),
                average_distance_m=float(average_distance_m),
                actual_distance_m=float(actual_distance_m),
                speed_scale=float(speed_scale),
                direction=str(direction),
                state=state,
                artifacts=artifacts,
                sample_count=len(sample_window.samples),
            )
            detail_payload = dict(payload.get("replanDetail") or {})
            self._persist_detail(int(mission_plan_id_out), detail_payload)
            payloads.append(payload)
            self._last_trigger_key_by_aircraft[int(aircraft_id)] = trigger_key
            logs.append(
                f"[QUALITY] 촬영품질 재계획 준비 "
                f"(aircraftID={aircraft_id}, currentWP={current_waypoint_id}, avgDistance={average_distance_m:.1f}, "
                f"threshold={trigger_threshold_m:.1f}, scale={speed_scale:.2f}, missionPlanID={mission_plan_id_out})"
            )

        return payloads, logs

    def _reset_samples(self, aircraft_id: int) -> None:
        if int(aircraft_id) in self._sample_windows:
            self._sample_windows[int(aircraft_id)] = _SampleWindow()

    def _rebuild_thresholds(self, mission_plan_id: int | None) -> None:
        self._thresholds_by_aircraft = {aircraft_id: {} for aircraft_id in TRACKED_UAV_IDS}
        view = build_uav_mission_view(mission_plan_id, uav_ids=TRACKED_UAV_IDS)
        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = _coerce_int(entry.get("aircraft_id"))
            if aircraft_id not in TRACKED_UAV_IDS:
                continue
            waypoint_map = self._thresholds_by_aircraft.setdefault(int(aircraft_id), {})
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                sep_m = _coerce_float(mission.get("sep_m"))
                if sep_m is None or sep_m <= 0.0:
                    continue
                meta = {
                    "sep_m": float(sep_m),
                    "width_m": _coerce_float(mission.get("width_m")),
                    "path_id": _coerce_int(mission.get("path_id")),
                    "individual_mission_id": _coerce_int(mission.get("individual_mission_id")),
                }
                for waypoint_id in mission.get("waypoint_ids") or []:
                    wid = _coerce_int(waypoint_id)
                    if wid is None or wid <= 0:
                        continue
                    waypoint_map[int(wid)] = dict(meta)

    def _build_payload(
        self,
        *,
        timestamp_ms: int,
        mission_plan_id: int,
        source_plan_id: int,
        aircraft_id: int,
        current_waypoint_id: int,
        flight_mode: int | None,
        base_sep_m: float | None,
        effective_width_m: float | None,
        sensor_fov_deg: float | None,
        trigger_threshold_m: float,
        lower_band_threshold_m: float,
        average_distance_m: float,
        actual_distance_m: float,
        speed_scale: float,
        direction: str,
        state: dict[str, Any],
        artifacts: SourceMissionArtifacts,
        sample_count: int,
    ) -> dict[str, Any]:
        input_ids = list(artifacts.input_mission_ids or [])
        if not input_ids:
            input_ids = [0]
        reason_text = f"UAV {int(aircraft_id) - 3} 촬영품질 개선"
        effective_half_width_m = None
        if effective_width_m is not None and float(effective_width_m) > 0.0:
            effective_half_width_m = float(effective_width_m) * 0.5
        detail_payload = {
            "trigger": "0401",
            "triggerType": TRIGGER_TYPE,
            "qualityAction": "searchSpeedAdjust",
            "sourceMissionPlanID": int(source_plan_id),
            "missionPlanID": int(mission_plan_id),
            "aircraftID": int(aircraft_id),
            "uavIndex": int(aircraft_id) - 3,
            "inputMissionPackageID": int(artifacts.input_mission_package_id)
            if artifacts.input_mission_package_id is not None
            else None,
            "individualMissionPackageID": int(artifacts.individual_mission_package_id),
            "individualMissionID": int(artifacts.individual_mission_id),
            "pathID": int(artifacts.path_id),
            "currentWaypointID": int(current_waypoint_id),
            "currentAircraftCoordinate": _normalize_coordinate(state.get("coordinate")),
            "sensorCenterCoordinate": _normalize_coordinate(state.get("sensor_center_coordinate")),
            "flightMode": int(flight_mode) if flight_mode is not None else None,
            "currentDistanceM": float(actual_distance_m),
            "averageDistanceM": float(average_distance_m),
            "thresholdDistanceM": float(trigger_threshold_m),
            "lowerBandDistanceM": float(lower_band_threshold_m),
            "baseSepM": float(base_sep_m) if base_sep_m is not None else None,
            "sensorFovDeg": float(sensor_fov_deg) if sensor_fov_deg is not None else None,
            "effectiveWidthM": float(effective_width_m) if effective_width_m is not None else None,
            "effectiveHalfWidthM": float(effective_half_width_m)
            if effective_half_width_m is not None
            else None,
            "currentSepM": float(actual_distance_m),
            "averageSepM": float(average_distance_m),
            "thresholdSepM": float(trigger_threshold_m),
            "lowerBandSepM": float(lower_band_threshold_m),
            "searchSpeedScale": float(speed_scale),
            "speedAdjustmentDirection": str(direction),
            "sampleCount": int(sample_count),
            "timestamp": int(timestamp_ms),
        }
        return {
            "timestamp": int(timestamp_ms),
            "source": "MSM",
            "inputMissionPackageID": int(artifacts.input_mission_package_id)
            if artifacts.input_mission_package_id is not None
            else 0,
            "replanRequestTime": {"replanRequestTimestamp": int(timestamp_ms)},
            "replanLevel": int(REPLAN_LEVEL),
            "replanRequest": str(reason_text),
            "replanReason": str(reason_text),
            "inputMissionIDList": [{"inputMissionID": int(mission_id)} for mission_id in input_ids],
            "individualMissionIDList": [
                {"individualMissionID": int(artifacts.individual_mission_id)}
            ],
            "pendingOptionList": [],
            "missionPlanIDList": [{"missionPlanID": int(mission_plan_id)}],
            "replanDetail": detail_payload,
        }

    def _resolve_source_artifacts(
        self,
        *,
        source_plan_id: int,
        aircraft_id: int,
        current_waypoint_id: int,
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
            path_id = _coerce_int(mission.get("pathID"))
            individual_mission_id = _coerce_int(mission.get("individualMissionID"))
            if path_id is None or path_id <= 0 or individual_mission_id is None or individual_mission_id <= 0:
                continue
            if int(current_waypoint_id) not in artifact_index.waypoint_ids(int(path_id)):
                continue

            related = mission.get("relatedMission") or {}
            input_mission_ids: list[int] = []
            related_input_mission_id = _coerce_int(
                related.get("inputMissionID") if isinstance(related, dict) else None
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
                input_mission_ids = [0]
            return SourceMissionArtifacts(
                input_mission_package_id=input_mission_package_id,
                input_mission_ids=input_mission_ids,
                individual_mission_package_id=int(imp_id),
                individual_mission_id=int(individual_mission_id),
                path_id=int(path_id),
            )
        return None

    def _persist_detail(self, mission_plan_id: int, detail_payload: dict[str, Any]) -> None:
        try:
            detail_path = imaging_schedule_replan_store.save_detail(int(mission_plan_id), detail_payload)
        except Exception:
            detail_path = None
        try:
            imaging_schedule_replan_store.save_event(
                "quality_monitor_dispatch",
                {
                    "missionPlanID": int(mission_plan_id),
                    "detailPath": str(detail_path) if detail_path is not None else None,
                    "detail": dict(detail_payload or {}),
                },
            )
        except Exception:
            pass
