from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..config import SIM_BASE_DT, SIM_POS_TOL, SIM_SPEED_LAH, SIM_SPEED_UAV, SIM_TIME_SCALE
from .geo import GeoConverter
from .lah import LAH, LAHParams
from .uav import UAV, UAVParams
from .controllers.waypoint_pid import WaypointPIDController, WaypointTarget, load_pid_gains_for_time_scale
from .operation_mode import OperationContext, OperationMode, build_operation_mode


def _agent_label(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return f"AC{aircraft_id}"


def _airframe_type(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return "lah"
    if 4 <= aircraft_id <= 6:
        return "uav"
    return "uav"


def _extract_waypoints(data: Dict[str, Any]) -> list[Dict[str, Any]]:
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        lst = data.get(key)
        if isinstance(lst, list):
            return lst
    return []


def _extract_coord(item: Dict[str, Any]) -> Optional[tuple[float, float, Optional[float]]]:
    coord = item.get("coordinate") or item.get("Coordinate")
    if not isinstance(coord, dict):
        return None
    lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
    lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
    alt = coord.get("altitude") if "altitude" in coord else coord.get("Altitude")
    if lat is None or lon is None:
        return None
    try:
        lat_v = float(lat)
        lon_v = float(lon)
        alt_v = float(alt) if alt is not None else None
    except Exception:
        return None
    return lat_v, lon_v, alt_v


def _order_waypoints(raw: list[Dict[str, Any]]) -> list[Dict[str, Any]]:
    if not raw:
        return []
    by_id: dict[int, Dict[str, Any]] = {}
    next_ids: set[int] = set()
    for wp in raw:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        if wid is None:
            continue
        try:
            wid_i = int(wid)
        except Exception:
            continue
        by_id[wid_i] = wp
        nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
        if nxt is None:
            continue
        try:
            nxt_i = int(nxt)
        except Exception:
            continue
        if nxt_i > 0:
            next_ids.add(nxt_i)

    if not by_id:
        return list(raw)

    start_id = None
    for wid in by_id:
        if wid not in next_ids:
            start_id = wid
            break

    ordered: list[Dict[str, Any]] = []
    visited: set[int] = set()
    if start_id is not None:
        curr = start_id
        while curr and curr in by_id and curr not in visited:
            wp = by_id[curr]
            ordered.append(wp)
            visited.add(curr)
            nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
            try:
                curr = int(nxt)
            except Exception:
                break
            if curr == 0:
                break

    # Append any leftover waypoints in original order.
    for wp in raw:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        if wid is None:
            ordered.append(wp)
            continue
        try:
            wid_i = int(wid)
        except Exception:
            ordered.append(wp)
            continue
        if wid_i not in visited:
            ordered.append(wp)

    return ordered


def _label_to_aircraft_id(label: str) -> Optional[int]:
    text = str(label or "").strip().upper()
    if text.startswith("LAH"):
        try:
            idx = int(text.replace("LAH", ""))
        except Exception:
            return None
        return idx if 1 <= idx <= 3 else None
    if text.startswith("UAV"):
        try:
            idx = int(text.replace("UAV", ""))
        except Exception:
            return None
        return idx + 3 if 1 <= idx <= 3 else None
    return None


def _extract_hover_time(item: Dict[str, Any]) -> Optional[float]:
    hover = (
        item.get("hovering")
        or item.get("Hovering")
        or item.get("hover_prop")
        or item.get("hoveringProperty")
        or {}
    )
    if isinstance(hover, dict):
        val = hover.get("time") or hover.get("Time")
        if val is not None:
            try:
                return float(val)
            except Exception:
                return None
    for key in ("hover_time", "hoverTime", "hover"):
        if key in item and item[key] is not None:
            try:
                return float(item[key])
            except Exception:
                return None
    return None


def _extract_loiter(item: Dict[str, Any]) -> Optional[dict]:
    loiter = item.get("loiter") or item.get("Loiter") or item.get("loiterProperty") or item.get("loiter_prop")
    if isinstance(loiter, dict):
        return loiter
    return None


@dataclass
class PathDefinition:
    label: str
    aircraft_id: int
    airframe: str
    path_id: int | None
    waypoints: list[dict]


@dataclass
class SimVehicle:
    label: str
    aircraft_id: int
    airframe: str
    vehicle: object
    controller: WaypointPIDController
    path_id: int | None


class SimulationService:
    def __init__(
        self,
        *,
        base_dt: float = SIM_BASE_DT,
        time_scale: float = SIM_TIME_SCALE,
        pos_tol: float = SIM_POS_TOL,
        speed_uav: float = SIM_SPEED_UAV,
        speed_lah: float = SIM_SPEED_LAH,
    ) -> None:
        self.base_dt = float(base_dt)
        self.time_scale = float(time_scale)
        self.dt = max(1e-4, self.base_dt * self.time_scale)
        self.pos_tol = float(pos_tol)
        self.speed_uav = float(speed_uav)
        self.speed_lah = float(speed_lah)

        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None

        self.running = False
        self.paused = True
        self.speed_factor = 1.0
        self.sim_time = 0.0
        self.step_count = 0
        self.last_error: str | None = None

        self.geo: GeoConverter | None = None
        self._paths: list[PathDefinition] = []
        self.vehicles: dict[str, SimVehicle] = {}
        self._block_indices: dict[int, dict[int, int]] = {}
        self._spawn_by_aircraft: dict[int, tuple[float, float, float]] = {}
        self._operation_handlers: dict[int, OperationMode] = {}
        self._filming_props: dict[str, dict | None] = {}
        self._filming_targets: dict[str, tuple[float, float, float] | None] = {}
        self._filming_wp_ids: dict[str, int | None] = {}
        self._line_search_state: dict[str, object | None] = {}
        self._line_search_debug: dict[str, object | None] = {}
        self.input_mission_order_by_aircraft: dict[int, list[int]] = {}
        self.current_input_mission_idx_by_aircraft: dict[int, int] = {}

    def _ensure_thread(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._shutdown.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def set_speed_factor(self, value: float) -> float:
        try:
            factor = float(value)
        except Exception:
            factor = 1.0
        factor = max(0.1, min(30.0, factor))
        with self._lock:
            self.speed_factor = factor
        return factor

    def play(self) -> dict:
        with self._lock:
            if not self.vehicles:
                return {"ok": False, "error": "no mission loaded"}
            self.running = True
            self.paused = False
        self._ensure_thread()
        return {"ok": True}

    def pause(self) -> dict:
        with self._lock:
            self.paused = True
        return {"ok": True}

    def stop(self) -> dict:
        with self._lock:
            self.running = False
            self.paused = True
        self.reset()
        return {"ok": True}

    def clear(self) -> dict:
        with self._lock:
            self.running = False
            self.paused = True
            self.sim_time = 0.0
            self.step_count = 0
            self.last_error = None
            self.geo = None
            self._paths = []
            self.vehicles = {}
            self._block_indices = {}
            self._spawn_by_aircraft = {}
            self.input_mission_order_by_aircraft = {}
            self.current_input_mission_idx_by_aircraft = {}
            self._filming_props = {}
            self._filming_targets = {}
            self._filming_wp_ids = {}
            self._line_search_state = {}
            self._line_search_debug = {}
        return {"ok": True}

    def reset(self) -> dict:
        with self._lock:
            self.sim_time = 0.0
            self.step_count = 0
            if self._paths:
                self._build_vehicles(self._paths)
        return {"ok": True}

    def _current_input_mission_id_for(self, aircraft_id: int) -> Optional[int]:
        order = self.input_mission_order_by_aircraft.get(aircraft_id) or []
        idx = self.current_input_mission_idx_by_aircraft.get(aircraft_id, 0)
        if idx < 0 or idx >= len(order):
            return None
        return order[idx]

    def _next_input_mission_id_for(self, aircraft_id: int) -> Optional[int]:
        order = self.input_mission_order_by_aircraft.get(aircraft_id) or []
        idx = self.current_input_mission_idx_by_aircraft.get(aircraft_id, 0) + 1
        if idx < 0 or idx >= len(order):
            return None
        return order[idx]

    def advance_input_mission(self, aircraft_id: Optional[int] = None) -> int:
        advanced = 0
        with self._lock:
            targets = (
                [aircraft_id]
                if aircraft_id is not None
                else sorted(self.input_mission_order_by_aircraft.keys())
            )
            for aid in targets:
                cur_id = self._current_input_mission_id_for(aid)
                if cur_id is None:
                    continue
                if self._next_input_mission_id_for(aid) is None:
                    continue
                label = _agent_label(aid)
                simv = self.vehicles.get(label)
                if not simv:
                    continue
                ap = simv.controller
                if getattr(ap, "blocked_input_id", None) != cur_id:
                    continue
                try:
                    ap.release_block()
                except Exception:
                    continue
                self.current_input_mission_idx_by_aircraft[aid] = (
                    self.current_input_mission_idx_by_aircraft.get(aid, 0) + 1
                )
                advanced += 1
        return advanced

    def load_mission(self, payload: dict) -> dict:
        flight_paths = payload.get("flightPaths") or payload.get("paths") or payload.get("flightpaths")
        if not isinstance(flight_paths, list):
            return {"ok": False, "error": "flightPaths list required"}
        mission_order = payload.get("missionOrder") or {}
        input_mission_plans = payload.get("inputMissionPlans") or []
        individual_mission_plans = payload.get("individualMissionPlans") or []
        take_over_list = payload.get("takeOverInfoList")
        if not isinstance(take_over_list, list):
            ref = payload.get("missionReference") or payload.get("missionReferenceInfo") or {}
            take_over_list = ref.get("takeOverInfoList") if isinstance(ref, dict) else []
        if not isinstance(take_over_list, list):
            take_over_list = []

        paths: list[PathDefinition] = []
        all_latlons: list[tuple[float, float]] = []
        flight_by_path: dict[int, dict] = {}
        flight_by_aircraft: dict[int, list[int]] = {}

        for entry in flight_paths:
            if not isinstance(entry, dict):
                continue
            data = entry.get("data") if isinstance(entry.get("data"), dict) else entry
            if not isinstance(data, dict):
                continue

            aircraft_id = data.get("aircraftID") or data.get("AircraftID") or entry.get("aircraftID")
            path_id = data.get("pathID") or data.get("PathID") or entry.get("pathID")
            try:
                aircraft_id = int(aircraft_id)
            except Exception:
                aircraft_id = -1
            try:
                path_id = int(path_id)
            except Exception:
                path_id = None

            waypoints_raw = _extract_waypoints(data)
            if not waypoints_raw:
                continue
            waypoints_raw = _order_waypoints(waypoints_raw)
            if path_id is not None:
                flight_by_path[path_id] = data
                if aircraft_id > 0:
                    flight_by_aircraft.setdefault(aircraft_id, []).append(path_id)

            waypoints: list[dict] = []
            for wp in waypoints_raw:
                if not isinstance(wp, dict):
                    continue
                coord = _extract_coord(wp)
                if coord is None:
                    continue
                lat, lon, alt = coord
                all_latlons.append((lon, lat))
                speed = wp.get("speed")
                try:
                    speed = float(speed) if speed is not None else None
                except Exception:
                    speed = None
                wp_id = wp.get("waypointID") or wp.get("WaypointID")
                try:
                    wp_id = int(wp_id) if wp_id is not None else None
                except Exception:
                    wp_id = None
                hover_time = _extract_hover_time(wp)
                loiter = _extract_loiter(wp)
                filming = wp.get("filmingProperty")
                waypoints.append(
                    {
                        "lat": lat,
                        "lon": lon,
                        "alt": alt,
                        "speed": speed,
                        "wp_id": wp_id,
                        "hover_time": hover_time,
                        "loiter": loiter,
                        "filming": filming,
                        "path_id": path_id,
                    }
                )

            if len(waypoints) < 2:
                continue

            label = _agent_label(aircraft_id)
            airframe = _airframe_type(aircraft_id)
            paths.append(
                PathDefinition(
                    label=label,
                    aircraft_id=aircraft_id,
                    airframe=airframe,
                    path_id=path_id,
                    waypoints=waypoints,
                )
            )

        seq_by_aircraft: dict[int, list[dict]] = {}
        for entry in individual_mission_plans:
            if not isinstance(entry, dict):
                continue
            try:
                aircraft_id = int(entry.get("aircraftID", 0))
            except Exception:
                aircraft_id = 0
            if not aircraft_id:
                continue
            seq = seq_by_aircraft.setdefault(aircraft_id, [])
            for im in entry.get("individualMissionList") or []:
                rel = im.get("relatedMission") or {}
                try:
                    input_id = int(rel.get("inputMissionID"))
                except Exception:
                    input_id = None
                try:
                    path_id = int(im.get("pathID"))
                except Exception:
                    path_id = None
                try:
                    individual_id = int(im.get("individualMissionID"))
                except Exception:
                    individual_id = None
                if path_id is None:
                    continue
                seq.append(
                    {
                        "input_mission_id": input_id,
                        "path_id": path_id,
                        "individual_mission_id": individual_id,
                    }
                )

        input_order: list[int] = []
        if isinstance(input_mission_plans, list) and input_mission_plans:
            latest = input_mission_plans[0]
            latest_ts = float(latest.get("timestamp") or 0)
            for plan in input_mission_plans:
                try:
                    ts = float(plan.get("timestamp") or 0)
                except Exception:
                    ts = 0
                if ts >= latest_ts:
                    latest = plan
                    latest_ts = ts
            for item in latest.get("inputMissionList") or []:
                try:
                    input_order.append(int(item.get("inputMissionID")))
                except Exception:
                    continue

        order_per_aircraft: dict[int, list[int]] = {}
        if not input_order and seq_by_aircraft:
            seen: set[int] = set()
            for seq in seq_by_aircraft.values():
                for entry in seq:
                    mid = entry.get("input_mission_id")
                    if mid is None or mid in seen:
                        continue
                    seen.add(mid)
                    input_order.append(mid)
        if seq_by_aircraft:
            for aircraft_id, seq in seq_by_aircraft.items():
                seen: set[int] = set()
                order: list[int] = []
                for entry in seq:
                    mid = entry.get("input_mission_id")
                    if mid is None or mid in seen:
                        continue
                    seen.add(mid)
                    order.append(mid)
                order_per_aircraft[aircraft_id] = order
        if input_order:
            for aid in list(order_per_aircraft.keys()):
                if not order_per_aircraft.get(aid):
                    order_per_aircraft[aid] = list(input_order)
            if not order_per_aircraft and input_order:
                for aid in flight_by_aircraft.keys():
                    order_per_aircraft[aid] = list(input_order)

        block_indices: dict[int, dict[int, int]] = {}

        if seq_by_aircraft:
            ordered_paths: list[PathDefinition] = []
            ordered_latlons: list[tuple[float, float]] = []
            for aircraft_id, seq in seq_by_aircraft.items():
                combined: list[dict] = []
                block_indices.setdefault(aircraft_id, {})
                last_input = None
                order_list = order_per_aircraft.get(aircraft_id) or []
                if order_list:
                    last_input = order_list[-1]
                last_path_for_input: dict[int, int] = {}
                for entry in seq:
                    mid = entry.get("input_mission_id")
                    pid = entry.get("path_id")
                    if mid is None or pid is None:
                        continue
                    last_path_for_input[mid] = pid

                for entry in seq:
                    pid = entry.get("path_id")
                    if pid is None:
                        continue
                    data = flight_by_path.get(pid)
                    if not isinstance(data, dict):
                        continue
                    wps = _extract_waypoints(data)
                    if not wps:
                        continue
                    wps = _order_waypoints(wps)
                    start_idx = len(combined)
                    for wp in wps:
                        if not isinstance(wp, dict):
                            continue
                        coord = _extract_coord(wp)
                        if coord is None:
                            continue
                        lat, lon, alt = coord
                        ordered_latlons.append((lon, lat))
                        speed = wp.get("speed")
                        try:
                            speed = float(speed) if speed is not None else None
                        except Exception:
                            speed = None
                        wp_id = wp.get("waypointID") or wp.get("WaypointID")
                        try:
                            wp_id = int(wp_id) if wp_id is not None else None
                        except Exception:
                            wp_id = None
                        hover_time = _extract_hover_time(wp)
                        loiter = _extract_loiter(wp)
                        filming = wp.get("filmingProperty")
                        combined.append(
                            {
                                "lat": lat,
                                "lon": lon,
                                "alt": alt,
                                "speed": speed,
                                "wp_id": wp_id,
                                "hover_time": hover_time,
                                "loiter": loiter,
                                "filming": filming,
                                "path_id": pid,
                                "input_mission_id": entry.get("input_mission_id"),
                                "individual_mission_id": entry.get("individual_mission_id"),
                            }
                        )
                    end_idx = len(combined) - 1
                    mid = entry.get("input_mission_id")
                    if (
                        mid is not None
                        and end_idx >= start_idx
                        and last_path_for_input.get(mid) == pid
                        and (last_input is None or mid != last_input)
                    ):
                        block_indices[aircraft_id][end_idx] = mid

                if len(combined) < 2:
                    continue
                label = _agent_label(aircraft_id)
                airframe = _airframe_type(aircraft_id)
                ordered_paths.append(
                    PathDefinition(
                        label=label,
                        aircraft_id=aircraft_id,
                        airframe=airframe,
                        path_id=None,
                        waypoints=combined,
                    )
                )

            if ordered_paths:
                paths = ordered_paths
                all_latlons = ordered_latlons
                self.input_mission_order_by_aircraft = order_per_aircraft
                self.current_input_mission_idx_by_aircraft = {
                    aid: 0 for aid in order_per_aircraft.keys()
                }
                self._block_indices = block_indices
        elif mission_order:
            ordered_paths: list[PathDefinition] = []
            ordered_latlons: list[tuple[float, float]] = []
            for key, path_ids in mission_order.items():
                if not isinstance(path_ids, list):
                    continue
                aircraft_id = None
                if isinstance(key, (int, float)):
                    aircraft_id = int(key)
                elif isinstance(key, str):
                    aircraft_id = _label_to_aircraft_id(key)
                    if aircraft_id is None:
                        try:
                            aircraft_id = int(key)
                        except Exception:
                            aircraft_id = None
                if aircraft_id is None:
                    continue

                combined: list[dict] = []
                for pid in path_ids:
                    try:
                        pid_int = int(pid)
                    except Exception:
                        continue
                    data = flight_by_path.get(pid_int)
                    if not isinstance(data, dict):
                        continue
                    wps = _extract_waypoints(data)
                    if not wps:
                        continue
                    wps = _order_waypoints(wps)
                    for wp in wps:
                        if not isinstance(wp, dict):
                            continue
                        coord = _extract_coord(wp)
                        if coord is None:
                            continue
                        lat, lon, alt = coord
                        ordered_latlons.append((lon, lat))
                        speed = wp.get("speed")
                        try:
                            speed = float(speed) if speed is not None else None
                        except Exception:
                            speed = None
                        wp_id = wp.get("waypointID") or wp.get("WaypointID")
                        try:
                            wp_id = int(wp_id) if wp_id is not None else None
                        except Exception:
                            wp_id = None
                        hover_time = _extract_hover_time(wp)
                        loiter = _extract_loiter(wp)
                        filming = wp.get("filmingProperty")
                        combined.append(
                            {
                                "lat": lat,
                                "lon": lon,
                                "alt": alt,
                                "speed": speed,
                                "wp_id": wp_id,
                                "hover_time": hover_time,
                                "loiter": loiter,
                                "filming": filming,
                                "path_id": pid_int,
                            }
                        )
                if len(combined) < 2:
                    continue
                label = _agent_label(aircraft_id)
                airframe = _airframe_type(aircraft_id)
                ordered_paths.append(
                    PathDefinition(
                        label=label,
                        aircraft_id=aircraft_id,
                        airframe=airframe,
                        path_id=None,
                        waypoints=combined,
                    )
                )

            if ordered_paths:
                paths = ordered_paths
                all_latlons = ordered_latlons
        else:
            # Fallback: map FlightPath by pathID prefix (1..6)
            combined_by_aircraft: dict[int, list[dict]] = {}
            for pid, data in flight_by_path.items():
                try:
                    prefix = int(str(pid)[0])
                except Exception:
                    continue
                if prefix < 1 or prefix > 6:
                    continue
                aircraft_id = prefix
                wps = _extract_waypoints(data)
                if not wps:
                    continue
                wps = _order_waypoints(wps)
                combined = combined_by_aircraft.setdefault(aircraft_id, [])
                for wp in wps:
                    if not isinstance(wp, dict):
                        continue
                    coord = _extract_coord(wp)
                    if coord is None:
                        continue
                    lat, lon, alt = coord
                    all_latlons.append((lon, lat))
                    speed = wp.get("speed")
                    try:
                        speed = float(speed) if speed is not None else None
                    except Exception:
                        speed = None
                    wp_id = wp.get("waypointID") or wp.get("WaypointID")
                    try:
                        wp_id = int(wp_id) if wp_id is not None else None
                    except Exception:
                        wp_id = None
                    hover_time = _extract_hover_time(wp)
                    loiter = _extract_loiter(wp)
                    filming = wp.get("filmingProperty")
                    combined.append(
                        {
                            "lat": lat,
                            "lon": lon,
                            "alt": alt,
                            "speed": speed,
                            "wp_id": wp_id,
                            "hover_time": hover_time,
                            "loiter": loiter,
                            "filming": filming,
                            "path_id": pid,
                        }
                    )
            if combined_by_aircraft:
                for aircraft_id, combined in combined_by_aircraft.items():
                    if len(combined) < 2:
                        continue
                    label = _agent_label(aircraft_id)
                    airframe = _airframe_type(aircraft_id)
                    paths.append(
                        PathDefinition(
                            label=label,
                            aircraft_id=aircraft_id,
                            airframe=airframe,
                            path_id=None,
                            waypoints=combined,
                        )
                    )

        if not paths:
            return {"ok": False, "error": "no valid flight paths"}

        if not all_latlons:
            return {"ok": False, "error": "waypoints missing coordinates"}

        lon_avg = sum(lon for lon, _ in all_latlons) / len(all_latlons)
        lat_avg = sum(lat for _, lat in all_latlons) / len(all_latlons)

        spawn_latlon: dict[int, tuple[float, float, float]] = {}
        for item in take_over_list:
            if not isinstance(item, dict):
                continue
            try:
                aircraft_id = int(item.get("aircraftID") or item.get("AircraftID") or 0)
            except Exception:
                aircraft_id = 0
            if aircraft_id <= 0:
                continue
            coord = _extract_coord(item)
            if coord is None:
                continue
            lat, lon, alt = coord
            spawn_latlon[aircraft_id] = (lat, lon, float(alt) if alt is not None else 0.0)

        with self._lock:
            self.geo = GeoConverter(lon_avg, lat_avg)
            geo = self.geo
            spawn_by_aircraft: dict[int, tuple[float, float, float]] = {}
            if geo and spawn_latlon:
                for aid, (lat, lon, alt) in spawn_latlon.items():
                    x, y = geo.lonlat_to_xy(lon, lat)
                    spawn_by_aircraft[aid] = (x, y, alt)
                # LAH spawns: 300m south of matching UAV if missing
                for lah_id in (1, 2, 3):
                    if lah_id in spawn_by_aircraft:
                        continue
                    uav_spawn = spawn_by_aircraft.get(lah_id + 3)
                    if uav_spawn is None:
                        continue
                    ux, uy, uz = uav_spawn
                    spawn_by_aircraft[lah_id] = (ux, uy - 300.0, uz)
            self._spawn_by_aircraft = spawn_by_aircraft
            self._paths = paths
            self._build_vehicles(paths)
            self.running = False
            self.paused = True
            self.sim_time = 0.0
            self.step_count = 0
            self.last_error = None
            if not seq_by_aircraft:
                self.input_mission_order_by_aircraft = {}
                self.current_input_mission_idx_by_aircraft = {}
                self._block_indices = {}
                self._spawn_by_aircraft = spawn_by_aircraft

        return {"ok": True, "count": len(paths)}

    def _ground_height(self, x: float, y: float) -> float:
        # Flat ground assumption for filming targets; replace with DEM if available.
        return 0.0

    def _default_downward_target(self, vehicle) -> tuple[float, float, float]:
        return (float(vehicle.s.x), float(vehicle.s.y), float(self._ground_height(vehicle.s.x, vehicle.s.y)))

    def _get_operation_handler(self, mode_id: int | None) -> OperationMode | None:
        if mode_id is None:
            return None
        handler = self._operation_handlers.get(mode_id)
        if handler is not None:
            return handler
        try:
            handler = build_operation_mode(mode_id)
        except Exception:
            return None
        self._operation_handlers[mode_id] = handler
        return handler

    def _update_filming_target(self, simv: SimVehicle, dt: float) -> None:
        geo = self.geo
        if geo is None:
            return
        label = simv.label
        controller = simv.controller
        tgt = controller.current_target()
        filming_prop = tgt.filming if tgt else None
        current_wp_id = tgt.wp_id if tgt else None

        if filming_prop is None:
            self._line_search_state[label] = None
            self._filming_props[label] = None
            self._filming_wp_ids[label] = current_wp_id
            self._filming_targets[label] = self._default_downward_target(simv.vehicle)
            return

        handler = self._get_operation_handler(filming_prop.get("operationMode"))
        if handler is None:
            self._line_search_state[label] = None
            self._filming_props[label] = filming_prop
            self._filming_wp_ids[label] = current_wp_id
            self._filming_targets[label] = self._default_downward_target(simv.vehicle)
            return

        ctx = OperationContext(
            geo=geo,
            ground_height_fn=self._ground_height,
            default_target_fn=self._default_downward_target,
        )
        prev_state = self._line_search_state.get(label)
        try:
            result = handler.apply(
                uav=simv.vehicle,
                filming_prop=filming_prop,
                ctx=ctx,
                dt=float(dt or 0.0),
                current_wp_id=current_wp_id,
                prev_state=prev_state,
            )
        except Exception:
            self._line_search_state[label] = None
            self._filming_props[label] = filming_prop
            self._filming_wp_ids[label] = current_wp_id
            self._filming_targets[label] = self._default_downward_target(simv.vehicle)
            return

        self._filming_props[label] = filming_prop
        self._filming_wp_ids[label] = current_wp_id
        self._filming_targets[label] = result.target or self._default_downward_target(simv.vehicle)
        self._line_search_state[label] = result.state
        if result.reset_debug or result.debug is not None:
            self._line_search_debug[label] = result.debug

    def _build_vehicles(self, paths: list[PathDefinition]) -> None:
        geo = self.geo
        if geo is None:
            return

        uav_db = Path(__file__).resolve().parent / "controllers" / "uav_pid_db.json"
        lah_db = Path(__file__).resolve().parent / "controllers" / "lah_pid_db.json"

        vehicles: dict[str, SimVehicle] = {}
        self._filming_props = {}
        self._filming_targets = {}
        self._filming_wp_ids = {}
        self._line_search_state = {}
        self._line_search_debug = {}
        spawn_by_aircraft = self._spawn_by_aircraft or {}
        for path in paths:
            wp_targets: list[WaypointTarget] = []
            for wp in path.waypoints:
                lat = wp.get("lat")
                lon = wp.get("lon")
                alt = wp.get("alt")
                if lat is None or lon is None:
                    continue
                x, y = geo.lonlat_to_xy(lon, lat)
                z = float(alt) if alt is not None else 0.0
                wp_targets.append(
                    WaypointTarget(
                        pos=(x, y, z),
                        speed=wp.get("speed"),
                        wp_id=wp.get("wp_id"),
                        hover_time=wp.get("hover_time"),
                        loiter=wp.get("loiter"),
                        filming=wp.get("filming"),
                        input_mission_id=wp.get("input_mission_id"),
                        individual_mission_id=wp.get("individual_mission_id"),
                        path_id=wp.get("path_id") or path.path_id,
                    )
                )

            if len(wp_targets) < 2:
                continue

            if path.airframe == "lah":
                vehicle = LAH(LAHParams())
                speed_target = self.speed_lah
                gains = load_pid_gains_for_time_scale(lah_db, self.time_scale)
                allow_hover = True
            else:
                vehicle = UAV(UAVParams())
                speed_target = self.speed_uav
                gains = load_pid_gains_for_time_scale(uav_db, self.time_scale)
                allow_hover = False

            first = wp_targets[0].pos
            spawn = spawn_by_aircraft.get(path.aircraft_id)
            if spawn is not None:
                vehicle.s.x = float(spawn[0])
                vehicle.s.y = float(spawn[1])
                vehicle.s.z = float(spawn[2])
                dx = first[0] - vehicle.s.x
                dy = first[1] - vehicle.s.y
                if abs(dx) + abs(dy) > 1e-6:
                    vehicle.s.yaw = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0
                elif len(wp_targets) >= 2:
                    dx = wp_targets[1].pos[0] - first[0]
                    dy = wp_targets[1].pos[1] - first[1]
                    if abs(dx) + abs(dy) > 1e-6:
                        vehicle.s.yaw = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0
            else:
                vehicle.s.x = float(first[0])
                vehicle.s.y = float(first[1])
                vehicle.s.z = float(first[2])
                if len(wp_targets) >= 2:
                    dx = wp_targets[1].pos[0] - first[0]
                    dy = wp_targets[1].pos[1] - first[1]
                    if abs(dx) + abs(dy) > 1e-6:
                        vehicle.s.yaw = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0
            init_speed = wp_targets[0].speed if wp_targets[0].speed is not None else speed_target
            if init_speed is not None:
                try:
                    vehicle.s.u = float(init_speed)
                except Exception:
                    pass

            controller = WaypointPIDController(
                vehicle,
                wp_targets,
                gains=gains,
                speed_target=float(speed_target),
                pos_tol=float(self.pos_tol),
                name=path.label,
                allow_hover=allow_hover,
            )
            block = self._block_indices.get(path.aircraft_id)
            if block:
                try:
                    controller.set_block_indices(block)
                except Exception:
                    pass

            vehicles[path.label] = SimVehicle(
                label=path.label,
                aircraft_id=path.aircraft_id,
                airframe=path.airframe,
                vehicle=vehicle,
                controller=controller,
                path_id=path.path_id,
            )

        self.vehicles = vehicles
        for simv in self.vehicles.values():
            self._update_filming_target(simv, 0.0)

    def _step_once(self, dt: float) -> None:
        for simv in self.vehicles.values():
            try:
                simv.controller.update(dt)
                self._update_filming_target(simv, dt)
                simv.vehicle.step(dt)
            except Exception:
                continue

    def _loop(self) -> None:
        last = time.perf_counter()
        accum = 0.0
        while not self._shutdown.is_set():
            with self._lock:
                running = self.running
                paused = self.paused
                speed = float(self.speed_factor)
                dt = float(self.dt)
                vehicles_ready = bool(self.vehicles)

            if not running or paused or not vehicles_ready:
                last = time.perf_counter()
                self._shutdown.wait(0.02)
                continue

            now = time.perf_counter()
            wall_dt = max(0.0, now - last)
            last = now

            accum += wall_dt * speed
            advanced = 0
            while accum >= dt:
                with self._lock:
                    if not (self.running and not self.paused and self.vehicles):
                        accum = 0.0
                        break
                    self._step_once(dt)
                    self.sim_time += dt
                    self.step_count += 1
                accum -= dt
                advanced += 1

            if advanced == 0:
                self._shutdown.wait(0.005)

    def build_snapshot(self) -> dict:
        with self._lock:
            geo = self.geo
            vehicles = list(self.vehicles.values())
            running = self.running
            paused = self.paused
            speed = float(self.speed_factor)
            dt = float(self.dt)
            sim_time = float(self.sim_time)
            step_count = int(self.step_count)
            error = self.last_error

        payload: dict[str, Any] = {
            "ok": True,
            "running": running,
            "paused": paused,
            "speedFactor": speed,
            "dt": dt,
            "simTime": sim_time,
            "step": step_count,
            "vehicles": {},
        }
        if error:
            payload["error"] = error

        if geo is None:
            return payload

        for simv in vehicles:
            s = simv.vehicle.s
            lon, lat = geo.xy_to_lonlat(s.x, s.y)
            payload["vehicles"][simv.label] = {
                "lat": float(lat),
                "lon": float(lon),
                "alt": float(s.z),
                "speed": float(getattr(s, "u", 0.0)),
                "heading": float(getattr(s, "yaw", 0.0)),
            }
            target = self._filming_targets.get(simv.label)
            if target is not None:
                t_lon, t_lat = geo.xy_to_lonlat(target[0], target[1])
                payload["vehicles"][simv.label]["filmingTarget"] = {
                    "lat": float(t_lat),
                    "lon": float(t_lon),
                    "alt": float(target[2]),
                }
            filming_prop = self._filming_props.get(simv.label)
            if isinstance(filming_prop, dict):
                payload["vehicles"][simv.label]["filmingMode"] = filming_prop.get("operationMode")
                fov = filming_prop.get("fieldOfView")
                if isinstance(fov, (int, float)):
                    payload["vehicles"][simv.label]["filmingFov"] = float(fov)
            if simv.label in self._filming_wp_ids:
                payload["vehicles"][simv.label]["filmingWpId"] = self._filming_wp_ids.get(simv.label)

        return payload
