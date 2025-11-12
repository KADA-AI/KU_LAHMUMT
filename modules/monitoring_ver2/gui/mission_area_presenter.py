# -*- coding: utf-8 -*-
"""Rendering helper for MissionAreaMonitoringTab."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, TYPE_CHECKING

from modules.common import db_paths
from modules.monitoring_ver2.config import SYSTEM_MODE_LABELS

if TYPE_CHECKING:
    from modules.monitoring_ver2.gui.tabs.MissionAreaMonitoringTab import MissionMapView


class MissionAreaPresenter:
    """Separates mission-area visualization logic from the tab widget."""

    # System mode 2 = "초기 임무 재계획 모드"
    INITIAL_PLANNING_MODES = {2}
    MISSION_EXECUTION_MODE = 3
    CURRENT_PLAN_MODES = INITIAL_PLANNING_MODES | {MISSION_EXECUTION_MODE}
    MAX_SENSOR_FOOTPRINTS = 600

    CUMULATIVE_COLORS = [
        ("#2563eb", "#dbeafe"),
        ("#dc2626", "#fee2e2"),
        ("#0d9488", "#ccfbf1"),
        ("#ca8a04", "#fef9c3"),
        ("#9333ea", "#f3e8ff"),
    ]
    CURRENT_PLAN_COLORS = [
        {"stroke": "#0f172a", "fill": "#cbd5f5", "path": "#2563eb"},
        {"stroke": "#115e59", "fill": "#ccfbf1", "path": "#0d9488"},
        {"stroke": "#7c2d12", "fill": "#fed7aa", "path": "#ea580c"},
        {"stroke": "#7e22ce", "fill": "#f5d0fe", "path": "#a855f7"},
        {"stroke": "#b45309", "fill": "#fef9c3", "path": "#f97316"},
    ]
    FILMING_MISSION_TYPES = {9}
    FILMING_PATTERN_TYPES = {12}
    SENSOR_FOOTPRINT_STYLE = {
        "opacity": 0.35,
        "width": 1.8,
        "z": 8.0,
    }
    FOOTPRINT_COLOR_PALETTE = [
        "#f97316",
        "#22c55e",
        "#0ea5e9",
        "#a855f7",
        "#eab308",
        "#ef4444",
    ]

    def __init__(
        self,
        manager,
        cumulative_view: "MissionMapView",
        current_view: "MissionMapView | None" = None,
        display_options: Optional[Dict[str, bool]] = None,
    ) -> None:
        self.manager = manager
        self.cumulative_view = cumulative_view
        self.current_view = current_view
        self._latest_0201: Any = None
        self._latest_0301: Any = None
        self._current_mode: Optional[int] = None
        self._latest_input_plan_id: Optional[int] = None
        self._latest_mission_plan_id: Optional[int] = None
        self._mission_plan_cache: Dict[Tuple[str, int], Dict[str, Any]] = {}
        self._cache_root: Optional[str] = None
        self.display_options = {
            "collab": True,
            "individual": True,
            "routes": True,
            "filming": True,
        }
        if display_options:
            self.display_options.update(display_options)
        self._footprint_history: List[Dict[str, Any]] = []
        self._footprint_color_map: Dict[int, str] = {}

    # ------------------------------------------------------------------ Public API
    def on_system_mode(self, mode_value: Optional[int]) -> None:
        previous_mode = self._current_mode
        self._current_mode = mode_value
        if (
            mode_value != self.MISSION_EXECUTION_MODE
            or previous_mode != self.MISSION_EXECUTION_MODE
        ):
            self._footprint_history.clear()
        self._redraw()

    def handle_update(self, source: Optional[str], key: Optional[str], payload: Any) -> None:
        if key == "SystemMode":
            return
        if key == "0201" and source == "receive":
            self._latest_0201 = payload or self._resolve_payload("0201")
            plan_id = self._extract_id(self._latest_0201, "inputMissionPackageID")
            if plan_id is not None:
                self._latest_input_plan_id = plan_id
            self._redraw()
        elif key in {"0301", "0302", "0303", "0304"} and source == "receive":
            if key == "0301":
                self._latest_0301 = payload or self._resolve_payload("0301")
                plan_id = self._extract_id(self._latest_0301, "missionPlanID")
            if plan_id is not None:
                self._latest_mission_plan_id = plan_id
            self._redraw()
        elif key == "0401" and source == "receive":
            if self._capture_sensor_footprints(payload):
                self._redraw()

    def update_display_options(self, options: Dict[str, bool]) -> None:
        self.display_options.update(options)
        self._redraw()

    def _redraw(self) -> None:
        if self.cumulative_view is None:
            return
        self.cumulative_view.clear_overlays()
        self._render_cumulative_plan()
        self._render_current_plan()
        self._render_sensor_footprints()

    # ------------------------------------------------------------------ Rendering (0201 ??mission accumulation)
    def _render_cumulative_plan(self) -> None:
        if not self.display_options.get("collab", True):
            return
        data = self._materialize_input_plan()
        if not data:
            return

        mission_list = data.get("inputMissionList") or []
        if not mission_list:
            return

        for seq, mission in enumerate(mission_list, start=1):
            detail = (mission or {}).get("missionDetail") or {}
            color_idx = (seq - 1) % len(self.CUMULATIVE_COLORS)
            stroke, fill = self.CUMULATIVE_COLORS[color_idx]
            label_point: Optional[Tuple[float, float]] = None

            for area in detail.get("areaList") or []:
                coords = self._coord_list(area.get("coordinateList"))
                if len(coords) < 3:
                    continue
                self.cumulative_view.add_polygon(coords, stroke=stroke, fill=fill, width=2.2, opacity=0.45)
                label_point = label_point or self._centroid(coords)

            for line in detail.get("lineList") or []:
                coords = self._coord_list(line.get("coordinateList"))
                if len(coords) < 2:
                    continue
                polygon = self._line_to_polygon(coords, line.get("width"))
                if polygon:
                    self.cumulative_view.add_polygon(polygon, stroke=stroke, fill=fill, width=1.5, opacity=0.35)
                    label_point = label_point or self._centroid(polygon)
                else:
                    width_px = self._line_pen_width(line.get("width"))
                    self.cumulative_view.add_polyline(coords, color=stroke, width=width_px)
                    label_point = label_point or self._midpoint(coords)

            coord_list = self._coord_list(detail.get("coordinateList"))
            if len(coord_list) >= 2:
                self.cumulative_view.add_polyline(coord_list, color=stroke, width=2.0, dash_pattern=[3, 3])
                label_point = label_point or self._midpoint(coord_list)
            elif len(coord_list) == 1:
                pt = coord_list[0]
                self.cumulative_view.add_point(pt[0], pt[1], radius=4, stroke=stroke, fill=fill)
                label_point = label_point or pt

            if label_point:
                self.cumulative_view.add_label(label_point[0], label_point[1], str(seq))

    # ------------------------------------------------------------------ Rendering (0301 ??current missions)
    def _render_current_plan(self) -> None:
        if not self._should_render_current():
            return
        options = self.display_options
        if not (options.get("individual", True) or options.get("filming", True) or options.get("routes", True)):
            return

        if self._latest_0301 is None:
            self._latest_0301 = self._resolve_payload("0301")
        payload = self._to_dict(self._latest_0301)
        plan_id = self._latest_mission_plan_id
        if plan_id is None:
            plan_id = self._safe_int((payload or {}).get("missionPlanID"))
        if plan_id is None:
            return

        bundle = self._collect_plan_geometry(plan_id)
        if not bundle:
            return

        view = self.cumulative_view
        for idx, aircraft_entry in enumerate(bundle):
            colors = self.CURRENT_PLAN_COLORS[idx % len(self.CURRENT_PLAN_COLORS)]
            stroke = colors["stroke"]
            fill = colors["fill"]
            for segment in aircraft_entry["missions"]:
                category = segment.get("category", "individual")
                if category == "filming" and not options.get("filming", True):
                    continue
                if category == "individual" and not options.get("individual", True):
                    continue
                points = segment.get("points")
                if segment["kind"] == "area" and points:
                    view.add_polygon(points, stroke=stroke, fill=fill, width=2.0, opacity=0.35)
                elif segment["kind"] == "line" and points:
                    view.add_polyline(
                        points,
                        color=stroke,
                        width=segment.get("width", 2.0),
                        dash_pattern=segment.get("dash"),
                    )
                elif segment["kind"] == "point":
                    lat, lon = segment["point"]
                    view.add_point(lat, lon, radius=4.5, stroke=stroke, fill=fill)

                label_pt = segment.get("label_point")
                if label_pt:
                    view.add_label(label_pt[0], label_pt[1], segment["label"], color=stroke)

            if options.get("routes", True):
                for path_points in aircraft_entry.get("paths", []):
                    for wp in path_points:
                        lat, lon = wp["position"]
                        label = wp.get("label")
                        view.add_point(
                            lat,
                            lon,
                            radius=1.25,
                            stroke=colors["path"],
                            fill="#ffffff",
                            z_value=7,
                            stroke_width=0,
                        )
                        if label:
                            view.add_label(lat, lon, label, color=colors["path"], font_size=8)

    def _render_sensor_footprints(self) -> None:
        if self.cumulative_view is None:
            return
        if not self.display_options.get("filming", True):
            return
        if not self._footprint_history:
            return

        style = self.SENSOR_FOOTPRINT_STYLE
        for entry in self._footprint_history:
            polygon = entry.get("points")
            if not polygon:
                continue
            aircraft_id = entry.get("aircraft")
            color = self._footprint_style_for(aircraft_id)
            self.cumulative_view.add_polygon(
                polygon,
                stroke=color,
                fill=color,
                width=style["width"],
                opacity=style["opacity"],
                z_value=style["z"],
            )

    def _capture_sensor_footprints(self, payload: Any) -> bool:
        if self.cumulative_view is None:
            return False
        if self._current_mode != self.MISSION_EXECUTION_MODE:
            return False

        data = self._to_dict(payload)
        if not data:
            return False

        added = False
        for agent_entry in data.get("agentStateList") or []:
            unmanned_info = self._get(agent_entry, "unmannedInfo")
            if not unmanned_info:
                continue
            sensor_info = self._get(unmanned_info, "sensorInfo")
            polygon = self._footprint_polygon(sensor_info)
            if not polygon:
                continue
            aircraft_id = self._safe_int(self._get(agent_entry, "aircraftID"))
            self._footprint_history.append({"points": polygon, "aircraft": aircraft_id})
            added = True

        if added and len(self._footprint_history) > self.MAX_SENSOR_FOOTPRINTS:
            overflow = len(self._footprint_history) - self.MAX_SENSOR_FOOTPRINTS
            del self._footprint_history[:overflow]
        return added

    def _footprint_polygon(self, sensor_info: Any) -> Optional[List[Tuple[float, float]]]:
        if not sensor_info:
            return None
        corners = self._get(sensor_info, "footprintCornerList") or []
        coords: List[Tuple[float, float]] = []
        for corner in corners:
            coord = self._coord_tuple(corner)
            if coord:
                coords.append(coord)
        if len(coords) < 3:
            return None
        if self._is_degenerate_polygon(coords):
            return None
        return coords

    # ------------------------------------------------------------------ Helpers
    def _materialize_input_plan(self) -> Optional[Dict[str, Any]]:
        if self._latest_0201 is None:
            self._latest_0201 = self._resolve_payload("0201")
        data = self._to_dict(self._latest_0201)
        plan_id = self._latest_input_plan_id
        if plan_id is None and data is not None:
            plan_id = self._safe_int(data.get("inputMissionPackageID"))
        if data and data.get("inputMissionList"):
            return data
        if plan_id is not None:
            file_data = self._load_json("InputMissionPlan", plan_id)
            if file_data:
                return file_data
        return data

    def _collect_plan_geometry(self, mission_plan_id: int) -> List[Dict[str, Any]]:
        plan = self._load_json("MissionPlan", mission_plan_id)
        if not plan:
            return []

        dataset: List[Dict[str, Any]] = []
        for aircraft in plan.get("aircraftList") or []:
            aircraft_id = aircraft.get("aircraftID")
            individual_pkg = aircraft.get("individualMissionPackageID")
            if individual_pkg is None:
                continue
            individual_plan = self._load_json("IndividualMissionPlan", individual_pkg)
            missions = []
            for mission in (individual_plan or {}).get("individualMissionList") or []:
                label = str(
                    mission.get("relatedMission", {}).get("inputMissionID")
                    or mission.get("individualMissionID")
                    or "?"
                )
                mission_info = mission.get("individualMissionInfo") or {}
                mission_type = mission_info.get("individualMissionType")
                pattern_type = mission_info.get("patternType")
                is_filming = (mission_type in self.FILMING_MISSION_TYPES) or (pattern_type in self.FILMING_PATTERN_TYPES)
                category = "filming" if is_filming else "individual"
                segments = self._extract_segments(mission_info, category=category)
                for seg in segments:
                    seg["label"] = label
                    missions.append(seg)

            path_sets: List[List[Dict[str, Tuple[float, float]]]] = []
            for mission in (individual_plan or {}).get("individualMissionList") or []:
                path_id = mission.get("pathID")
                if path_id:
                    coords = self._load_path_coordinates(path_id)
                    if coords:
                        path_sets.append(coords)
            dataset.append(
                {
                    "aircraft": aircraft_id,
                    "missions": missions,
                    "paths": path_sets,
                }
            )
        return dataset

    def _extract_segments(self, info: Dict[str, Any], *, category: str) -> List[Dict[str, Any]]:
        segments: List[Dict[str, Any]] = []

        for area in info.get("areaList") or []:
            coords = self._coord_list(area.get("coordinateList"))
            if len(coords) >= 3:
                segments.append(
                    {"kind": "area", "points": coords, "label_point": self._centroid(coords), "category": category}
                )

        for line in info.get("lineList") or []:
            coords = self._coord_list(line.get("coordinateList"))
            if len(coords) >= 2:
                polygon = self._line_to_polygon(coords, line.get("width"))
                if polygon:
                    segments.append(
                        {
                            "kind": "area",
                            "points": polygon,
                            "label_point": self._centroid(polygon),
                        }
                    )
                else:
                    segments.append(
                        {
                            "kind": "line",
                            "points": coords,
                        "width": self._line_pen_width(line.get("width")),
                        "label_point": self._midpoint(coords),
                        "category": category,
                    }
                )

        coords = self._coord_list(info.get("coordinateList"))
        if len(coords) > 1:
            segments.append(
                {
                    "kind": "line",
                    "points": coords,
                    "width": 2.0,
                    "dash": [3, 3],
                    "label_point": self._midpoint(coords),
                    "category": category,
                }
            )
        elif len(coords) == 1:
            segments.append(
                {
                    "kind": "point",
                    "point": coords[0],
                    "label_point": coords[0],
                    "category": category,
                }
            )
        return segments

    def _load_json(self, subdir: str, identifier: Any) -> Optional[Dict[str, Any]]:
        try:
            ident_int = int(identifier)
        except (TypeError, ValueError):
            return None

        cache_key = (subdir, ident_int)
        cached = self._mission_plan_cache.get(cache_key)
        if cached:
            return cached

        base = self._get_active_root()
        path = base / subdir / f"{ident_int}.json"
        if not path.exists():
            self._log("WARN", f"{subdir} ?뚯씪??李얠쓣 ???놁뒿?덈떎: {path.name}")
            return None
        try:
            with path.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
                self._mission_plan_cache[cache_key] = data
                return data
        except Exception as exc:
            self._log("ERROR", f"{path.name} ?쎄린 ?ㅽ뙣: {exc}")
            return None

    def _load_path_coordinates(self, path_id: Any) -> List[Dict[str, Tuple[float, float]]]:
        data = self._load_json("FlightPath", path_id)
        if not data:
            return []
        waypoints = (
            data.get("lahWaypointList")
            or data.get("waypointList")
            or data.get("flightWaypointList")
            or []
        )
        coords: List[Dict[str, Tuple[float, float]]] = []
        for idx, wp in enumerate(waypoints, start=1):
            coord = self._coord_tuple(wp.get("coordinate") if isinstance(wp, dict) else None)
            if coord:
                label = wp.get("waypointID") if isinstance(wp, dict) else None
                if label is None:
                    label = wp.get("id") if isinstance(wp, dict) else None
                if label is None:
                    label = idx
                coords.append({"position": coord, "label": f"WP{label}" if isinstance(label, int) else str(label)})
        return coords

    def _resolve_payload(self, key: str) -> Any:
        store = getattr(self.manager, "receive_store", None)
        if store is None:
            return None
        try:
            return store.get_data(key)
        except Exception:
            return None

    def _should_render_current(self) -> bool:
        if self._current_mode is None:
            return True
        return self._current_mode in self.CURRENT_PLAN_MODES

    def _get_active_root(self) -> Path:
        root = str(db_paths.get_active_db_root())
        if self._cache_root != root:
            self._mission_plan_cache.clear()
            self._cache_root = root
        return Path(root)

    @staticmethod
    def _line_pen_width(width_meters: Any) -> float:
        try:
            width_val = float(width_meters)
        except (TypeError, ValueError):
            width_val = 600.0
        # Convert meters into a reasonable pen width (pixel space).
        return max(1.5, min(8.0, width_val / 400.0))

    @staticmethod
    def _centroid(points: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        if not points:
            return None
        lat = sum(p[0] for p in points) / len(points)
        lon = sum(p[1] for p in points) / len(points)
        return (lat, lon)

    @staticmethod
    def _midpoint(points: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        if not points:
            return None
        if len(points) == 1:
            return points[0]
        total = 0.0
        cumulative = [0.0]
        for idx in range(1, len(points)):
            seg = MissionAreaPresenter._distance(points[idx - 1], points[idx])
            total += seg
            cumulative.append(total)
        half = total / 2.0 if total else 0.0
        for idx, dist in enumerate(cumulative):
            if dist >= half:
                return points[idx]
        return points[-1]

    @staticmethod
    def _distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    @staticmethod
    def _coord_list(values: Optional[Iterable[Any]]) -> List[Tuple[float, float]]:
        coords: List[Tuple[float, float]] = []
        for item in values or []:
            coord = MissionAreaPresenter._coord_tuple(item)
            if coord:
                coords.append(coord)
        return coords

    @staticmethod
    def _coord_tuple(value: Any) -> Optional[Tuple[float, float]]:
        if value is None:
            return None
        lat = MissionAreaPresenter._get(value, "latitude")
        lon = MissionAreaPresenter._get(value, "longitude")
        if lat is None or lon is None:
            return None
        try:
            return float(lat), float(lon)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _get(obj: Any, key: str, default: Any = None) -> Any:
        if obj is None:
            return default
        if isinstance(obj, dict):
            return obj.get(key, default)
        return getattr(obj, key, default)

    @staticmethod
    def _extract_id(payload: Any, field: str) -> Optional[int]:
        if payload is None:
            return None
        if isinstance(payload, dict):
            value = payload.get(field)
        elif hasattr(payload, field):
            value = getattr(payload, field, None)
        else:
            value = MissionAreaPresenter._get(payload, field)
        return MissionAreaPresenter._safe_int(value)

    def _footprint_style_for(self, aircraft_id: Optional[int]) -> str:
        if aircraft_id is None:
            return self.FOOTPRINT_COLOR_PALETTE[0]
        if aircraft_id not in self._footprint_color_map:
            idx = len(self._footprint_color_map) % len(self.FOOTPRINT_COLOR_PALETTE)
            self._footprint_color_map[aircraft_id] = self.FOOTPRINT_COLOR_PALETTE[idx]
        return self._footprint_color_map[aircraft_id]

    @staticmethod
    def _is_degenerate_polygon(points: Sequence[Tuple[float, float]]) -> bool:
        if len(points) < 3:
            return True
        unique = {(round(lat, 6), round(lon, 6)) for lat, lon in points}
        return len(unique) < 3

    @staticmethod
    def _line_to_polygon(coords: Sequence[Tuple[float, float]], width_meters: Any) -> Optional[List[Tuple[float, float]]]:
        try:
            width = float(width_meters)
        except (TypeError, ValueError):
            width = 0.0
        if len(coords) < 2 or width <= 0:
            return None
        half = max(1.0, width / 2.0)

        lat0, lon0 = coords[0]
        meters_per_lat = 111320.0
        meters_per_lon = max(1e-6, 111320.0 * math.cos(math.radians(lat0)))

        def to_xy(lat: float, lon: float) -> Tuple[float, float]:
            return (lon - lon0) * meters_per_lon, (lat - lat0) * meters_per_lat

        def to_latlon(x: float, y: float) -> Tuple[float, float]:
            return y / meters_per_lat + lat0, x / meters_per_lon + lon0

        xy_points = [to_xy(lat, lon) for lat, lon in coords]
        left: List[Tuple[float, float]] = []
        right: List[Tuple[float, float]] = []

        for idx, (x, y) in enumerate(xy_points):
            if idx == 0:
                dx = xy_points[1][0] - x
                dy = xy_points[1][1] - y
            elif idx == len(xy_points) - 1:
                dx = x - xy_points[idx - 1][0]
                dy = y - xy_points[idx - 1][1]
            else:
                dx = xy_points[idx + 1][0] - xy_points[idx - 1][0]
                dy = xy_points[idx + 1][1] - xy_points[idx - 1][1]
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            nx = -dy / length
            ny = dx / length
            left.append((x + nx * half, y + ny * half))
            right.append((x - nx * half, y - ny * half))

        if len(left) < 2 or len(right) < 2:
            return None
        polygon_xy = left + right[::-1]
        return [to_latlon(x, y) for x, y in polygon_xy]

    @staticmethod
    def _safe_int(value: Any) -> Optional[int]:
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _to_dict(payload: Any) -> Optional[Dict[str, Any]]:
        if payload is None:
            return None
        if isinstance(payload, dict):
            return payload
        if is_dataclass(payload):
            return asdict(payload)
        attr_dict = getattr(payload, "__dict__", None)
        if attr_dict:
            return dict(attr_dict)
        return None

    def _log(self, level: str, message: str) -> None:
        log_fn = getattr(self.manager, "_log", None)
        if callable(log_fn):
            try:
                log_fn("MSM_AREA", level, message)
                return
            except TypeError:
                pass
        print(f"[MSM_AREA][{level}] {message}")

