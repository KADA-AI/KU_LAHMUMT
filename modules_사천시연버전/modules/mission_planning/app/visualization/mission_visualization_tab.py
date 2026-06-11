"""Mission plan map visualization tab."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable

import folium
from PyQt5.QtCore import QUrl
from PyQt5.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
except Exception:  # pragma: no cover - depends on local Qt install
    QWebEngineView = None


def _default_map_html_path() -> Path:
    project_root = Path(__file__).resolve().parents[4]
    temp_dir = project_root / "temp"
    temp_dir.mkdir(parents=True, exist_ok=True)
    return temp_dir / "mission_planning_map.html"


class MissionVisualizationTab(QWidget):
    def __init__(
        self,
        plan_id_provider: Callable[[], object],
        db_root_provider: Callable[[], Path],
        log_cb: Callable[[str], None] | None,
        parent=None,
        *,
        map_html_path: Path | None = None,
    ):
        super().__init__(parent)
        self._plan_id_provider = plan_id_provider
        self._db_root_provider = db_root_provider
        self._log = log_cb or (lambda _msg: None)
        self._map_view_state: dict | None = None
        self._map_html_path = Path(map_html_path) if map_html_path is not None else _default_map_html_path()
        self._show_waypoints = True
        self._show_geometry = True
        self._map_view: QWebEngineView | None = None

        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        self._info_label = QLabel("plan_ids: -")
        self._btn_refresh = QPushButton("Refresh")
        self._btn_refresh.clicked.connect(self.refresh)
        self._chk_wps = QCheckBox("Show WPs")
        self._chk_wps.setChecked(True)
        self._chk_wps.toggled.connect(self._on_toggle_options)
        self._chk_geo = QCheckBox("Show Mission Geometry")
        self._chk_geo.setChecked(True)
        self._chk_geo.toggled.connect(self._on_toggle_options)
        row.addWidget(self._btn_refresh)
        row.addWidget(self._chk_wps)
        row.addWidget(self._chk_geo)
        row.addStretch(1)
        row.addWidget(self._info_label)
        layout.addLayout(row)

        if QWebEngineView is None:
            layout.addWidget(QLabel("QtWebEngine not available."))
        else:
            self._map_view = QWebEngineView()
            self._map_view.loadFinished.connect(self._on_map_load_finished)
            layout.addWidget(self._map_view)
            self._build_map()

    def _log_line(self, msg: str) -> None:
        try:
            self._log(f"[VIS] {msg}")
        except Exception:
            pass

    def _on_toggle_options(self, checked: bool) -> None:
        self._show_waypoints = bool(self._chk_wps.isChecked())
        self._show_geometry = bool(self._chk_geo.isChecked())
        self.refresh()

    def refresh(self) -> None:
        if not self._map_view:
            return
        self._capture_map_view_state(self._reload_map_content)

    def _build_map(self) -> None:
        if not self._map_view:
            return
        self._write_map_html()
        self._map_view.setUrl(QUrl.fromLocalFile(str(self._map_html_path)))

    def _reload_map_content(self) -> None:
        self._write_map_html()
        try:
            if self._map_view:
                self._map_view.reload()
        except Exception:
            pass

    def _capture_map_view_state(self, callback=None) -> None:
        if not self._map_view:
            if callback:
                callback()
            return
        script = """
            (function() {
                var map = null;
                for (var k in window) {
                    if (window[k] instanceof L.Map) { map = window[k]; break; }
                }
                if (!map) { return null; }
                var c = map.getCenter();
                return {lat: c.lat, lng: c.lng, zoom: map.getZoom()};
            })();
        """

        def _store_view(result):
            if isinstance(result, dict) and "lat" in result and "lng" in result:
                self._map_view_state = result
            if callback:
                callback()

        try:
            self._map_view.page().runJavaScript(script, _store_view)
        except Exception:
            if callback:
                callback()

    def _on_map_load_finished(self, ok: bool) -> None:
        if not ok or not self._map_view_state:
            return
        lat = float(self._map_view_state.get("lat", 0.0))
        lng = float(self._map_view_state.get("lng", 0.0))
        zoom = int(self._map_view_state.get("zoom", 14))
        script = f"""
            (function() {{
                var map = null;
                for (var k in window) {{
                    if (window[k] instanceof L.Map) {{ map = window[k]; break; }}
                }}
                if (map) {{
                    map.setView([{lat}, {lng}], {zoom});
                }}
            }})();
        """
        try:
            if self._map_view:
                self._map_view.page().runJavaScript(script)
        except Exception:
            pass

    def _resolve_plan_ids(self, db_root: Path) -> list[int]:
        raw = []
        try:
            raw = list(self._plan_id_provider() or [])
        except Exception:
            raw = []
        plan_ids: list[int] = []
        for val in raw:
            try:
                plan_ids.append(int(val))
            except Exception:
                continue
        if plan_ids:
            return plan_ids
        dir_mp = db_root / "MissionPlan"
        if not dir_mp.exists():
            return []
        candidates = sorted(dir_mp.glob("*.json"), key=lambda p: p.stat().st_mtime)
        if not candidates:
            return []
        latest = candidates[-1]
        try:
            with latest.open(encoding="utf-8") as fh:
                data = json.load(fh)
            val = data.get("missionPlanID")
            if val is not None:
                return [int(val)]
        except Exception:
            pass
        if latest.stem.isdigit():
            return [int(latest.stem)]
        return []

    def _coords_from_list(self, items) -> list[tuple[float, float]]:
        coords = []
        for item in items or []:
            lat = item.get("latitude")
            lon = item.get("longitude")
            if lat is None or lon is None:
                continue
            coords.append((float(lat), float(lon)))
        return coords

    def _collect_plan_data(self, plan_ids: list[int], db_root: Path) -> dict:
        dir_mp = db_root / "MissionPlan"
        dir_imp = db_root / "IndividualMissionPlan"
        dir_fp = db_root / "FlightPath"
        path_meta: dict[int, dict] = {}
        line_geoms: list[dict] = []
        area_geoms: list[dict] = []
        points: list[tuple[float, float]] = []

        for plan_id in plan_ids:
            mp_path = dir_mp / f"{int(plan_id)}.json"
            if not mp_path.exists():
                continue
            try:
                mp_json = json.loads(mp_path.read_text(encoding="utf-8"))
            except Exception:
                continue
            for entry in mp_json.get("aircraftList", []):
                aid = entry.get("aircraftID")
                imp_id = entry.get("individualMissionPackageID") or entry.get("individualMissionPlanPackageID")
                if imp_id is None:
                    continue
                imp_path = dir_imp / f"{int(imp_id)}.json"
                if not imp_path.exists():
                    continue
                try:
                    imp_json = json.loads(imp_path.read_text(encoding="utf-8"))
                except Exception:
                    continue
                aircraft_id = imp_json.get("aircraftID", aid)
                for im in imp_json.get("individualMissionList", []):
                    path_id = im.get("pathID")
                    if path_id is not None:
                        try:
                            path_meta[int(path_id)] = {
                                "aircraftID": aircraft_id,
                                "missionID": im.get("individualMissionID"),
                                "missionType": im.get("individualMissionInfo", {}).get("individualMissionType"),
                            }
                        except Exception:
                            pass
                    info = im.get("individualMissionInfo", {}) or {}
                    for line in info.get("lineList", []) or []:
                        coords = self._coords_from_list(line.get("coordinateList"))
                        if len(coords) >= 2:
                            line_geoms.append(
                                {
                                    "coords": coords,
                                    "aircraftID": aircraft_id,
                                    "missionID": im.get("individualMissionID"),
                                    "pathID": path_id,
                                }
                            )
                            points.extend(coords)
                    for area in info.get("areaList", []) or []:
                        coords = self._coords_from_list(area.get("coordinateList"))
                        if len(coords) >= 3:
                            area_geoms.append(
                                {
                                    "coords": coords,
                                    "aircraftID": aircraft_id,
                                    "missionID": im.get("individualMissionID"),
                                    "pathID": path_id,
                                }
                            )
                            points.extend(coords)

        paths: list[dict] = []
        for path_id, meta in path_meta.items():
            fp_path = dir_fp / f"{int(path_id)}.json"
            if not fp_path.exists():
                continue
            try:
                fp_json = json.loads(fp_path.read_text(encoding="utf-8"))
            except Exception:
                continue
            waypoints = fp_json.get("waypointList") or fp_json.get("lahWaypointList") or []
            coords = []
            for wp in waypoints:
                coord = (wp or {}).get("coordinate") or {}
                lat = coord.get("latitude")
                lon = coord.get("longitude")
                if lat is None or lon is None:
                    continue
                coords.append((float(lat), float(lon)))
            if len(coords) >= 2:
                paths.append(
                    {
                        "pathID": path_id,
                        "aircraftID": fp_json.get("aircraftID", meta.get("aircraftID")),
                        "missionID": meta.get("missionID"),
                        "coords": coords,
                        "waypoints": waypoints,
                    }
                )
                points.extend(coords)

        return {
            "paths": paths,
            "lines": line_geoms,
            "areas": area_geoms,
            "points": points,
        }

    def _write_map_html(self) -> None:
        db_root = self._db_root_provider()
        plan_ids = self._resolve_plan_ids(db_root)
        label = ", ".join(str(pid) for pid in plan_ids) if plan_ids else "-"
        self._info_label.setText(f"plan_ids: {label}")

        data = self._collect_plan_data(plan_ids, db_root)
        points = data.get("points") or []
        state = self._map_view_state or {}
        if points:
            avg_lat = sum(p[0] for p in points) / len(points)
            avg_lon = sum(p[1] for p in points) / len(points)
        else:
            avg_lat = float(state.get("lat", 38.128774))
            avg_lon = float(state.get("lng", 127.318005))
        zoom = int(state.get("zoom", 13))

        fmap = folium.Map(location=[avg_lat, avg_lon], zoom_start=zoom)
        color_map = {1: "green", 2: "orange", 3: "purple", 4: "red", 5: "blue", 6: "brown"}

        if self._show_geometry:
            for line in data.get("lines", []):
                coords = line["coords"]
                aid = line.get("aircraftID")
                color = color_map.get(aid, "gray")
                label = f"A{aid} M{line.get('missionID')} P{line.get('pathID')}"
                folium.PolyLine(coords, color=color, weight=2, dash_array="6,6", tooltip=label).add_to(fmap)
            for area in data.get("areas", []):
                coords = area["coords"]
                aid = area.get("aircraftID")
                color = color_map.get(aid, "gray")
                label = f"A{aid} M{area.get('missionID')} P{area.get('pathID')}"
                folium.Polygon(coords, color=color, weight=2, fill=True, fill_opacity=0.2, tooltip=label).add_to(fmap)

        for path in data.get("paths", []):
            aid = path.get("aircraftID")
            color = color_map.get(aid, "gray")
            label = f"A{aid} M{path.get('missionID')} P{path.get('pathID')}"
            folium.PolyLine(path["coords"], color=color, weight=3, tooltip=label).add_to(fmap)
            if self._show_waypoints:
                for idx, wp in enumerate(path.get("waypoints") or []):
                    coord = (wp or {}).get("coordinate") or {}
                    lat = coord.get("latitude")
                    lon = coord.get("longitude")
                    if lat is None or lon is None:
                        continue
                    folium.CircleMarker(
                        [float(lat), float(lon)],
                        radius=2,
                        color=color,
                        fill=True,
                        fill_opacity=1.0,
                        tooltip=f"WP {idx + 1}",
                    ).add_to(fmap)

        try:
            fmap.save(str(self._map_html_path))
        except Exception as exc:
            self._log_line(f"map save failed: {exc}")
