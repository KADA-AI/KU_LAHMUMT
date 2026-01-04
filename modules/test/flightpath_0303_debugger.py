#!/usr/bin/env python3
"""
Interactive 0303 flight-path debugger.

Usage
-----
$ python modules/test/flightpath_0303_debugger.py

Left-click to add latitude/longitude points (polyline order). Press ENTER to
input corridor width (meters) and number of UAVs. The script uses the same
0303 flight-path logic (d0303.build_flight_plans) to generate:
  - flight paths (waypointList)
  - filming plans (lineSearch coordinate lists)

Press ESC to clear all points/results.
Mouse wheel zooms, right-click drag pans the view.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Tuple

import matplotlib

try:
    matplotlib.use("TkAgg")
except Exception:
    pass

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.widgets import CheckButtons
import numpy as np
import re

try:
    import rasterio
except ImportError:
    rasterio = None

try:
    from PIL import Image
except ImportError:
    Image = None


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
MISSION_PLANNER_DIR = REPO_ROOT / "modules" / "mission_planning" / "MissionPlanner"
BACKGROUND_TIF = REPO_ROOT / "resource" / "38_e127_1arc_v3.tif"
if not BACKGROUND_TIF.exists():
    matches = sorted((REPO_ROOT / "resource").glob("*38_e127_1arc_v3.tif"))
    if matches:
        BACKGROUND_TIF = matches[0]

for path in (REPO_ROOT, MISSION_PLANNER_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.append(path_str)

from modules.mission_planning.MissionPlanner.AnS.mission_pipeline import (  # type: ignore
    divide_corridor_polyline,
)
from modules.mission_planning.MissionPlanner.data_def import d0303  # type: ignore


BOUND_PAD_DEG = 0.0005
CRUISE_SPEED = 40.0
TURN_STEP_DEG = 15.0

PointLL = Tuple[float, float]  # (lat, lon)


class FlightPlanDebugger:
    def __init__(self) -> None:
        self.points: List[PointLL] = []
        self.strip_polygons: List[List[dict]] = []
        self.flight_plans_by_algo: dict[str, List[dict]] = {}
        self.filming_lines_by_algo: dict[str, List[Tuple[int, List[dict]]]] = {}
        self.last_width: float | None = None
        self.last_uav_cnt: int | None = None
        self.view_xlim: Tuple[float, float] | None = None
        self.view_ylim: Tuple[float, float] | None = None
        self.view_fixed: bool = False
        self.dem_image: np.ndarray | None = None
        self.dem_extent: Tuple[float, float, float, float] | None = None
        self.show_filming: bool = True

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self._algo_checks: CheckButtons | None = None
        self._algo_mode_checks: CheckButtons | None = None
        self._algo_checked = {"dtatrim": True, "linear": True, "algo3": True}
        self._dim_unchecked = False
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self._load_background()
        self._init_algo_checks()
        self._refresh_plot()

        self._is_panning = False
        self._pan_anchor: tuple[float, float] | None = None
        self._pan_xlim: Tuple[float, float] | None = None
        self._pan_ylim: Tuple[float, float] | None = None

        print(
            "Instructions:\n"
            "  - Left-click: add LL point (lon on X, lat on Y)\n"
            "  - Right-click drag: pan view\n"
            "  - ENTER: input width + UAV count, then build 0303 plans\n"
            "  - ESC: clear all points/results\n"
            "  - Mouse wheel: zoom in/out\n"
            "  - R: reset view to background extent\n"
            "  - F: toggle filming plan lines\n"
            "  - Algorithm checkboxes: show/hide algorithm plans\n"
            "  - Dim unselected: fade unchecked algorithms"
        )

    def _on_click(self, event):
        if event.inaxes != self.ax:
            return
        if event.button == 1 and event.xdata is not None and event.ydata is not None:
            lat = float(event.ydata)
            lon = float(event.xdata)
            self.points.append((lat, lon))
            self._expand_view(lat, lon)
            print(f"[+] point added  lat={lat:.6f}, lon={lon:.6f}")
            self._clear_results()
            self._refresh_plot()
        elif event.button == 3 and event.xdata is not None and event.ydata is not None:
            self._is_panning = True
            self._pan_anchor = (event.xdata, event.ydata)
            self._pan_xlim = self.ax.get_xlim()
            self._pan_ylim = self.ax.get_ylim()

    def _on_key(self, event):
        if event.key == "enter":
            self._handle_enter()
        elif event.key == "escape":
            self.points.clear()
            if not self.view_fixed:
                self.view_xlim = None
                self.view_ylim = None
            self._clear_results()
            print("[*] cleared all points.")
            self._refresh_plot()
        elif event.key == "r":
            if self.dem_extent is not None:
                lon_min, lon_max, lat_min, lat_max = self.dem_extent
                self.view_xlim = (lon_min, lon_max)
                self.view_ylim = (lat_min, lat_max)
                self.view_fixed = True
                self._refresh_plot()
        elif event.key == "f":
            self.show_filming = not self.show_filming
            self._refresh_plot()

    def _on_release(self, event):
        if event.button == 3:
            self._is_panning = False
            self._pan_anchor = None
            self._pan_xlim = None
            self._pan_ylim = None

    def _on_scroll(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        scale = 0.9 if event.button == "up" else 1.1
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        width = (x1 - x0) * scale
        height = (y1 - y0) * scale
        cx, cy = event.xdata, event.ydata
        new_x0 = cx - (cx - x0) * scale
        new_x1 = new_x0 + width
        new_y0 = cy - (cy - y0) * scale
        new_y1 = new_y0 + height
        self.view_xlim = (new_x0, new_x1)
        self.view_ylim = (new_y0, new_y1)
        self.view_fixed = True
        self._refresh_plot()

    def _on_motion(self, event):
        if not self._is_panning or event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        if (
            self._pan_anchor is None
            or self._pan_xlim is None
            or self._pan_ylim is None
        ):
            return
        dx = event.xdata - self._pan_anchor[0]
        dy = event.ydata - self._pan_anchor[1]
        new_xlim = (self._pan_xlim[0] - dx, self._pan_xlim[1] - dx)
        new_ylim = (self._pan_ylim[0] - dy, self._pan_ylim[1] - dy)
        self.view_xlim = new_xlim
        self.view_ylim = new_ylim
        self.view_fixed = True
        self._refresh_plot()

    def _clear_results(self) -> None:
        self.strip_polygons.clear()
        self.flight_plans_by_algo.clear()
        self.filming_lines_by_algo.clear()
        self.last_width = None
        self.last_uav_cnt = None

    def _handle_enter(self) -> None:
        if len(self.points) < 2:
            print("[!] need at least two points to form a corridor.")
            return
        try:
            width = float(input("Enter corridor width (meters): "))
            if width <= 0:
                raise ValueError
        except ValueError:
            print("[!] width must be a positive number.")
            return
        try:
            uav_cnt = int(input("Enter number of UAVs (>=1): "))
            if uav_cnt < 1:
                raise ValueError
        except ValueError:
            print("[!] UAV count must be an integer >= 1.")
            return

        try:
            self._generate_plans(width, uav_cnt)
            self.last_width = width
            self.last_uav_cnt = uav_cnt
            print(f"[OK] generated 0303 plans for width={width}m, UAVs={uav_cnt}")
        except Exception as exc:
            print(f"[!] failed to generate 0303 plans: {exc}")
        finally:
            self._refresh_plot()

    def _generate_plans(self, width: float, uav_cnt: int) -> None:
        line_seg = {
            "width": width,
            "coordinateList": [
                {"latitude": lat, "longitude": lon, "altitude": 0.0}
                for lat, lon in self.points
            ],
        }
        strips = divide_corridor_polyline(line_seg, uav_cnt)
        self.strip_polygons = [strip["coordinateList"] for strip in strips]

        if uav_cnt > 3:
            print("[info] UAV IDs are limited to 4-6; reusing IDs for extra strips.")

        missions: List[dict] = []
        for idx, strip in enumerate(strips):
            aircraft_id = 4 + (idx % 3)
            mission = {
                "aircraftID": aircraft_id,
                "pathID": 100_000_000 + idx + 1,
                "individualMissionID": idx + 1,
                "relatedMission": {
                    "relatedMissionType": 1,
                    "inputMissionID": 1,
                    "priorMissionID": 0,
                },
                "individualMissionInfo": {
                    "individualMissionType": 6,
                    "patternType": 4,
                    "autoZoomIn": True,
                    "lineList": [
                        {
                            "width": strip.get("width", width),
                            "coordinateList": strip.get("Centerline", []),
                        }
                    ],
                },
            }
            missions.append(mission)

        algos = [
            ("dtatrim", "dtatrim"),
            ("linear", "linear"),
            ("algo3", "algo3"),
        ]
        original_algo = getattr(d0303, "ROUTE_PLANNER_NAME", "dtatrim")

        self.flight_plans_by_algo = {}
        self.filming_lines_by_algo = {}
        for algo_key, planner_name in algos:
            d0303.set_route_planner(planner_name)
            wp_alloc = d0303._WPAllocator(1)
            plans = d0303.build_flight_plans(
                missions,
                wp_alloc=wp_alloc,
                cruise_speed=CRUISE_SPEED,
                turn_step_deg=TURN_STEP_DEG,
            )
            self.flight_plans_by_algo[algo_key] = plans

            algo_lines: List[Tuple[int, List[dict]]] = []
            for fp in plans:
                aid = int(fp.get("aircraftID", 0))
                for wp in fp.get("waypointList") or []:
                    filming = wp.get("filmingProperty") or {}
                    if int(filming.get("operationMode", 0)) != d0303.OPMODE_LINE:
                        continue
                    line_search = filming.get("lineSearch") or {}
                    coords = line_search.get("coordinateList") or []
                    if len(coords) >= 2:
                        algo_lines.append((aid, coords))
            self.filming_lines_by_algo[algo_key] = algo_lines

        d0303.set_route_planner(original_algo)

        for strip in self.strip_polygons:
            for coord in strip:
                self._expand_view(coord["latitude"], coord["longitude"])
        for plans in self.flight_plans_by_algo.values():
            for fp in plans:
                for wp in fp.get("waypointList") or []:
                    coord = wp.get("coordinate") or {}
                    if "latitude" in coord and "longitude" in coord:
                        self._expand_view(coord["latitude"], coord["longitude"])

    def _infer_extent_from_name(self, path: Path) -> Tuple[float, float, float, float] | None:
        stem = path.stem.lower()
        lat = None
        lon = None

        lat_match = re.search(r"([ns])(\\d+)", stem)
        if lat_match:
            sign = 1 if lat_match.group(1) == "n" else -1
            lat = sign * float(lat_match.group(2))
        else:
            leading_digits = ""
            for ch in stem:
                if ch.isdigit():
                    leading_digits += ch
                else:
                    break
            if leading_digits:
                lat = float(leading_digits)

        lon_match = re.search(r"([ew])(\\d+)", stem)
        if lon_match:
            sign = 1 if lon_match.group(1) == "e" else -1
            lon = sign * float(lon_match.group(2))

        if lat is None or lon is None:
            return None

        lat_min, lat_max = (lat, lat + 1.0) if lat >= 0 else (lat - 1.0, lat)
        lon_min, lon_max = (lon, lon + 1.0) if lon >= 0 else (lon - 1.0, lon)
        return lon_min, lon_max, lat_min, lat_max

    def _load_background(self) -> None:
        if self.dem_image is not None:
            return
        if not BACKGROUND_TIF.exists():
            print(f"[info] background file not found: {BACKGROUND_TIF}")
            return

        img = None
        extent = None

        if rasterio is not None:
            try:
                with rasterio.open(BACKGROUND_TIF) as src:
                    data = src.read(1).astype(np.float32)
                    nodata = src.nodata
                    if nodata is not None:
                        data = np.where(data == nodata, np.nan, data)
                    img = data
                    bounds = src.bounds
                    extent = (bounds.left, bounds.right, bounds.bottom, bounds.top)
            except Exception as exc:
                print(f"[info] rasterio failed to load background ({exc}); falling back.")

        if img is None and Image is not None:
            try:
                with Image.open(BACKGROUND_TIF) as pil_img:
                    img = np.array(pil_img.convert("L"), dtype=np.float32)
            except Exception as exc:
                print(f"[info] Pillow failed to read background ({exc}).")

        if img is None:
            print("[info] unable to load background image; drawing plain axes.")
            return

        if extent is None:
            inferred = self._infer_extent_from_name(BACKGROUND_TIF)
            if inferred is None:
                print("[info] could not infer GeoTIFF extent; using default view.")
            else:
                extent = inferred

        if extent is None:
            extent = (0.0, 1.0, 0.0, 1.0)

        self.dem_image = img
        self.dem_extent = extent
        lon_min, lon_max, lat_min, lat_max = extent
        self.view_xlim = (lon_min, lon_max)
        self.view_ylim = (lat_min, lat_max)
        self.view_fixed = True
        print(f"[info] background loaded: lon[{lon_min},{lon_max}] lat[{lat_min},{lat_max}]")

    def _expand_view(self, lat: float, lon: float) -> None:
        if self.view_fixed:
            return
        pad = BOUND_PAD_DEG
        if self.view_xlim is None:
            self.view_xlim = (lon - pad, lon + pad)
        else:
            lo, hi = self.view_xlim
            self.view_xlim = (min(lo, lon - pad), max(hi, lon + pad))
        if self.view_ylim is None:
            self.view_ylim = (lat - pad, lat + pad)
        else:
            lo, hi = self.view_ylim
            self.view_ylim = (min(lo, lat - pad), max(hi, lat + pad))

    def _refresh_plot(self) -> None:
        self.ax.clear()
        self.ax.set_xlabel("Longitude (deg)")
        self.ax.set_ylabel("Latitude (deg)")
        self.ax.set_aspect("equal", adjustable="datalim")

        title = "Click to add LL points. ENTER: generate 0303 plans."
        if self.last_width and self.last_uav_cnt:
            title += f"  [width={self.last_width:.1f}m, UAVs={self.last_uav_cnt}]"
        self.ax.set_title(title)

        if self.dem_image is not None and self.dem_extent is not None:
            self.ax.imshow(
                self.dem_image,
                extent=self.dem_extent,
                cmap="gray",
                origin="upper",
                alpha=0.6,
                zorder=0,
            )

        # Base polyline
        if self.points:
            lats = [p[0] for p in self.points]
            lons = [p[1] for p in self.points]
            self.ax.plot(lons, lats, marker="o", color="black", linewidth=1.5)

        # Corridor strips
        colors = ["tab:blue", "tab:orange", "tab:green", "tab:purple", "tab:red", "tab:brown"]
        for idx, poly in enumerate(self.strip_polygons):
            lats = [p["latitude"] for p in poly] + [poly[0]["latitude"]]
            lons = [p["longitude"] for p in poly] + [poly[0]["longitude"]]
            color = colors[idx % len(colors)]
            self.ax.fill(lons, lats, alpha=0.12, color=color, edgecolor=color, linewidth=1)

        # Flight paths (emphasize waypoints over lines)
        handles: List[Line2D] = []
        labels: List[str] = []
        algo_colors = {
            "dtatrim": "tab:blue",
            "linear": "tab:orange",
            "algo3": "tab:green",
        }
        selected = [k for k, v in self._algo_checked.items() if v]
        if not selected:
            selected = list(algo_colors.keys())

        for algo_key, plans in self.flight_plans_by_algo.items():
            color = algo_colors.get(algo_key, "gray")
            if self._dim_unchecked and algo_key not in selected:
                path_alpha = 0.15
                wp_alpha = 0.15
            else:
                path_alpha = 0.85
                wp_alpha = 0.85
            for fp in plans:
                wps = fp.get("waypointList") or []
                if not wps:
                    continue
                lats = [wp["coordinate"]["latitude"] for wp in wps]
                lons = [wp["coordinate"]["longitude"] for wp in wps]
                if len(wps) >= 2:
                    self.ax.plot(lons, lats, color=color, linewidth=0.8, alpha=path_alpha)
                self.ax.scatter(
                    lons,
                    lats,
                    s=36,
                    color=color,
                    edgecolors="black",
                    linewidths=0.5,
                    alpha=wp_alpha,
                    zorder=3,
                )
            if algo_key not in labels:
                handles.append(Line2D([0], [0], color=color, marker="o", linestyle="None", markersize=6))
                labels.append(algo_key)

        # Filming plans (lineSearch)
        if self.show_filming:
            for algo_key, lines in self.filming_lines_by_algo.items():
                color = algo_colors.get(algo_key, "gray")
                if self._dim_unchecked and algo_key not in selected:
                    alpha = 0.1
                else:
                    alpha = 0.4
                for _, coords in lines:
                    lats = [c["latitude"] for c in coords]
                    lons = [c["longitude"] for c in coords]
                    self.ax.plot(lons, lats, color=color, linewidth=1.0, alpha=alpha, linestyle="--")

        if handles:
            self.ax.legend(handles, labels, loc="upper right", fontsize=8)

        self.ax.grid(True, linestyle="--", alpha=0.4)
        if self.view_xlim:
            self.ax.set_xlim(*self.view_xlim)
        if self.view_ylim:
            self.ax.set_ylim(*self.view_ylim)
        self.fig.canvas.draw_idle()

    def _init_algo_checks(self) -> None:
        ax_check = self.fig.add_axes([0.82, 0.72, 0.16, 0.18])
        labels = ["DTAutoTrim", "Algorithm-2", "Algorithm-3"]
        states = [self._algo_checked["dtatrim"], self._algo_checked["linear"], self._algo_checked["algo3"]]
        self._algo_checks = CheckButtons(ax_check, labels, states)
        ax_check.set_title("Algorithm")

        def _on_check(label: str) -> None:
            if self._algo_checks is None:
                return
            statuses = list(self._algo_checks.get_status())
            mapping = {"DTAutoTrim": "dtatrim", "Algorithm-2": "linear", "Algorithm-3": "algo3"}
            for idx, current_label in enumerate(labels):
                self._algo_checked[mapping[current_label]] = statuses[idx]
            self._refresh_plot()

        self._algo_checks.on_clicked(_on_check)

        ax_mode = self.fig.add_axes([0.82, 0.64, 0.16, 0.06])
        self._algo_mode_checks = CheckButtons(ax_mode, ["Dim unselected"], [self._dim_unchecked])
        ax_mode.set_title("Display")

        def _on_mode_check(label: str) -> None:
            if self._algo_mode_checks is None:
                return
            self._dim_unchecked = bool(self._algo_mode_checks.get_status()[0])
            self._refresh_plot()

        self._algo_mode_checks.on_clicked(_on_mode_check)


def main() -> None:
    app = FlightPlanDebugger()
    plt.show()
    _ = app


if __name__ == "__main__":
    main()
