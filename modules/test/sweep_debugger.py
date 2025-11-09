#!/usr/bin/env python3
"""
Interactive corridor/sweep reproducer.

Usage
-----
$ python modules/test/sweep_debugger.py

Left-click to add latitude/longitude points (polyline order).  Right-click
removes the most recent point.  Press ENTER when done to input the corridor
width (meters) and the number of UAV strips.  The script will:
    1) Split the corridor exactly as divide_corridor_polyline() does.
    2) Run UAVMissionPlanner to build the sweep lines.
    3) Plot the corridor, split polygons, and sweep lines.

Press ESC to clear everything and start over.
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
from modules.mission_planning.MissionPlanner.UAV_missionPlanning import (  # type: ignore
    UAVMissionPlanner,
)


DEFAULT_SEPARATION = 850.0  # meters
DEFAULT_FOV = 10.0          # degrees
BOUND_PAD_DEG = 0.0005


PointLL = Tuple[float, float]  # (lat, lon)


class SweepReproducer:
    def __init__(self) -> None:
        self.points: List[PointLL] = []
        self.strip_polygons: List[List[dict]] = []
        self.sweep_groups: List[List[Tuple[PointLL, PointLL]]] = []
        self.last_width: float | None = None
        self.last_uav_cnt: int | None = None
        self.view_xlim: Tuple[float, float] | None = None
        self.view_ylim: Tuple[float, float] | None = None
        self.view_fixed: bool = False
        self.dem_image: np.ndarray | None = None
        self.dem_extent: Tuple[float, float, float, float] | None = None
        self.show_sweeps: bool = False

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self._load_background()
        self._refresh_plot()

        self._is_panning = False
        self._pan_anchor: tuple[float, float] | None = None
        self._pan_xlim: Tuple[float, float] | None = None
        self._pan_ylim: Tuple[float, float] | None = None

        print(
            "Instructions:\n"
            "  - Left-click: add LL point (lon on X, lat on Y)\n"
            "  - Right-click drag: pan view (drag map)\n"
            "  - ENTER: input width + UAV count, then plot corridor + sweeps\n"
            "  - ESC: clear all points/results\n"
            "  - Mouse wheel: zoom in/out\n"
            "  - R: reset view to background extent\n"
            "  - S: toggle sweep-line visibility"
        )

    # ------------------------------------------------------------------ events
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
        elif event.key == "s":
            self.show_sweeps = not self.show_sweeps
            self._refresh_plot()

    def _on_release(self, event):
        if event.button == 3:
            self._is_panning = False
            self._pan_anchor = None
            self._pan_xlim = None
            self._pan_ylim = None

    # ---------------------------------------------------------------- helpers
    def _clear_results(self) -> None:
        self.strip_polygons.clear()
        self.sweep_groups.clear()
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
            self._generate_corridor(width, uav_cnt)
            self.last_width = width
            self.last_uav_cnt = uav_cnt
            print(f"[OK] generated sweeps for width={width}m, UAVs={uav_cnt}")
        except Exception as exc:
            print(f"[!] failed to generate sweeps: {exc}")
        finally:
            self._refresh_plot()

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

    def _generate_corridor(self, width: float, uav_cnt: int) -> None:
        line_seg = {
            "width": width,
            "coordinateList": [
                {"latitude": lat, "longitude": lon, "altitude": 0.0}
                for lat, lon in self.points
            ],
        }
        strips = divide_corridor_polyline(line_seg, uav_cnt)
        self.strip_polygons = [strip["coordinateList"] for strip in strips]

        planner = UAVMissionPlanner(
            [(lat, lon) for lat, lon in self.points],
            corridor_width=width,
            separation=DEFAULT_SEPARATION,
            fov_deg=DEFAULT_FOV,
            crs="lla",
        )

        self.sweep_groups = []

        if getattr(planner, "_proj_back", None) is None:
            raise RuntimeError("UAVMissionPlanner missing _proj_back transformer.")

        sweeps_per_strip = len(planner.sweeps) // max(1, uav_cnt)
        idx = 0
        for strip in self.strip_polygons:
            group: List[Tuple[PointLL, PointLL]] = []
            for _ in range(sweeps_per_strip):
                if idx >= len(planner.sweeps):
                    break
                s_xy, e_xy = planner.sweeps[idx]
                idx += 1
                lat_s, lon_s = planner._proj_back(s_xy[0], s_xy[1])[::-1]
                lat_e, lon_e = planner._proj_back(e_xy[0], e_xy[1])[::-1]
                group.append(((lat_s, lon_s), (lat_e, lon_e)))
                self._expand_view(lat_s, lon_s)
                self._expand_view(lat_e, lon_e)
            if group:
                self.sweep_groups.append(group)

        for poly in self.strip_polygons:
            for coord in poly:
                self._expand_view(coord["latitude"], coord["longitude"])

    def _infer_extent_from_name(self, path: Path) -> Tuple[float, float, float, float] | None:
        stem = path.stem.lower()
        lat = None
        lon = None

        lat_match = re.search(r'([ns])(\d+)', stem)
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

        lon_match = re.search(r'([ew])(\d+)', stem)
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

        title = "Click to add LL points. ENTER: generate sweeps."
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
        colors = ["tab:red", "tab:green", "tab:blue", "tab:purple", "tab:orange", "tab:brown"]
        for idx, poly in enumerate(self.strip_polygons):
            lats = [p["latitude"] for p in poly] + [poly[0]["latitude"]]
            lons = [p["longitude"] for p in poly] + [poly[0]["longitude"]]
            color = colors[idx % len(colors)]
            self.ax.fill(lons, lats, alpha=0.15, color=color, edgecolor=color, linewidth=1, label=f"Strip {idx+1}")

        # Sweep lines per group
        if self.show_sweeps:
            for idx, group in enumerate(self.sweep_groups):
                color = colors[idx % len(colors)]
                for line in group:
                    lat_pair = [line[0][0], line[1][0]]
                    lon_pair = [line[0][1], line[1][1]]
                    self.ax.plot(lon_pair, lat_pair, color=color, linewidth=1.0, alpha=0.9)

        handles: List[Line2D] = []
        labels: List[str] = []
        if self.strip_polygons:
            if self.show_sweeps:
                for idx in range(len(self.strip_polygons)):
                    color = colors[idx % len(colors)]
                    handles.append(Line2D([0], [0], color=color, lw=4))
                    labels.append(f"UAV {idx+1}")
            else:
                handles.append(Line2D([0], [0], color="gray", lw=4))
                labels.append("Sub-areas")
            self.ax.legend(handles, labels, loc="upper right", fontsize=8)

        self.ax.grid(True, linestyle="--", alpha=0.4)
        if self.view_xlim:
            self.ax.set_xlim(*self.view_xlim)
        if self.view_ylim:
            self.ax.set_ylim(*self.view_ylim)
        self.fig.canvas.draw_idle()


def main() -> None:
    app = SweepReproducer()
    plt.show()
    # Keep reference to avoid garbage collection
    _ = app


if __name__ == "__main__":
    main()
