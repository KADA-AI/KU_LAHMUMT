#!/usr/bin/env python3
"""
Heading alignment debugger with lat/lon background.
Behavior matches modules/test/dubins_connector.py but keeps map + LL coords.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import matplotlib

try:
    matplotlib.use("TkAgg")
except Exception:
    pass

import matplotlib.pyplot as plt
import numpy as np

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

for path in (SCRIPT_DIR, REPO_ROOT, MISSION_PLANNER_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.append(path_str)

import dubins_connector as dc

from modules.mission_planning.MissionPlanner.data_def.coord_transform import (  # type: ignore
    llh_to_xy,
    xy_to_llh,
)

BOUND_PAD_DEG = 0.0005
DEFAULT_TURN_RADIUS_M = 60.0
DEFAULT_SAMPLE_STEP_M = 5.0

PointLL = Tuple[float, float]  # (lat, lon)


def _heading_xy(p0: PointLL, p1: PointLL, ref: PointLL) -> float | None:
    x0, y0 = llh_to_xy(p0[0], p0[1], ref[0], ref[1])
    x1, y1 = llh_to_xy(p1[0], p1[1], ref[0], ref[1])
    dx = x1 - x0
    dy = y1 - y0
    if math.hypot(dx, dy) < 1e-9:
        return None
    return math.atan2(dy, dx)


def _distance_m(p0: PointLL, p1: PointLL) -> float:
    x, y = llh_to_xy(p1[0], p1[1], p0[0], p0[1])
    return math.hypot(x, y)


@dataclass
class Mission:
    p0: PointLL
    p1: PointLL

    def heading(self, ref: PointLL) -> float | None:
        return _heading_xy(self.p0, self.p1, ref)


class HeadingAlignDebugger:
    def __init__(self) -> None:
        self.turn_radius_m = DEFAULT_TURN_RADIUS_M
        self.sample_step_m = DEFAULT_SAMPLE_STEP_M

        self.current_points: List[PointLL] = []
        self.missions: List[Mission] = []
        self.connectors: List[List[PointLL]] = []

        self.dem_image: np.ndarray | None = None
        self.dem_extent: Tuple[float, float, float, float] | None = None
        self.view_xlim: Tuple[float, float] | None = None
        self.view_ylim: Tuple[float, float] | None = None
        self.view_fixed = False

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect("equal", adjustable="datalim")

        self._panning = False
        self._pan_press_xy = (0.0, 0.0)
        self._pan_xlim = (0.0, 1.0)
        self._pan_ylim = (0.0, 1.0)

        self.fig.canvas.mpl_connect("button_press_event", self.on_mouse_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_mouse_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_mouse_move)
        self.fig.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        self._load_background()
        self._refresh_plot()
        self.print_help()

    def print_help(self) -> None:
        print(
            "\n[Help]\n"
            "- Left-click: add 2 points (one mission)\n"
            "- Enter: commit current 2-point mission\n"
            "- Esc: clear all points/missions\n"
            "- Backspace: remove last mission\n"
            "- T: set turn radius (meters)\n"
            "- S: set sample step (meters)\n"
            "- R: reset view to background extent\n"
            "- H: print help\n"
            "- Right-click drag: pan, Mouse wheel: zoom\n"
        )

    def _infer_extent_from_name(self, path: Path) -> Tuple[float, float, float, float] | None:
        stem = path.stem.lower()
        try:
            import re
        except Exception:
            return None

        lat = None
        lon = None

        m = re.search(r"([ns])(\d+(?:\.\d+)?)", stem)
        if m:
            lat = (1.0 if m.group(1) == "n" else -1.0) * float(m.group(2))

        m = re.search(r"([ew])(\d+(?:\.\d+)?)", stem)
        if m:
            lon = (1.0 if m.group(1) == "e" else -1.0) * float(m.group(2))

        if lat is None or lon is None:
            return None

        lat_min, lat_max = (lat, lat + 1.0) if lat >= 0 else (lat - 1.0, lat)
        lon_min, lon_max = (lon, lon + 1.0) if lon >= 0 else (lon - 1.0, lon)
        return lon_min, lon_max, lat_min, lat_max

    def _load_background(self) -> None:
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
                print(f"[info] rasterio failed ({exc}); falling back.")

        if img is None and Image is not None:
            try:
                with Image.open(BACKGROUND_TIF) as pil_img:
                    img = np.array(pil_img.convert("L"), dtype=np.float32)
            except Exception as exc:
                print(f"[info] Pillow failed ({exc}).")

        if img is None:
            print("[info] unable to load background image; drawing plain axes.")
            return

        if extent is None:
            extent = self._infer_extent_from_name(BACKGROUND_TIF)

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

    def _apply_view(self) -> None:
        if self.view_xlim and self.view_ylim:
            self.ax.set_xlim(*self.view_xlim)
            self.ax.set_ylim(*self.view_ylim)

    def _compute_connector(self, prev: Mission, cur: Mission, log: bool) -> List[PointLL]:
        ref = prev.p1
        h0 = prev.heading(ref)
        h1 = cur.heading(ref)
        if h0 is None or h1 is None:
            return [prev.p1, cur.p0]

        dx, dy = llh_to_xy(cur.p0[0], cur.p0[1], ref[0], ref[1])
        q0 = (0.0, 0.0, h0)
        q1 = (dx, dy, h1)

        path = dc.dubins_shortest_path(q0, q1, rho=self.turn_radius_m)
        if path is None:
            conn_xy = np.asarray([[0.0, 0.0], [dx, dy]], dtype=float)
        else:
            conn_xy = dc.dubins_sample_xy(path, step_m=self.sample_step_m)

        conn_ll: List[PointLL] = []
        for x, y in conn_xy:
            lat, lon = xy_to_llh(float(x), float(y), ref[0], ref[1])
            conn_ll.append((lat, lon))

        if log:
            print(
                f"connector {len(self.connectors) + 1}: "
                f"type={path.word if path else 'N/A'}, "
                f"len={path.length if path else float('nan'):.2f} m, "
                f"R={self.turn_radius_m:.2f}, step={self.sample_step_m:.2f}"
            )
        return conn_ll

    def _rebuild_connectors(self, log: bool = False) -> None:
        self.connectors.clear()
        if len(self.missions) < 2:
            return
        for i in range(1, len(self.missions)):
            prev = self.missions[i - 1]
            cur = self.missions[i]
            conn = self._compute_connector(prev, cur, log=log)
            self.connectors.append(conn)

    def _clear_all(self) -> None:
        self.current_points.clear()
        self.missions.clear()
        self.connectors.clear()
        if self.dem_extent is None:
            self.view_fixed = False
        self._refresh_plot()

    def _pop_last_mission(self) -> None:
        if not self.missions:
            return
        self.missions.pop()
        if self.connectors:
            self.connectors.pop()
        if self.dem_extent is None:
            self.view_fixed = False
        self._refresh_plot()

    def _commit_current_mission(self) -> None:
        if len(self.current_points) != 2:
            print("[info] need exactly 2 points to commit a mission.")
            return

        p0, p1 = self.current_points
        if _distance_m(p0, p1) < 1e-6:
            print("[info] points are too close; ignored.")
            return

        new_mission = Mission(p0=p0, p1=p1)
        self.missions.append(new_mission)

        if len(self.missions) >= 2:
            prev = self.missions[-2]
            cur = self.missions[-1]
            conn = self._compute_connector(prev, cur, log=True)
            self.connectors.append(conn)

        self.current_points.clear()
        if self.dem_extent is None:
            self.view_fixed = False
        self._refresh_plot()

    def _reset_view(self) -> None:
        if self.dem_extent is not None:
            lon_min, lon_max, lat_min, lat_max = self.dem_extent
            self.view_xlim = (lon_min, lon_max)
            self.view_ylim = (lat_min, lat_max)
            self.view_fixed = True
        else:
            self.view_fixed = False
        self._refresh_plot()

    def _refresh_plot(self) -> None:
        self.ax.clear()
        self.ax.set_xlabel("Longitude (deg)")
        self.ax.set_ylabel("Latitude (deg)")
        self.ax.set_aspect("equal", adjustable="datalim")

        if self.dem_image is not None and self.dem_extent is not None:
            self.ax.imshow(
                self.dem_image,
                extent=self.dem_extent,
                cmap="gray",
                origin="upper",
                alpha=0.5,
            )

        for mission in self.missions:
            lat0, lon0 = mission.p0
            lat1, lon1 = mission.p1
            self.ax.plot([lon0, lon1], [lat0, lat1], lw=2.5)
            self._expand_view(lat0, lon0)
            self._expand_view(lat1, lon1)

        for conn in self.connectors:
            lons = [p[1] for p in conn]
            lats = [p[0] for p in conn]
            self.ax.plot(lons, lats, lw=2.0)
            for lat, lon in conn:
                self._expand_view(lat, lon)

        if self.current_points:
            lons = [p[1] for p in self.current_points]
            lats = [p[0] for p in self.current_points]
            self.ax.plot(lons, lats, "rx", ms=8)
            for lat, lon in self.current_points:
                self._expand_view(lat, lon)

        self.ax.text(
            0.01,
            0.99,
            f"missions={len(self.missions)}  connectors={len(self.connectors)}\n"
            f"R={self.turn_radius_m:.2f} m  step={self.sample_step_m:.2f} m\n"
            f"current_points={len(self.current_points)}/2",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=10,
        )

        self._apply_view()
        self.fig.canvas.draw_idle()

    def on_mouse_press(self, event) -> None:
        if event.inaxes != self.ax:
            return
        if event.button == 3:
            self._panning = True
            self._pan_press_xy = (event.xdata, event.ydata)
            self._pan_xlim = self.ax.get_xlim()
            self._pan_ylim = self.ax.get_ylim()
            return

        if event.button == 1:
            if event.xdata is None or event.ydata is None:
                return
            if len(self.current_points) >= 2:
                return
            lat = float(event.ydata)
            lon = float(event.xdata)
            self.current_points.append((lat, lon))
            self._refresh_plot()

    def on_mouse_release(self, event) -> None:
        if event.button == 3:
            self._panning = False

    def on_mouse_move(self, event) -> None:
        if not self._panning:
            return
        if event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return

        x0, y0 = self._pan_press_xy
        dx = x0 - event.xdata
        dy = y0 - event.ydata

        self.view_xlim = (self._pan_xlim[0] + dx, self._pan_xlim[1] + dx)
        self.view_ylim = (self._pan_ylim[0] + dy, self._pan_ylim[1] + dy)
        self.view_fixed = True
        self._apply_view()
        self.fig.canvas.draw_idle()

    def on_scroll(self, event) -> None:
        if event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return

        base_scale = 1.2
        if event.button == "up":
            scale_factor = 1 / base_scale
        elif event.button == "down":
            scale_factor = base_scale
        else:
            return

        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        x = event.xdata
        y = event.ydata

        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor

        relx = (x - cur_xlim[0]) / (cur_xlim[1] - cur_xlim[0] + 1e-12)
        rely = (y - cur_ylim[0]) / (cur_ylim[1] - cur_ylim[0] + 1e-12)

        self.view_xlim = (x - new_width * relx, x + new_width * (1 - relx))
        self.view_ylim = (y - new_height * rely, y + new_height * (1 - rely))
        self.view_fixed = True
        self._apply_view()
        self.fig.canvas.draw_idle()

    def on_key(self, event) -> None:
        if event.key in ("h", "H"):
            self.print_help()
            return
        if event.key == "escape":
            self._clear_all()
            return
        if event.key == "backspace":
            self._pop_last_mission()
            return
        if event.key in ("enter", "return"):
            self._commit_current_mission()
            return
        if event.key in ("t", "T"):
            try:
                val = input(f"turn radius R (meters), current={self.turn_radius_m:.3f}: ").strip()
                if not val:
                    return
                r = float(val)
                if r <= 0:
                    print("R must be > 0")
                    return
                self.turn_radius_m = r
                self._rebuild_connectors()
                self._refresh_plot()
            except Exception as exc:
                print(f"failed to set R: {exc}")
            return
        if event.key in ("s", "S"):
            try:
                val = input(f"sample step (meters), current={self.sample_step_m:.3f}: ").strip()
                if not val:
                    return
                s = float(val)
                if s <= 0:
                    print("step must be > 0")
                    return
                self.sample_step_m = s
                self._rebuild_connectors()
                self._refresh_plot()
            except Exception as exc:
                print(f"failed to set step: {exc}")
            return
        if event.key in ("r", "R"):
            self._reset_view()
            return


def main() -> None:
    _app = HeadingAlignDebugger()
    plt.show()


if __name__ == "__main__":
    main()
