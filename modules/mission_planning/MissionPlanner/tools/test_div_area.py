"""
Interactive tester for divide_search_area_clip.

Usage:
    python MissionPlanner/tools/test_div_area.py

Steps:
 1) Left-click to define polygon vertices (lat/lon). Right-click to finish.
 2) After polygon is closed, left-click once to set the previous-mission point.
 3) When prompted, type aircraft count (1~3). The tool splits the polygon and
    visualizes the resulting sub-areas using the same logic as mission_pipeline.
"""
from __future__ import annotations

import math
import random
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.text import Text
import matplotlib as mpl
from tkinter import Tk, simpledialog

import sys

# Ensure repo root import
ROOT = Path(__file__).resolve().parents[2]
MP_DIR = Path(__file__).resolve().parents[1]
MODULES_DIR = ROOT.parents[0]
REPO_ROOT = ROOT.parents[1]
for path in (REPO_ROOT, MODULES_DIR, ROOT, MP_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from MissionPlanner.AnS import mission_pipeline  # type: ignore
from MissionPlanner.AnS.mission_pipeline import (
    divide_search_area_clip,
    _resolve_area_bearing,
)


class AreaSplitTester:
    def __init__(self) -> None:
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_title("TEST DIV AREA – Left-click to draw polygon, right-click to finish")
        self.ax.set_xlabel("Longitude (deg)")
        self.ax.set_ylabel("Latitude (deg)")
        self.ax.grid(True)
        self.ax.set_xlim(126.5, 127.8)
        self.ax.set_ylim(37.5, 38.5)

        self.poly_points: List[tuple[float, float]] = []
        self.prev_point: tuple[float, float] | None = None
        self.poly_artist: MplPolygon | None = None
        self.prev_artist = None
        self.split_polys: list[PolyCollection] = []
        self.status: Text = self.ax.text(
            0.02,
            0.98,
            "Left-click to add vertices. Right-click to finalize polygon.",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=10,
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"),
        )

        self.mode = "polygon"  # polygon -> prev_point -> done
        self.cid = self.fig.canvas.mpl_connect("button_press_event", self.on_click)

    def update_status(self, text: str) -> None:
        self.status.set_text(text)
        self.fig.canvas.draw_idle()

    def on_click(self, event) -> None:
        if event.inaxes != self.ax:
            return
        if event.button == 1:
            if self.mode == "polygon":
                self.add_vertex(event.xdata, event.ydata)
            elif self.mode == "prev":
                self.set_prev_point(event.xdata, event.ydata)
        elif event.button == 3 and self.mode == "polygon":
            self.finalize_polygon()

    def add_vertex(self, lon: float, lat: float) -> None:
        self.poly_points.append((lat, lon))
        self.draw_polygon()
        self.update_status(f"Vertices: {len(self.poly_points)}. Right-click to finish polygon.")

    def draw_polygon(self) -> None:
        if self.poly_artist:
            self.poly_artist.remove()
        if len(self.poly_points) >= 2:
            self.poly_artist = MplPolygon(
                [(lon, lat) for lat, lon in self.poly_points],
                closed=False,
                fill=False,
                edgecolor="tab:blue",
                linewidth=1.5,
            )
            self.ax.add_patch(self.poly_artist)
        self.fig.canvas.draw_idle()

    def finalize_polygon(self) -> None:
        if len(self.poly_points) < 3:
            self.update_status("Need at least 3 vertices before finalizing.")
            return
        if self.poly_artist:
            self.poly_artist.remove()
        self.poly_artist = MplPolygon(
            [(lon, lat) for lat, lon in self.poly_points],
            closed=True,
            fill=False,
            edgecolor="tab:blue",
            linewidth=2.0,
        )
        self.ax.add_patch(self.poly_artist)
        self.mode = "prev"
        self.update_status("Polygon locked. Left-click once to mark previous mission point.")
        self.fig.canvas.draw_idle()

    def set_prev_point(self, lon: float, lat: float) -> None:
        self.prev_point = (lat, lon)
        if self.prev_artist:
            self.prev_artist.remove()
        self.prev_artist = self.ax.scatter(
            [lon], [lat], c="red", marker="x", s=80, label="Prev"
        )
        self.update_status("Prev point set. Enter aircraft count (1~3) in prompt.")
        self.fig.canvas.draw_idle()
        self.request_aircraft_count()

    def request_aircraft_count(self) -> None:
        root = Tk()
        root.withdraw()
        count = simpledialog.askinteger(
            "Aircraft count",
            "Enter number of UAVs (1-3):",
            minvalue=1,
            maxvalue=3,
            parent=root,
        )
        root.destroy()
        if count is None:
            self.update_status("Input cancelled. Right-click window to close or try again.")
            return
        self.split_and_draw(count)

    def split_and_draw(self, aircraft_cnt: int) -> None:
        for coll in self.split_polys:
            coll.remove()
        self.split_polys.clear()

        poly_llh = [
            {"latitude": lat, "longitude": lon, "altitude": 0}
            for lat, lon in self.poly_points
        ]
        prev = (
            {"latitude": self.prev_point[0], "longitude": self.prev_point[1]}
            if self.prev_point
            else None
        )
        center, bearing = _resolve_area_bearing(prev, poly_llh)  # type: ignore[arg-type]
        print(
            f"[TEST_DIV] center=({center['latitude']:.6f},{center['longitude']:.6f}) "
            f"bearing={bearing:.2f}°"
        )
        subs = divide_search_area_clip(poly_llh, aircraft_cnt, bearing)
        colors = plt.cm.get_cmap("tab20", aircraft_cnt)
        for idx, sub in enumerate(subs):
            coords = sub["coordinateList"]
            poly = [(pt["longitude"], pt["latitude"]) for pt in coords]
            coll = self.ax.fill(
                [p[0] for p in poly],
                [p[1] for p in poly],
                color=colors(idx),
                alpha=0.3,
                edgecolor="k",
                linewidth=1.2,
                label=f"UAV #{idx+1}",
            )
            self.split_polys.extend(coll)
        self.update_status(
            f"Split into {aircraft_cnt} sub-areas using bearing {bearing:.2f}°. Close window to exit."
        )
        self.fig.canvas.draw_idle()


def main() -> None:
    tester = AreaSplitTester()
    plt.show()


if __name__ == "__main__":
    main()
