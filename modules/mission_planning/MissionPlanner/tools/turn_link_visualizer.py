"""
DTA 기반 턴 연결 시각화 도구
────────────────────────────────────────────
 1) 이전 임무의 마지막 두 점(위·경도)을 순서대로 좌클릭한다.
 2) 이어서 다음 임무의 시작 두 점을 좌클릭한다.
 3) 입력이 끝나면 속도/최대 롤각/거리 제한을 물어본 뒤, DTA 기반 접선 구간
    (previous_end -> tangent_start -> tangent_end -> next_start)을 계산해 표시한다.
 4) 'r' 키를 누르면 초기화하여 다시 측정할 수 있다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib import font_manager, rcParams
from tkinter import Tk, simpledialog

# ── import path 설정 ────────────────────────────────────────────
THIS_DIR = Path(__file__).resolve().parent
ROOT_DIR = THIS_DIR.parents[1]
REPO_ROOT = THIS_DIR.parents[2]
for path in (REPO_ROOT, ROOT_DIR, THIS_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from DTA import get_dta, get_radius  # type: ignore


def _configure_font() -> None:
    """Ensure Korean labels render properly."""
    candidates = [
        "Malgun Gothic",
        "MalgunGothic",
        "Segoe UI",
        "AppleGothic",
        "NanumGothic",
    ]
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            rcParams["font.family"] = name
            break
    rcParams["axes.unicode_minus"] = False


_configure_font()

Point = Tuple[float, float]  # (lat, lon)
_R_EARTH = 6_378_137.0
DEFAULT_REF_LAT = 37.0
DEFAULT_REF_LON = 127.0


def _llh_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = dlon * _R_EARTH * math.cos(lat0_r)
    y = dlat * _R_EARTH
    return x, y


def _xy_to_llh(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat = lat0 + math.degrees(y / _R_EARTH)
    lon = lon0 + math.degrees(x / (_R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


def _unit(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm <= 1e-9:
        return np.zeros_like(vec)
    return vec / norm


def _heading_deg(vec: np.ndarray) -> float:
    return (math.degrees(math.atan2(vec[1], vec[0])) + 360.0) % 360.0


def _wrap_delta(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


def _latlon_to_xy_km(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    x_m, y_m = _llh_to_xy(lat, lon, lat0, lon0)
    return x_m / 1000.0, y_m / 1000.0


def _xy_km_to_latlon(x_km: float, y_km: float, lat0: float, lon0: float) -> Tuple[float, float]:
    return _xy_to_llh(x_km * 1000.0, y_km * 1000.0, lat0, lon0)


@dataclass
class TurnSolution:
    exit_heading: float
    entry_heading: float
    delta_deg: float
    dta_m: float
    radius_m: float
    tangent_start: Point  # lat/lon
    tangent_end: Point


class TurnLinkVisualizer:
    def __init__(self) -> None:
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_title("Turn-Link Visualizer (좌클릭: 점 입력, 우클릭: 이동, 'r': 초기화)")
        self.ax.set_xlabel("East [km]")
        self.ax.set_ylabel("North [km]")
        self.ax.grid(True, linestyle="--", alpha=0.5)

        self.prev_points: List[Point] = []
        self.next_points: List[Point] = []
        self.ref_latlon: Point | None = (DEFAULT_REF_LAT, DEFAULT_REF_LON)
        self.view_extent_km = 20.0
        self.view_center: Tuple[float, float] | None = (0.0, 0.0)  # (x_km, y_km)
        self.view_locked = False
        self.is_panning = False
        self._pan_start_xy: Tuple[float, float] | None = None
        self._pan_start_xlim: Tuple[float, float] | None = None
        self._pan_start_ylim: Tuple[float, float] | None = None
        self.use_xy_display = True
        self.all_scatter = self.ax.scatter([], [])
        self.solution_artists: List[Line2D] = []
        self.summary_text = self.ax.text(
            0.02,
            0.98,
            "이전 임무 마지막 두 점 → 다음 임무 첫 두 점 순으로 좌클릭",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=10,
            bbox=dict(facecolor="white", alpha=0.85, edgecolor="none"),
        )

        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        self._set_view_window(force=True)

    # ── 이벤트 핸들러 ───────────────────────────────────────
    def on_click(self, event) -> None:
        if event.inaxes != self.ax:
            return
        if event.button == 3:
            if event.xdata is None or event.ydata is None:
                return
            self.is_panning = True
            self._pan_start_xy = (event.xdata, event.ydata)
            self._pan_start_xlim = self.ax.get_xlim()
            self._pan_start_ylim = self.ax.get_ylim()
            return
        if event.button != 1:
            return
        if self.use_xy_display:
            if not self.ref_latlon:
                return
            lat, lon = _xy_km_to_latlon(event.xdata, event.ydata, *self.ref_latlon)
        else:
            lat, lon = event.ydata, event.xdata
        if self.ref_latlon is None:
            self.ref_latlon = (lat, lon)
        pt = (lat, lon)
        total = len(self.prev_points) + len(self.next_points)
        if total < 2:
            self.prev_points.append(pt)
        elif total < 4:
            self.next_points.append(pt)
        self._refresh_points()
        if not self.use_xy_display and self.ref_latlon is not None:
            self._switch_to_xy_display()
        if len(self.prev_points) == 2 and len(self.next_points) == 2:
            self._compute_and_draw()

    def on_key(self, event) -> None:
        if event.key.lower() == "r":
            self._reset()

    def on_release(self, event) -> None:
        if event.button == 3:
            self.is_panning = False
            self._pan_start_xy = None
            self._pan_start_xlim = None
            self._pan_start_ylim = None

    def on_motion(self, event) -> None:
        if not self.use_xy_display or not self.is_panning or event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        if not self._pan_start_xy or not self._pan_start_xlim or not self._pan_start_ylim:
            return
        dx = event.xdata - self._pan_start_xy[0]
        dy = event.ydata - self._pan_start_xy[1]
        x0, x1 = self._pan_start_xlim
        y0, y1 = self._pan_start_ylim
        self.ax.set_xlim(x0 - dx, x1 - dx)
        self.ax.set_ylim(y0 - dy, y1 - dy)
        if self.use_xy_display:
            new_cx = (self.ax.get_xlim()[0] + self.ax.get_xlim()[1]) / 2.0
            new_cy = (self.ax.get_ylim()[0] + self.ax.get_ylim()[1]) / 2.0
            self.view_center = (new_cx, new_cy)
        self.fig.canvas.draw_idle()

    def on_scroll(self, event) -> None:
        if event.inaxes != self.ax or not self.use_xy_display:
            return
        if event.xdata is None or event.ydata is None:
            return
        factor = 0.9 if getattr(event, "button", None) == "up" else 1.1
        self.view_extent_km = max(1.0, min(200.0, self.view_extent_km * factor))
        self.view_locked = False
        self.view_center = (event.xdata, event.ydata)
        self._set_view_window(force=True)

    # ── 내부 유틸 ───────────────────────────────────────────
    def _set_view_window(self, force: bool = False) -> None:
        if not self.use_xy_display or not self.ref_latlon:
            return
        if self.view_center is None:
            self.view_center = (0.0, 0.0)
        if not force and self.view_locked:
            return
        cx, cy = self.view_center
        half = self.view_extent_km / 2.0
        self.ax.set_xlim(cx - half, cx + half)
        self.ax.set_ylim(cy - half, cy + half)
        self.ax.set_aspect("equal", adjustable="box")
        self.view_locked = True
        self.fig.canvas.draw_idle()

    def _reset(self) -> None:
        self.prev_points.clear()
        self.next_points.clear()
        self.ref_latlon = (DEFAULT_REF_LAT, DEFAULT_REF_LON)
        self.view_center = (0.0, 0.0)
        self.view_locked = False
        self.use_xy_display = True
        for artist in self.solution_artists:
            artist.remove()
        self.solution_artists.clear()
        self.summary_text.set_text("이전/다음 임무 점을 다시 입력하세요 ('r'=리셋).")
        self._refresh_points()
        self.ax.set_xlabel("East [km]")
        self.ax.set_ylabel("North [km]")
        self.view_extent_km = 20.0
        self._set_view_window(force=True)
        self.fig.canvas.draw_idle()

    def _refresh_points(self) -> None:
        pts = self.prev_points + self.next_points
        xs: List[float] = []
        ys: List[float] = []
        if self.use_xy_display and self.ref_latlon:
            for lat, lon in pts:
                x_km, y_km = _latlon_to_xy_km(lat, lon, *self.ref_latlon)
                xs.append(x_km)
                ys.append(y_km)
        else:
            xs = [lon for lat, lon in pts]
            ys = [lat for lat, _ in pts]
        self.all_scatter.remove()
        colors = (
            ["tab:blue"] * len(self.prev_points)
            + ["tab:orange"] * len(self.next_points)
        )
        self.all_scatter = self.ax.scatter(xs, ys, c=colors, s=60, zorder=3)
        self.fig.canvas.draw_idle()

    def _switch_to_xy_display(self) -> None:
        if self.use_xy_display or not self.ref_latlon:
            return
        self.use_xy_display = True
        self.view_center = (0.0, 0.0)
        self.view_locked = False
        self.ax.set_xlabel("East [km]")
        self.ax.set_ylabel("North [km]")
        self._set_view_window(force=True)
        self._refresh_points()

    def _prompt_params(self) -> Tuple[float, float, float]:
        root = Tk()
        root.withdraw()
        try:
            V = simpledialog.askfloat(
                "속도",
                "속도 V (m/s)",
                minvalue=5.0,
                maxvalue=150.0,
                initialvalue=40.0,
                parent=root,
            )
            max_roll = simpledialog.askfloat(
                "최대 롤각",
                "최대 롤각 (deg)",
                minvalue=5.0,
                maxvalue=60.0,
                initialvalue=30.0,
                parent=root,
            )
            dlim = simpledialog.askfloat(
                "거리 제한",
                "거리 제한 dlim (m)",
                minvalue=10.0,
                maxvalue=5000.0,
                initialvalue=1500.0,
                parent=root,
            )
        finally:
            root.destroy()
        if None in (V, max_roll, dlim):
            raise RuntimeError("사용자가 입력을 취소했습니다.")
        return float(V), float(max_roll), float(dlim)

    def _to_xy(self, lat: float, lon: float) -> np.ndarray:
        if not self.ref_latlon:
            return np.zeros(2)
        lat0, lon0 = self.ref_latlon
        return np.array(_llh_to_xy(lat, lon, lat0, lon0), dtype=float)

    def _compute_solution(self, V: float, max_roll: float, dlim: float) -> TurnSolution:
        prev_vec = self._to_xy(*self.prev_points[1]) - self._to_xy(*self.prev_points[0])
        next_vec = self._to_xy(*self.next_points[1]) - self._to_xy(*self.next_points[0])
        exit_dir = _unit(prev_vec)
        entry_dir = _unit(next_vec)
        exit_heading = _heading_deg(exit_dir)
        entry_heading = _heading_deg(entry_dir)
        delta = _wrap_delta(entry_heading - exit_heading)
        dta = get_dta(V, max_roll, delta, dlim)
        radius = get_radius(V, max_roll)

        prev_end_xy = self._to_xy(*self.prev_points[1])
        next_start_xy = self._to_xy(*self.next_points[0])
        tangent_start_xy = prev_end_xy + exit_dir * dta
        tangent_end_xy = next_start_xy - entry_dir * dta

        if self.ref_latlon:
            lat0, lon0 = self.ref_latlon
            tangent_start_llh = _xy_to_llh(tangent_start_xy[0], tangent_start_xy[1], lat0, lon0)
            tangent_end_llh = _xy_to_llh(tangent_end_xy[0], tangent_end_xy[1], lat0, lon0)
        else:
            tangent_start_llh = self.prev_points[1]
            tangent_end_llh = self.next_points[0]

        return TurnSolution(
            exit_heading=exit_heading,
            entry_heading=entry_heading,
            delta_deg=delta,
            dta_m=dta,
            radius_m=radius,
            tangent_start=tangent_start_llh,
            tangent_end=tangent_end_llh,
        )

    def _compute_and_draw(self) -> None:
        for artist in self.solution_artists:
            artist.remove()
        self.solution_artists.clear()
        try:
            V, max_roll, dlim = self._prompt_params()
        except RuntimeError:
            self._reset()
            return

        sol = self._compute_solution(V, max_roll, dlim)

        pts = [
            self.prev_points[1],
            sol.tangent_start,
            sol.tangent_end,
            self.next_points[0],
        ]
        if self.use_xy_display and self.ref_latlon:
            coords = [
                _latlon_to_xy_km(lat, lon, *self.ref_latlon)
                for lat, lon in pts
            ]
            xs = [c[0] for c in coords]
            ys = [c[1] for c in coords]
        else:
            xs = [lon for _, lon in pts]
            ys = [lat for lat, _ in pts]
        line = Line2D(xs, ys, color="tab:green", linewidth=2.0, linestyle="--")
        self.ax.add_line(line)
        self.solution_artists.append(line)

        if self.ref_latlon:
            tang_xy = [
                _latlon_to_xy_km(lat, lon, *self.ref_latlon)
                for lat, lon in (sol.tangent_start, sol.tangent_end)
            ]
            tang_x = [p[0] for p in tang_xy]
            tang_y = [p[1] for p in tang_xy]
        else:
            tang_x = [sol.tangent_start[1], sol.tangent_end[1]]
            tang_y = [sol.tangent_start[0], sol.tangent_end[0]]

        tang_scatter = self.ax.scatter(
            tang_x,
            tang_y,
            c="tab:green",
            marker="s",
            s=80,
            zorder=4,
        )
        self.solution_artists.append(tang_scatter)

        summary = (
            f"V={V:.1f} m/s, 롤={max_roll:.1f}°, dlim={dlim:.0f} m\n"
            f"exit hdg={sol.exit_heading:.1f}°, entry hdg={sol.entry_heading:.1f}° "
            f"(Δ={sol.delta_deg:.1f}°)\n"
            f"DTA={sol.dta_m:.1f} m, 최소 선회반경={sol.radius_m:.1f} m"
        )
        self.summary_text.set_text(summary)
        print(
            "[TURN-LINK]",
            summary.replace("\n", " | "),
            f"TangentStart(lat,lon)={sol.tangent_start}",
            f"TangentEnd(lat,lon)={sol.tangent_end}",
        )
        self.fig.canvas.draw_idle()


def main() -> None:
    _ = TurnLinkVisualizer()
    plt.show()


if __name__ == "__main__":
    main()
