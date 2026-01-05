#!/usr/bin/env python3
"""
두 점(직선) 미션들을 연속으로 등록하고, 미션 사이를 Dubins 커넥터로 자동 연결하는 디버거

동작
- 좌클릭 2번: 현재 미션의 (시작, 끝) 점 입력
- Enter: 현재 2점 미션 확정
  - 두 번째 미션부터는, 직전 미션의 출구 heading(직전 미션 시작->끝) 과
    신규 미션의 입구 heading(신규 미션 시작->끝)을 만족하는 Dubins 경로를 생성해 연결

키
- Enter      : 현재 2점 미션 확정
- Esc        : 전체 초기화
- Backspace  : 마지막 확정 미션 제거(연결 경로도 같이 제거)
- T          : 선회 반경 R(미터) 변경
- S          : 샘플링 간격 step(미터) 변경
- R          : 화면 범위 리셋(전체 보기)
- H          : 도움말 출력

마우스
- 휠 스크롤  : 줌
- 우클릭 드래그: 팬

의존성
- numpy, matplotlib
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import matplotlib

try:
    matplotlib.use("TkAgg")
except Exception:
    pass

import matplotlib.pyplot as plt
import numpy as np


# ----------------------------
# Dubins (최소 선회반경, 전진만)
# ----------------------------
TAU = 2.0 * math.pi


def mod2pi(x: float) -> float:
    return x - TAU * math.floor(x / TAU)


def wrap_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def _dubins_LSL(alpha: float, beta: float, d: float):
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p2 = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp1 = math.atan2((math.cos(beta) - math.cos(alpha)), tmp0)
    t = mod2pi(-alpha + tmp1)
    q = mod2pi(beta - tmp1)
    return t, p, q


def _dubins_RSR(alpha: float, beta: float, d: float):
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p2 = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp1 = math.atan2((math.cos(alpha) - math.cos(beta)), tmp0)
    t = mod2pi(alpha - tmp1)
    q = mod2pi(-beta + tmp1)
    return t, p, q


def _dubins_LSR(alpha: float, beta: float, d: float):
    p2 = -2 + d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp2 = (
        math.atan2((-math.cos(alpha) - math.cos(beta)), (d + math.sin(alpha) + math.sin(beta)))
        - math.atan2(-2.0, p)
    )
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-beta + tmp2)
    return t, p, q


def _dubins_RSL(alpha: float, beta: float, d: float):
    p2 = -2 + d * d + 2 * math.cos(alpha - beta) - 2 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp2 = (
        math.atan2((math.cos(alpha) + math.cos(beta)), (d - math.sin(alpha) - math.sin(beta)))
        - math.atan2(2.0, p)
    )
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)
    return t, p, q


def _dubins_RLR(alpha: float, beta: float, d: float):
    tmp0 = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))) / 8
    if abs(tmp0) > 1.0:
        return None
    p = mod2pi(2 * math.pi - math.acos(tmp0))
    t = mod2pi(
        alpha
        - math.atan2((math.cos(alpha) - math.cos(beta)), (d - math.sin(alpha) + math.sin(beta)))
        + p / 2
    )
    q = mod2pi(alpha - beta - t + p)
    return t, p, q


def _dubins_LRL(alpha: float, beta: float, d: float):
    tmp0 = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))) / 8
    if abs(tmp0) > 1.0:
        return None
    p = mod2pi(2 * math.pi - math.acos(tmp0))
    t = mod2pi(
        -alpha
        - math.atan2((math.cos(alpha) - math.cos(beta)), (d + math.sin(alpha) - math.sin(beta)))
        + p / 2
    )
    q = mod2pi(beta - alpha - t + p)
    return t, p, q


@dataclass
class DubinsPath:
    q0: Tuple[float, float, float]   # (x,y,heading)
    rho: float
    theta: float
    alpha: float
    beta: float
    d: float
    word: str
    params: Tuple[float, float, float]  # (t,p,q) in normalized units
    length: float  # meters


def dubins_shortest_path(
    q0: Tuple[float, float, float],
    q1: Tuple[float, float, float],
    rho: float
) -> Optional[DubinsPath]:
    if rho <= 0:
        raise ValueError("rho must be > 0")

    x0, y0, th0 = q0
    x1, y1, th1 = q1

    dx = x1 - x0
    dy = y1 - y0
    D = math.hypot(dx, dy)

    # 거의 같은 점이면, 연결 자체를 생략하거나 아주 짧게 처리
    if D < 1e-9:
        # heading 차이가 크더라도 여기서는 경로를 만들지 않고 None 처리
        return None

    d = D / rho
    theta = math.atan2(dy, dx)
    alpha = mod2pi(th0 - theta)
    beta = mod2pi(th1 - theta)

    best: Optional[DubinsPath] = None

    candidates = [
        ("LSL", _dubins_LSL),
        ("LSR", _dubins_LSR),
        ("RSL", _dubins_RSL),
        ("RSR", _dubins_RSR),
        ("RLR", _dubins_RLR),
        ("LRL", _dubins_LRL),
    ]

    for word, fn in candidates:
        res = fn(alpha, beta, d)
        if res is None:
            continue
        t, p, q = res
        length = (t + p + q) * rho
        if best is None or length < best.length:
            best = DubinsPath(
                q0=q0, rho=rho, theta=theta, alpha=alpha, beta=beta, d=d,
                word=word, params=(t, p, q), length=length
            )
    return best


def dubins_sample_xy(path: DubinsPath, step_m: float) -> np.ndarray:
    """
    path를 polyline으로 샘플링해서 (N,2) 반환
    step_m: 샘플링 간격(미터)
    """
    step_m = max(float(step_m), 1e-6)
    ds = step_m / path.rho  # normalized step

    t, p, q = path.params
    word = path.word

    # rotated frame: start = (0,0,alpha), goal = (d,0,beta)
    x = 0.0
    y = 0.0
    hd = path.alpha

    pts = []

    def push():
        cg = math.cos(path.theta)
        sg = math.sin(path.theta)
        X = path.q0[0] + path.rho * (cg * x - sg * y)
        Y = path.q0[1] + path.rho * (sg * x + cg * y)
        pts.append((X, Y))

    push()

    seg_lens = (t, p, q)
    for seg_type, seg_len in zip(word, seg_lens):
        rem = float(seg_len)
        while rem > 1e-9:
            inc = min(ds, rem)

            if seg_type == "S":
                x += inc * math.cos(hd)
                y += inc * math.sin(hd)
            elif seg_type == "L":
                x += math.sin(hd + inc) - math.sin(hd)
                y += -math.cos(hd + inc) + math.cos(hd)
                hd = mod2pi(hd + inc)
            elif seg_type == "R":
                x += -math.sin(hd - inc) + math.sin(hd)
                y += math.cos(hd - inc) - math.cos(hd)
                hd = mod2pi(hd - inc)
            else:
                raise RuntimeError(f"unknown segment type: {seg_type}")

            rem -= inc
            push()

    return np.asarray(pts, dtype=float)


# ----------------------------
# 인터랙티브 앱
# ----------------------------
@dataclass
class Mission:
    p0: np.ndarray  # (2,)
    p1: np.ndarray  # (2,)

    @property
    def heading(self) -> float:
        v = self.p1 - self.p0
        return math.atan2(float(v[1]), float(v[0]))


class DubinsConnectorApp:
    def __init__(self):
        self.turn_radius_m = 450.0
        self.sample_step_m = 5.0
        self.default_view_size_m = 3000.0
        self.dta_speed_mps = 40.0
        self.dta_v_extra = 3.0
        self.dta_limit_m = 800.0
        self.dta_max_phi_deg = 90.0

        self.current_points: List[np.ndarray] = []
        self.missions: List[Mission] = []
        self.connectors: List[np.ndarray] = []  # list of (N,2) polylines
        self.dta_wp_points: List[Optional[np.ndarray]] = []
        self.dta_tangent_points: List[Optional[np.ndarray]] = []
        self.mid_points: List[Optional[np.ndarray]] = []

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title("Dubins Connector Debugger (2-point missions)")

        # plot artists
        self._artist_current_pts = self.ax.plot([], [], "ro", ms=6, zorder=5)[0]
        self._artist_current_seg = self.ax.plot([], [], "r--", lw=1.5, zorder=4)[0]

        self._artist_missions = []   # list of Line2D
        self._artist_connectors = [] # list of Line2D

        self._artist_dta_wp = self.ax.scatter(
            [], [], c="tab:green", marker="D", s=55, zorder=6
        )
        self._artist_dta_tangent = self.ax.scatter(
            [], [], c="tab:cyan", marker="s", s=40, zorder=6
        )
        self._artist_mid = self.ax.scatter(
            [], [], c="tab:orange", marker="o", s=45, zorder=6
        )

        self._status_text = self.ax.text(
            0.01, 0.99, "", transform=self.ax.transAxes,
            va="top", ha="left", fontsize=10
        )

        # pan/zoom state
        self._panning = False
        self._pan_press_xy = (0.0, 0.0)
        self._pan_xlim = (0.0, 1.0)
        self._pan_ylim = (0.0, 1.0)

        # connect events
        self.fig.canvas.mpl_connect("button_press_event", self.on_mouse_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_mouse_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_mouse_move)
        self.fig.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        self._reset_view()
        self._redraw()

    def print_help(self):
        print(
            "\n[Help]\n"
            "- 좌클릭 2번: 미션 시작/끝 점 입력\n"
            "- Enter: 현재 2점 미션 확정 (이후 자동 Dubins 커넥터 생성)\n"
            "- Esc: 전체 초기화\n"
            "- Backspace: 마지막 확정 미션 제거\n"
            "- T: 선회 반경 R 변경\n"
            "- S: 샘플링 간격 step 변경\n"
            "- R: 화면 전체 보기\n"
            "- 우클릭 드래그: 팬, 휠: 줌\n"
        )

    def on_mouse_press(self, event):
        if event.inaxes != self.ax:
            return

        # right click: start panning
        if event.button == 3:
            self._panning = True
            self._pan_press_xy = (event.xdata, event.ydata)
            self._pan_xlim = self.ax.get_xlim()
            self._pan_ylim = self.ax.get_ylim()
            return

        # left click: add point
        if event.button == 1:
            if event.xdata is None or event.ydata is None:
                return
            if len(self.current_points) >= 2:
                # 이미 2점이면 추가 입력은 무시
                return
            self.current_points.append(np.array([event.xdata, event.ydata], dtype=float))
            self._redraw()

    def on_mouse_release(self, event):
        if event.button == 3:
            self._panning = False

    def on_mouse_move(self, event):
        if not self._panning:
            return
        if event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return

        x0, y0 = self._pan_press_xy
        dx = x0 - event.xdata
        dy = y0 - event.ydata

        self.ax.set_xlim(self._pan_xlim[0] + dx, self._pan_xlim[1] + dx)
        self.ax.set_ylim(self._pan_ylim[0] + dy, self._pan_ylim[1] + dy)
        self.fig.canvas.draw_idle()

    def on_scroll(self, event):
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

        self.ax.set_xlim(x - new_width * relx, x + new_width * (1 - relx))
        self.ax.set_ylim(y - new_height * rely, y + new_height * (1 - rely))
        self.fig.canvas.draw_idle()

    def on_key(self, event):
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
            self._set_turn_radius()
            return

        if event.key in ("s", "S"):
            self._set_sample_step()
            return

        if event.key in ("r", "R"):
            self._reset_view()
            return

    def _compute_dta_distance(self, prev: Mission, cur: Mission) -> float:
        delta = wrap_pi(cur.heading - prev.heading)
        phi = abs(math.degrees(delta))
        if phi < 1e-6:
            return 0.0
        phi = min(phi, self.dta_max_phi_deg)
        dta = self.turn_radius_m * math.tan(math.radians(phi) / 2.0) + self.dta_v_extra * self.dta_speed_mps
        return min(dta, self.dta_limit_m)

    @staticmethod
    def _point_on_polyline(poly: np.ndarray, seg: np.ndarray, dist: float) -> np.ndarray:
        if len(poly) == 0:
            return np.zeros(2, dtype=float)
        if len(poly) == 1:
            return poly[0].copy()
        total = float(np.sum(seg))
        if total < 1e-9:
            return poly[-1].copy()
        dist = max(0.0, min(float(dist), total))
        acc = 0.0
        for i, length in enumerate(seg):
            if acc + length >= dist:
                if length < 1e-9:
                    return poly[i].copy()
                t = (dist - acc) / length
                return poly[i] + t * (poly[i + 1] - poly[i])
            acc += length
        return poly[-1].copy()

    def _dta_points_on_path(self, conn_xy: np.ndarray, dta: float) -> Optional[np.ndarray]:
        if conn_xy is None or len(conn_xy) == 0:
            return None
        if len(conn_xy) == 1:
            return np.vstack([conn_xy[0], conn_xy[0]])
        seg = np.linalg.norm(conn_xy[1:] - conn_xy[:-1], axis=1)
        total = float(np.sum(seg))
        if total < 1e-9:
            return np.vstack([conn_xy[-1], conn_xy[-1]])
        dta = max(0.0, min(float(dta), total * 0.49))
        p_in = self._point_on_polyline(conn_xy, seg, dta)
        p_out = self._point_on_polyline(conn_xy, seg, total - dta)
        return np.vstack([p_in, p_out])

    def _path_midpoint(self, conn_xy: np.ndarray) -> Optional[np.ndarray]:
        if conn_xy is None or len(conn_xy) == 0:
            return None
        if len(conn_xy) == 1:
            return conn_xy[0].copy()
        seg = np.linalg.norm(conn_xy[1:] - conn_xy[:-1], axis=1)
        total = float(np.sum(seg))
        if total < 1e-9:
            return conn_xy[-1].copy()
        return self._point_on_polyline(conn_xy, seg, total * 0.5)

    @staticmethod
    def _advance_segment(x: float, y: float, hd: float, seg_type: str, seg_len: float):
        if seg_type == "S":
            x += seg_len * math.cos(hd)
            y += seg_len * math.sin(hd)
        elif seg_type == "L":
            x += math.sin(hd + seg_len) - math.sin(hd)
            y += -math.cos(hd + seg_len) + math.cos(hd)
            hd = mod2pi(hd + seg_len)
        elif seg_type == "R":
            x += -math.sin(hd - seg_len) + math.sin(hd)
            y += math.cos(hd - seg_len) - math.cos(hd)
            hd = mod2pi(hd - seg_len)
        else:
            raise RuntimeError(f"unknown segment type: {seg_type}")
        return x, y, hd

    def _dubins_straight_tangents(self, path: DubinsPath) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if "S" not in path.word:
            return None
        t, p, q = path.params
        x, y, hd = 0.0, 0.0, path.alpha
        start = None
        end = None
        for seg_type, seg_len in zip(path.word, (t, p, q)):
            x0, y0, hd0 = x, y, hd
            x, y, hd = self._advance_segment(x, y, hd, seg_type, seg_len)
            if seg_type == "S":
                start = np.array([x0, y0], dtype=float)
                end = np.array([x, y], dtype=float)
                break
        if start is None or end is None:
            return None
        cg = math.cos(path.theta)
        sg = math.sin(path.theta)

        def to_world(pt: np.ndarray) -> np.ndarray:
            xw = path.q0[0] + path.rho * (cg * pt[0] - sg * pt[1])
            yw = path.q0[1] + path.rho * (sg * pt[0] + cg * pt[1])
            return np.array([xw, yw], dtype=float)

        return to_world(start), to_world(end)

    def _compute_dta_markers(
        self, prev: Mission, cur: Mission, path: Optional[DubinsPath], conn_xy: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        dta = self._compute_dta_distance(prev, cur)
        dta_points = self._dta_points_on_path(conn_xy, dta)
        tangents = None
        if path is not None:
            tangent_pair = self._dubins_straight_tangents(path)
            if tangent_pair is not None:
                tangents = np.vstack(tangent_pair)
        return dta_points, tangents

    def _set_turn_radius(self):
        try:
            val = input(f"turn radius R (meters), current={self.turn_radius_m:.3f}: ").strip()
            if not val:
                return
            r = float(val)
            if r <= 0:
                print("R must be > 0")
                return
            self.turn_radius_m = r
            # 기존 커넥터 재계산
            self._rebuild_connectors()
            self._redraw()
        except Exception as e:
            print(f"failed to set R: {e}")

    def _set_sample_step(self):
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
            self._redraw()
        except Exception as e:
            print(f"failed to set step: {e}")

    def _clear_all(self):
        self.current_points.clear()
        self.missions.clear()
        self.connectors.clear()
        self.dta_wp_points.clear()
        self.dta_tangent_points.clear()
        self.mid_points.clear()

        for a in self._artist_missions:
            a.remove()
        for a in self._artist_connectors:
            a.remove()
        self._artist_missions.clear()
        self._artist_connectors.clear()

        self._redraw()

    def _pop_last_mission(self):
        if not self.missions:
            return
        self.missions.pop()
        if self.connectors:
            self.connectors.pop()
            if self.dta_wp_points:
                self.dta_wp_points.pop()
            if self.dta_tangent_points:
                self.dta_tangent_points.pop()
            if self.mid_points:
                self.mid_points.pop()

        if self._artist_missions:
            a = self._artist_missions.pop()
            a.remove()
        if self._artist_connectors:
            a = self._artist_connectors.pop()
            a.remove()

        self._redraw()

    def _commit_current_mission(self):
        if len(self.current_points) != 2:
            print("현재 미션은 점 2개가 필요합니다")
            return

        p0 = self.current_points[0].copy()
        p1 = self.current_points[1].copy()

        # 너무 가까우면 무시
        if float(np.hypot(*(p1 - p0))) < 1e-9:
            print("두 점이 너무 가깝습니다")
            return

        new_mission = Mission(p0=p0, p1=p1)
        self.missions.append(new_mission)

        # 미션 라인 추가
        line = self.ax.plot([p0[0], p1[0]], [p0[1], p1[1]], lw=2.5)[0]
        self._artist_missions.append(line)

        # 커넥터 생성 (두 번째 미션부터)
        if len(self.missions) >= 2:
            prev = self.missions[-2]
            cur = self.missions[-1]

            q0 = (float(prev.p1[0]), float(prev.p1[1]), float(prev.heading))
            q1 = (float(cur.p0[0]), float(cur.p0[1]), float(cur.heading))

            path = dubins_shortest_path(q0, q1, rho=self.turn_radius_m)
            if path is None:
                print("Dubins 경로를 만들지 못했습니다 (점이 너무 가깝거나 수치 문제일 수 있음)")
                conn_xy = np.asarray([[q0[0], q0[1]], [q1[0], q1[1]]], dtype=float)
            else:
                conn_xy = dubins_sample_xy(path, step_m=self.sample_step_m)
            dta_points, tangents = self._compute_dta_markers(prev, cur, path, conn_xy)
            mid = self._path_midpoint(conn_xy)

            self.connectors.append(conn_xy)
            conn_line = self.ax.plot(conn_xy[:, 0], conn_xy[:, 1], lw=2.0)[0]
            self._artist_connectors.append(conn_line)
            self.dta_wp_points.append(dta_points)
            self.dta_tangent_points.append(tangents)
            self.mid_points.append(mid)

            print(
                f"connector {len(self.connectors)}: "
                f"type={path.word if path else 'N/A'}, "
                f"len={path.length if path else float('nan'):.2f} m, "
                f"R={self.turn_radius_m:.2f}, step={self.sample_step_m:.2f}"
            )

        # 현재 입력 초기화
        self.current_points.clear()
        self._redraw()

    def _rebuild_connectors(self):
        self.connectors.clear()
        for a in self._artist_connectors:
            a.remove()
        self._artist_connectors.clear()
        self.dta_wp_points.clear()
        self.dta_tangent_points.clear()
        self.mid_points.clear()

        if len(self.missions) < 2:
            return

        for i in range(1, len(self.missions)):
            prev = self.missions[i - 1]
            cur = self.missions[i]
            q0 = (float(prev.p1[0]), float(prev.p1[1]), float(prev.heading))
            q1 = (float(cur.p0[0]), float(cur.p0[1]), float(cur.heading))

            path = dubins_shortest_path(q0, q1, rho=self.turn_radius_m)
            if path is None:
                conn_xy = np.asarray([[q0[0], q0[1]], [q1[0], q1[1]]], dtype=float)
            else:
                conn_xy = dubins_sample_xy(path, step_m=self.sample_step_m)
            dta_points, tangents = self._compute_dta_markers(prev, cur, path, conn_xy)
            mid = self._path_midpoint(conn_xy)

            self.connectors.append(conn_xy)
            conn_line = self.ax.plot(conn_xy[:, 0], conn_xy[:, 1], lw=2.0)[0]
            self._artist_connectors.append(conn_line)
            self.dta_wp_points.append(dta_points)
            self.dta_tangent_points.append(tangents)
            self.mid_points.append(mid)

    def _reset_view(self):
        half = self.default_view_size_m / 2.0
        pts = []
        for m in self.missions:
            pts.append(m.p0)
            pts.append(m.p1)
        for c in self.connectors:
            if len(c) > 0:
                pts.append(c.min(axis=0))
                pts.append(c.max(axis=0))
        for p in self.dta_wp_points:
            if p is not None:
                for row in p:
                    pts.append(row)
        for t in self.dta_tangent_points:
            if t is not None:
                pts.append(t[0])
                pts.append(t[1])
        for p in self.mid_points:
            if p is not None:
                pts.append(p)
        for p in self.current_points:
            pts.append(p)

        if not pts:
            self.ax.set_xlim(-half, half)
            self.ax.set_ylim(-half, half)
            self.fig.canvas.draw_idle()
            return

        P = np.vstack(pts)
        xmin, ymin = P.min(axis=0)
        xmax, ymax = P.max(axis=0)
        cx = (xmin + xmax) * 0.5
        cy = (ymin + ymax) * 0.5
        self.ax.set_xlim(cx - half, cx + half)
        self.ax.set_ylim(cy - half, cy + half)
        self.fig.canvas.draw_idle()

    def _redraw(self):
        # current points
        if self.current_points:
            xs = [p[0] for p in self.current_points]
            ys = [p[1] for p in self.current_points]
        else:
            xs, ys = [], []
        self._artist_current_pts.set_data(xs, ys)

        # current preview segment
        if len(self.current_points) == 2:
            p0, p1 = self.current_points
            self._artist_current_seg.set_data([p0[0], p1[0]], [p0[1], p1[1]])
        else:
            self._artist_current_seg.set_data([], [])

        wp_list = [p for p in self.dta_wp_points if p is not None]
        if wp_list:
            wp_pts = np.vstack(wp_list)
        else:
            wp_pts = np.empty((0, 2), dtype=float)
        self._artist_dta_wp.set_offsets(wp_pts)

        tangent_list = [p for p in self.dta_tangent_points if p is not None]
        if tangent_list:
            tangent_pts = np.vstack(tangent_list)
        else:
            tangent_pts = np.empty((0, 2), dtype=float)
        self._artist_dta_tangent.set_offsets(tangent_pts)

        mid_list = [p for p in self.mid_points if p is not None]
        if mid_list:
            mid_pts = np.vstack(mid_list)
        else:
            mid_pts = np.empty((0, 2), dtype=float)
        self._artist_mid.set_offsets(mid_pts)

        self._status_text.set_text(
            f"missions={len(self.missions)}  connectors={len(self.connectors)}\n"
            f"R={self.turn_radius_m:.2f} m  step={self.sample_step_m:.2f} m\n"
            f"current_points={len(self.current_points)}/2"
        )

        self.fig.canvas.draw_idle()


def main():
    app = DubinsConnectorApp()
    app.print_help()
    plt.show()


if __name__ == "__main__":
    main()
