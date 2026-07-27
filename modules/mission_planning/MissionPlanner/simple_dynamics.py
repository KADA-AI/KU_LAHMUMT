"""
Very‑simple 2‑D fixed‑wing / quad‑transition model & autopilot
================================================================
• **Speed limits:** 28 – 55 m/s  (table: 임무 운용 속도)
• **Turn performance:** bank‑angle limited to ±30 °, giving radii ≈340–560 m
  (table: 선회반경 30 m/s→340 m, 40 m/s→450 m, 50 m/s→560 m)
• **Modes:**
    – *Quad mode* (hover)   : |v| < 5 m/s  →👇 position hold only
    – *Fixed‑wing mode*     : otherwise    →👇 coordinated turn kinematics

The code keeps everything extremely compact so you can copy–paste the whole file
into your project.  All units are SI (metres, seconds, radians).
"""

from __future__ import annotations
import math
from typing import List, Tuple

from modules.common.turn_dynamics import (
    advance_roll_toward_target,
    coordinated_turn_rate_nav_dps,
)

# ────────────────────────────────────────────────
# ①  CONSTANTS (table‑based)
# ────────────────────────────────────────────────
V_MIN   = 28.0                    # m/s   (임무 운용 속도 하한)
V_MAX   = 55.0                    # m/s   (임무 운용 속도 상한)
PHI_MAX = math.radians(30.0)      # ±30° bank‑angle limit
HOVER_THR = 5.0                   # <5 m/s → Quad mode

# ────────────────────────────────────────────────
# ②  DYNAMICS :   Simple 2‑D point‑mass aircraft
# ────────────────────────────────────────────────
class SimpleUAV:
    """Extremely lightweight 2‑D kinematic model.

    State vector : (x [m], y [m], ψ [rad], v [m/s])
                  heading ψ is measured clockwise from +X (E‑axis).
    Control      : (φ_cmd [rad], v_cmd [m/s]) – bank and speed commands.
                   φ_cmd is ignored in quad‑mode.
    """
    def __init__(self,
                 x0: float = 0.0,
                 y0: float = 0.0,
                 heading_deg: float = 0.0,
                 v0: float = 30.0,
                 tau_v: float = 2.0,
                 max_roll_rate_dps: float = 40.0,
                 roll_time_constant_s: float = 0.45):
        self.x   = x0
        self.y   = y0
        self.psi = math.radians(heading_deg)
        self.v   = max(min(v0, V_MAX), V_MIN)
        self.phi = 0.0
        self._tau_v = tau_v          # speed time-constant [s]
        self._max_roll_rate_dps = max(1.0, float(max_roll_rate_dps))
        self._roll_time_constant_s = max(0.05, float(roll_time_constant_s))

    # ───────────────────────────────────
    def step(self, dt: float, phi_cmd: float, v_cmd: float):
        """Integrate one time-step (Euler fwd)."""
        # 1) Saturations & mode-switch
        v_cmd = max(min(v_cmd, V_MAX), V_MIN)
        hover = abs(self.v) < HOVER_THR and abs(v_cmd) < HOVER_THR
        if not hover:
            phi_cmd = max(min(phi_cmd, PHI_MAX), -PHI_MAX)
        else:
            phi_cmd = 0.0                    # hover → bank 무시

        # 2) First-order speed response
        self.v += (v_cmd - self.v) * dt / self._tau_v

        # 3) Heading / position propagation
        if hover:            # 쿼드 모드일 땐 위치 고정
            return

        next_roll_deg = advance_roll_toward_target(
            math.degrees(self.phi),
            math.degrees(phi_cmd),
            dt,
            bank_limit_deg=math.degrees(PHI_MAX),
            roll_rate_limit_dps=self._max_roll_rate_dps,
            roll_time_constant_s=self._roll_time_constant_s,
        )
        self.phi = math.radians(float(next_roll_deg or 0.0))
        turn_rate_dps = coordinated_turn_rate_nav_dps(math.degrees(self.phi), self.v) or 0.0
        self.psi += math.radians(turn_rate_dps) * dt

        # ★ y 축은 ‘시계방향 +’ 기준이므로 부호 반전
        self.x += self.v * math.cos(self.psi) * dt
        self.y -= self.v * math.sin(self.psi) * dt

    # Convenience --------------------------------------------------
    @property
    def pos(self) -> Tuple[float, float]:
        return self.x, self.y

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.psi) % 360.0

    # Debug print --------------------------------------------------
    def __repr__(self):
        return (f"UAV(x={self.x:.1f}, y={self.y:.1f}, ψ={self.heading_deg:5.1f}°, "
                f"v={self.v:4.1f})")

# ────────────────────────────────────────────────
# ③  AUTOPILOT :   straight‑line & waypoint sequencer
# ────────────────────────────────────────────────
class VerySimpleAutopilot:
    """Bare-bones lateral logic using proportional bank control."""
    def __init__(self, waypoints: List[Tuple[float, float]],
                 v_cruise: float = 40.0,
                 kp_phi: float = 2.0,
                 arrival_tol: float = 50.0):
        self.wps          = waypoints
        self._idx         = 0
        self.v_cmd        = v_cruise
        self.kp_phi       = kp_phi
        self.arrival_tol2 = arrival_tol ** 2
        self.done         = False          # ★ 새 플래그

    # -------------------------------------------------------------
    def control(self, uav: 'SimpleUAV') -> Tuple[float, float]:
        """
        Return (φ_cmd, v_cmd).

        • 선 통과 + 거리 오차 판정으로 WP 전환
        • 마지막 WP에 도달하면 self.done = True
        """
        # ── 이미 끝났으면 0,0 반환 ───────────────
        if self.done:
            return 0.0, 0.0

        # ── 현재·이전 WP 좌표 ───────────────────
        tx, ty = self.wps[self._idx]
        px, py = (self.wps[self._idx - 1] if self._idx > 0
                  else (uav.x, uav.y))

        dx, dy = tx - uav.x, ty - uav.y
        dist2  = dx*dx + dy*dy                   # 거리²

        # ── 선 통과 여부 ------------------------
        abx, aby = tx - px, ty - py
        ab_len2  = abx*abx + aby*aby or 1e-6
        proj     = (dx*abx + dy*aby) / ab_len2   # <0 ⇒ 선을 넘어섬

        arrived = (dist2 <= self.arrival_tol2) or (proj < 0.0)

        # ── 마지막 WP?  → 종료(hover) ───────────
        if arrived and self._idx == len(self.wps) - 1:
            self.done = True
            return 0.0, 0.0

        # ── 중간 WP 도착 → 다음 WP ─────────────
        if arrived:
            self._idx += 1
            return self.control(uav)             # 재귀로 새 WP 처리

        # ── 뱅크 명령 계산 ────────────────────
        desired_psi = math.atan2(-dy, dx)        # 시계방향 +Y
        err = _wrap_pi(desired_psi - uav.psi)
        phi_cmd = max(min(self.kp_phi * err, PHI_MAX), -PHI_MAX)
        return phi_cmd, self.v_cmd


# ────────────────────────────────────────────────
# ④  Helper : angle wrapping
# ────────────────────────────────────────────────

def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2*math.pi) - math.pi

# ────────────────────────────────────────────────
# ⑤  QUICK DEMO      (python simple_uav_model.py)
# ────────────────────────────────────────────────
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Waypoints in metres (ENU frame)
    WPS = [
        (-2616.454, -2121.978),
        (-2602.240, -1972.598),
        (-2588.231, -1822.422),
        (-2574.366, -1671.430),
        (-2560.580, -1519.611),
        (-2546.805, -1366.958),
        (-2532.969, -1213.471),
        (-2518.999, -1059.159),
        (-2504.822, -904.034),
        (-2490.364, -748.121),
        (-2475.553, -591.448),
        (-2460.319, -434.051),
        (-2444.597, -275.974),
        (-2428.323, -117.266),
        (-2411.443,   42.020),
        (-2393.906,  201.825),
        (-2375.669,  362.089),
        (-2356.694,  522.746),
        (-2336.954,  683.733),
        (-2316.425,  844.985),
        (-2295.093, 1006.436),
        (-2272.949, 1168.024),   # ← 중복 하나만
        (-2151.083, 1205.458),
        (-2028.710, 1245.531),
        (-1905.508, 1288.242),
        (-1781.129, 1333.520),
        (-1655.214, 1381.218),
        (-1527.407, 1431.106),
        (-1397.381, 1482.872),
        (-1264.851, 1536.129),
        (-1129.599, 1590.429),
        (-991.485,  1645.280),
        (-850.457,  1700.178),
        (-706.551,  1754.623),
        (-559.885,  1808.150),
        (-410.649,  1860.349),
        (-259.086,  1910.873),
        (-105.475,  1959.453),
        (  49.887,  2005.893),
        ( 206.701,  2050.071),   # ← 중복 하나만
        ( 331.045,  1978.877),
        ( 455.723,  1907.675),
        ( 580.734,  1836.446),
        ( 706.075,  1765.171),
        ( 831.745,  1693.830),
        ( 957.739,  1622.406),
        (1084.050,  1550.878),
        (1210.673,  1479.228),
        (1337.599,  1407.438),
        (1464.820,  1335.491),
        (1592.326,  1263.368),
        (1720.106,  1191.054),
        (1848.148,  1118.532),
        (1976.440,  1045.788),
        (2104.968,   972.808),
        (2233.718,   899.578),
        (2362.677,   826.087),
    ]

    # 1) 첫 두 WP
    x0, y0 = WPS[0]           # 시작 위치
    x1, y1 = WPS[1]           # 그다음 WP

    # 2) 첫 WP → 두번째 WP 방향각 (시계방향 +X 기준)
    hdg0_rad = math.atan2(-(y1 - y0), x1 - x0)   # 라디안
    hdg0_deg = math.degrees(hdg0_rad)            # 디그리로 변환

    # 3) UAV/Plane 초기화
    uav = SimpleUAV(x0, y0, heading_deg=hdg0_deg, v0=40)
    ap   = VerySimpleAutopilot(WPS, v_cruise=40.0)

    xs, ys = [], []
    DT = 0.1
    for _ in range(6000):                 # 600 s max
        phi_cmd, v_cmd = ap.control(uav)

        if ap.done:                       # ★ 마지막 WP 도달 → 종료
            break

        uav.step(DT, phi_cmd, v_cmd)
        xs.append(uav.x); ys.append(uav.y)

    # ── Plot ------------------------------------------------------
    plt.figure(figsize=(6,6))
    plt.plot(xs, ys, label="trajectory")
    wpx, wpy = zip(*WPS)
    plt.scatter(wpx, wpy, c="red", label="waypoints")
    plt.gca().set_aspect('equal', 'box')
    plt.title("Very‑simple UAV guidance demo")
    plt.xlabel("East [m]"); plt.ylabel("North [m]")
    plt.legend(); plt.grid(True)
    plt.show()
