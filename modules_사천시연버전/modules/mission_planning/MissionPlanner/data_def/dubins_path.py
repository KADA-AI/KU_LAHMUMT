"""
dubins_path.py
--------------------------------------------
Minimal‐dependency Dubins path solver & sampler
for missionPlanner.  This file is **stand‑alone**
(No external libraries required).

A Dubins path is the shortest curve that joins two
poses q0=(x, y, yaw) and q1=(x, y, yaw) for a forward‑
only vehicle with (signed) bounded curvature 1/R.

Implemented path types  (6 canonical):
    LSL, LSR, RSL, RSR, RLR, LRL

Typical usage
-------------
>>> import math
>>> from dubins_path import sample_dubins_path
>>> q0 = (0.0, 0.0, math.radians( 45))
>>> q1 = (80.0, 50.0, math.radians(-90))
>>> pts = sample_dubins_path(q0, q1, R=15.0, step=1.0)

Functions
---------
• mod2pi(theta)                         → θ ∈ [0, 2π)
• dubins_shortest_path(q0, q1, R)       → dict{length, seg_lengths, types}
• sample_dubins_path(q0, q1, R, step)   → List[(x, y, yaw, s)]

All units are **meters** and **radians**.
"""

from __future__ import annotations
import math
from typing import Tuple, List, Dict

# ──────────────────────────────────────────────
# 1) Helpers
# ──────────────────────────────────────────────

def mod2pi(theta: float) -> float:
    """Wrap angle to [0, 2π)."""
    return theta % (2.0 * math.pi)


def _polar(x: float, y: float) -> Tuple[float, float]:
    """Return (r, theta) in radians (theta ∈ [0, 2π))."""
    r = math.hypot(x, y)
    th = mod2pi(math.atan2(y, x))
    return r, th


# ──────────────────────────────────────────────
# 2) Segment‑specific primitive solvers
#    Each returns (t, p, q) in **radians** (arc lengths / R) or
#    linear length / R for the middle segment.
#    If no feasible path, returns None.
# ──────────────────────────────────────────────

def _LSL(alpha: float, beta: float, d: float):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    tmp = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (sa - sb)
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    t = mod2pi(math.atan2(cb - ca, d + sa - sb) - alpha)
    q = mod2pi(beta - math.atan2(cb - ca, d + sa - sb))
    return t, p, q


def _RSR(alpha: float, beta: float, d: float):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    tmp = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (sb - sa)
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    t = mod2pi(alpha - math.atan2(cb - ca, d - sa + sb))
    q = mod2pi(beta - alpha + t - p)
    return t, p, q


def _LSR(alpha: float, beta: float, d: float):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    tmp = -2 + d * d + 2 * math.cos(alpha - beta) + 2 * d * (sa + sb)
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    t = mod2pi(math.atan2(-ca - cb, d + sa + sb) - alpha)
    q = mod2pi(math.atan2(-ca - cb, d + sa + sb) - beta)
    return t, p, q


def _RSL(alpha: float, beta: float, d: float):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    tmp = d * d - 2 + 2 * math.cos(alpha - beta) - 2 * d * (sa + sb)
    if tmp < 0:
        return None
    p = math.sqrt(tmp)
    t = mod2pi(alpha - math.atan2(ca + cb, d - sa - sb))
    q = mod2pi(beta - math.atan2(ca + cb, d - sa - sb))
    return t, p, q


def _RLR(alpha: float, beta: float, d: float):
    tmp = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))) / 8
    if abs(tmp) > 1:
        return None
    p = mod2pi(2 * math.pi - math.acos(tmp))
    t = mod2pi(alpha - math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta)) + p / 2)
    q = mod2pi(alpha - beta - t + mod2pi(p))
    return t, p, q


def _LRL(alpha: float, beta: float, d: float):
    tmp = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))) / 8
    if abs(tmp) > 1:
        return None
    p = mod2pi(2 * math.pi - math.acos(tmp))
    t = mod2pi(-alpha - math.atan2(math.cos(alpha) - math.cos(beta), d + math.sin(alpha) - math.sin(beta)) + p / 2)
    q = mod2pi(beta - alpha - t + mod2pi(p))
    return t, p, q


_PATH_FUNCS = {
    "LSL": _LSL,
    "RSR": _RSR,
    "LSR": _LSR,
    "RSL": _RSL,
    "RLR": _RLR,
    "LRL": _LRL,
}

# ──────────────────────────────────────────────
# 3) Shortest path search
# ──────────────────────────────────────────────

def dubins_shortest_path(q0: Tuple[float, float, float],
                         q1: Tuple[float, float, float],
                         R: float) -> Dict:
    """Return the shortest Dubins path between q0 and q1.

    q = (x, y, yaw) with yaw in radians.
    R = min turning radius (m)

    Returns dict with keys:
        'length'       total length (m)
        'seg_lengths'  [l1, l2, l3] in meters
        'types'        str, e.g. 'LSL'
        'params'       (t, p, q) in radians
    """
    (x0, y0, th0) = q0
    (x1, y1, th1) = q1

    # Transform to origin frame of q0
    dx = (x1 - x0) / R
    dy = (y1 - y0) / R
    d, theta = _polar(dx, dy)
    alpha = mod2pi(th0 - theta)
    beta  = mod2pi(th1 - theta)

    best = None
    for key, fn in _PATH_FUNCS.items():
        res = fn(alpha, beta, d)
        if res is None:
            continue
        t, p, q = res
        length = (t + p + q) * R
        if (best is None) or (length < best['length']):
            best = {
                'length': length,
                'seg_lengths': [t*R, p*R, q*R],
                'types': key,
                'params': (t, p, q),
                'alpha': alpha, 'beta': beta, 'd': d, 'theta': theta,
            }
    if best is None:
        raise ValueError("No feasible Dubins path found (possibly R too small)")
    return best

# ──────────────────────────────────────────────
# 4) Path sampler
# ──────────────────────────────────────────────

def _segment(state, seg_type: str, seg_length: float, R: float, step: float,
             out: List):
    """Advance along one segment, append points to `out`."""
    (x, y, th) = state
    traveled = 0.0
    while traveled < seg_length - 1e-6:
        ds = min(step, seg_length - traveled)
        if seg_type == 'S':
            x += ds * math.cos(th)
            y += ds * math.sin(th)
        elif seg_type == 'L':
            dtheta = ds / R
            x += R * (math.sin(th + dtheta) - math.sin(th))
            y += R * (-math.cos(th + dtheta) + math.cos(th))
            th = mod2pi(th + dtheta)
        else:  # 'R'
            dtheta = ds / R
            x += R * (-math.sin(th - dtheta) + math.sin(th))
            y += R * ( math.cos(th - dtheta) - math.cos(th))
            th = mod2pi(th - dtheta)
        traveled += ds
        out.append((x, y, th))
    return (x, y, th)


def sample_dubins_path(q0: Tuple[float, float, float],
                       q1: Tuple[float, float, float],
                       R: float,
                       step: float = 1.0) -> List[Tuple[float, float, float, float]]:
    """Sample the shortest Dubins path every `step` metres.

    Returns list of (x, y, yaw, s) where s is cumulative arc‑length (m).
    First point is q0 (s=0), last point equals q1 (approx)."""
    info = dubins_shortest_path(q0, q1, R)
    (t, p, q) = info['params']
    segs = list(info['types'])
    seg_lengths = [t*R, p*R, q*R]
    pts: List[Tuple[float, float, float, float]] = [(q0[0], q0[1], mod2pi(q0[2]), 0.0)]

    state = (q0[0], q0[1], mod2pi(q0[2]))
    s_total = 0.0
    for s_type, seg_len in zip(segs, seg_lengths):
        state = _segment(state, s_type, seg_len, R, step, [])
        # Sampling done separately to accumulate points at constant step
        # Append internal points produced by _segment (if any)
        x, y, th = state
        s_total += seg_len
        pts.append((x, y, th, s_total))

    # Adjust last pose to exactly q1 (numerical clean‑up)
    pts[-1] = (q1[0], q1[1], mod2pi(q1[2]), info['length'])
    return pts

# ──────────────────────────────────────────────
# End of file
# ──────────────────────────────────────────────
