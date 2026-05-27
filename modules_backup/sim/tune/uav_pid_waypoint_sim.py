"""
UAV waypoint-following + PID tuning utilities (5Hz-friendly).

- Uses modules.sim.runtime.uav UAV dynamics.
- Provides simulate_waypoints(), tune_pid_gains(), sweep_time_scales().
- Includes straight-line smoothness penalty (oscillation) in scoring.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Tuple

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from modules.sim.config import clamp, wrap_deg
from modules.sim.runtime.controllers.waypoint_pid import WaypointPIDController, WaypointTarget
from modules.sim.runtime.uav import UAV, UAVParams


# -----------------------------
# Gains
# -----------------------------
@dataclass
class PIDGains:
    yaw: float = 0.9
    yaw_i: float = 0.0
    pitch_rate: float = 1.4
    pitch_damp: float = 1.0
    alt_pitch: float = 0.03
    alt_i: float = 0.0
    throttle: float = 0.1
    throttle_alt: float = 0.0008
    throttle_i: float = 0.0
    lookahead_m: float = 120.0
    freeze_yaw_dist: float = 50.0
    freeze_alt_ratio: float = 0.5


# Straight-line smoothness (oscillation) penalty config
OSC_TURN_MAX_DEG = 12.0
OSC_TURN_DIST_MULT = 1.5


# -----------------------------
# Save / Load helpers (atomic)
# -----------------------------
def _save_json_atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def save_gains(path: str | Path, gains: PIDGains) -> None:
    _save_json_atomic(Path(path), gains.__dict__)


def save_report(path: str | Path, report: dict) -> None:
    _save_json_atomic(Path(path), report)


def load_gains(path: str | Path) -> PIDGains | None:
    p = Path(path)
    if not p.exists():
        return None
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        if isinstance(data, dict) and "gains" in data and isinstance(data["gains"], dict):
            data = data["gains"]
        if not isinstance(data, dict):
            return None
        return PIDGains(**data)
    except Exception as exc:
        print(f"[load] failed to load {p}: {exc}")
        return None


# -----------------------------
# Simulation
# -----------------------------
def simulate_waypoints(
    waypoints: Iterable[Tuple[float, float, float]],
    dt: float = 0.01,
    total_time: float = 120.0,
    speed_target: float = 80.0,
    pos_tol: float = 30.0,
    gains: PIDGains = PIDGains(),
    return_errors: bool = False,
    start_offset_xy: Tuple[float, float] = (-300.0, -300.0),
    start_yaw_deg: float | None = None,
    start_speed: float | None = None,
    return_info: bool = False,
):
    uav = UAV(UAVParams())
    wp_list: List[Tuple[float, float, float]] = list(waypoints)
    if not wp_list:
        raise ValueError("waypoints list is empty")

    uav.s.x = wp_list[0][0] + float(start_offset_xy[0])
    uav.s.y = wp_list[0][1] + float(start_offset_xy[1])
    uav.s.z = wp_list[0][2]
    if start_yaw_deg is not None:
        uav.s.yaw = wrap_deg(float(start_yaw_deg))
    else:
        init_dx = wp_list[0][0] - uav.s.x
        init_dy = wp_list[0][1] - uav.s.y
        if abs(init_dx) + abs(init_dy) > 1e-6:
            uav.s.yaw = wrap_deg(math.degrees(math.atan2(-init_dy, init_dx)))
    if start_speed is not None:
        uav.s.u = float(start_speed)

    targets = [WaypointTarget(pos=(float(x), float(y), float(z))) for (x, y, z) in wp_list]
    controller = WaypointPIDController(
        uav,
        targets,
        gains=gains,
        speed_target=float(speed_target),
        pos_tol=float(pos_tol),
        name="uav-tune",
        allow_hover=False,
        hold_on_complete=False,
    )

    traj: list[tuple[float, float, float]] = []

    t = 0.0
    errs_xy: list[float] = []
    errs_alt: list[float] = []

    prev_x = uav.s.x
    prev_y = uav.s.y
    prev_heading = None
    turn_sum = 0.0
    turn_dist = 0.0
    osc_turn_max = math.radians(OSC_TURN_MAX_DEG)

    yaw_int_max = 100.0
    alt_int_max = 10.0
    speed_int_max = 10.0

    sat_yaw = 0
    sat_pitch = 0
    sat_roll = 0
    sat_thr = 0
    effort = 0.0
    min_u = float("inf")
    aborted = False

    eps = 1e-6

    max_roll_rate = getattr(uav.p, "max_roll_rate_dps", None)
    if max_roll_rate is None:
        max_roll_rate = 1e9
    max_roll_rate = float(max_roll_rate)
    while t < total_time:
        target = controller.current_target()
        if target is None:
            break
        tx, ty, tz = target.pos
        dx = float(tx - uav.s.x)
        dy = float(ty - uav.s.y)
        dz_wp = float(tz - uav.s.z)
        dist_xy = math.hypot(dx, dy)

        updated = controller.update(dt)
        yaw_rate_limit = float(getattr(uav, "turn_rate_limit_dps", lambda: uav.p.max_yaw_rate_dps)())
        yaw_rate_limit = max(yaw_rate_limit, 0.0)

        if abs(uav.cmd_yaw_rate) >= max(yaw_rate_limit - eps, 0.0):
            sat_yaw += 1
        if abs(uav.cmd_pitch_rate) >= uav.p.max_pitch_rate_dps - eps:
            sat_pitch += 1
        if abs(uav.cmd_roll_rate) >= max_roll_rate - eps:
            sat_roll += 1
        if abs(uav.cmd_throttle) >= 1.0 - eps:
            sat_thr += 1

        effort += (
            abs(uav.cmd_yaw_rate) / max(yaw_rate_limit, eps)
            + abs(uav.cmd_pitch_rate) / (uav.p.max_pitch_rate_dps + eps)
            + abs(uav.cmd_roll_rate) / (max_roll_rate + eps)
            + abs(uav.cmd_throttle)
        ) * dt

        uav.step(dt)

        dx_step = uav.s.x - prev_x
        dy_step = uav.s.y - prev_y
        step_dist = math.hypot(dx_step, dy_step)
        if step_dist > 1e-6:
            heading = math.atan2(dy_step, dx_step)
            if prev_heading is not None:
                dhead = ((heading - prev_heading + math.pi) % (2 * math.pi)) - math.pi
                dist_xy2 = math.hypot(float(tx - uav.s.x), float(ty - uav.s.y))
                if dist_xy2 > pos_tol * OSC_TURN_DIST_MULT and abs(dhead) <= osc_turn_max:
                    turn_sum += abs(dhead)
                    turn_dist += step_dist
            prev_heading = heading
        prev_x = uav.s.x
        prev_y = uav.s.y

        if not (np.isfinite(uav.s.x) and np.isfinite(uav.s.y) and np.isfinite(uav.s.z) and np.isfinite(uav.s.u)):
            aborted = True
            break
        if abs(uav.s.z) > 20000.0:
            aborted = True
            break

        traj.append((uav.s.x, uav.s.y, uav.s.z))
        errs_xy.append(dist_xy)
        errs_alt.append(abs(dz_wp))
        min_u = min(min_u, float(uav.s.u))
        t += dt
        if controller.finished or not updated:
            break

    traj_arr = np.array(traj) if traj else np.zeros((0, 3), dtype=float)

    info = {
        "finished": bool(controller.finished),
        "curr_idx": int(getattr(controller, "curr_idx", 0)),
        "t": float(t),
        "steps": int(len(traj)),
        "effort": float(effort),
        "sat_yaw": int(sat_yaw),
        "sat_pitch": int(sat_pitch),
        "sat_roll": int(sat_roll),
        "sat_throttle": int(sat_thr),
        "sat_total": int(sat_yaw + sat_pitch + sat_roll + sat_thr),
        "aborted": bool(aborted),
        "min_u": float(min_u) if min_u != float("inf") else float("nan"),
        "turn_sum": float(turn_sum),
        "turn_dist": float(turn_dist),
        "turn_per_km": float((turn_sum / max(turn_dist, 1e-6)) * 1000.0) if turn_dist > 1e-6 else 0.0,
    }

    if return_errors and return_info:
        return traj_arr, wp_list, np.array(errs_xy), np.array(errs_alt), info
    if return_errors:
        return traj_arr, wp_list, np.array(errs_xy), np.array(errs_alt)
    if return_info:
        return traj_arr, wp_list, info
    return traj_arr, wp_list


# -----------------------------
# Tuning utilities
# -----------------------------
@dataclass(frozen=True)
class ParamSpec:
    name: str
    low: float
    high: float
    scale: str = "linear"  # "linear" | "log"


def _rotate_waypoints_xy(wps: List[Tuple[float, float, float]], yaw_deg: float) -> List[Tuple[float, float, float]]:
    ang = math.radians(yaw_deg)
    c, s = math.cos(ang), math.sin(ang)
    out = []
    for x, y, z in wps:
        xr = c * x - s * y
        yr = s * x + c * y
        out.append((xr, yr, z))
    return out


def _scale_waypoints_xy(wps: List[Tuple[float, float, float]], scale: float) -> List[Tuple[float, float, float]]:
    return [(x * scale, y * scale, z) for (x, y, z) in wps]


def make_tuning_suite(base_wps: List[Tuple[float, float, float]]) -> List[List[Tuple[float, float, float]]]:
    suite: list[list[tuple[float, float, float]]] = []
    suite.append(list(base_wps))
    suite.append(_rotate_waypoints_xy(base_wps, 90.0))
    suite.append(_rotate_waypoints_xy(base_wps, 180.0))
    suite.append(_scale_waypoints_xy(base_wps, 1.25))
    suite.append([(x, y, z + 80.0) for (x, y, z) in base_wps])
    if base_wps:
        x0, y0, z0 = base_wps[0]
        suite.append(
            [
                (float(x0), float(y0), float(z0)),
                (float(x0) + 450.0, float(y0), float(z0)),
                (float(x0) + 450.0, float(y0) + 450.0, float(z0)),
                (float(x0) + 900.0, float(y0) + 450.0, float(z0)),
            ]
        )
    return suite


def _default_param_specs(*, tight: bool = False) -> List[ParamSpec]:
    yaw_hi = 3.0 if tight else 2.5
    yaw_i_hi = 0.28 if tight else 0.2
    pitch_rate_hi = 4.2 if tight else 3.2
    pitch_damp_hi = 3.6 if tight else 2.5
    alt_pitch_hi = 0.12 if tight else 0.09
    alt_i_hi = 0.28 if tight else 0.2
    throttle_hi = 0.45 if tight else 0.35
    throttle_alt_hi = 5e-3 if tight else 2.5e-3
    throttle_i_hi = 0.32 if tight else 0.2
    lookahead_lo = 0.0 if tight else 10.0
    lookahead_hi = 140.0 if tight else 220.0
    freeze_dist_hi = 110.0 if tight else 160.0

    return [
        ParamSpec("yaw", 0.2, yaw_hi, "linear"),
        ParamSpec("yaw_i", 0.0, yaw_i_hi, "linear"),
        ParamSpec("pitch_rate", 0.4, pitch_rate_hi, "linear"),
        ParamSpec("pitch_damp", 0.2, pitch_damp_hi, "linear"),
        ParamSpec("alt_pitch", 0.005, alt_pitch_hi, "linear"),
        ParamSpec("alt_i", 0.0, alt_i_hi, "linear"),
        ParamSpec("throttle", 0.02, throttle_hi, "linear"),
        ParamSpec("throttle_alt", 1e-4, throttle_alt_hi, "log"),
        ParamSpec("throttle_i", 0.0, throttle_i_hi, "linear"),
        ParamSpec("lookahead_m", lookahead_lo, lookahead_hi, "linear"),
        ParamSpec("freeze_yaw_dist", 0.0, freeze_dist_hi, "linear"),
    ]


def _gains_to_params(g: PIDGains, specs: List[ParamSpec]) -> dict[str, float]:
    d = g.__dict__.copy()
    return {s.name: float(d[s.name]) for s in specs}


def _params_to_gains(params: dict[str, float]) -> PIDGains:
    return PIDGains(**params)


def _spec_bounds_x(specs: List[ParamSpec]) -> tuple[np.ndarray, np.ndarray]:
    lb = []
    ub = []
    for s in specs:
        if s.scale == "log":
            lb.append(math.log(s.low))
            ub.append(math.log(s.high))
        else:
            lb.append(s.low)
            ub.append(s.high)
    return np.array(lb, dtype=float), np.array(ub, dtype=float)


def _params_to_x(params: dict[str, float], specs: List[ParamSpec]) -> np.ndarray:
    x = np.zeros(len(specs), dtype=float)
    for i, s in enumerate(specs):
        v = float(params[s.name])
        x[i] = math.log(v) if s.scale == "log" else v
    return x


def _x_to_params(x: np.ndarray, specs: List[ParamSpec]) -> dict[str, float]:
    out: dict[str, float] = {}
    for i, s in enumerate(specs):
        out[s.name] = float(math.exp(x[i])) if s.scale == "log" else float(x[i])
    return out


def _lhs_candidates(specs: List[ParamSpec], n: int, rng: np.random.Generator) -> List[dict[str, float]]:
    d = len(specs)
    u = np.empty((n, d), dtype=float)
    for j in range(d):
        perm = rng.permutation(n)
        u[:, j] = (perm + rng.uniform(size=n)) / n

    cands: list[dict[str, float]] = []
    for i in range(n):
        p: dict[str, float] = {}
        for j, s in enumerate(specs):
            uj = float(u[i, j])
            if s.scale == "log":
                lo, hi = math.log(s.low), math.log(s.high)
                p[s.name] = math.exp(lo + uj * (hi - lo))
            else:
                p[s.name] = s.low + uj * (s.high - s.low)
        cands.append(p)
    return cands


def _score_run(
    errs_xy: np.ndarray,
    errs_alt: np.ndarray,
    info: dict,
    dt: float,
    speed_target: float,
    *,
    tight: bool = False,
) -> float:
    if info.get("aborted", False) or errs_xy.size == 0:
        return 3e6

    finished = bool(info.get("finished", False))
    steps = max(1, int(info.get("steps", 1)))

    iae_xy = float(np.sum(errs_xy) * dt)
    iae_alt = float(np.sum(errs_alt) * dt)

    p95_xy = float(np.percentile(errs_xy, 95))
    p95_alt = float(np.percentile(errs_alt, 95))
    max_xy = float(np.max(errs_xy))
    max_alt = float(np.max(errs_alt))

    sat_ratio = float(info.get("sat_total", 0)) / float(steps)

    t = max(1e-3, float(info.get("t", steps * dt)))
    eff = float(info.get("effort", 0.0)) / t

    min_u = float(info.get("min_u", float("nan")))
    speed_pen = 0.0
    if np.isfinite(min_u):
        speed_pen = max(0.0, (speed_target * 0.35 - min_u)) * 40.0

    turn_per_km = float(info.get("turn_per_km", 0.0))
    if not np.isfinite(turn_per_km):
        turn_per_km = 0.0

    if tight:
        w_iae_xy = 1.4
        w_p95_xy = 0.9
        w_max_xy = 0.45
        w_iae_alt = 1.8
        w_p95_alt = 1.0
        w_max_alt = 0.55
        w_sat = 320.0
        w_eff = 8.0
        w_turn = 1200.0
    else:
        w_iae_xy = 1.0
        w_p95_xy = 0.45
        w_max_xy = 0.15
        w_iae_alt = 1.25
        w_p95_alt = 0.55
        w_max_alt = 0.15
        w_sat = 220.0
        w_eff = 6.0
        w_turn = 600.0

    final_pen = 0.0
    if tight and errs_xy.size > 0:
        final_pen = float(errs_xy[-1] * 12.0 + errs_alt[-1] * 14.0)

    base = (
        w_iae_xy * iae_xy
        + w_p95_xy * p95_xy
        + w_max_xy * max_xy
        + w_iae_alt * iae_alt
        + w_p95_alt * p95_alt
        + w_max_alt * max_alt
        + w_sat * sat_ratio
        + w_eff * eff
        + w_turn * turn_per_km
        + speed_pen
        + final_pen
    )

    if not finished:
        extra = 1.4e6 if tight else 1e6
        base += extra + 8.0 * max_xy + 8.0 * max_alt

    return float(base)


def _evaluate_params(
    params: dict[str, float],
    waypoint_sets: List[List[Tuple[float, float, float]]],
    *,
    dt: float,
    total_time: float,
    speed_target: float,
    pos_tol: float,
    start_cases: List[tuple[Tuple[float, float], float, float]],
    tight: bool = False,
    cache: dict[tuple, float] | None = None,
) -> float:
    key = None
    if cache is not None:
        key = tuple((k, round(float(v), 6)) for k, v in sorted(params.items()))
        key = key + (("dt", round(dt, 4)), ("T", round(total_time, 2)), ("V", round(speed_target, 2)), ("tight", tight))
        if key in cache:
            return cache[key]

    gains = _params_to_gains(params)

    total = 0.0
    n = 0
    for wps in waypoint_sets:
        for (offset_xy, yaw0, u0) in start_cases:
            _, _, exy, ealt, info = simulate_waypoints(
                wps,
                dt=dt,
                total_time=total_time,
                speed_target=speed_target,
                pos_tol=pos_tol,
                gains=gains,
                return_errors=True,
                start_offset_xy=offset_xy,
                start_yaw_deg=yaw0,
                start_speed=u0,
                return_info=True,
            )
            total += _score_run(exy, ealt, info, dt=dt, speed_target=speed_target, tight=tight)
            n += 1

    score = total / max(1, n)
    if cache is not None and key is not None:
        cache[key] = score
    return float(score)


def _local_refine(
    start_params: dict[str, float],
    start_score: float,
    specs: List[ParamSpec],
    eval_fn,
    *,
    max_iters: int = 70,
    step_frac: float = 0.18,
    min_step_frac: float = 0.004,
    on_try=None,
) -> tuple[dict[str, float], float]:
    lb, ub = _spec_bounds_x(specs)
    best_x = _params_to_x(start_params, specs)
    best_score = float(start_score)

    step = (ub - lb) * float(step_frac)
    min_step = (ub - lb) * float(min_step_frac)

    for _ in range(max_iters):
        improved = False
        for i in range(len(specs)):
            for sgn in (-1.0, 1.0):
                xt = best_x.copy()
                xt[i] = float(np.clip(xt[i] + sgn * step[i], lb[i], ub[i]))
                pt = _x_to_params(xt, specs)
                sc = float(eval_fn(pt))
                if on_try is not None:
                    on_try(pt, sc)
                if sc < best_score:
                    best_score = sc
                    best_x = xt
                    improved = True

        if not improved:
            step *= 0.5
            if float(np.max(step)) < float(np.max(min_step)):
                break

    return _x_to_params(best_x, specs), float(best_score)


def tune_pid_gains(
    base_waypoints: List[Tuple[float, float, float]],
    *,
    speed_target: float = 90.0,
    pos_tol: float = 30.0,
    n_global: int = 160,
    keep_top: int = 10,
    n_restarts: int = 6,
    local_iters: int = 70,
    seed: int = 0,
    dt_coarse: float = 0.02,
    T_coarse: float = 70.0,
    dt_fine: float = 0.01,
    T_fine: float = 95.0,
    tight: bool = False,
    save_path: str | None = None,
    report_path: str | None = None,
) -> tuple[PIDGains, dict]:
    rng = np.random.default_rng(seed)
    specs = _default_param_specs(tight=tight)
    waypoint_sets = make_tuning_suite(base_waypoints)

    start_cases = [
        ((-300.0, -300.0), 0.0, speed_target * 0.9),
        ((-520.0, 180.0), 90.0, speed_target * 0.8),
        ((220.0, -520.0), 180.0, speed_target * 1.0),
    ]

    cache_coarse: dict[tuple, float] = {}
    cache_fine: dict[tuple, float] = {}

    def eval_coarse(p: dict[str, float]) -> float:
        return _evaluate_params(
            p, waypoint_sets,
            dt=dt_coarse, total_time=T_coarse,
            speed_target=speed_target, pos_tol=pos_tol,
            start_cases=start_cases,
            tight=tight,
            cache=cache_coarse,
        )

    def eval_fine(p: dict[str, float]) -> float:
        return _evaluate_params(
            p, waypoint_sets,
            dt=dt_fine, total_time=T_fine,
            speed_target=speed_target, pos_tol=pos_tol,
            start_cases=start_cases,
            tight=tight,
            cache=cache_fine,
        )

    candidates: list[dict[str, float]] = []
    candidates.append(_gains_to_params(PIDGains(), specs))

    if save_path is not None:
        prev = load_gains(save_path)
        if prev is not None:
            candidates.append(_gains_to_params(prev, specs))
            print(f"[tune] resume candidate loaded from {save_path}: {prev}")

    candidates.extend(_lhs_candidates(specs, n_global, rng))

    best_fine_score = float("inf")
    best_params: dict[str, float] | None = None
    best_stage = "init"

    def checkpoint(params: dict[str, float], fine_score: float, stage: str) -> None:
        nonlocal best_fine_score, best_params, best_stage
        if fine_score + 1e-9 < best_fine_score:
            best_fine_score = float(fine_score)
            best_params = dict(params)
            best_stage = stage

            best_gains = _params_to_gains(best_params)
            print(f"[tune][best] score={best_fine_score:.2f} stage={best_stage} gains={best_gains}")

            if save_path is not None:
                save_gains(save_path, best_gains)
                print(f"[tune][save] updated {save_path}")

            if report_path is not None:
                rep = {
                    "timestamp": datetime.now().isoformat(timespec="seconds"),
                    "best_score": best_fine_score,
                    "best_stage": best_stage,
                    "best_params": {k: float(v) for k, v in best_params.items()},
                    "speed_target": float(speed_target),
                    "pos_tol": float(pos_tol),
                    "n_global": int(n_global),
                    "keep_top": int(keep_top),
                    "n_restarts": int(n_restarts),
                    "local_iters": int(local_iters),
                    "dt_coarse": float(dt_coarse),
                    "T_coarse": float(T_coarse),
                    "dt_fine": float(dt_fine),
                    "T_fine": float(T_fine),
                    "suite_size": int(len(waypoint_sets)),
                    "start_cases": start_cases,
                    "seed": int(seed),
                    "tight": bool(tight),
                }
                save_report(report_path, rep)

    scored_coarse: list[tuple[float, dict[str, float]]] = []
    best_coarse = float("inf")

    for i, p in enumerate(candidates):
        sc = float(eval_coarse(p))
        scored_coarse.append((sc, p))
        if sc < best_coarse:
            best_coarse = sc
            sc_f = float(eval_fine(p))
            checkpoint(p, sc_f, stage="coarse-best")
        if (i + 1) % 25 == 0 or (i + 1) == len(candidates):
            print(f"[tune:coarse] {i+1}/{len(candidates)} coarse_best={best_coarse:.2f}")

    scored_coarse.sort(key=lambda x: x[0])
    top = [p for (_, p) in scored_coarse[:max(keep_top, n_restarts)]]

    refined_seed: list[tuple[float, dict[str, float]]] = []
    for p in top:
        sc_f = float(eval_fine(p))
        refined_seed.append((sc_f, p))
        checkpoint(p, sc_f, stage="fine-seed")
    refined_seed.sort(key=lambda x: x[0])
    print(f"[tune:fine] seed best={refined_seed[0][0]:.2f}")

    overall_best_score = float("inf")
    overall_best_params: dict[str, float] | None = None

    for r in range(min(n_restarts, len(refined_seed))):
        s0, p0 = refined_seed[r]

        def on_try(pt, sc):
            checkpoint(pt, float(sc), stage=f"local-r{r+1}")

        p_ref, s_ref = _local_refine(
            p0, s0, specs, eval_fine,
            max_iters=local_iters,
            on_try=on_try,
        )
        print(f"[tune:local] restart {r+1}/{min(n_restarts, len(refined_seed))} score={s_ref:.2f}")
        checkpoint(p_ref, float(s_ref), stage=f"local-final-r{r+1}")

        if s_ref < overall_best_score:
            overall_best_score = float(s_ref)
            overall_best_params = dict(p_ref)

    if overall_best_params is None:
        overall_best_params = refined_seed[0][1]
        overall_best_score = refined_seed[0][0]

    best_gains = _params_to_gains(overall_best_params)

    if save_path is not None:
        save_gains(save_path, best_gains)
        print(f"[tune][save] final {save_path}")

    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "best_score": float(overall_best_score),
        "best_stage": "final",
        "best_params": {k: float(v) for k, v in overall_best_params.items()},
        "speed_target": float(speed_target),
        "pos_tol": float(pos_tol),
        "n_global": int(n_global),
        "keep_top": int(keep_top),
        "n_restarts": int(n_restarts),
        "local_iters": int(local_iters),
        "dt_coarse": float(dt_coarse),
        "T_coarse": float(T_coarse),
        "dt_fine": float(dt_fine),
        "T_fine": float(T_fine),
        "suite_size": int(len(waypoint_sets)),
        "start_cases": start_cases,
        "seed": int(seed),
        "tight": bool(tight),
    }
    if report_path is not None:
        save_report(report_path, report)

    return best_gains, report


def sweep_time_scales(
    base_waypoints: List[Tuple[float, float, float]],
    time_scales: List[float],
    *,
    speed_target: float = 90.0,
    pos_tol: float = 30.0,
    n_global: int = 160,
    keep_top: int = 10,
    n_restarts: int = 6,
    local_iters: int = 70,
    seed: int = 0,
    dt_coarse: float = 0.02,
    T_coarse: float = 70.0,
    dt_fine: float = 0.01,
    T_fine: float = 95.0,
    save_dir: str | Path | None = None,
    db_name: str = "uav_pid_db.json",
    aggregate_path: str | Path | None = None,
    tight: bool = False,
):
    sim_dir = Path(__file__).resolve().parents[1]
    if save_dir is None:
        save_dir = sim_dir / "tune" / "db"
    if aggregate_path is None:
        aggregate_path = sim_dir / "runtime" / "controllers" / "uav_pid_db.json"

    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    db_path = save_dir / db_name

    def _load_db_records(path: Path) -> list[dict]:
        if not path.exists():
            return []
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(data, dict) and isinstance(data.get("records"), list):
                return list(data["records"])
        except Exception:
            pass
        return []

    def _merge_records(path: Path, updates: list[dict]) -> list[dict]:
        existing = {float(r.get("time_scale", i)): r for i, r in enumerate(_load_db_records(path))}
        for r in updates:
            ts = float(r.get("time_scale", len(existing)))
            existing[ts] = r
        merged = sorted(existing.values(), key=lambda r: float(r.get("time_scale", 0.0)))
        _save_json_atomic(path, {"records": merged})
        return merged

    prefix = Path(db_name).stem.replace("_db", "")
    records = []
    for ts in time_scales:
        ts = float(ts)
        ts_tight = bool(tight or ts >= 5.0)
        ts_is_large = ts >= 5.0
        n_global_ts = int(n_global * (2.0 if ts_is_large else 1.0))
        keep_top_ts = max(keep_top, int(keep_top * (1.6 if ts_is_large else 1.0)))
        n_restarts_ts = max(n_restarts, int(n_restarts * (1.5 if ts_is_large else 1.0)))
        local_iters_ts = int(local_iters * (1.6 if ts_is_large else 1.0))
        T_coarse_ts = T_coarse * (ts if ts_is_large else 1.0)
        T_fine_ts = T_fine * (ts if ts_is_large else 1.0)
        rep_path = save_dir / f"{prefix}_scale_{ts:.2f}.report.json"
        gains_path = save_dir / f"{prefix}_scale_{ts:.2f}.json"
        print(f"[sweep] time_scale={ts:.2f} dt_coarse={dt_coarse*ts:.4f} dt_fine={dt_fine*ts:.4f}")
        gains_ts, rep_ts = tune_pid_gains(
            base_waypoints,
            speed_target=speed_target,
            pos_tol=pos_tol,
            n_global=n_global_ts,
            keep_top=keep_top_ts,
            n_restarts=n_restarts_ts,
            local_iters=local_iters_ts,
            seed=seed,
            dt_coarse=dt_coarse * ts,
            T_coarse=T_coarse_ts,
            dt_fine=dt_fine * ts,
            T_fine=T_fine_ts,
            save_path=gains_path,
            report_path=rep_path,
            tight=ts_tight,
        )
        records.append(
            {
                "time_scale": ts,
                "dt_coarse": dt_coarse * ts,
                "dt_fine": dt_fine * ts,
                "T_coarse": T_coarse_ts,
                "T_fine": T_fine_ts,
                "best_score": float(rep_ts["best_score"]),
                "best_stage": rep_ts.get("best_stage"),
                "gains": gains_ts.__dict__,
                "report_path": str(rep_path),
                "gains_path": str(gains_path),
                "tight": bool(ts_tight),
                "n_global": n_global_ts,
                "keep_top": keep_top_ts,
                "n_restarts": n_restarts_ts,
                "local_iters": local_iters_ts,
            }
        )
    merged_db = _merge_records(db_path, records)
    if aggregate_path is not None:
        agg_path = Path(aggregate_path)
        agg_path.parent.mkdir(parents=True, exist_ok=True)
        merged_agg = _merge_records(agg_path, records)
        print(f"[sweep] saved aggregate DB to {agg_path} ({len(merged_agg)} total)")
    print(f"[sweep] saved DB with {len(merged_db)} entries to {db_path}")
    return db_path, merged_db


# -----------------------------
# Optional CLI (minimal)
# -----------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UAV PID tuning helper")
    parser.add_argument("--tune", action="store_true", help="run tuning once")
    parser.add_argument("--sweep-time-scales", type=str, help="comma-separated time_scale values")
    parser.add_argument("--tight", action="store_true", help="use tight scoring/bounds")
    parser.add_argument("--save", type=str, default=None, help="save gains json")
    parser.add_argument("--report", type=str, default=None, help="save report json")
    args = parser.parse_args()

    wp_demo = [
        (0.0, 0.0, 300.0),
        (800.0, 0.0, 300.0),
        (800.0, 800.0, 350.0),
        (0.0, 800.0, 350.0),
        (-400.0, 400.0, 400.0),
    ]

    if args.sweep_time_scales:
        scales = [float(s.strip()) for s in args.sweep_time_scales.split(",") if s.strip()]
        sweep_time_scales(
            wp_demo,
            scales,
            speed_target=90.0,
            pos_tol=30.0,
            n_global=180,
            keep_top=12,
            n_restarts=7,
            local_iters=80,
            seed=0,
            dt_coarse=0.02,
            T_coarse=70.0,
            dt_fine=0.01,
            T_fine=95.0,
            save_dir=None,
            db_name="uav_pid_db.json",
            aggregate_path=None,
            tight=args.tight,
        )
        raise SystemExit(0)

    if args.tune:
        tuned, rep = tune_pid_gains(
            wp_demo,
            speed_target=90.0,
            pos_tol=30.0,
            n_global=180,
            keep_top=12,
            n_restarts=7,
            local_iters=80,
            seed=0,
            dt_coarse=0.02,
            T_coarse=70.0,
            dt_fine=0.01,
            T_fine=95.0,
            save_path=args.save,
            report_path=args.report,
            tight=args.tight,
        )
        print(f"[tune] final best gains: {tuned} (score {rep['best_score']:.2f})")
