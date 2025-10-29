# -*- coding: utf-8 -*-
"""
d0304_rl_3d.py – RL-기반 LAH FlightPlan (3-DoF / 새 Gym Env 대응)
────────────────────────────────────────────────────────────────
• 기존 d0304_rl.py 와 100% API 호환
• 달라진 점
  1) ENV_ID = "LAH3D-v0"  (gym.register 된 3-D 환경)
  2) env_kwargs 로 추가 파라미터 전달 가능
  3) sim 속성명이 lon/lat/alt → state.lon 등으로 바뀐 경우 자동 대응
  4) _traj_to_wplist() : heading-based down-sampling 옵션 포함
"""
from __future__ import annotations
import math, os, csv, tempfile
from collections import OrderedDict
from typing import List, Tuple, Dict, Any

import gymnasium as gym
import numpy as np
import torch as th
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from .mission_helpers import now_ms_since_2000
from .id_allocator import next_waypoint_id as _next_waypoint_id

# ────────────────────────────────────────────────────────────
# 0. VecEnv 래퍼 풀기
# ────────────────────────────────────────────────────────────
def _extract_base_env(venv):
    if isinstance(venv, VecNormalize):
        venv = venv.venv
    return venv                      # DummyVecEnv


# ────────────────────────────────────────────────────────────
# 1. WaypointID 할당기
# ────────────────────────────────────────────────────────────
class _WPAllocator:
    def __init__(self, start: int | None = None):
        self._local_next = start
        self._use_global = start is None
    def alloc(self) -> int:
        if self._use_global:
            return int(_next_waypoint_id())
        if self._local_next is None:
            raise RuntimeError("Waypoint allocator misconfigured (local start unset)")
        if self._local_next > 65_535:
            raise RuntimeError("WaypointID pool exhausted")
        wid = self._local_next
        self._local_next += 1
        return wid


# ────────────────────────────────────────────────────────────
# 2. RL 궤적 → WaypointList
# ────────────────────────────────────────────────────────────
def _haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    dφ, dλ = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2*R*math.asin(min(1, math.sqrt(a)))


def _bearing(lat1, lon1, lat2, lon2):
    dλ = math.radians(lon2 - lon1)
    y  = math.sin(dλ) * math.cos(math.radians(lat2))
    x  = (math.cos(math.radians(lat1))*math.sin(math.radians(lat2))
          - math.sin(math.radians(lat1))*math.cos(math.radians(lat2))*math.cos(dλ))
    return (math.degrees(math.atan2(y, x)) + 360) % 360


# d0304_rl.py
# ─────────────────────────────────────────────────────────────
def _traj_to_wplist(
        lons: List[float],
        lats: List[float],
        alts: List[float],
        wp_alloc: _WPAllocator,
        cruise_spd: float = 40.0,
        heading_tol_deg: float = 15,     # ☆ 추가 : 헤딩 변화 임계값
) -> List[dict]:
    """
    RL 궤적 → WaypointList 
    · heading_tol_deg° 이상 방향이 바뀌는 지점만 WP 로 채택해
      ‘뱅글뱅글’ 현상 최소화.
    """
    import math

    # ── 헤딩 계산 → down-sample ───────────────────────────────
    def _bearing(lat1, lon1, lat2, lon2):
        dλ = math.radians(lon2 - lon1)
        y  = math.sin(dλ) * math.cos(math.radians(lat2))
        x  = (math.cos(math.radians(lat1))*math.sin(math.radians(lat2))
              - math.sin(math.radians(lat1))*math.cos(math.radians(lat2))*math.cos(dλ))
        return (math.degrees(math.atan2(y, x)) + 360) % 360

    keep = [0]
    if len(lats) > 2:
        ref = _bearing(lats[0], lons[0], lats[1], lons[1])
        for k in range(2, len(lats)):
            hdg = _bearing(lats[k-1], lons[k-1], lats[k], lons[k])
            if abs((hdg - ref + 180) % 360 - 180) > heading_tol_deg:
                keep.append(k-1)
                ref = hdg
    keep.append(len(lats) - 1)

    lons = [lons[i] for i in keep]
    lats = [lats[i] for i in keep]
    alts = [alts[i] for i in keep]

    # ── 거리·ETA·ECF 계산(기존과 동일) ─────────────────────────
    def _haversine(lat1, lon1, lat2, lon2):
        R = 6_371_000.0
        φ1, φ2 = math.radians(lat1), math.radians(lat2)
        dφ, dλ = math.radians(lat2-lat1), math.radians(lon2-lon1)
        a = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
        return 2*R*math.asin(min(1, math.sqrt(a)))

    total_d = sum(_haversine(lats[i-1], lons[i-1], lats[i], lons[i])
                  for i in range(1, len(lats))) or 1.0

    cum_d = eta_ms = 0.0
    wplist = []
    for i, (lon, lat, alt) in enumerate(zip(lons, lats, alts)):
        if i:
            seg = _haversine(lats[i-1], lons[i-1], lat, lon)
            cum_d += seg
            eta_ms += seg / cruise_spd * 1000.0
        wplist.append(OrderedDict([
            ("waypointID", wp_alloc.alloc()),
            ("coordinate", {
                "latitude":  round(lat, 6),
                "longitude": round(lon, 6),
                "altitude":  round(alt, 1),   # ← 610 → 궤적 alt 그대로
            }),
            ("speed",          cruise_spd),
            ("eta",            int(round(eta_ms))),
            ("ecf",            round(cum_d/total_d, 2)),
            ("nextWaypointID", 0),
            ("hovering",       {}),
            ("loiter",         {}),
            ("attack",         {}),
        ]))
    for i in range(len(wplist)-1):
        wplist[i]["nextWaypointID"] = wplist[i+1]["waypointID"]
    return wplist

# ────────────────────────────────────────────────────────────
# 3. 0302 미션 → 시나리오 CSV (변함 없음)
# ────────────────────────────────────────────────────────────
def _mission_to_wp_rows(miss: dict, idx: int, alt_m: float | None = None):
    """
    • 각 점에 altitude 필드가 있으면 그대로 사용
    • 없으면 alt_m(기본 DEM+200 등) 또는 900 m fallback
    """
    info = miss["individualMissionInfo"]
    typ  = info["individualMissionType"]

    if   typ == 1: pts = info["areaList"][0]["coordinateList"]
    elif typ == 2: pts = info["lineList"][0]["coordinateList"]
    else:          pts = info["coordinateList"]

    rows = []
    for p in pts:
        z = p.get("altitude")
        if z is None:
            z = alt_m if alt_m is not None else 900.0
        rows.append((f"LAH-WP{idx}",
                     p["longitude"], p["latitude"], z, "✓"))
        idx += 1
    return rows, idx


def _build_scenario_csv(missions: List[dict]) -> str:
    """
    0302 미션 목록 → 시나리오 CSV 경로 반환
    * ✓ 문자를 포함하므로 반드시 UTF-8 로 저장
    """
    header = ["ID", "x", "y", "z", "Status"]
    rows = [header]
    idx = 1
    for m in missions:
        add, idx = _mission_to_wp_rows(m, idx)
        rows.extend(add)

    # ※ encoding="utf-8" 지정
    tmp = tempfile.NamedTemporaryFile(
        "w", newline="", suffix=".csv", delete=False, encoding="utf-8"
    )
    with tmp as f:
        csv.writer(f).writerows(rows)
    return tmp.name



# ────────────────────────────────────────────────────────────
# 4. 모델 캐시
# ────────────────────────────────────────────────────────────
_MODEL_CACHE = {}
def _get_model(model_path: str) -> PPO:
    if model_path not in _MODEL_CACHE:
        device = th.device("cuda" if th.cuda.is_available() else "cpu")
        _MODEL_CACHE[model_path] = PPO.load(model_path, device=device)
    return _MODEL_CACHE[model_path]


# ────────────────────────────────────────────────────────────
# 5. 메인 API
# ────────────────────────────────────────────────────────────
# d0304_RL.py
# ------------------------------------------------------------
def build_lah_flight_plans_rl(
        missions: List[dict],
        *,
        model_path: str,
        vecnorm_path: str | None = None,
        cruise_speed: float = 40.0,
        wp_alloc: _WPAllocator | None = None,
        deterministic: bool = True,
):
    """
    0302 미션 → PPO 모델 → 0304 패킷
    • gymnasium 0.29+ 의 OrderEnforcing 래퍼도 자동 해제한다.
    """
    if not missions:
        return []

    wp_alloc = wp_alloc or _WPAllocator()
    now_ms   = now_ms_since_2000()
    packets  : List[dict] = []
    model    = _get_model(model_path)

    # 0) ENV 등록 (없으면) ------------------------------------------
    from envs.gym_framework import LAHEnv
    try:
        gym.spec("LAH-v0")
    except gym.error.Error:
        LAHEnv.register_default()

    # 1) aircraftID 1·2·3만 처리 -----------------------------------
    for miss in missions:
        if miss["aircraftID"] not in (1, 2, 3):
            continue

        # ① 단독 시나리오 CSV 생성
        scn_csv = _build_scenario_csv([miss])

        # ② VecEnv 구성
        env = DummyVecEnv([lambda: gym.make(
            "LAH-v0",
            scenario_csv=scn_csv,
            max_steps=10_000,
        )])
        if vecnorm_path and os.path.exists(vecnorm_path):
            env = VecNormalize.load(vecnorm_path, env)
            env.training = False
            env.norm_reward = False

        obs      = env.reset()
        sim_env  = _extract_base_env(env).envs[0]

        # ▼ NEW: OrderEnforcing / TimeLimit 등 래퍼 벗겨서 .sim 찾기
        base_env = sim_env
        while hasattr(base_env, "env"):          # gymnasium 래퍼 체인
            if hasattr(base_env, "sim"):
                break
            base_env = base_env.env
        if not hasattr(base_env, "sim") and hasattr(base_env, "unwrapped"):
            base_env = base_env.unwrapped
        if not hasattr(base_env, "sim"):
            raise AttributeError("LAH env에 .sim 속성을 찾을 수 없습니다.")
        sim = base_env.sim

        # ③ rollout -------------------------------------------------
        lons, lats, alts = [], [], []
        while True:
            lons.append(sim.lon); lats.append(sim.lat); alts.append(sim.alt)
            action, _ = model.predict(obs, deterministic=deterministic)
            step_out  = env.step(action)

            # Gymnasium(5-값) vs SB3(4-값) 호환
            if len(step_out) == 5:
                obs, _, term, trunc, _ = step_out
                done = bool(term[0] or trunc[0])
            else:
                obs, _, done_vec, _ = step_out
                done = bool(done_vec[0])

            if done:
                break

        # ④ WaypointList 변환 & 패킷 조립
        wplist = _traj_to_wplist(lons, lats, alts, wp_alloc, cruise_speed)
        packets.append(OrderedDict([
            ("timestamp",    now_ms),
            ("pathID",       miss["pathID"]),
            ("aircraftID",   miss["aircraftID"]),
            ("waypointList", wplist),
        ]))

        env.close()
        os.remove(scn_csv)      # 임시 CSV 정리

    return packets


