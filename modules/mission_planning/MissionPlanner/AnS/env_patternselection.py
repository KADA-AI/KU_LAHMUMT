"""
env_patternselection.py
───────────────────────────────────────────────────────────────
• 강화학습 / 추론‑전용 Mission‑Pattern 선택 환경
• 입력  :  processed_areas 리스트  (← mission_pipeline 에서 직접 주입)
          └ 각 원소  = {
              "MissionID"         : str,
              "coordinateList"    : [(x, y), …]   # 평면‑XY (m) 4 코너
              "meanAltitude"      : float,        # m
              "altitudeVariance"  : float         # (m²)
            }
• observation : 10‑D  
      0‑7   → 4 코너 XY 를 0‑1 로 정규화(패딩)  
      8     → meanAltitude / 1000  
      9     → AltVariance  / 1e4
• action      : len(mission_patterns)  (패턴 인덱스)
"""

from __future__ import annotations
# NumPy 2.x 환경에서 구버전 gym(0.26)은 네이티브 ABI 불일치로 프로세스가
# access violation(0xC0000005)으로 죽는다. gymnasium 은 동일 API 의 유지보수
# 대체재이므로 우선 사용하고, 없을 때만 legacy gym 으로 폴백한다.
try:
    import gymnasium as gym
except Exception:  # pragma: no cover - gymnasium 미설치 환경 폴백
    import gym
import numpy as np

from .task_patterns_ver2 import mission_patterns
from .mission_effectiveness_ver2 import calculate_mission_effectiveness
from .coord_transform import llh_to_xy 


class UnifiedMissionEnvironment(gym.Env):
    def __init__(self, processed_areas: list[dict]):
        super().__init__()

        if not processed_areas:
            raise ValueError("processed_areas 리스트가 비어 있음")

        self.processed_missions = processed_areas
        self.current_mission_index: int = 0

        # observation: (10,)
        self.observation_space = gym.spaces.Box(
            low=0.0, high=1.0, shape=(10,), dtype=np.float32
        )
        self.action_space = gym.spaces.Discrete(len(mission_patterns))

        # 가중합 비율 (외부에서 수정 가능)
        self.flight_weight  = 0.5
        self.imaging_weight = 0.5

    # ────────────────────────────────────────
    # Helper
    def _area_to_obs(self, area: dict) -> np.ndarray:
        """
        coordinateList 가
          • [{'latitude': …, 'longitude': …}, …]  →  LLH  ➜ XY 변환
          • [(x, y), …]                          →  이미 XY
        둘 다 허용.
        최종적으로 (x, y) 4 쌍을 0‑1 로 정규화해 8‑D, Alt/Var 2‑D 더해 10‑D 반환
        """
        # ── ① 좌표 형식 구분 ───────────────────────────
        if isinstance(area["coordinateList"][0], dict):      # LLH 형식
            lat0 = area["coordinateList"][0]["latitude"]
            lon0 = area["coordinateList"][0]["longitude"]
            xy = [
                llh_to_xy(p["latitude"], p["longitude"], lat0, lon0)
                for p in area["coordinateList"]
            ]
        else:                                                # 이미 (x, y)
            xy = area["coordinateList"]

        # ── ② 0‑1 정규화 & 패딩(≤4코너 대비) ───────────
        coords_norm = [(x / 500.0, y / 500.0) for x, y in xy]
        flat = np.array(coords_norm, dtype=np.float32).flatten()

        if flat.size > 8:                 # ▶  추가. 초과분 절단
            flat = flat[:8]
        elif flat.size < 8:               # 기존 패딩 로직
            flat = np.pad(flat, (0, 8 - flat.size))

        # ── ③ 고도 정보 추가 후 반환 ───────────────────
        obs_extra = np.array([
            area.get("meanAltitude", 0.0)      / 1000.0,
            area.get("altitudeVariance", 0.0)  / 1e4
        ], dtype=np.float32)

        return np.concatenate([flat, obs_extra])
    # ────────────────────────────────────────
    # Gym 인터페이스
    def reset(self, **kwargs):
        self.current_mission_index = 0
        return self._area_to_obs(self.processed_missions[0])

    def step(self, action: int):
        area   = self.processed_missions[self.current_mission_index]
        pattern = list(mission_patterns.values())[action]

        # 비행/촬영 효과도 → reward
        flight_eff, img_eff = calculate_mission_effectiveness(pattern, area)
        reward = (
            self.flight_weight  * flight_eff +
            self.imaging_weight * img_eff
        )

        # 다음 state
        self.current_mission_index += 1
        done = self.current_mission_index >= len(self.processed_missions)
        obs  = None if done else self._area_to_obs(
            self.processed_missions[self.current_mission_index])

        return obs, reward, done, {}

    def render(self, mode="human"):
        pass
