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
import gym
import os
import numpy as np
import cv2
import random
from typing import List, Tuple
from stable_baselines3 import PPO

from .task_patterns_ver2 import mission_patterns
from .mission_effectiveness_ver2 import calculate_mission_effectiveness
from .coord_transform import llh_to_xy


# DEM 전역 설정
DEM_PATH = "/DEM.jpg"  # 경로 조정 가능
DEM_SIZE = 500

# DEM 이미지 좌표 → 실세계 위경도 변환
MAP_CORNER_COORDS = {
    "TL": (38.146458, 127.294782),
    "TR": (38.147111, 127.340401),
    "BL": (38.110432, 127.295620),
    "BR": (38.111084, 127.341217)
}

# ─────────────────────────────────────
# 픽셀 <-> 위경도 변환 유틸
# ─────────────────────────────────────
def pixel_to_llh(x: float, y: float) -> Tuple[float, float]:
    """픽셀 좌표 (x, y) → 위경도"""
    ratio_x = x / DEM_SIZE
    ratio_y = y / DEM_SIZE

    lat_top = MAP_CORNER_COORDS["TL"][0]
    lat_bottom = MAP_CORNER_COORDS["BL"][0]
    lon_left = MAP_CORNER_COORDS["TL"][1]
    lon_right = MAP_CORNER_COORDS["TR"][1]

    lat = lat_top - (lat_top - lat_bottom) * ratio_y
    lon = lon_left + (lon_right - lon_left) * ratio_x

    return lat, lon

def sample_random_mission_areas(dem_array: np.ndarray, num_areas: int = 20) -> List[dict]:
    """DEM에서 랜덤 임무영역 생성"""
    h, w = dem_array.shape
    area_list = []

    for _ in range(num_areas):
        # 1. 크기 제한된 사각형 랜덤 선택 (예: 40x40 ~ 100x100)
        x1 = random.randint(0, w - 100)
        y1 = random.randint(0, h - 100)
        width = random.randint(40, 100)
        height = random.randint(40, 100)
        x2 = x1 + width
        y2 = y1 + height

        # 2. 꼭짓점 위경도 계산
        corners_llh = [
            pixel_to_llh(x1, y1),
            pixel_to_llh(x2, y1),
            pixel_to_llh(x2, y2),
            pixel_to_llh(x1, y2),
        ]

        # 3. DEM 고도 통계 추출
        crop = dem_array[y1:y2, x1:x2]
        mean_alt = float(np.mean(crop))
        var_alt  = float(np.var(crop))

        # 4. 하나의 mission dict 구성
        area = {
            "MissionID": f"Area_{x1}_{y1}",
            "coordinateList": [{"latitude": lat, "longitude": lon} for lat, lon in corners_llh],
            "meanAltitude": mean_alt,
            "altitudeVariance": var_alt,
            "inputMissionType": random.randint(1, 6)
        }
        area_list.append(area)

    return area_list




class UnifiedMissionEnvironment(gym.Env):
    def __init__(self, processed_areas: list[dict]):
        super().__init__()

        if not processed_areas:
            raise ValueError("processed_areas 리스트가 비어 있음")

        self.processed_missions = processed_areas
        self.current_mission_index: int = 0

        # observation: (10,)
        self.observation_space = gym.spaces.Box(
            low=0.0, high=1.0, shape=(16,), dtype=np.float32
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
        if flat.size < 8:                       # 8‑D 맞추기
            flat = np.pad(flat, (0, 8 - flat.size))

        # ── ③ 고도 정보 추가 후 반환 ───────────────────
        obs_extra = np.array([
            area.get("meanAltitude", 0.0)      / 1000.0,
            area.get("altitudeVariance", 0.0)  / 1e4
        ], dtype=np.float32)
        
        # ── 4. inputMissionType → one-hot (6D) ───────────────────
        type_id = area.get("inputMissionType", 0)
        one_hot = np.zeros(6, dtype=np.float32)
        if 1 <= type_id <= 6:
            one_hot[type_id - 1] = 1.0

        return np.concatenate([flat, obs_extra, one_hot])
    # ────────────────────────────────────────
    # Gym 인터페이스
    def reset(self, **kwargs):
        self.current_mission_index = 0
        return self._area_to_obs(self.processed_missions[0])

    def step(self, action: int):
        # print(f"[STEP] Mission {self.current_mission_index}, Action: {action}")
        area = self.processed_missions[self.current_mission_index]
        pattern = list(mission_patterns.values())[action]

        # 비행/촬영 효과도 계산
        flight_eff, img_eff = calculate_mission_effectiveness(pattern, area)

        # ▶ 임무 유형 및 선택된 임무 패턴 이름 추출
        input_mission_type = area.get("inputMissionType", 0)
        selected_pattern_name = pattern.get("임무 패턴 명", "")

        # ▶ 보상 계산 (임무유형 기반 패턴 선호 반영)
        reward = self.calculate_reward(
            flight_eff,
            img_eff,
            input_mission_type,
            selected_pattern_name
        )

        # 다음 상태 이동
        self.current_mission_index += 1
        done = self.current_mission_index >= len(self.processed_missions)
        obs = None if done else self._area_to_obs(
            self.processed_missions[self.current_mission_index]
        )

        return obs, reward, done, {}

    def calculate_reward(self, flight_eff, img_eff, inputMissionType, selected_pattern_name):
        
        base_reward = 0.5 * flight_eff + 0.5 * img_eff

        preferred_patterns = {
            (1, 4, 5): ["구간중심종단-선형반복주사촬영", "구간중심종단-자동반복주사촬영"],
            (2, 3, 6): ["직하방-BF촬영", "이격-BF촬영", "구간왕복-BF촬영", "선형반복주사-BF촬영"]
        }

        bonus = 0.0
        for types, names in preferred_patterns.items():
            if inputMissionType in types and selected_pattern_name in names:
                bonus = 0.2
        # print(f"[REWARD] flight_eff={flight_eff}, img_eff={img_eff}, inputMissionType={inputMissionType}, pattern={selected_pattern_name}")
        return base_reward + bonus


    def render(self, mode="human"):
        pass

