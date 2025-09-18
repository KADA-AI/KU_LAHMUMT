# KU/AnS/train.py
import os
import cv2
import numpy as np
from pathlib import Path
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, CallbackList

from env_patternselection import UnifiedMissionEnvironment, sample_random_mission_areas

# ── 0) 경로 고정: 이 파일 기준 절대 경로
THIS_DIR = Path(__file__).resolve().parent          # KU/AnS
ROOT_DIR = THIS_DIR.parent.parent                   # 프로젝트 루트 추정 (필요시 조정)
SAVE_DIR = THIS_DIR / "Training"                    # KU/AnS/Training
MODEL_DIR = SAVE_DIR / "SavedModels"
LOG_DIR   = SAVE_DIR / "TensorBoardLogs"
for d in (SAVE_DIR, MODEL_DIR, LOG_DIR):
    d.mkdir(parents=True, exist_ok=True)

print("[CWD]     ", Path.cwd())
print("[SAVE_DIR]", SAVE_DIR)

# ── 1) DEM 경로 (사용자 메모에 따르면 gui/assets/DEM.jpg)
DEM_PATH = ROOT_DIR / "KU" / "ANS" / "DEM.jpg"
assert DEM_PATH.exists(), f"DEM 이미지가 존재하지 않습니다: {DEM_PATH}"

dem_array = cv2.imread(str(DEM_PATH), cv2.IMREAD_GRAYSCALE)
assert dem_array is not None, f"DEM 이미지 로딩 실패: {DEM_PATH}"

# ── 2) 학습용 영역 샘플링
processed_areas = sample_random_mission_areas(dem_array, num_areas=20)
# print("processed_areas:", len(processed_areas))

# ── 3) 가중치 케이스
cases = [
    {"flight_weight": 0.9, "imaging_weight": 0.1, "case_name": "BestFlightCase"},
    {"flight_weight": 0.1, "imaging_weight": 0.9, "case_name": "BestImagingCase"},
    {"flight_weight": 0.5, "imaging_weight": 0.5, "case_name": "BalancedCase"},
]

for case in cases:
    print(f"\n🔹 Training: {case['case_name']}")
    env = UnifiedMissionEnvironment(processed_areas)
    env.flight_weight = case["flight_weight"]
    env.imaging_weight = case["imaging_weight"]

    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=str(LOG_DIR), device="cpu")

    # (선택) 체크포인트/베스트 저장 콜백
    checkpoint_cb = CheckpointCallback(
        save_freq=10_000,                         # 스텝 키우면 주기적으로 저장
        save_path=str(MODEL_DIR),
        name_prefix=f"ppo_{case['case_name']}"
    )
    eval_cb = EvalCallback(
        eval_env=UnifiedMissionEnvironment(processed_areas),
        best_model_save_path=str(MODEL_DIR / f"{case['case_name']}_best"),
        log_path=str(LOG_DIR),
        eval_freq=5_000,
        n_eval_episodes=5,
        deterministic=True
    )
    cbs = CallbackList([checkpoint_cb, eval_cb])

    # 학습 & 반드시 저장
    try:
        model.learn(total_timesteps=100_000, tb_log_name=f"PPO_{case['case_name']}", progress_bar=True, callback=cbs)
    finally:
        model_path = MODEL_DIR / f"{case['case_name']}_Model"
        model.save(str(model_path))
        print(f"✅ 모델 저장 완료: {model_path.with_suffix('.zip')}")
