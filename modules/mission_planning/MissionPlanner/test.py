#!/usr/bin/env python
import os, sys, json
import matplotlib.pyplot as plt
import numpy as np

# ────────────────────────────────────────────────
# 0) 패키지 경로 추가 + AnS 패키지 import
# ────────────────────────────────────────────────
ROOT = os.path.abspath(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.append(ROOT)                 # missionPlanner 최상위

from AnS.mission_pipeline import divide_corridor_polyline, llh_to_xy

# ────────────────────────────────────────────────
# 1) 테스트용 CMPK(0201) JSON 로드
# ────────────────────────────────────────────────
CMPK_PATH = os.path.join(
    ROOT, "plannedMission", "InputMissionPlan", "100_001.json")
with open(CMPK_PATH, "r", encoding="utf-8") as f:
    cmpk = json.load(f)

# corridor 임무 하나 추출
line_seg = next(
    im["missionDetail"]["lineList"][0]
    for im in cmpk["inputMissionList"]
    if im["inputMissionType"] in (1, 4, 5)
)

# ────────────────────────────────────────────────
# 2) Strip 분할
# ────────────────────────────────────────────────
UAV_CNT = 3
strips = divide_corridor_polyline(line_seg, UAV_CNT)

# ────────────────────────────────────────────────
# 3) 시각화
# ────────────────────────────────────────────────
lat0, lon0 = line_seg["coordinateList"][0]["latitude"], line_seg["coordinateList"][0]["longitude"]
to_xy = lambda p: llh_to_xy(p["latitude"], p["longitude"], lat0, lon0)

fig, ax = plt.subplots(figsize=(8, 6))

# (a) 원본 centerline + 포인트 라벨
orig_xy = list(map(to_xy, line_seg["coordinateList"]))
ax.plot(*zip(*orig_xy), "k--", lw=2, label="Centerline")
ax.scatter(*zip(*orig_xy), c="k", s=30, zorder=5)
for idx, (x, y) in enumerate(orig_xy):
    ax.text(x, y, f"P{idx}", fontsize=9, ha="right", va="bottom")

# (b) Centerline 기준 폭 300 m(±150) 표시  ▸ 회색 반투명
half_w = line_seg["width"] * 0.5            # = 150
# 양 끝점에서 폭축으로 ±half_w 이동
v = np.array(orig_xy[1]) - np.array(orig_xy[0])
v /= np.linalg.norm(v)
w = np.array([-v[1], v[0]])                 # 폭축
left = [np.array(p) + w * half_w for p in orig_xy]
right = [np.array(p) - w * half_w for p in reversed(orig_xy)]
poly = np.vstack([left, right])
ax.fill(poly[:, 0], poly[:, 1], color="lightgray", alpha=0.4,
        label="Width 300 m corridor")

# (c) 각 UAV Strip 폴리곤 + Centerline
colors = ["r", "g", "b", "c", "m", "y"]
for idx, st in enumerate(strips):
    col = colors[idx % len(colors)]
    xs, ys = zip(*map(to_xy, st["coordinateList"]))
    ax.fill(xs, ys, alpha=0.25, color=col, label=f"Strip {idx+1}")
    cls_xy = list(map(to_xy, st["Centerline"]))
    ax.plot(*zip(*cls_xy), color=col, lw=2)

ax.set_aspect("equal")
ax.legend()
ax.set_title("Corridor ±150 m 폭 및 UAV Strip 시각화")
plt.tight_layout()
plt.show()
