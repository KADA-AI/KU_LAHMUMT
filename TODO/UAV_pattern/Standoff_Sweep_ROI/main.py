from Stdoff_ROI import AisleSweepPlanner
from datetime import timedelta
import math
# 다각형 
polygon_vertices = [(0,0), (2000,0), (2000,2000), (0,2000)]
# 시작점(Before_ACP)
start_point = (-1000, -200)

planner = AisleSweepPlanner(
    polygon_vertices,
    start_point,
    show_plot=True,          # 시각화 
    # 필요시 파라미터 수정 
    # width=340.0, separation_dist=850.0, uav_height=610.0,
    # fov_deg=2.5, R_min=360.0
)

result = planner.run()

for item in result["ordered_output"]:
    if item["type"] == "seg":
        print(item["label"], item["p1"], "->", item["p2"])
    elif item["type"] == "wp":
        print(item["label"], item["p"])
    elif item["type"] == "dubins":
        print(item["label"], item["p"])

# 세부 컨테이너 접근 
all_parts_lines    = result["all_parts_lines"]     # 폴리곤 내부 세그먼트(라인 단위)
all_bbox_lines     = result["all_bbox_lines"]      # 바운딩박스 내부 세그먼트(라인 단위)
all_parts_wps_bbox = result["all_parts_wps_bbox"]  # 각 파트의 WP 리스트
dubins_between     = result["dubins_between_parts"]# {(i,i+1): [분기점들]}

# ===== 경로 좌표(시각화 순서 그대로)만 추출 =====
full_path = []   # [(x, y), ...] 최종 경로 좌표열

# 1) 파트 번호 범위 결정 (all_parts_wps_bbox의 키는 1..N)
part_ids = sorted(result["all_parts_wps_bbox"].keys())

# 2) part1 WPs → Dubins(1→2) → part2 WPs → Dubins(2→3) → ... 순서로 이어붙임
for i, pid in enumerate(part_ids, start=1):
    # 현재 파트 WPs
    wps = result["all_parts_wps_bbox"].get(pid, [])
    for (wx, wy) in wps:
        full_path.append((float(wx), float(wy)))
    # 다음 파트로 넘어가는 Dubins 분기점들
    if pid < part_ids[-1]:
        mids = result["dubins_between_parts"].get((pid, pid+1), [])
        for (mx, my) in mids:
            full_path.append((float(mx), float(my)))

# 3) 좌표만 출력 (원하면 full_path 리스트만 사용)
print("\n[Full path coordinates in visualized order]")
for idx, (x, y) in enumerate(full_path):
    print(f"{idx:03d}: ({x:.6f}, {y:.6f})")
    

def polyline_length(points):
    """연속 좌표들을 직선으로 이은 총 길이(m).
       (중복 좌표/NaN/None 방지 포함)"""
    if not points or len(points) < 2:
        return 0.0
    total = 0.0
    prev = None
    for x, y in points:
        if x is None or y is None:
            continue
        if prev is not None:
            dx = x - prev[0]
            dy = y - prev[1]
            seg = math.hypot(dx, dy)
            if seg > 0:   # 동일 좌표(0 길이) 무시
                total += seg
        prev = (x, y)
    return total

# 1) 총 경로 길이(m)
total_length_m = polyline_length(full_path)

# 2) 속도(100 km/h)를 m/s로 환산
speed_kmh = 125.0
speed_mps = speed_kmh * 1000.0 / 3600.0  # = 27.777...

# 3) 소요 시간(s) 및 보기 좋은 형식(HH:MM:SS)
if speed_mps > 0:
    time_seconds = total_length_m / speed_mps
else:
    time_seconds = float('inf')

time_hms = str(timedelta(seconds=round(time_seconds)))

# 4) 결과 출력 (테이블 형식)
print("\n================= 경로 길이 및 소요 시간 =================")
print(f"{'총 경로 길이':<16}: {total_length_m:,.2f} m  ({total_length_m/1000.0:,.3f} km)")
print(f"{'평균 속도':<16}: {speed_kmh:.1f} km/h ({speed_mps:.3f} m/s)")
print(f"{'소요 시간(초)':<16}: {time_seconds:,.2f} s")
print(f"{'소요 시간(HH:MM:SS)':<16}: {time_hms}")
print("==========================================================\n")