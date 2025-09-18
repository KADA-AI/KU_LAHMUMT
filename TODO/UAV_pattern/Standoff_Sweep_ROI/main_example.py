# other_script.py
from Stdoff_ROI import AisleSweepPlanner

# ① 입력: 다각형 꼭짓점들(시계/반시계 상관없음)
polygon_vertices = [(0,0), (500,50), (520,420), (100,480), (-80,220)]
# ② 입력: 시작점(Before_ACP)
start_point = (200, -200)

# 옵션: show_plot=True면 시각화 창 띄움(서버/콘솔이면 False 추천)
planner = AisleSweepPlanner(
    polygon_vertices,
    start_point,
    show_plot=True,          # 시각화 끄려면 False
    # 필요시 파라미터 수정 가능:
    # width=340.0, separation_dist=850.0, uav_height=610.0,
    # fov_deg=2.5, R_min=360.0
)

result = planner.run()

# i_Seg_k / i_WP_k / DUBINS_i_to_{i+1}_k 모두 ordered_output에 들어있음
for item in result["ordered_output"]:
    if item["type"] == "seg":
        print(item["label"], item["p1"], "->", item["p2"])
    elif item["type"] == "wp":
        print(item["label"], item["p"])
    elif item["type"] == "dubins":
        print(item["label"], item["p"])

# # 세부 컨테이너 접근 (원하면)
# all_parts_lines    = result["all_parts_lines"]     # 폴리곤 내부 세그먼트(라인 단위)
# all_bbox_lines     = result["all_bbox_lines"]      # 바운딩박스 내부 세그먼트(라인 단위)
# all_parts_wps_bbox = result["all_parts_wps_bbox"]  # 각 파트의 WP 리스트
# dubins_between     = result["dubins_between_parts"]# {(i,i+1): [분기점들]}
