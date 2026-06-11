import math
import numpy as np
import matplotlib.pyplot as plt

from shapely.geometry import Polygon, LineString, box, MultiPolygon, MultiLineString, GeometryCollection
from shapely.affinity import rotate, translate

# =========================================================
# 설정
# =========================================================
AX_MIN = -800
AX_MAX =  800
USE_SHOOT_PLAN = True  # Aisle_Sweep_CPP_shoot_plan.RectanglePath 사용 시 True

RectanglePath = None
def try_import_shoot_plan():
    global RectanglePath
    if RectanglePath is not None:
        return
    try:
        from Aisle_Sweep_CPP_shoot_plan import RectanglePath as _RectPath
        RectanglePath = _RectPath
        print("[INFO] Aisle_Sweep_CPP_shoot_plan.RectanglePath 로드 성공")
    except Exception as e:
        RectanglePath = None
        print(f"[INFO] 촬영 계획 모듈을 찾지 못해 연동을 건너뜁니다: {e}")

# =========================================================
# 기하 유틸
# =========================================================
def ring_coords_no_close(poly: Polygon):
    coords = list(poly.exterior.coords)
    if len(coords) > 1 and coords[0] == coords[-1]:
        coords = coords[:-1]
    return coords

def polygon_vertices_list(poly: Polygon):
    coords = ring_coords_no_close(poly)
    return [tuple(map(float, xy)) for xy in coords]

def longest_edge_info(coords):
    max_len = -1.0
    best = None
    n = len(coords)
    for i in range(n):
        a = np.array(coords[i], dtype=float)
        b = np.array(coords[(i+1) % n], dtype=float)
        v = b - a
        L = np.linalg.norm(v)
        if L > max_len:
            max_len = L
            angle = math.degrees(math.atan2(v[1], v[0]))
            best = (L, tuple(a), tuple(b), i, angle)
    return best

def rotate_polygon_to_axis(poly: Polygon, angle_deg):
    return rotate(poly, -angle_deg, origin=(0, 0), use_radians=False)

def make_horizontal_strips(bounds, width):
    minx, miny, maxx, maxy = bounds
    strips = []
    y = miny
    eps = 1e-9
    while y < maxy - eps:
        y_top = min(y + width, maxy)
        strips.append(box(minx, y, maxx, y_top))
        y = y_top
    return strips

def slice_polygon_by_width_using_longest_edge(polygon: Polygon, width: float):
    base_coords = ring_coords_no_close(polygon)
    _, _, _, _, angle = longest_edge_info(base_coords)

    rotated = rotate_polygon_to_axis(polygon, angle)
    minx, miny, maxx, maxy = rotated.bounds
    strips = make_horizontal_strips((minx, miny, maxx, maxy), width)

    parts = []
    for s in strips:
        inter = rotated.intersection(s)
        if inter.is_empty:
            continue
        if isinstance(inter, Polygon):
            parts.append(inter)
        elif isinstance(inter, MultiPolygon):
            parts.extend([g for g in inter.geoms if isinstance(g, Polygon)])

    restored = [rotate(p, angle, origin=(0, 0), use_radians=False) for p in parts]

    def centroid_y_in_rot_space(poly):
        c = rotate_polygon_to_axis(poly, angle).centroid.y
        return c
    restored.sort(key=centroid_y_in_rot_space)
    return restored

def oriented_bbox_from_polygon(poly: Polygon, angle_deg: float) -> Polygon:
    rotated = rotate(poly, -angle_deg, origin=(0, 0), use_radians=False)
    minx, miny, maxx, maxy = rotated.bounds
    rect = box(minx, miny, maxx, maxy)
    restored = rotate(rect, angle_deg, origin=(0, 0), use_radians=False)
    return restored

def parts_to_oriented_bboxes(parts: list, angle_deg: float) -> list:
    return [oriented_bbox_from_polygon(p, angle_deg) for p in parts if not p.is_empty]

# =========================================================
# interval l 계산 (RectanglePath 우선, 없으면 동일식 수동계산)
# =========================================================
def compute_interval_l(separation_dist: float, uav_height: float, fov_deg: float = 2.5):
    """
    RectanglePath.footprint()의 'l'과 동일한 공식을 사용.
    fov1=fov2=fov_deg, theta=0 가정.
    """
    if USE_SHOOT_PLAN:
        try_import_shoot_plan()
    if RectanglePath is not None:
        try:
            rp = RectanglePath(point=(0.0, 0.0),
                               rectangle_vertices=[(0,0),(1,0),(1,1),(0,1)],  # dummy
                               separation_dist=separation_dist,
                               UAV_height=uav_height)
            _, l, _, _ = rp.footprint(fov_deg, fov_deg)
            return float(l)
        except Exception:
            pass

    # 동일 수식 직접 계산
    h = uav_height
    angle = math.degrees(math.asin(h / separation_dist))  # derive_angle
    af = angle                                          # theta = 0
    def cot(deg):
        rad = math.radians(deg)
        return 1.0 / math.tan(rad)
    l = h * (cot(af - fov_deg/2) - cot(af + fov_deg/2))
    return float(l)

# =========================================================
# 평행선(진행방향 ⟂) 생성 + 클리핑
#   - 기준변을 x축으로 회전 → "수직선 x=const" 생성 (진행방향에 수직)
#   - x 간격 = interval_l, 선들의 '배치 순서'만 L→R 또는 R→L
# =========================================================
def _to_rot(p, ang):
    return rotate(p, -ang, origin=(0,0), use_radians=False)

def _to_org(p, ang):
    return rotate(p,  ang, origin=(0,0), use_radians=False)

def intersect_vertical_lines_and_clip(poly_rot: Polygon, x_positions, ypad=1000.0):
    minx, miny, maxx, maxy = poly_rot.bounds
    segs_per_x = []
    full_lines = []
    for x in x_positions:
        full_line = LineString([(x, miny - ypad), (x, maxy + ypad)])  # 진행방향에 수직
        full_lines.append(full_line)

        inter = poly_rot.intersection(full_line)
        if inter.is_empty:
            segs_per_x.append([]);  continue

        if isinstance(inter, LineString):
            segs_per_x.append([inter])
        elif isinstance(inter, MultiLineString):
            geoms = list(inter.geoms)
            geoms.sort(key=lambda g: (g.coords[0][1] + g.coords[-1][1]) * 0.5)  # y중심 정렬
            segs_per_x.append(geoms)
        elif isinstance(inter, GeometryCollection):
            lines = [g for g in inter.geoms if isinstance(g, LineString)]
            lines.sort(key=lambda g: (g.coords[0][1] + g.coords[-1][1]) * 0.5)
            segs_per_x.append(lines)
        else:
            segs_per_x.append([])
    return segs_per_x, full_lines

def generate_perpendicular_sweeps_for_part(part_poly: Polygon, slice_angle_deg: float,
                                           interval_l: float, start_L_to_R: bool):
    """
    파트 하나에 대해 '진행방향(L→R 또는 R→L)에 수직'인 평행선(x=const) 생성.
    - 첫 라인은 시작 에지에서 interval_l * (1/3) 만큼 안쪽에서 시작
    - 이후엔 interval_l 간격
    - 기준변을 x축으로 회전 → x=const 수직선들 배치 → 폴리곤과 교차
    반환:
      clipped_segments_org: List[LineString]
      full_parallel_org:    List[LineString]
    """
    prot = _to_rot(part_poly, slice_angle_deg)
    minx, miny, maxx, maxy = prot.bounds

    # --- x 포지션: 1/3*interval 오프셋 적용 ---
    eps = 1e-9
    offset = interval_l / 3.0

    if start_L_to_R:
        # L→R: minx + offset 부터 interval_l 간격으로 증가
        start = minx + offset
        if start > maxx:  # 파트 폭이 너무 좁은 예외 케이스 방어
            start = (minx + maxx) * 0.5
        xs = np.arange(start, maxx + eps, interval_l)
    else:
        # R→L: maxx - offset 부터 interval_l 간격으로 감소 (배치 순서가 우→좌)
        start = maxx - offset
        if start < minx:
            start = (minx + maxx) * 0.5
        xs = np.arange(start, minx - eps, -interval_l)

    # 최소 1개 방어
    if xs.size == 0:
        xs = np.array([ (minx+maxx)*0.5 ], dtype=float)

    # 교차 계산 (회전공간)
    segs_per_x, full_lines = intersect_vertical_lines_and_clip(prot, xs, ypad=1000.0)

    # 원좌표계로 변환
    clipped_segments_org = []
    full_parallel_org = []
    for ln in full_lines:
        full_parallel_org.append(_to_org(ln, slice_angle_deg))
    for segs in segs_per_x:
        for s in segs:
            clipped_segments_org.append(_to_org(s, slice_angle_deg))

    return clipped_segments_org, full_parallel_org


# =========================================================
# 클릭 클래스
# =========================================================
class ClickPolygonBuilder:
    def __init__(self, ax, max_points=500):
        self.ax = ax
        self.max_points = max_points
        self.points = []
        self.cid_click = None
        her=self
        self.cid_key = None
        self.finished = False
        self.scatter = None
        self.line = None

    def connect(self, fig):
        self.cid_click = fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_key = fig.canvas.mpl_connect('key_press_event', self.on_key)

    def disconnect(self, fig):
        if self.cid_click is not None:
            fig.canvas.mpl_disconnect(self.cid_click)
        if self.cid_key is not None:
            fig.canvas.mpl_disconnect(self.cid_key)

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        if len(self.points) >= self.max_points:
            return
        if event.button == 1:
            self.points.append((event.xdata, event.ydata))
            self.update_drawing()
        elif event.button == 3:
            if self.points:
                self.points.pop()
                self.update_drawing()

    def on_key(self, event):
        if event.key == 'enter':
            if len(self.points) >= 3:
                self.finished = True
            plt.close(self.ax.figure)

    def update_drawing(self):
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        if self.scatter is None:
            self.scatter = self.ax.scatter(xs, ys, s=20)
        else:
            self.scatter.set_offsets(np.c_[xs, ys])
        if len(self.points) >= 2:
            closed = self.points + [self.points[0]]
            if self.line is None:
                self.line, = self.ax.plot([p[0] for p in closed], [p[1] for p in closed], lw=1.5)
            else:
                self.line.set_data([p[0] for p in closed], [p[1] for p in closed])
        self.ax.figure.canvas.draw_idle()

    def to_polygon(self):
        if len(self.points) < 3:
            return None
        poly = Polygon(self.points)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty or not isinstance(poly, Polygon):
            return None
        return poly

# =========================================================
# 메인
# =========================================================
def main():
    width = 340.0
    separation_dist = 850.0
    uav_height = 610.0

    # ----- 폴리곤 클릭 창 -----
    fig, ax = plt.subplots()
    # ax.set_title("좌클릭: 꼭짓점 추가 / 우클릭: 마지막 점 취소 / Enter: 완료")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(AX_MIN, AX_MAX)
    ax.set_ylim(AX_MIN, AX_MAX)

    builder = ClickPolygonBuilder(ax)
    builder.connect(fig)
    plt.show()

    polygon = builder.to_polygon()
    if polygon is None:
        print("유효한 다각형을 만들지 못했습니다. 최소 3개 이상의 꼭짓점이 필요합니다.")
        return

    # 기준변 정보(분할 방향)
    base_coords = ring_coords_no_close(polygon)
    _, a, b, _, angle = longest_edge_info(base_coords)

    # 분할
    parts = slice_polygon_by_width_using_longest_edge(polygon, width)

    # 각 파트의 '분할 방향' 정렬 바운딩 박스
    bboxes = parts_to_oriented_bboxes(parts, angle)

    # interval = l 계산
    interval_l = compute_interval_l(separation_dist, uav_height, fov_deg=2.5)
    print(f"\n[INFO] footprint 기반 interval l = {interval_l:.6f}")

    # ----- 결과 표시 창 -----
    fig2, ax2 = plt.subplots()
    ax2.set_title(f"\n"
                  f"")
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(AX_MIN, AX_MAX)
    ax2.set_ylim(AX_MIN, AX_MAX)

    # 원본 폴리곤 + 기준변
    x0, y0 = polygon.exterior.xy
    ax2.plot(x0, y0, lw=2, label="원본 폴리곤")
    ax2.plot([a[0], b[0]], [a[1], b[1]], lw=3, alpha=0.85, label="가장 긴 변(진행방향 기준)")

    # 파트별 처리
    for i, (p, bb) in enumerate(zip(parts, bboxes), start=1):
        # 파트/바운딩박스
        px, py = p.exterior.xy
        ax2.fill(px, py, alpha=0.22)
        ax2.plot(px, py, lw=1.2)
        bx, by = bb.exterior.xy
        ax2.plot(bx, by, lw=1.2, linestyle="--")

        # 라벨
        cx, cy = p.centroid.x, p.centroid.y
        ax2.text(cx, cy, str(i), fontsize=9, ha='center', va='center')

        # 배치 순서: 홀수파트 L→R, 짝수파트 R→L
        start_L_to_R = (i % 2 == 1)

        # 진행방향 ⟂ 평행선 생성
        clipped_segments, full_parallels = generate_perpendicular_sweeps_for_part(
            p, angle, interval_l, start_L_to_R
        )

        # 시각화
        for ln in full_parallels:
            xs, ys = ln.xy
            ax2.plot(xs, ys, lw=0.8, linestyle=":", color='gray', alpha=0.7)  # 전체 평행선(점선)
        for seg in clipped_segments:
            xs, ys = seg.xy
            ax2.plot(xs, ys, lw=1.4, color='k', alpha=0.9)  # 내부 스윕(진한선)
            
    part_lines = []  # [ line1_segs, line2_segs, ... ] ; 각 원소는 [ [(x1,y1),(x2,y2)], ... ]
    for line_idx, ln in enumerate(full_parallels, start=1):
        inter = p.intersection(ln)
        line_segs = []

        def _push(ls):
            if len(ls.coords) >= 2:
                x1, y1 = ls.coords[0]
                x2, y2 = ls.coords[-1]
                if (line_idx % 2) == 0:
                    # 짝수번째 라인: 두 점 순서 반전해서 저장
                    line_segs.append([(x2, y2), (x1, y1)])
                else:
                    line_segs.append([(x1, y1), (x2, y2)])

        if isinstance(inter, LineString):
            _push(inter)
        elif isinstance(inter, MultiLineString):
            for g in inter.geoms:
                _push(g)
        elif isinstance(inter, GeometryCollection):
            for g in inter.geoms:
                if isinstance(g, LineString):
                    _push(g)
        # Point/empty는 무시

        part_lines.append(line_segs)

    # 파트별로 바로 출력
        print(f"\n[Part {i}] lines (짝수 라인은 좌표 순서 반전하여 저장)")
        for li, seg_list in enumerate(part_lines, start=1):
            print(f"  Line {li}: segments={len(seg_list)}")
            if not seg_list:
                print("    (no intersection)")
            else:
                for si, ((x1, y1), (x2, y2)) in enumerate(seg_list, start=1):
                    print(f"    seg {si}: ({x1:.6f}, {y1:.6f}) -> ({x2:.6f}, {y2:.6f})")
    # ax2.legend(loc="best")
    plt.show()

    # 좌표 출력
    # print("\n=== 분할된 다각형 꼭짓점 좌표 (첫점 반복 없음) ===")
    # for i, p in enumerate(parts, start=1):
    #     verts = polygon_vertices_list(p)
    #     print(f"\n[Part {i}] (N={len(verts)})")
    #     for xy in verts:
    #         print(f"{xy[0]:.6f}, {xy[1]:.6f}")

    # print("\n=== 파트별 바운딩 박스 꼭짓점 좌표 (첫점 반복 없음) ===")
    # for i, bb in enumerate(bboxes, start=1):
    #     verts = polygon_vertices_list(bb)
    #     print(f"\n[BBox {i}] (N={len(verts)})")
    #     for xy in verts:
    #         print(f"{xy[0]:.6f}, {xy[1]:.6f}")

if __name__ == "__main__":
    main()
