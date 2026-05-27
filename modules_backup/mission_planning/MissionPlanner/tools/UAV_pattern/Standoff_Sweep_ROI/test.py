import math
import numpy as np
import matplotlib.pyplot as plt

from shapely.geometry import Polygon, LineString, box, MultiPolygon, MultiLineString, GeometryCollection
from shapely.affinity import rotate

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

# ===== Dubins 모듈 로드 (추가) =====
HAS_DUBINS = True
try:
    from Dubins_Path import DubinsPath
except Exception as e:
    HAS_DUBINS = False
    DubinsPath = None
    print(f"[INFO] Dubins_Path 모듈을 로드하지 못했습니다: {e}")

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
#   - x 간격 = interval_l, 첫 라인은 interval_l/3 오프셋
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
      clipped_segments_org: List[LineString]  # 폴리곤 내부 세그먼트(시각화·출력 대상)
      full_parallel_org:    List[LineString]  # 전체 평행선(라인 기준)
    """
    prot = _to_rot(part_poly, slice_angle_deg)
    minx, miny, maxx, maxy = prot.bounds

    eps = 1e-9
    offset = interval_l / 3.0

    if start_L_to_R:
        start = minx + offset
        if start > maxx:
            start = (minx + maxx) * 0.5
        xs = np.arange(start, maxx + eps, interval_l)
    else:
        start = maxx - offset
        if start < minx:
            start = (minx + maxx) * 0.5
        xs = np.arange(start, minx - eps, -interval_l)

    if xs.size == 0:
        xs = np.array([ (minx+maxx)*0.5 ], dtype=float)

    segs_per_x, full_lines = intersect_vertical_lines_and_clip(prot, xs, ypad=1000.0)

    clipped_segments_org = []
    full_parallel_org = []
    for ln in full_lines:
        full_parallel_org.append(_to_org(ln, slice_angle_deg))
    for segs in segs_per_x:
        for s in segs:
            clipped_segments_org.append(_to_org(s, slice_angle_deg))

    return clipped_segments_org, full_parallel_org

def generate_bbox_limited_lines_for_part(part_poly: Polygon, slice_angle_deg: float,
                                         interval_l: float, start_L_to_R: bool):
    """
    '파트의 오리엔티드 바운딩박스 내부'에서만 잘린 평행선(=x=const ∩ bbox)을 생성하여 반환.
    - 라인 배치 규칙(첫 l/3, 이후 l 간격)과 L↔R 시작 규칙은 위 함수와 동일.
    반환:
      bbox_segments_org_by_line: List[List[LineString]]  # 라인별 세그먼트(보통 0 또는 1개)
      full_parallel_org:         List[LineString]        # 라인 자체(원좌표계) - 필요 시 사용
    """
    # 회전 공간으로
    prot = _to_rot(part_poly, slice_angle_deg)
    minx, miny, maxx, maxy = prot.bounds
    bbox_rot = box(minx, miny, maxx, maxy)  # 회전공간에서 축정렬 bbox == 오리엔티드 bbox

    eps = 1e-9
    offset = interval_l / 3.0

    if start_L_to_R:
        start = minx + offset
        if start > maxx:
            start = (minx + maxx) * 0.5
        xs = np.arange(start, maxx + eps, interval_l)
    else:
        start = maxx - offset
        if start < minx:
            start = (minx + maxx) * 0.5
        xs = np.arange(start, minx - eps, -interval_l)

    if xs.size == 0:
        xs = np.array([ (minx+maxx)*0.5 ], dtype=float)

    # 수직 라인과 bbox_rot의 교차(보통 라인당 0 또는 1개 세그먼트)
    bbox_segs_by_line_rot = []
    full_lines_rot = []
    for x in xs:
        ln = LineString([(x, miny-1000.0), (x, maxy+1000.0)])
        full_lines_rot.append(ln)
        inter = bbox_rot.intersection(ln)
        if inter.is_empty:
            bbox_segs_by_line_rot.append([])
        elif isinstance(inter, LineString):
            bbox_segs_by_line_rot.append([inter])
        elif isinstance(inter, MultiLineString):
            geoms = list(inter.geoms)
            geoms.sort(key=lambda g: (g.coords[0][1] + g.coords[-1][1]) * 0.5)
            bbox_segs_by_line_rot.append(geoms)
        elif isinstance(inter, GeometryCollection):
            lines = [g for g in inter.geoms if isinstance(g, LineString)]
            lines.sort(key=lambda g: (g.coords[0][1] + g.coords[-1][1]) * 0.5)
            bbox_segs_by_line_rot.append(lines)
        else:
            bbox_segs_by_line_rot.append([])

    # 원좌표계로 복원
    full_parallel_org = [ _to_org(ln, slice_angle_deg) for ln in full_lines_rot ]
    bbox_segments_org_by_line = []
    for geoms in bbox_segs_by_line_rot:
        bbox_segments_org_by_line.append([ _to_org(g, slice_angle_deg) for g in geoms ])

    return bbox_segments_org_by_line, full_parallel_org

# ===== (추가) 헤딩 + Dubins 결과 파싱 =====
def _heading_from_two_points(p1, p2):
    return math.atan2(p2[1]-p1[1], p2[0]-p1[0])  # radians

def _extract_midpoints_from_dubins_result(res, start_xy, end_xy, tol=1e-6):
    """
    get_branch_points() 반환값에서 (시작/끝 제외) 분기점들만 [(x,y), ...] 추출
    - ndarray (N,2) 또는 (N,>=2)
    - list/tuple도 처리
    """
    mids = []
    if res is None:
        return mids
    arr = np.array(res, dtype=float)
    if arr.ndim == 2 and arr.shape[1] >= 2:
        for row in arr:
            x, y = float(row[0]), float(row[1])
            if (abs(x-start_xy[0])<tol and abs(y-start_xy[1])<tol): 
                continue
            if (abs(x-end_xy[0])<tol and abs(y-end_xy[1])<tol): 
                continue
            mids.append((x, y))
    return mids

# =========================================================
# 클릭 클래스
# =========================================================
class ClickPolygonBuilder:
    def __init__(self, ax, max_points=500):
        self.ax = ax
        self.max_points = max_points
        self.points = []
        self.cid_click = None
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
    # 고정 파라미터
    width = 340.0
    separation_dist = 850.0
    uav_height = 610.0
    print(f"[CFG] width={width}, separation_dist={separation_dist}, uav_height={uav_height}")

    # ----- 폴리곤 클릭 창 -----
    fig, ax = plt.subplots()
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
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(AX_MIN, AX_MAX)
    ax2.set_ylim(AX_MIN, AX_MAX)

    # 원본 폴리곤 + 기준변 (연한 회색)
    x0, y0 = polygon.exterior.xy
    ax2.plot(x0, y0, lw=1.2, color='0.5', label="원본 폴리곤")
    ax2.plot([a[0], b[0]], [a[1], b[1]], lw=1.2, alpha=0.7, color='0.5', linestyle='--', label="가장 긴 변")

    # ===== 저장 컨테이너 =====
    all_parts_lines       = {}  # {part_idx: [ [ (x1,y1),(x2,y2) ], ... ] per line}  # 폴리곤 내부 세그먼트
    all_bbox_lines        = {}  # {part_idx: [ [ (x1,y1),(x2,y2) ], ... ] per line}  # 바운딩박스 내부 세그먼트
    all_parts_wps_bbox    = {}  # {part_idx: [ (x,y), ... ] }  # bbox 세그 중앙점 기반 WP

    # 파트별 색상 팔레트
    cmap = plt.cm.get_cmap('tab20', max(len(parts), 1))

    # 진행방향 단위 벡터(+x_rot → 원좌표계)
    theta = math.radians(angle)
    dir_u = np.array([math.cos(theta), math.sin(theta)], dtype=float)

    # 파트별 처리
    for i, (p, bb) in enumerate(zip(parts, bboxes), start=1):
        color = cmap(i-1)

        # 파트 윤곽
        px, py = p.exterior.xy
        ax2.fill(px, py, alpha=0.10, color=color)
        ax2.plot(px, py, lw=1.2, color=color)

        # 바운딩 박스 윤곽(점선)
        bx, by = bb.exterior.xy
        ax2.plot(bx, by, lw=1.0, linestyle="--", color=color, alpha=0.7)

        # 라벨
        cx, cy = p.centroid.x, p.centroid.y
        ax2.text(cx, cy, f"{i}", fontsize=10, ha='center', va='center', color=color)

        # 홀수 파트 L→R, 짝수 파트 R→L
        start_L_to_R = (i % 2 == 1)

        # 1) 평행선 생성(폴리곤 내부 세그먼트)
        clipped_segments, full_parallels = generate_perpendicular_sweeps_for_part(
            p, angle, interval_l, start_L_to_R
        )

        # 라인별로 교차 세그먼트 묶기(짝수 라인 반전)
        part_lines = []  # [ line1_segs, line2_segs, ... ]
        for line_idx, ln in enumerate(full_parallels, start=1):
            inter = p.intersection(ln)
            line_segs = []
            def _push(ls):
                if len(ls.coords) >= 2:
                    x1, y1 = ls.coords[0]
                    x2, y2 = ls.coords[-1]
                    if (line_idx % 2) == 0:  # 짝수 라인 역순
                        line_segs.append(((x2, y2), (x1, y1)))
                    else:
                        line_segs.append(((x1, y1), (x2, y2)))
            if isinstance(inter, LineString):
                _push(inter)
            elif isinstance(inter, MultiLineString):
                for g in inter.geoms: _push(g)
            elif isinstance(inter, GeometryCollection):
                for g in inter.geoms:
                    if isinstance(g, LineString): _push(g)
            part_lines.append(line_segs)
        all_parts_lines[i] = part_lines

        # 시각화: 폴리곤 내부 세그먼트 = 굵은 실선
        for li, seg_list in enumerate(part_lines, start=1):
            for (p1, p2) in seg_list:
                ax2.plot([p1[0], p2[0]], [p1[1], p2[1]], lw=1.8, color=color, alpha=0.95)

        # 2) 바운딩박스 내부 세그먼트(라인별) 생성
        bbox_segs_by_line, full_parallels_bbox = generate_bbox_limited_lines_for_part(
            p, angle, interval_l, start_L_to_R
        )
        # 저장용: 좌표쌍 형태로 변환
        bbox_lines_for_store = []
        for geoms in bbox_segs_by_line:
            seg_list = []
            for ls in geoms:
                if len(ls.coords) >= 2:
                    x1, y1 = ls.coords[0]
                    x2, y2 = ls.coords[-1]
                    seg_list.append(((x1, y1), (x2, y2)))  # bbox는 방향 반전 없이 보관
            bbox_lines_for_store.append(seg_list)
        all_bbox_lines[i] = bbox_lines_for_store

        # 시각화: 바운딩박스 내부 세그먼트 = 점선
        for geoms in bbox_segs_by_line:
            for ls in geoms:
                xs, ys = ls.xy
                ax2.plot(xs, ys, lw=1.0, linestyle=":", color=color, alpha=0.8)

        # 3) 바운딩박스 세그 중앙점으로 WP 생성 (기존 로직 유지!)
        offset_dir = (-dir_u) if start_L_to_R else (dir_u)  # L->R: L방향(-u), R->L: R방향(+u)
        wps = []
        for line_idx, geoms in enumerate(bbox_segs_by_line, start=1):
            for ls in geoms:
                if len(ls.coords) >= 2:
                    (x1, y1) = ls.coords[0]
                    (x2, y2) = ls.coords[-1]
                    mid = ((x1+x2)*0.5, (y1+y2)*0.5)
                    wp = (mid[0] + separation_dist*offset_dir[0],
                          mid[1] + separation_dist*offset_dir[1])
                    wps.append(wp)
        all_parts_wps_bbox[i] = wps

        # WP 시각화(점 + 연결선)
        if wps:
            wpx = [w[0] for w in wps];  wpy = [w[1] for w in wps]
            ax2.plot(wpx, wpy, lw=1.2, linestyle='-', color=color, alpha=0.9, label=f"Part {i} WPs")
            ax2.scatter(wpx, wpy, s=22, color=color, edgecolor='k', linewidth=0.4, zorder=3)

        # ===== 출력 (세그먼트 + WP만) =====
        seg_counter = 1
        for seg_list in all_bbox_lines[i]:
            for ((x1, y1), (x2, y2)) in seg_list:
                print(f"{i}_Seg_{seg_counter}: ({x1:.6f}, {y1:.6f}) -> ({x2:.6f}, {y2:.6f})")
                seg_counter += 1

        for wi, (wx, wy) in enumerate(wps, start=1):
            print(f"{i}_WP_{wi}: ({wx:.6f}, {wy:.6f})")

    # ===== (추가) 파트 간 Dubins 분기점: get_branch_points([n1, n2]) 사용 =====
    if HAS_DUBINS and len(parts) >= 2:
        try:
            dubins = DubinsPath(R_min=360.0)  # 회전반경(필요시 조정)
        except Exception:
            # 생성자 인자가 없을 수도 있음
            dubins = DubinsPath()

        for i in range(1, len(parts)):
            wps_prev = all_parts_wps_bbox.get(i, [])
            wps_next = all_parts_wps_bbox.get(i+1, [])
            if len(wps_prev) == 0 or len(wps_next) == 0:
                continue

            # 시작/종료 헤딩: (마지막-1 → 마지막), (처음 → 두번째)
            if len(wps_prev) >= 2:
                heading_s = _heading_from_two_points(wps_prev[-2], wps_prev[-1])
            else:
                heading_s = math.radians(angle)  # 폴백
            if len(wps_next) >= 2:
                heading_e = _heading_from_two_points(wps_next[0], wps_next[1])
            else:
                heading_e = math.radians(angle)  # 폴백

            start_xy = wps_prev[-1]
            end_xy   = wps_next[0]

            # Dubins 입력 형식: [x, y, z(0.0), heading(rad)]
            n1 = np.array([start_xy[0], start_xy[1], 0.0, heading_s], dtype=float)
            n2 = np.array([end_xy[0],   end_xy[1],   0.0, heading_e], dtype=float)

            try:
                br = dubins.get_branch_points([n1, n2])     # <== 핵심
                mids = _extract_midpoints_from_dubins_result(br, start_xy, end_xy)

                # 시각화: 이전 마지막 → (분기점들) → 다음 첫 WP (검은 점선), 분기점은 ★
                chain = [start_xy] + mids + [end_xy]
                cx = [p[0] for p in chain]; cy = [p[1] for p in chain]
                ax2.plot(cx, cy, 'k--', lw=1.2, alpha=0.9)
                for k, pt in enumerate(mids, start=1):
                    ax2.scatter([pt[0]], [pt[1]], s=80, marker='*', color='k', zorder=6)
                    print(f"DUBINS_{i}_to_{i+1}_{k}: ({pt[0]:.6f}, {pt[1]:.6f})")
            except Exception as e:
                print(f"[WARN] Dubins get_branch_points 실패(i={i}): {e}")
    else:
        if len(parts) >= 2:
            print("[INFO] Dubins_Path 불가: 모듈 미탑재이거나 파트 수가 부족합니다.")

    ax2.legend(loc="best")
    plt.show()

    # 필요 시: all_parts_lines, all_bbox_lines, all_parts_wps_bbox 이용해 저장/후처리 가능

if __name__ == "__main__":
    main()
