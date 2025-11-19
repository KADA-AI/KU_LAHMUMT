# main.py  –  Corridor 패치 + WP-기준 Start/End + FOV Sweep Lines
WAYPOINTS       = [(-2400, -1300), (-1610, 1700), (1020, 1803), (3050, 326)]
CORRIDOR_WIDTH  = 1000.0   # m
SEPARATION      = 850.0    # m
FOV_DEG         = 10      # 카메라 수평 FOV
SAVE_JSON       = None     # "patch_sweeps.json"

# ─── 새로 추가 ↓ (WAYPOINTS 바로 아래 넣으세요) ─────────────────
RECT_MODE        = True                       # True → 사각형 스윕 실행
RECT_POINTS      = [(-500, -500), (1500, -300),
                    (1500, 1200), (-1500, 1200)]   # 시계·역OK
RECT_START_EDGE  = 0    # 남쪽(0), 동(1), 북(2), 서(3) … 시계방향 기준
RECT_END_EDGE    = 2    # 탈출 Edge

from typing import List, Tuple
import json, math, matplotlib.pyplot as plt
import numpy as np
from dividing_Aisle_class import PolygonProcessor
from simple_dynamics import SimpleUAV, VerySimpleAutopilot

Point = Tuple[float, float]
Line  = Tuple[Point, Point]

# ─── PolygonProcessor → 패치 ──────────────────────────────
def build_patches(wps: List[Point]):
    pp = PolygonProcessor(n=len(wps), width=CORRIDOR_WIDTH,
                          points=wps, separation_dist=SEPARATION)
    pp.calculate_polygons()
    return pp.get_sub_polygons()

def edge_mid(p: Point, q: Point) -> Point:
    return ((p[0]+q[0])/2, (p[1]+q[1])/2)


# ─── 유틸 ────────────────────────────────────────────────
def unit(v: Point) -> Point:
    l = math.hypot(*v)
    return (v[0]/l, v[1]/l) if l else (0.0, 0.0)

def perp_offset(p1: Point, p2: Point, dist: float) -> Point:
    """선분 p1-p2 중점에서 시계 90°로 dist 만큼 평행 이동한 점"""
    mid = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
    vx, vy = p2[0]-p1[0], p2[1]-p1[1]
    ux, uy = unit((vy, -vx))          # 시계 90° 단위벡터
    return (mid[0]+ux*dist, mid[1]+uy*dist)

def intersect_ray_seg(o: Point, d: Point,
                      a: Point, b: Point, eps=1e-9):
    """Ray o+d*s 와 선분 ab 교차, 교차점·s 반환 (없으면 None)"""
    x1, y1 = o; dx1, dy1 = d
    x2, y2 = a; dx2, dy2 = b[0]-x2, b[1]-y2
    det = dx1*dy2 - dy1*dx2
    if abs(det) < eps:
        return None
    s = ((x2-x1)*dy2 - (y2-y1)*dx2) / det
    t = ((x2-x1)*dy1 - (y2-y1)*dx1) / det
    if 0-eps <= t <= 1+eps:
        return (x1+s*dx1, y1+s*dy1), s
    return None

def sweep_lines(patch: List[Point],
                start_edge: Line, end_edge: Line,
                fov_deg: float, separation: float) -> List[Line]:
    """패치(4점) 내부에 시작→종료선으로 점진 회전 스윕라인 생성"""
    # 앵커·방향
    a0, a1 = edge_mid(*start_edge), edge_mid(*end_edge)
    mov    = (a1[0]-a0[0], a1[1]-a0[1])
    v0     = unit((start_edge[1][0]-start_edge[0][0],
                   start_edge[1][1]-start_edge[0][1]))
    v1     = unit((end_edge[1][0]-end_edge[0][0],
                   end_edge[1][1]-end_edge[0][1]))
    if v0[0]*v1[0] + v0[1]*v1[1] < 0:  #  >90° 틀어지면 뒤집기
        v0 = (-v0[0], -v0[1])

    # 간격·개수
    spacing = 2*separation*math.tan(math.radians(fov_deg)/2)
    n       = max(int(math.ceil(math.hypot(*mov)/spacing))+1, 3)

    segs = [(patch[i], patch[(i+1)%4]) for i in range(4)]
    lines: List[Line] = []

    for i in range(n):
        t  = i/(n-1) if n>1 else 0.0
        ax = a0[0]+mov[0]*t
        ay = a0[1]+mov[1]*t
        vx = v0[0]*(1-t)+v1[0]*t
        vy = v0[1]*(1-t)+v1[1]*t
        vx, vy = unit((vx, vy))

        hits=[]
        for a,b in segs:
            hit = intersect_ray_seg((ax,ay),(vx,vy),a,b)
            if hit: hits.append(hit)
        if len(hits)>=2:
            hits.sort(key=lambda x: x[1])
            lines.append((hits[0][0], hits[-1][0]))
    return lines

# ─── 메인 ────────────────────────────────────────────────
def _sample_path(xs: list[float], ys: list[float], n: int) -> list[tuple[float, float]]:
    """누적거리 기준으로 궤적을 균등 분할해 n개 샘플 반환(시작‧끝 포함)."""
    if n <= 2:
        return [(xs[0], ys[0]), (xs[-1], ys[-1])]

    # 누적 거리 계산
    seg_len = [math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(len(xs)-1)]
    cum = [0.0]
    for l in seg_len:
        cum.append(cum[-1] + l)
    total = cum[-1]

    # 목표 거리 위치
    targets = [i * total / (n-1) for i in range(n)]
    samples = []
    j = 0
    for t in targets:
        while j < len(cum)-1 and cum[j+1] < t:
            j += 1
        # 선형 보간
        ratio = (t - cum[j]) / max(cum[j+1] - cum[j], 1e-9)
        x = xs[j] + (xs[j+1] - xs[j]) * ratio
        y = ys[j] + (ys[j+1] - ys[j]) * ratio
        samples.append((x, y))
    return samples

# ─── RECTANGLE SWEEP 생성기 ──────────────────────────────────
def rectangle_sweeps(
    rect: List[Point],          # 시계(또는 역)방향 4점
    start_edge_idx: int,        # 0~3  : 진입 Edge 인덱스
    end_edge_idx:   int,        # 0~3  : 탈출 Edge 인덱스
    fov_deg: float,
    separation: float,
) -> List[Line]:
    """
    직사각형(rect) 내부를 시작 Edge → 종료 Edge 로 회전하며 스윕 라인 생성.
    • rect[0] ~ rect[3] 은 꼭짓점 순서(시계/반시계 상관없음)
    • start_edge_idx, end_edge_idx 는 rect[i]–rect[i+1] 엣지 번호
      예) 좌하단이 0, 시계방향이면 0:남, 1:동, 2:북, 3:서
    반환 : [(p0,p1), (p0,p1), …]  # 각 스윕 라인 세그먼트
    """
    # ── 엣지·중앙 계산 ───────────────────────────────────
    edges = [(rect[i], rect[(i + 1) % 4]) for i in range(4)]
    a0 = edge_mid(*edges[start_edge_idx])
    a1 = edge_mid(*edges[end_edge_idx])
    mov = (a1[0] - a0[0], a1[1] - a0[1])

    v0 = unit((edges[start_edge_idx][1][0] - edges[start_edge_idx][0][0],
               edges[start_edge_idx][1][1] - edges[start_edge_idx][0][1]))
    v1 = unit((edges[end_edge_idx][1][0]   - edges[end_edge_idx][0][0],
               edges[end_edge_idx][1][1]   - edges[end_edge_idx][0][1]))
    if v0[0] * v1[0] + v0[1] * v1[1] < 0:      # 90° 초과면 뒤집기
        v0 = (-v0[0], -v0[1])

    # ── 스윕 개수/간격 ─────────────────────────────────
    spacing = 2 * separation * math.tan(math.radians(fov_deg) / 2)
    n = max(int(math.ceil(math.hypot(*mov) / spacing)) + 1, 3)

    segs  = edges
    lines: List[Line] = []

    for i in range(n):
        t  = i / (n - 1) if n > 1 else 0.0
        ax = a0[0] + mov[0] * t
        ay = a0[1] + mov[1] * t
        vx = v0[0] * (1 - t) + v1[0] * t
        vy = v0[1] * (1 - t) + v1[1] * t
        vx, vy = unit((vx, vy))

        hits = []
        for a, b in segs:
            hit = intersect_ray_seg((ax, ay), (vx, vy), a, b)
            if hit:
                hits.append(hit)
        if len(hits) >= 2:
            hits.sort(key=lambda x: x[1])   # s 값 기준 정렬
            lines.append((hits[0][0], hits[-1][0]))

    return lines


# ─── 유틸 끝부분에 추가 ─────────────────────────────────────
def reorder_rect_cw_from_topleft(rect: List[Point],
                                 heading_deg: float) -> List[Point]:
    """
    rect : 임의 순서 4점
    heading_deg : UAV 진행 방향(0°=E →, 90°=N ↑)
    반환        : '좌측-상단 → 시계방향' 4점
    """
    # 1) 진행 방향을 +X 로 놓는 회전행렬
    rad = math.radians(-heading_deg)          # 시계 → 좌표계 회전
    cos_t, sin_t = math.cos(rad), math.sin(rad)
    def roto(pt):
        x, y = pt
        return (x*cos_t - y*sin_t, x*sin_t + y*cos_t)

    rot_pts = [roto(p) for p in rect]
    # 2) Y(상단) 큰 순 → X(좌측) 작은 순 정렬 → 좌상단 찾기
    idx0 = min(range(4), key=lambda i: (-rot_pts[i][1], rot_pts[i][0]))
    # 3) 시계방향 여부 → CW 유지
    cw = []
    for k in range(4):
        cw.append(rect[(idx0 + k) % 4])
    # CCW일 경우 뒤집어 CW로
    area = sum(cw[i][0]*cw[(i+1)%4][1] - cw[(i+1)%4][0]*cw[i][1] for i in range(4))
    if area < 0:
        cw = [cw[0]] + cw[:0:-1]   # 첫 점 고정 후 반전
    return cw


# ─── 메인 ────────────────────────────────────────────────
def main_corridor():
    patches = build_patches(WAYPOINTS)

    fig, ax = plt.subplots(figsize=(8, 8))
    offset_pts = []                             # 검정점 → Waypoints (시각화 X)

    for idx, poly in enumerate(patches):
        if len(poly) != 4 or idx >= len(WAYPOINTS) - 1:
            continue

        px, py = zip(*(poly + [poly[0]]))
        ax.fill(px, py, color="lightgray", alpha=0.25, zorder=0)

        wp_s, wp_e = WAYPOINTS[idx], WAYPOINTS[idx + 1]
        edges = [(poly[i], poly[(i+1) % 4]) for i in range(4)]
        mids  = [edge_mid(*e) for e in edges]
        start_edge = edges[int(np.argmin([math.hypot(mp[0]-wp_s[0], mp[1]-wp_s[1]) for mp in mids]))]
        end_edge   = edges[int(np.argmin([math.hypot(mp[0]-wp_e[0], mp[1]-wp_e[1]) for mp in mids]))]

        sweeps = sweep_lines(poly, start_edge, end_edge, FOV_DEG, SEPARATION)
        for ln in sweeps:
            ax.plot(*zip(*ln), color="blue", lw=1)

            # ★ 검정점 scatter 제거, 좌표만 저장
            offset_pts.append(perp_offset(*ln, SEPARATION))

        ax.plot(*zip(*start_edge), color="green", lw=2)
        ax.plot(*zip(*end_edge),   color="red",   lw=2)

    # ── 동역학 시뮬레이션 ────────────────────────────────
    wps = [offset_pts[0]]
    for p in offset_pts[1:]:
        if math.hypot(p[0]-wps[-1][0], p[1]-wps[-1][1]) > 1e-3:
            wps.append(p)

    x0, y0 = wps[0]; x1, y1 = wps[1]
    hdg0   = math.degrees(math.atan2(-(y1 - y0), x1 - x0))

    uav = SimpleUAV(x0, y0, heading_deg=hdg0, v0=40)
    ap  = VerySimpleAutopilot(wps, v_cruise=40.0, arrival_tol=100)

    xs, ys = [uav.x], [uav.y]
    DT = 0.1
    for _ in range(10000):             # 충분히 큰 step cap
        phi_cmd, v_cmd = ap.control(uav)
        if ap.done:
            break
        uav.step(DT, phi_cmd, v_cmd)
        xs.append(uav.x); ys.append(uav.y)

    # ── 주황색 균등 샘플링 점 ──────────────────────────
    orange_pts = _sample_path(xs, ys, len(wps))     # ★ 궤적 → n개 샘플
    ox, oy = zip(*orange_pts)
    ax.scatter(ox, oy, c="orange", s=40, zorder=5, label="sampled pts")

    # ── 궤적 & 마무리 ──────────────────────────────────
    ax.plot(xs, ys, color="magenta", lw=2, label="trajectory")
    ax.set_aspect('equal', 'box')
    ax.set_title("Corridor sweeps & UAV trajectory (orange samples)")
    ax.legend(); ax.grid(True)
    plt.show()

# ─── 메인 ────────────────────────────────────────────────
def main_area():
    """
    ▶ Corridor 모드 (기본)
       · WAYPOINTS → 회랑 패치 → 스윕 & 오프셋 Waypoints
    ▶ Rectangle 모드
       · RECT_POINTS, RECT_START_EDGE / END_EDGE 로 스윕 생성
    """
    # ── 1) 스윕 대상 도형 준비 ────────────────────────────
    if RECT_MODE:
        # a) 원하는 진행 heading (deg) 지정
        rect_heading = 45.0                 # 예: 북동 45°
        # b) 좌측-상단부터 시계방향 재배열
        rect = reorder_rect_cw_from_topleft(RECT_POINTS, rect_heading)
        poly_list = [rect]
        wp_pairs  = [(None, None)]
    else:                                            # 회랑 Corridor
        poly_list = build_patches(WAYPOINTS)
        wp_pairs  = list(zip(WAYPOINTS[:-1], WAYPOINTS[1:]))

    fig, ax = plt.subplots(figsize=(8, 8))
    offset_pts: List[Point] = []

    # ── 2) 도형별 스윕 라인 & 오프셋 포인트 ───────────────
    for idx, poly in enumerate(poly_list):
        if len(poly) != 4:
            continue

        # 폴리곤 시각화
        px, py = zip(*(poly + [poly[0]]))
        ax.fill(px, py, color="lightgray", alpha=0.25, zorder=0)

        # ── 스윕 계산 분기
        if RECT_MODE:
            sweeps = rectangle_sweeps(
                poly, RECT_START_EDGE, RECT_END_EDGE, FOV_DEG, SEPARATION)
        else:
            wp_s, wp_e = wp_pairs[idx]
            edges = [(poly[i], poly[(i + 1) % 4]) for i in range(4)]
            mids  = [edge_mid(*e) for e in edges]
            start_edge = edges[int(np.argmin(
                [math.hypot(mp[0]-wp_s[0], mp[1]-wp_s[1]) for mp in mids]))]
            end_edge   = edges[int(np.argmin(
                [math.hypot(mp[0]-wp_e[0], mp[1]-wp_e[1]) for mp in mids]))]
            sweeps = sweep_lines(poly, start_edge, end_edge, FOV_DEG, SEPARATION)

            # 시작·종료 엣지 시각화 (Corridor 전용)
            ax.plot(*zip(*start_edge), color="green", lw=2)
            ax.plot(*zip(*end_edge),   color="red",   lw=2)

        # 스윕 라인 & 오프셋 점
        for ln in sweeps:
            ax.plot(*zip(*ln), color="blue", lw=1)
            offset_pts.append(perp_offset(*ln, SEPARATION))

    # ── 3) Waypoint 리스트 정리 ────────────────────────────
    wps = [offset_pts[0]]
    for p in offset_pts[1:]:
        if math.hypot(p[0]-wps[-1][0], p[1]-wps[-1][1]) > 1e-3:
            wps.append(p)

    # ── 4) 동역학 시뮬레이션 (단순) ───────────────────────
    x0, y0 = wps[0]; x1, y1 = wps[1]
    hdg0   = math.degrees(math.atan2(-(y1 - y0), x1 - x0))

    uav = SimpleUAV(x0, y0, heading_deg=hdg0, v0=40)
    ap  = VerySimpleAutopilot(wps, v_cruise=40.0, arrival_tol=100)

    xs, ys = [uav.x], [uav.y]
    DT = 0.1
    for _ in range(10000):
        phi_cmd, v_cmd = ap.control(uav)
        if ap.done:
            break
        uav.step(DT, phi_cmd, v_cmd)
        xs.append(uav.x); ys.append(uav.y)

    # ── 5) 균등 샘플링 Waypoints 시각화 ──────────────────
    orange_pts = _sample_path(xs, ys, len(wps))
    ox, oy = zip(*orange_pts)
    ax.scatter(ox, oy, c="orange", s=40, zorder=5, label="sampled pts")

    # ── 6) 궤적 & 마무리 ─────────────────────────────────
    ax.plot(xs, ys, color="magenta", lw=2, label="trajectory")
    ax.set_aspect('equal', 'box')
    title = "Rectangle sweeps" if RECT_MODE else "Corridor sweeps"
    ax.set_title(f"{title} & UAV trajectory (orange samples)")
    ax.legend(); ax.grid(True)
    plt.show()


if __name__ == "__main__":
    main_area()
