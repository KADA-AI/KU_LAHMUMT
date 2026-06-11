import math
import numpy as np
import matplotlib.pyplot as plt

from shapely.geometry import Polygon, LineString, box, MultiPolygon, MultiLineString, GeometryCollection
from shapely.affinity import rotate

# ========= Optional: Aisle_Sweep_CPP_shoot_plan =========
USE_SHOOT_PLAN = True
RectanglePath = None
def _try_import_shoot_plan():
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

# ========= Optional: Dubins =========
HAS_DUBINS = True
try:
    from Dubins_Path import DubinsPath
except Exception as e:
    HAS_DUBINS = False
    DubinsPath = None
    print(f"[INFO] Dubins_Path 모듈을 로드하지 못했습니다: {e}")


class AisleSweepPlanner:


    AX_MIN = -800
    AX_MAX =  800

    def __init__(self, polygon_vertices, Before_ACP,
                 width=370.0, separation_dist=1000.0, uav_height=610.0,
                 fov_deg=2.7, R_min=360.0, show_plot=True):
        self.polygon_vertices = [(float(x), float(y)) for (x,y) in polygon_vertices]
        self.Before_ACP = (float(Before_ACP[0]), float(Before_ACP[1]))

        self.width = float(width)
        self.separation_dist = float(separation_dist)
        self.uav_height = float(uav_height)
        self.fov_deg = float(fov_deg)
        self.R_min = float(R_min)
        self.show_plot = bool(show_plot)

        # containers for results
        self.all_parts_lines = {}     # polygon 내부 세그먼트(라인 단위)
        self.all_bbox_lines = {}      # bbox 내부 세그먼트(라인 단위)
        self.all_parts_wps_bbox = {}  # bbox 세그 중앙점 기반 WP
        self.dubins_between_parts = {}  # {(i,i+1): [midpoints ...]}
        self.ordered_output = []      # 출력 순서 기록(segments -> WPs -> dubins -> ...)

    # ====== 기하 유틸 ======
    @staticmethod
    def ring_coords_no_close(poly: Polygon):
        coords = list(poly.exterior.coords)
        if len(coords) > 1 and coords[0] == coords[-1]:
            coords = coords[:-1]
        return coords

    @staticmethod
    def polygon_vertices_list(poly: Polygon):
        coords = AisleSweepPlanner.ring_coords_no_close(poly)
        return [tuple(map(float, xy)) for xy in coords]

    @staticmethod
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

    @staticmethod
    def rotate_polygon_to_axis(poly: Polygon, angle_deg):
        return rotate(poly, -angle_deg, origin=(0, 0), use_radians=False)

    @staticmethod
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

    @classmethod
    def slice_polygon_by_width_using_longest_edge(cls, polygon: Polygon, width: float):
        base_coords = cls.ring_coords_no_close(polygon)
        _, _, _, _, angle = cls.longest_edge_info(base_coords)

        rotated = cls.rotate_polygon_to_axis(polygon, angle)
        minx, miny, maxx, maxy = rotated.bounds
        strips = cls.make_horizontal_strips((minx, miny, maxx, maxy), width)

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
            c = cls.rotate_polygon_to_axis(poly, angle).centroid.y
            return c
        restored.sort(key=centroid_y_in_rot_space)
        return restored, angle

    @staticmethod
    def oriented_bbox_from_polygon(poly: Polygon, angle_deg: float) -> Polygon:
        rotated = rotate(poly, -angle_deg, origin=(0, 0), use_radians=False)
        minx, miny, maxx, maxy = rotated.bounds
        rect = box(minx, miny, maxx, maxy)
        restored = rotate(rect, angle_deg, origin=(0, 0), use_radians=False)
        return restored

    @classmethod
    def parts_to_oriented_bboxes(cls, parts: list, angle_deg: float) -> list:
        return [cls.oriented_bbox_from_polygon(p, angle_deg) for p in parts if not p.is_empty]

    # ====== interval l 계산 ======
    def compute_interval_l(self):
        if USE_SHOOT_PLAN:
            _try_import_shoot_plan()
        if RectanglePath is not None:
            try:
                rp = RectanglePath(point=(0.0, 0.0),
                                   rectangle_vertices=[(0,0),(1,0),(1,1),(0,1)],
                                   separation_dist=self.separation_dist,
                                   UAV_height=self.uav_height)
                _, l, _, _ = rp.footprint(self.fov_deg, self.fov_deg)
                return float(l)
            except Exception:
                pass

        h = self.uav_height
        angle = math.degrees(math.asin(h / self.separation_dist))
        af = angle
        def cot(deg):
            rad = math.radians(deg)
            return 1.0 / math.tan(rad)
        l = h * (cot(af - self.fov_deg/2) - cot(af + self.fov_deg/2))
        return float(l)

    # ====== 평행선 생성/클리핑 ======
    @staticmethod
    def _to_rot(p, ang):
        return rotate(p, -ang, origin=(0,0), use_radians=False)

    @staticmethod
    def _to_org(p, ang):
        return rotate(p,  ang, origin=(0,0), use_radians=False)

    @classmethod
    def intersect_vertical_lines_and_clip(cls, poly_rot: Polygon, x_positions, ypad=1000.0):
        minx, miny, maxx, maxy = poly_rot.bounds
        segs_per_x = []
        full_lines = []
        for x in x_positions:
            full_line = LineString([(x, miny - ypad), (x, maxy + ypad)])
            full_lines.append(full_line)

            inter = poly_rot.intersection(full_line)
            if inter.is_empty:
                segs_per_x.append([]);  continue

            if isinstance(inter, LineString):
                segs_per_x.append([inter])
            elif isinstance(inter, MultiLineString):
                geoms = list(inter.geoms)
                geoms.sort(key=lambda g: (g.coords[0][1] + g.coords[-1][1]) * 0.5)
                segs_per_x.append(geoms)
            elif isinstance(inter, GeometryCollection):
                lines = [g for g in inter.geoms if isinstance(g, LineString)]
                lines.sort(key=lambda g: (g.coords[0][1] + g.coords[-1][1]) * 0.5)
                segs_per_x.append(lines)
            else:
                segs_per_x.append([])
        return segs_per_x, full_lines

    @classmethod
    def generate_perpendicular_sweeps_for_part(cls, part_poly: Polygon, slice_angle_deg: float,
                                               interval_l: float, start_L_to_R: bool):
        prot = cls._to_rot(part_poly, slice_angle_deg)
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

        segs_per_x, full_lines = cls.intersect_vertical_lines_and_clip(prot, xs, ypad=1000.0)

        clipped_segments_org = []
        full_parallel_org = []
        for ln in full_lines:
            full_parallel_org.append(cls._to_org(ln, slice_angle_deg))
        for segs in segs_per_x:
            for s in segs:
                clipped_segments_org.append(cls._to_org(s, slice_angle_deg))

        return clipped_segments_org, full_parallel_org

    @classmethod
    def generate_bbox_limited_lines_for_part(cls, part_poly: Polygon, slice_angle_deg: float,
                                             interval_l: float, start_L_to_R: bool):
        prot = cls._to_rot(part_poly, slice_angle_deg)
        minx, miny, maxx, maxy = prot.bounds
        bbox_rot = box(minx, miny, maxx, maxy)

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

        full_parallel_org = [ cls._to_org(ln, slice_angle_deg) for ln in full_lines_rot ]
        bbox_segments_org_by_line = []
        for geoms in bbox_segs_by_line_rot:
            bbox_segments_org_by_line.append([ cls._to_org(g, slice_angle_deg) for g in geoms ])

        return bbox_segments_org_by_line, full_parallel_org

    # ====== L/R 중심점 계산 ======
    @staticmethod
    def _rotate_point(pt, angle_deg):
        th = math.radians(angle_deg)
        c, s = math.cos(th), math.sin(th)
        x, y = pt
        return (c*x - s*y, s*x + c*y)

    @classmethod
    def left_right_edge_centers_from_bbox_in_slice_direction(cls, bbox: Polygon, slice_angle_deg: float):
        brot = rotate(bbox, -slice_angle_deg, origin=(0,0), use_radians=False)
        minx, miny, maxx, maxy = brot.bounds
        cy = 0.5*(miny+maxy)
        left_rot  = (minx, cy)
        right_rot = (maxx, cy)
        left  = cls._rotate_point(left_rot,  slice_angle_deg)
        right = cls._rotate_point(right_rot, slice_angle_deg)
        return left, right  # (L, R)

    # ====== heading / dubins 유틸 ======
    @staticmethod
    def _heading_from_two_points(p1, p2):
        return math.atan2(p2[1]-p1[1], p2[0]-p1[0])

    @staticmethod
    def _extract_midpoints_from_dubins_result(res, start_xy, end_xy, tol=1e-6):
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

    # ====== 실행 ======
    def run(self):
        # 0) 입력 폴리곤 생성
        polygon = Polygon(self.polygon_vertices)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        if polygon.is_empty or not isinstance(polygon, Polygon):
            raise ValueError("유효한 다각형이 아닙니다.")

        start_point = self.Before_ACP

        # 1) 분할 + 기준 각도
        parts, angle = self.slice_polygon_by_width_using_longest_edge(polygon, self.width)
        if len(parts) == 0:
            print("[WARN] 분할 결과가 비었습니다.")
            return self._make_return()

        bboxes = self.parts_to_oriented_bboxes(parts, angle)

        # 2) interval l 계산
        interval_l = self.compute_interval_l()
        print(f"\n[INFO] footprint 기반 interval l = {interval_l:.6f}")

        # 3) 시작 방향(L1/R1 가까운 쪽) 결정
        L1, R1 = self.left_right_edge_centers_from_bbox_in_slice_direction(bboxes[0], angle)
        dL = math.hypot(start_point[0]-L1[0], start_point[1]-L1[1])
        dR = math.hypot(start_point[0]-R1[0], start_point[1]-R1[1])
        base_L_to_R = True if dL <= dR else False
        chosen = "L1" if base_L_to_R else "R1"
        print(f"[START] 시작점에 더 가까운 변: {chosen}  (dL={dL:.2f}, dR={dR:.2f})")

        # 4) 시각화 준비
        if self.show_plot:
            fig2, ax2 = plt.subplots()
            ax2.set_aspect("equal")
            ax2.grid(True, alpha=0.3)
            ax2.set_xlim(self.AX_MIN, self.AX_MAX)
            ax2.set_ylim(self.AX_MIN, self.AX_MAX)

            # 입력 폴리곤 + 기준변 + 시작점
            x0, y0 = polygon.exterior.xy
            ax2.plot(x0, y0, lw=1.2, color='0.5', label="Polygon")
            # 기준변(가장 긴 변)
            _, a, b, _, _ = self.longest_edge_info(self.ring_coords_no_close(polygon))
            ax2.plot([a[0], b[0]], [a[1], b[1]], lw=1.2, alpha=0.7, color='0.5', linestyle='--')
            ax2.scatter([start_point[0]], [start_point[1]], s=50, marker='x', color='k', zorder=5, label='Start')
            # L1, R1 시각화
            ax2.scatter([L1[0], R1[0]], [L1[1], R1[1]], s=30, color='k')
            ax2.text(L1[0], L1[1], "L1", fontsize=9, ha='right', va='bottom')
            ax2.text(R1[0], R1[1], "R1", fontsize=9, ha='left', va='bottom')
        else:
            fig2, ax2 = None, None

        # 5) 방향/색/단위벡터
        theta = math.radians(angle)
        dir_u = np.array([math.cos(theta), math.sin(theta)], dtype=float)
        cmap = plt.cm.get_cmap('tab20', max(len(parts), 1))

        # 6) 파트별 처리
        for i, (p, bb) in enumerate(zip(parts, bboxes), start=1):
            color = cmap(i-1)
            if self.show_plot:
                px, py = p.exterior.xy
                ax2.fill(px, py, alpha=0.10, color=color)
                ax2.plot(px, py, lw=1.2, color=color)
                bx, by = bb.exterior.xy
                ax2.plot(bx, by, lw=1.0, linestyle="--", color=color, alpha=0.7)
                cx, cy = p.centroid.x, p.centroid.y
                ax2.text(cx, cy, f"{i}", fontsize=10, ha='center', va='center', color=color)

            # 진행 방향: 시작 파트 기준 교대
            start_L_to_R = ( (i % 2 == 1) if base_L_to_R else (i % 2 == 0) )

            # (1) 폴리곤 내부 세그먼트
            clipped_segments, full_parallels = self.generate_perpendicular_sweeps_for_part(
                p, angle, interval_l, start_L_to_R
            )
            part_lines = []
            for line_idx, ln in enumerate(full_parallels, start=1):
                inter = p.intersection(ln)
                line_segs = []
                def _push(ls):
                    if len(ls.coords) >= 2:
                        x1, y1 = ls.coords[0]
                        x2, y2 = ls.coords[-1]
                        if (line_idx % 2) == 0:
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
            self.all_parts_lines[i] = part_lines

            if self.show_plot:
                for li, seg_list in enumerate(part_lines, start=1):
                    for (p1, p2) in seg_list:
                        ax2.plot([p1[0], p2[0]], [p1[1], p2[1]], lw=1.8, color=color, alpha=0.95)

            # (2) 바운딩박스 내부 세그먼트
            bbox_segs_by_line, _ = self.generate_bbox_limited_lines_for_part(
                p, angle, interval_l, start_L_to_R
            )
            bbox_lines_for_store = []
            for geoms in bbox_segs_by_line:
                seg_list = []
                for ls in geoms:
                    if len(ls.coords) >= 2:
                        x1, y1 = ls.coords[0]
                        x2, y2 = ls.coords[-1]
                        seg_list.append(((x1, y1), (x2, y2)))
                bbox_lines_for_store.append(seg_list)
            self.all_bbox_lines[i] = bbox_lines_for_store

            if self.show_plot:
                for geoms in bbox_segs_by_line:
                    for ls in geoms:
                        xs, ys = ls.xy
                        ax2.plot(xs, ys, lw=1.0, linestyle=":", color=color, alpha=0.8)

            # (3) WP: bbox 세그 중앙점 기준
            offset_dir = (-dir_u) if start_L_to_R else (dir_u)
            wps = []
            for line_idx, geoms in enumerate(bbox_segs_by_line, start=1):
                for ls in geoms:
                    if len(ls.coords) >= 2:
                        (x1, y1) = ls.coords[0]
                        (x2, y2) = ls.coords[-1]
                        mid = ((x1+x2)*0.5, (y1+y2)*0.5)
                        wp = (mid[0] + self.separation_dist*offset_dir[0],
                              mid[1] + self.separation_dist*offset_dir[1])
                        wps.append(wp)
            self.all_parts_wps_bbox[i] = wps

            if self.show_plot and wps:
                wpx = [w[0] for w in wps];  wpy = [w[1] for w in wps]
                ax2.plot(wpx, wpy, lw=1.2, linestyle='-', color=color, alpha=0.9, label=f"Part {i} WPs")
                ax2.scatter(wpx, wpy, s=22, color=color, edgecolor='k', linewidth=0.4, zorder=3)

            # ===== 출력 (세그먼트 + WP) =====
            seg_counter = 1
            for seg_list in self.all_bbox_lines[i]:
                for ((x1, y1), (x2, y2)) in seg_list:
                    label = f"{i}_Seg_{seg_counter}"
                    print(f"{label}: ({x1:.6f}, {y1:.6f}) -> ({x2:.6f}, {y2:.6f})")
                    self.ordered_output.append({'type':'seg','label':label,'part':i,'p1':(x1,y1),'p2':(x2,y2)})
                    seg_counter += 1
            for wi, (wx, wy) in enumerate(wps, start=1):
                label = f"{i}_WP_{wi}"
                print(f"{label}: ({wx:.6f}, {wy:.6f})")
                self.ordered_output.append({'type':'wp','label':label,'part':i,'p':(wx,wy)})

            # ===== 파트 사이 Dubins: 다음 루프에서 처리(파트 i 끝난 뒤, i<i_last 일 때) =====

        # 7) 파트 간 Dubins 분기점(파트 i 출력이 끝난 직후 → i+1 출력 전에 끼워 넣음)
        if HAS_DUBINS and len(parts) >= 2:
            try:
                dubins = DubinsPath(R_min=self.R_min)
            except Exception:
                dubins = DubinsPath()

            # ordered_output 사이에 끼워넣을 수 있도록, 여기서는 출력만 추가/시각화만 수행
            # (실제 "순서"는 이미 파트별 출력이 완료되었으므로, part i의 마지막 출력 이후 위치로 append)
            for i in range(1, len(parts)):
                wps_prev = self.all_parts_wps_bbox.get(i, [])
                wps_next = self.all_parts_wps_bbox.get(i+1, [])
                if len(wps_prev) == 0 or len(wps_next) == 0:
                    continue

                if len(wps_prev) >= 2:
                    heading_s = self._heading_from_two_points(wps_prev[-2], wps_prev[-1])
                else:
                    heading_s = math.radians(angle)
                if len(wps_next) >= 2:
                    heading_e = self._heading_from_two_points(wps_next[0], wps_next[1])
                else:
                    heading_e = math.radians(angle)

                start_xy = wps_prev[-1]
                end_xy   = wps_next[0]

                n1 = np.array([start_xy[0], start_xy[1], 0.0, heading_s], dtype=float)
                n2 = np.array([end_xy[0],   end_xy[1],   0.0, heading_e], dtype=float)

                try:
                    br = dubins.get_branch_points([n1, n2])
                    mids = self._extract_midpoints_from_dubins_result(br, start_xy, end_xy)
                    self.dubins_between_parts[(i, i+1)] = mids

                    # 출력(파트 i와 i+1 사이)
                    for k, pt in enumerate(mids, start=1):
                        label = f"DUBINS_{i}_to_{i+1}_{k}"
                        print(f"{label}: ({pt[0]:.6f}, {pt[1]:.6f})")
                        # ordered_output에 '분기점'을 파트 i 블록의 끝에 이어 붙인다
                        self.ordered_output.append({'type':'dubins','label':label,'between':(i,i+1),'p':(pt[0],pt[1])})

                    # 시각화
                    if self.show_plot:
                        chain = [start_xy] + mids + [end_xy]
                        cx = [p[0] for p in chain]; cy = [p[1] for p in chain]
                        ax2.plot(cx, cy, 'k--', lw=1.2, alpha=0.9)
                        for pt in mids:
                            ax2.scatter([pt[0]], [pt[1]], s=80, marker='*', color='k', zorder=6)

                except Exception as e:
                    print(f"[WARN] Dubins get_branch_points 실패(i={i}): {e}")
        else:
            if len(parts) >= 2:
                print("[INFO] Dubins_Path 불가: 모듈 미탑재이거나 파트 수가 부족합니다.")

        if self.show_plot:
            ax2.legend(loc="best")
            plt.show()

        return self._make_return()

    # ====== 반환 dict 구성 ======
    def _make_return(self):
        return {
            'all_parts_lines': self.all_parts_lines,
            'all_bbox_lines': self.all_bbox_lines,
            'all_parts_wps_bbox': self.all_parts_wps_bbox,
            'dubins_between_parts': self.dubins_between_parts,
            'ordered_output': self.ordered_output
        }


# =========================
# 사용 예시
# =========================
# if __name__ == "__main__":
#     # 예제: 임의의 5각형과 시작점
#     polygon_vertices = [(0,0),(500,50),(520,420),(100,480),(-80,220)]
#     Before_ACP = (200, -200)

#     planner = AisleSweepPlanner(polygon_vertices, Before_ACP, show_plot=True)
#     result = planner.run()
    # result 딕셔너리로도 접근 가능
