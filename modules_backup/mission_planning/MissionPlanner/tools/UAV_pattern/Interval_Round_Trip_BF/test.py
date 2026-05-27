import numpy as np
import matplotlib.pyplot as plt
import math
from shapely.geometry import LineString, Polygon, Point, MultiPoint, GeometryCollection
from fractions import Fraction
from Dubins_Path import DubinsPath

class IntervalRoundFlightBF:
    def __init__(self, ROI, before_ACP, horizontal_dist, theta=0, fov1=2.6, fov2=2.6, h=610, R_min=360):
        # Region of Interest vertices
        self.ROI = [tuple(p) for p in ROI]
        # Point before ACP
        self.before_ACP = tuple(before_ACP)
        # Desired parallel flight line distance
        self.horizontal_dist = horizontal_dist
        # Angle offset
        self.theta = theta
        # Field of view parameters
        self.fov1 = fov1
        self.fov2 = fov2
        # Altitude
        self.h = h
        # Dubins path planner
        self.dubins = DubinsPath(R_min)

    def find_closest_edge(self, before_ACP, ROI):
        # Find two closest edges by point-to-line distance
        before = np.array(before_ACP)
        ROI = np.array(ROI)
        def point_to_line_distance(point, line_start, line_end):
            line_vec = line_end - line_start
            point_vec = point - line_start
            line_len = np.linalg.norm(line_vec)
            line_unit_vec = line_vec / line_len if line_len != 0 else np.array([0, 0])
            proj_len = np.dot(point_vec, line_unit_vec)
            if proj_len < 0:
                return np.linalg.norm(point - line_start)
            elif proj_len > line_len:
                return np.linalg.norm(point - line_end)
            else:
                proj_point = line_start + proj_len * line_unit_vec
                return np.linalg.norm(point - proj_point)
        edges = []
        for i in range(len(ROI)):
            a = ROI[i]
            b = ROI[(i + 1) % len(ROI)]
            dist = point_to_line_distance(before, a, b)
            length = np.linalg.norm(b - a)
            edges.append((dist, length, i, a, b))
        edges = sorted(edges, key=lambda x: (x[0], -x[1], x[2]))
        closest = edges[:2]
        if closest[0][0] == closest[1][0]:
            if closest[0][1] == closest[1][1]:
                return closest[0]
            else:
                return max(closest, key=lambda x: x[1])
        else:
            return closest[0]

    def rotate_point(self, point, angle, origin=(0, 0)):
        ox, oy = origin
        px, py = point
        cos_t = math.cos(angle)
        sin_t = math.sin(angle)
        qx = ox + cos_t * (px - ox) - sin_t * (py - oy)
        qy = oy + sin_t * (px - ox) + cos_t * (py - oy)
        return (qx, qy)

    def calculate_rotated_bounding_box_with_selected_edge(self, ROI, selected_edge):
        ROI = np.array(ROI)
        start, end = selected_edge[3], selected_edge[4]
        angle = -math.atan2(end[1] - start[1], end[0] - start[0])
        rotated_points = [self.rotate_point(p, angle, origin=start) for p in ROI]
        xs, ys = zip(*rotated_points)
        bbox = [(min(xs), min(ys)), (max(xs), min(ys)), (max(xs), max(ys)), (min(xs), max(ys))]
        unrotated_bbox = [self.rotate_point(point, -angle, origin=start) for point in bbox]
        return unrotated_bbox, angle

    def find_bbox_edge(self, selected_edge, bbox):
        bbox_edges = [(bbox[i], bbox[(i + 1) % len(bbox)]) for i in range(len(bbox))]
        for edge in bbox_edges:
            if (np.allclose(edge[0], selected_edge[3]) and np.allclose(edge[1], selected_edge[4])) or \
               (np.allclose(edge[0], selected_edge[4]) and np.allclose(edge[1], selected_edge[3])):
                opposite_index = (bbox_edges.index(edge) + 2) % len(bbox_edges)
                return bbox_edges[opposite_index]
        return bbox_edges[0]

    def find_opposite_edge(self, bbox_edge, bbox, tol=1e-6):
        bbox_edges = [(tuple(bbox[i]), tuple(bbox[(i + 1) % 4])) for i in range(4)]
        def dist(p, q): return math.hypot(p[0]-q[0], p[1]-q[1])
        for idx, (a, b) in enumerate(bbox_edges):
            d1 = dist(bbox_edge[0], a) + dist(bbox_edge[1], b)
            d2 = dist(bbox_edge[0], b) + dist(bbox_edge[1], a)
            if min(d1, d2) <= tol:
                return bbox_edges[(idx + 2) % 4]
        raise ValueError("Selected edge not found within tolerance in bbox edges")

    def draw_parallel_line_outside_bbox(self, ax, bbox_edge, horizontal_dist, before_ACP):
        bbox_start = np.array(bbox_edge[0], dtype=float)
        bbox_end   = np.array(bbox_edge[1], dtype=float)
        before_pt  = np.array(before_ACP, dtype=float)
        direction_vec = bbox_end - bbox_start
        direction_vec /= np.linalg.norm(direction_vec)
        normal_vec = np.array([-direction_vec[1], direction_vec[0]])
        to_before = before_pt - bbox_start
        if np.dot(to_before, normal_vec) < 0: normal_vec = -normal_vec
        parallel_start = bbox_start + horizontal_dist * normal_vec
        parallel_end   = bbox_end   + horizontal_dist * normal_vec
        ax.plot([parallel_start[0], parallel_end[0]], [parallel_start[1], parallel_end[1]],
                color='purple', linestyle='--', linewidth=2, label="Parallel Line Outside BBox")
        return [(tuple(parallel_start), tuple(parallel_end))]

    def distance_between_parallel_segments(self, a_seg, b_seg, tol=1e-6):
        (x1,y1),(x2,y2) = a_seg; (x3,y3),(x4,y4) = b_seg
        v1 = np.array([x2-x1, y2-y1], float)
        v2 = np.array([x4-x3, y4-y3], float)
        cross_mag = abs(v1[0]*v2[1] - v1[1]*v2[0])
        if np.linalg.norm(v1)>0 and np.linalg.norm(v2)>0 and cross_mag/(np.linalg.norm(v1)*np.linalg.norm(v2)) > tol:
            raise ValueError("두 선분은 평행하지 않습니다.")
        a =  y2 - y1; b = x1 - x2; c = -(a*x1 + b*y1)
        return abs(a*x3 + b*y3 + c) / math.hypot(a, b)

    def parallel_interval(self, opposite_edge, bbox_edge, horizontal_dist):
        between_edge_dist = self.distance_between_parallel_segments(bbox_edge, opposite_edge)
        between_flight_line_bbox_edge = horizontal_dist - between_edge_dist
        return between_flight_line_bbox_edge, between_edge_dist

    def af_angle(self, h, dist):
        return math.degrees(math.atan(h / dist))

    def cot(self, angle):
        return 1 / math.tan(math.radians(angle))

    def footprint(self, af):
        w2 = (2 * self.h * math.tan(math.radians(self.fov2 / 2))) / math.sin(math.radians(af - self.theta + (self.fov1 / 2)))
        l = self.h * (self.cot(af - self.theta - self.fov1 / 2) - self.cot(af - self.theta + self.fov1 / 2))
        return l

    def Intervals(self, h, between_flight_line_bbox_edge, horizontal_dist):
        intervals = []
        adjusted_intervals = []
        af1 = self.af_angle(h, between_flight_line_bbox_edge)
        l1 = self.footprint(af1)
        start_interval = between_flight_line_bbox_edge + l1 / 2
        intervals.append(start_interval)
        adjusted_intervals.append(start_interval - between_flight_line_bbox_edge)
        next_interval = start_interval
        while True:
            af = self.af_angle(h, next_interval)
            l = self.footprint(af)
            next_interval += l
            if next_interval >= horizontal_dist: break
            intervals.append(next_interval)
            adjusted_intervals.append(next_interval - between_flight_line_bbox_edge)
        return intervals, adjusted_intervals

    def visualize_bounding_box_with_roi(self, before_ACP, ROI, selected_edge, bbox):
        ROI = np.array(ROI)
        selected_start, selected_end = selected_edge[3], selected_edge[4]
        ROI_closed = np.vstack([ROI, ROI[0]])
        plt.plot(ROI_closed[:, 0], ROI_closed[:, 1], label="ROI", linewidth=2)
        plt.scatter(before_ACP[0], before_ACP[1], color='red', label="Before_ACP", zorder=5)
        plt.plot([selected_start[0], selected_end[0]], [selected_start[1], selected_end[1]],
                 color='green', linewidth=3, label="Selected Edge")
        bbox_arr = np.array(bbox)
        bbox_closed = np.vstack([bbox_arr, bbox_arr[0]])
        plt.plot(bbox_closed[:, 0], bbox_closed[:, 1], color='blue', linestyle='--', label="Rotated Bounding Box", linewidth=2)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Visualization of ROI, Before_ACP, Selected Edge, and Bounding Box")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        return plt.gca()
    
    def draw_parallel_lines_for_intervals(self, ax, bbox_edge, adjusted_intervals):
        """Calculate and draw parallel lines at specified intervals and compute intersections"""
        s, e = bbox_edge
        s0, e0 = np.array(s), np.array(e)
        direction = (e0 - s0) / np.linalg.norm(e0 - s0)
        perpendicular = np.array([-direction[1], direction[0]])

        parallels = []
        intersections = []
        for adj in adjusted_intervals:
            start_pt = s0 + perpendicular * adj
            end_pt = e0 + perpendicular * adj
            ax.plot([start_pt[0], end_pt[0]], [start_pt[1], end_pt[1]], '--', linewidth=2)
            parallels.append((tuple(start_pt), tuple(end_pt)))
            for seg in self.find_intersections((tuple(start_pt), tuple(end_pt)), self.ROI):
                intersections.extend(seg)
        return parallels, intersections

    def find_intersections(self, line, ROI):
        """Find intersection point pairs between line segment and ROI boundary"""
        ls = LineString(line)
        poly = Polygon(ROI)
        inter = ls.intersection(poly.boundary)
        pts = []
        if isinstance(inter, Point):
            pts.append((inter.x, inter.y))
        elif isinstance(inter, (MultiPoint, GeometryCollection)):
            for g in inter.geoms:
                if isinstance(g, Point):
                    pts.append((g.x, g.y))
        elif isinstance(inter, LineString):
            coords = list(inter.coords)
            pts.append(tuple(coords[0])); pts.append(tuple(coords[-1]))
        # Pair points
        pairs = []
        for i in range(0, len(pts)-1, 2):
            pairs.append((pts[i], pts[i+1]))
        return pairs

    def transform_point_pairs(self, point_pairs):
        """짝수만 바꾼 것, 홀수만 바꾼 것 두 리스트 반환"""
        even_swapped = []
        odd_swapped  = []
        for idx, (p_start, p_end) in enumerate(point_pairs):
            # 짝수 index만 순서를 바꿈
            if idx % 2 == 0:
                even_swapped.append((p_end, p_start))
                odd_swapped.append((p_start, p_end))
            else:
                even_swapped.append((p_start, p_end))
                odd_swapped.append((p_end, p_start))

        # flat 리스트도 만들어줌
        even_flat = [pt for pair in even_swapped for pt in pair]
        odd_flat  = [pt for pair in odd_swapped  for pt in pair]

        return even_swapped, even_flat, odd_swapped, odd_flat

    def sort_pair_along_line(self, pair, line_start, line_end):
        """line_start → line_end 진행방향으로 정렬"""
        direction = np.array(line_end) - np.array(line_start)
        direction /= np.linalg.norm(direction)
        proj1 = np.dot(np.array(pair[0]) - np.array(line_start), direction)
        proj2 = np.dot(np.array(pair[1]) - np.array(line_start), direction)
        return (pair[0], pair[1]) if proj1 < proj2 else (pair[1], pair[0])
    
    
    def draw_segments_on_parallel_line(self, ax, bbox_edge, horizontal_dist, intersection_pairs, before_ACP):
        """Offset intersection segments to parallel line and draw them"""
        start, end = bbox_edge
        s0, e0 = np.array(start), np.array(end)
        direction = (e0 - s0) / np.linalg.norm(e0 - s0)
        normal = np.array([-direction[1], direction[0]])
        if np.dot(np.array(before_ACP) - s0, normal) < 0:
            normal = -normal
        origin = s0 + normal * horizontal_dist

        offset_segments = []
        for p_start, p_end in intersection_pairs:
            vs = np.array(p_start) - s0
            ve = np.array(p_end)   - s0
            t_s = np.dot(vs, direction)
            t_e = np.dot(ve, direction)
            s_off = origin + direction * t_s
            e_off = origin + direction * t_e
            ax.plot([s_off[0], e_off[0]], [s_off[1], e_off[1]], color='blue', linewidth=2)
            offset_segments.append((tuple(s_off), tuple(e_off)))
        return offset_segments

    def Path_calculation(self, offset_segs):
        """Compute full path including Dubins branch points"""
        grouped_pi = []
        for p0, p1 in offset_segs:
            theta = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            frac = Fraction(theta / math.pi).limit_denominator(16)
            n, d = frac.numerator, frac.denominator
            if n == 0:
                expr = "0"
            elif abs(n) == 1:
                expr = f"{'-' if n<0 else ''}np.pi/{d}"
            else:
                expr = f"{'-' if n<0 else ''}{abs(n)}*np.pi/{d}"
            grouped_pi.append(((p0[0], p0[1], expr), (p1[0], p1[1], expr)))

        full_path = []
        first = grouped_pi[0][0]
        full_path.append((first[0], first[1], self.h))
        for i, (p0, p1) in enumerate(grouped_pi):
            full_path.append((p1[0], p1[1], self.h))
            if i < len(grouped_pi) - 1:
                nxt = grouped_pi[i+1][0]
                h1 = eval(p1[2], {'np': np})
                h2 = eval(nxt[2], {'np': np})
                n1 = np.array([p1[0], p1[1], 0.0, h1])
                n2 = np.array([nxt[0], nxt[1], 0.0, h2])
                br = self.dubins.get_branch_points([n1, n2])
                full_path.extend([
                    (float(br[0,0]), float(br[0,1]), self.h),
                    (float(br[1,0]), float(br[1,1]), self.h),
                    (nxt[0], nxt[1], self.h)
                ])
        return full_path

    def generate_full_path(self):
        """End-to-end path generation and visualization"""
        # 1) Select edge
        selected = self.find_closest_edge(self.before_ACP, self.ROI)
        # 2) Bounding box
        bbox, angle = self.calculate_rotated_bounding_box_with_selected_edge(self.ROI, selected)
        # 3) Edges
        bbox_edge = self.find_bbox_edge(selected, bbox)
        opposite_edge = self.find_opposite_edge(bbox_edge, bbox)
        # 4) Visualize initial
        fig, ax = plt.subplots()
        ax = self.visualize_bounding_box_with_roi(self.before_ACP, self.ROI, selected, bbox)
        # 5) Parallel flight line
        _ = self.draw_parallel_line_outside_bbox(ax, bbox_edge, self.horizontal_dist, self.before_ACP)
        # 6) Intervals
        bf_dist, edge_dist = self.parallel_interval(opposite_edge, bbox_edge, self.horizontal_dist)
        intervals, adjs = self.Intervals(self.h, bf_dist, self.horizontal_dist)
        # 7) Sweep lines & intersections
        _, inters = self.draw_parallel_lines_for_intervals(ax, bbox_edge, adjs)
       
        s0, e0 = np.array(bbox_edge[0]), np.array(bbox_edge[1])
        pairs = [(inters[i], inters[i+1]) for i in range(0, len(inters), 2)]
        sorted_pairs = [self.sort_pair_along_line(pair, s0, e0) for pair in pairs]

        # ★★★ transform_point_pairs 적용 ★★★
        even, even_flat, odd, odd_flat = self.transform_point_pairs(sorted_pairs)

        # 8) Offset segments
        offset_segs = self.draw_segments_on_parallel_line(ax, bbox_edge, self.horizontal_dist, odd, self.before_ACP)

        
        print("\n\npath : ", offset_segs)
        print(len(offset_segs))
        # 9) Dubins integration
        full = self.Path_calculation(offset_segs)
        
        
        for start, end in offset_segs:
            plt.plot([start[0], end[0]], [start[1], end[1]], marker='o', linewidth=2)
        xy_points = [(p[0], p[1]) for p in full]
        x_vals = [p[0] for p in xy_points]
        y_vals = [p[1] for p in xy_points]
        plt.plot(x_vals, y_vals, 'o-', label='Path')  # 점과 선을 함께
        ax.scatter(*zip(*self.ROI), color='black')
        plt.legend()
        plt.show()
        return full


# 1) 입력 데이터
before_ACP       = [-1500, -980]
ROI              = [[0, 0], [500, 50], [600, 500], [0, 500]]
horizontal_dist  = 1000
theta            = 0
fov1 = fov2     = 2.6
h                = 610      # 기본값 사용
R_min            = 360      # 기본값 사용

# 2) 클래스 인스턴스 생성
bf = IntervalRoundFlightBF(
    ROI=ROI,
    before_ACP=before_ACP,
    horizontal_dist=horizontal_dist,
    theta=theta,
    fov1=fov1,
    fov2=fov2,
    h=h,
    R_min=R_min
)

# 3) 전체 경로 생성 및 시각화
full_path = bf.generate_full_path()

# 4) 결과 확인
print(full_path)
print(len(full_path))