import matplotlib.pyplot as plt
import numpy as np
import math
from fractions import Fraction
from Dubins_Path import DubinsPath
from BF_DB_Generate import BFDBGenerate
class BFPlanner:
    """
    Coverage Path Planner with Dubins transition integration.
    Given polygon vertices, start/goal, camera FOV, and altitude,
    computes a CPP path and inserts Dubins branch points.
    """
    def __init__(self, points, start_point, goal_point, R_min=360):
        self.points = points
        self.start_point = start_point  # [x, y, h]
        self.goal_point = goal_point
        self.h = start_point[2]
        self.fov = BFDBGenerate().find_max_fov_under_limit()         # diagonal FOV in degrees
        self.dubins = DubinsPath(R_min)

    # --------- FOV & Footprint Utilities ---------
    def calculate_horizontal_vertical_fov(self, diagonal_fov_deg):
        aspect_ratio = 16.0 / 9.0
        diag_rad = math.radians(diagonal_fov_deg)
        tan_half = math.tan(diag_rad / 2)
        denom = math.sqrt(1 + aspect_ratio**2)
        vertical_fov = 2 * math.atan(tan_half / denom)
        horizontal_fov = 2 * math.atan((aspect_ratio * tan_half) / denom)
        return math.degrees(horizontal_fov), math.degrees(vertical_fov)

    def cot(self, angle_deg):
        return 1 / math.tan(math.radians(angle_deg))

    def Footprint(self, fov_v, fov_h, h, af=90):
        w1 = (2 * h * math.tan(math.radians(fov_h / 2))) / math.sin(math.radians(af - fov_v / 2))
        w2 = (2 * h * math.tan(math.radians(fov_h / 2))) / math.sin(math.radians(af + fov_v / 2))
        l = h * (self.cot(af - fov_v / 2) - self.cot(af + fov_v / 2))
        W = ((w1 + w2) / 2) * l
        return w1, w2, l, W

    # --------- Geometry Utilities ---------
    def side_distance_from_point(self, point, line_start, line_end):
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2*y1 - y2*x1)
        den = math.hypot(y2 - y1, x2 - x1)
        return num / den

    def nearest_line_to_start_point(self):
        min_d = float('inf')
        nearest = None
        for i in range(len(self.points)):
            a = self.points[i]
            b = self.points[(i+1) % len(self.points)]
            d = self.side_distance_from_point(self.start_point[:2], a, b)
            if d < min_d:
                min_d, nearest = d, (a, b)
        center = ((nearest[0][0] + nearest[1][0]) / 2,
                  (nearest[0][1] + nearest[1][1]) / 2)
        return nearest, center

    def farthest_point_from_line(self, line):
        max_d = -float('inf')
        far_pt = None
        for p in self.points:
            d = self.side_distance_from_point(p, line[0], line[1])
            if d > max_d:
                max_d, far_pt = d, p
        return far_pt

    def rotate_point(self, point, angle):
        px, py = point
        return (px * math.cos(angle) - py * math.sin(angle),
                px * math.sin(angle) + py * math.cos(angle))

    def draw_rotated_bounding_box(self, ax, side):
        start, end = side
        ang = math.atan2(end[1] - start[1], end[0] - start[0])
        rot_pts = [self.rotate_point(p, -ang) for p in self.points]
        xs, ys = zip(*rot_pts)
        bbox = [(min(xs), min(ys)), (max(xs), min(ys)),
                (max(xs), max(ys)), (min(xs), max(ys))]
        unrot = [self.rotate_point(p, ang) for p in bbox]
        ax.add_patch(plt.Polygon(unrot, closed=True, fill=None, edgecolor='blue'))
        return unrot, ang

    def line_intersection(self, line1, line2):
        (x1, y1), (x2, y2) = line1
        (x3, y3), (x4, y4) = line2
        def det(a, b): return a[0]*b[1] - a[1]*b[0]
        xdiff = (x1-x2, x3-x4)
        ydiff = (y1-y2, y3-y4)
        div = det(xdiff, ydiff)
        if abs(div) < 1e-6: return None
        d = (det((x1,y1),(x2,y2)), det((x3,y3),(x4,y4)))
        return det(d, xdiff)/div, det(d, ydiff)/div

    def is_point_left_of_line(self, start, end, pt):
        return ((end[0]-start[0])*(pt[1]-start[1])
                - (end[1]-start[1])*(pt[0]-start[0])) > 0

    def is_point_on_line_segment(self, start, end, pt):
        return (min(start[0], end[0]) <= pt[0] <= max(start[0], end[0]) and
                min(start[1], end[1]) <= pt[1] <= max(start[1], end[1]))

    def calculate_path_length(self, path, start, end):
        total = 0
        for i in range(0, len(path)-1, 2):
            total += math.dist(path[i], path[i+1])
        total += min(math.dist(path[0], start[:2]), math.dist(path[-1], start[:2]))
        total += min(math.dist(path[-1], end[:2]), math.dist(path[0], end[:2]))

        return total

    def LTR_CPP(self, pts, p1, p2):
        path = []
        n = len(pts) // 2
        for i in range(n):
            if i % 2 == 0:
                path.extend([p1[i], p2[i]])
            else:
                path.extend([p2[i], p1[i]])
        return path, self.calculate_path_length(path, self.start_point, self.goal_point)

    def RTL_CPP(self, pts, p1, p2):
        path = []
        n = len(pts) // 2
        for i in range(n):
            if i % 2 == 0:
                path.extend([p2[i], p1[i]])
            else:
                path.extend([p1[i], p2[i]])
        return path, self.calculate_path_length(path, self.start_point, self.goal_point)

    def draw_CPP(self, path):
        xs, ys = zip(*path)
        plt.plot(xs, ys, marker='o', markersize=5, linewidth=4)

    def calculate_cost(self, pts, p1, p2):
        cpp1, d1 = self.LTR_CPP(pts, p1, p2)
        cpp2, d2 = self.RTL_CPP(pts, p1, p2)
        # print(f"path1: {int(d1)} meter")
        # print(f"path2: {int(d2)} meter")
        if d1 <= d2:
            self.draw_CPP(cpp1)
            return cpp1
        else:
            self.draw_CPP(cpp2)
            return cpp2

    def side_draw_parallel_lines(self, ax, start, end, bbox, center, far_pt, interval):
        interval = interval - 6.5
        path1, path2, intersects = [], [], []
        base = np.subtract(end, start)
        norm = base / np.linalg.norm(base)
        perp = np.array([-norm[1], norm[0]])
        max_len = max(
            math.dist(bbox[0], bbox[1]),
            math.dist(bbox[1], bbox[2]),
            math.dist(bbox[2], bbox[3])
        )
        for off in np.arange(interval/2, max_len, interval):
            ns = np.add(start, perp*off)
            ne = np.add(end, perp*off)
            for i in range(len(self.points)):
                a, b = self.points[i], self.points[(i+1)%len(self.points)]
                ip = self.line_intersection((ns, ne), (a, b))
                if ip and self.is_point_on_line_segment(a, b, ip):
                    intersects.append(ip)
                    if self.is_point_left_of_line(center, far_pt, ip):
                        path1.append(ip)
                    else:
                        path2.append(ip)
                    ax.scatter(*ip, color='red', s=13)
        return self.calculate_cost(intersects, path1, path2)

    # Main CPP arrangement
    def CPP_run_arrangement(self):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        fov_h, fov_v = self.calculate_horizontal_vertical_fov(self.fov)
        near, center = self.nearest_line_to_start_point()
        far_pt = self.farthest_point_from_line(near)
        bbox, _ = self.draw_rotated_bounding_box(ax, near)
        _, w2, _, _ = self.Footprint(fov_v, fov_h, self.h)
        cpp_path = self.side_draw_parallel_lines(ax, near[0], near[1], bbox, center, far_pt, w2)
        final_path = [tuple(cpp_path[i:i+2]) for i in range(0, len(cpp_path), 2)]
        grouped_by, grouped_pi = [], []
        for i in range(0, len(cpp_path), 2):
            x1, y1 = cpp_path[i]
            x2, y2 = cpp_path[i+1]
            pt1 = (float(x1), float(y1))
            pt2 = (float(x2), float(y2))
            grouped_by.append((pt1, pt2))
            plt.show()
        print(grouped_by)
        print("\n")
        for p0, p1 in grouped_by:
            x0, y0 = p0; x1, y1 = p1
            theta = math.atan2(y1-y0, x1-x0)
            frac = Fraction(theta / math.pi).limit_denominator(16)
            n, d = frac.numerator, frac.denominator
            if n == 0:
                expr = "0"
            elif abs(n) == 1:
                sign = "-" if n < 0 else ""
                expr = f"{sign}np.pi/{d}"
            else:
                sign = "-" if n < 0 else ""
                expr = f"{sign}{abs(n)}*np.pi/{d}"
            grouped_pi.append(((x0, y0, expr),(x1, y1, expr)))
        # print(grouped_by)
        print("\n")
        print(grouped_pi)
        return final_path, grouped_by, grouped_pi

    def plan(self):
        _, _, segments = self.CPP_run_arrangement()
        full_path = []
        first = segments[0][0]
        full_path.append((first[0], first[1], self.h))
        for i, (p0, p1) in enumerate(segments):
            full_path.append((p1[0], p1[1], self.h))
            if i < len(segments)-1:
                nxt = segments[i+1][0]
                h1 = eval(p1[2], {'np': np}); h2 = eval(nxt[2], {'np': np})
                n1 = np.array([p1[0], p1[1], 0.0, h1])
                n2 = np.array([nxt[0], nxt[1], 0.0, h2])
                br = self.dubins.get_branch_points([n1, n2])
                w1 = (float(br[0,0]), float(br[0,1]), self.h)
                w2 = (float(br[1,0]), float(br[1,1]), self.h)
                full_path.extend([w1, w2, (nxt[0], nxt[1], self.h)])
        return full_path, self.fov
  
if __name__ == "__main__":
    points      = [(0, 0), (2000, 0), (2000, 2000), (0, 2000)]
    start_point = (-120, 10, 610)
    goal_point  = (-100, 1000, 610)


    planner = BFPlanner(points, start_point, goal_point)
    full_path, fov = planner.plan()

    print(full_path)
    print("\n",fov)