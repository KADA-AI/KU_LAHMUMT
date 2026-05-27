import math
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PatchPolygon
from shapely.geometry import LineString, Polygon as ShapelyPolygon
from Dubins_Path import DubinsPath
from fractions import Fraction
import numpy as np
from Standoff_BF_DB_Generate import BFDBGenerate 

class Standoff_BF:
    def __init__(self, polygon, start, goal, R_min=360):
        self.start = start    # now (x,y,z)
        self.goal = goal
        self.h = start[2]
        self.polygon = polygon
        self.dubins = DubinsPath(R_min)

    # ======================== 계산 함수 ======================== #

    def find_valid_fov_af_pairs_with_geometry(self):
        calc = BFDBGenerate()
        df = calc.find_valid_fov_af_pairs_with_geometry()

        max_row = df.loc[df["Footprint Length (m)"].idxmax()]
        base_length = max_row["Base Length (m)"]
        footprint_length = max_row["Footprint Length (m)"]
        fov = max_row["FOV (deg)"]
        af = max_row["AF (deg)"]

        return base_length, footprint_length, fov, af


    # ======================== 기하 함수 ======================== #
    def distance(self, p1, p2): return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
    def midpoint(self, p1, p2): return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
    def rotate_point(self, p, ang): return (p[0]*math.cos(ang)-p[1]*math.sin(ang), p[0]*math.sin(ang)+p[1]*math.cos(ang))
    def rotate_points(self, pts, ang): return [self.rotate_point(p, ang) for p in pts]

    def calculate_rotated_bounding_box(self):
        poly = self.polygon
        max_l, sides = -1, []
        for i in range(len(poly)):
            p1, p2 = poly[i], poly[(i+1)%len(poly)]
            d = self.distance(p1, p2)
            if d > max_l:
                max_l, sides = d, [(p1, p2)]
            elif math.isclose(d, max_l, abs_tol=1e-6):
                sides.append((p1, p2))
        ref = min(sides, key=lambda s: self.distance(self.midpoint(*s), self.start))
        ang = math.atan2(ref[1][1]-ref[0][1], ref[1][0]-ref[0][0])
        rot = self.rotate_points(poly, -ang)
        xs, ys = zip(*rot)
        box = [(min(xs),min(ys)),(max(xs),min(ys)),(max(xs),max(ys)),(min(xs),max(ys))]
        return self.rotate_points(box, ang)

    def parallel_lines(self, bbox, fp_len):
        s, e, o = bbox[0], bbox[1], bbox[3]
        dx, dy = o[0]-s[0], o[1]-s[1]
        dist = math.hypot(dx, dy); ux, uy = dx/dist, dy/dist
        lines, off = [], (fp_len/2)*0.95
        while off < dist:
            lines.append([(s[0]+ux*off, s[1]+uy*off),(e[0]+ux*off, e[1]+uy*off)])
            off += fp_len
        return lines

    def get_intersections(self, bbox, parallels):
        poly = ShapelyPolygon(self.polygon)
        pts = []
        for seg in parallels:
            i = poly.intersection(LineString(seg))
            if i.is_empty: continue
            if i.geom_type=='Point': pts.append([(i.x,i.y)])
            elif i.geom_type=='MultiPoint': pts.append([(p.x,p.y) for p in i.geoms])
            elif i.geom_type=='LineString': pts.append(list(i.coords))
        return pts

    def reverse_even(self, inters): return [pts[::-1] if i%2==0 else pts for i,pts in enumerate(inters)]
    def reverse_odd(self,  inters): return [pts[::-1] if i%2==1 else pts for i,pts in enumerate(inters)]
    def translate(self, segs, unit, horiz, rev=False):
        ux,uy = unit; sc = -1 if rev else 1; dx,dy = ux*horiz*sc, uy*horiz*sc
        return [[(x+dx,y+dy) for x,y in seg] for seg in segs]
    def flatten(self, nested): return [pt for seg in nested for pt in seg]
    def clean_best_coordinates(self, best):
        return [(float(x), float(y)) for x, y in best]
    def compute(self):
        bbox = self.calculate_rotated_bounding_box()
        horiz, fp_len, fov, af = self.find_valid_fov_af_pairs_with_geometry()
        parallels = self.parallel_lines(bbox, fp_len)
        inters = self.get_intersections(bbox, parallels)

        even, odd = self.reverse_even(inters), self.reverse_odd(inters)
        unit = ((bbox[3][0] - bbox[0][0]) / math.hypot(bbox[3][0] - bbox[0][0], bbox[3][1] - bbox[0][1]),
                (bbox[3][1] - bbox[0][1]) / math.hypot(bbox[3][0] - bbox[0][0], bbox[3][1] - bbox[0][1]))

        candidates, original_groups, directions = [], [], []
        for group, label in zip((even, odd), ('even', 'odd')):
            for rev in (True, False):
                translated = self.translate(group, unit, horiz, rev)
                flat = self.flatten(translated)
                candidates.append(flat)
                original_groups.append(group)
                directions.append((label, rev))

        best, idx, md = None, None, float('inf')
        for i, path in enumerate(candidates):
            for p in (path, list(reversed(path))):
                td = sum(self.distance(p[j], p[j + 1]) for j in range(len(p) - 1)) \
                    + self.distance(self.start[:2], p[0]) + self.distance(p[-1], self.goal[:2])
                if td < md:
                    best, idx, md = p, i, td

        group, rev = directions[idx]
        original = original_groups[idx]
        if best != candidates[idx]:
            original = [list(reversed(seg)) for seg in reversed(original)]

        Imaging_plan = self.flatten(original)
        grouped_by = []
        for i in range(0, len(best), 2):
            x1, y1 = best[i]
            x2, y2 = best[i + 1]
            grouped_by.append(((float(x1), float(y1)), (float(x2), float(y2))))

        # 기존 grouped_pi 계산
        poly_cx = sum([p[0] for p in self.polygon]) / len(self.polygon)
        poly_cy = sum([p[1] for p in self.polygon]) / len(self.polygon)
        grouped_pi = []
        for p0, p1 in grouped_by:
            x0, y0 = p0
            x1, y1 = p1
            theta = math.atan2(y1 - y0, x1 - x0)
            frac = Fraction(theta / math.pi).limit_denominator(16)
            n, d = frac.numerator, frac.denominator
            expr = "0" if n == 0 else f"{'-' if n<0 else ''}{abs(n)}*np.pi/{d}" if abs(n)!=1 else f"{'-' if n<0 else ''}np.pi/{d}"

            # 다각형 중심 기준 cross product -> 오른쪽이면 +90, 왼쪽이면 -90
            cross = (x1 - x0)*(poly_cy - y0) - (y1 - y0)*(poly_cx - x0)
            perp_deg = -90 if cross > 0 else 90

            grouped_pi.append(((x0, y0, expr, perp_deg), (x1, y1, expr, perp_deg)))

        # 기존 full_flight_path
        full_flight_path = []
        first = grouped_pi[0][0]
        full_flight_path.append((first[0], first[1], self.h))
        for i, (p0, p1) in enumerate(grouped_pi):
            full_flight_path.append((p1[0], p1[1], self.h))
            if i < len(grouped_pi) - 1:
                nxt = grouped_pi[i + 1][0]
                h1, h2 = eval(p1[2], {'np': np}), eval(nxt[2], {'np': np})
                n1, n2 = np.array([p1[0], p1[1], 0.0, h1]), np.array([nxt[0], nxt[1], 0.0, h2])
                br = self.dubins.get_branch_points([n1, n2])
                w1 = (float(br[0, 0]), float(br[0, 1]), self.h)
                w2 = (float(br[1, 0]), float(br[1, 1]), self.h)
                full_flight_path.extend([w1, w2, (nxt[0], nxt[1], self.h)])

        # 추가: yaw 포함된 경로
        full_flight_path_with_yaw = []
        first = grouped_pi[0][0]
        full_flight_path_with_yaw.append((first[0], first[1], self.h, grouped_pi[0][0][3]))

        for i, (p0, p1) in enumerate(grouped_pi):
            full_flight_path_with_yaw.append((p1[0], p1[1], self.h, p1[3]))
            if i < len(grouped_pi) - 1:
                nxt = grouped_pi[i + 1][0]
                h1, h2 = eval(p1[2], {'np': np}), eval(nxt[2], {'np': np})
                n1, n2 = np.array([p1[0], p1[1], 0.0, h1]), np.array([nxt[0], nxt[1], 0.0, h2])
                br = self.dubins.get_branch_points([n1, n2])

                next_yaw = nxt[3]
                w1 = (float(br[0, 0]), float(br[0, 1]), self.h, next_yaw)
                w2 = (float(br[1, 0]), float(br[1, 1]), self.h, next_yaw)
                full_flight_path_with_yaw.extend([w1, w2, (nxt[0], nxt[1], self.h, next_yaw)])

        return group, best, original, Imaging_plan, full_flight_path, full_flight_path_with_yaw, fov, af





    def visualize(self):
        group, best, original, Imaging_plan, full_flight_path, full_flight_path_with_yaw, fov, af = self.compute()
        bbox = self.calculate_rotated_bounding_box()
        horiz, fp_len, fov, af = self.find_valid_fov_af_pairs_with_geometry()
        parallels = self.parallel_lines(bbox, fp_len)

        fig, ax = plt.subplots()
        ax.set_title(f"Standoff BF - {group.upper()} CPP")
        ax.set_aspect('equal'); ax.grid(True)

        ax.add_patch(PatchPolygon(self.polygon,closed=True,edgecolor='red',fill=False,linewidth=2))
        ax.scatter(self.start[0], self.start[1], color='blue', s=40)
        ax.scatter(self.goal[0], self.goal[1], color='green', s=40)
        ax.add_patch(PatchPolygon(bbox,closed=True,edgecolor='blue',fill=False,linewidth=2))
        for seg in parallels: ax.plot(*zip(*seg),color='gray',linestyle='--',linewidth=1)
        
        
        # flatten된 원본 그룹 경로
        if Imaging_plan and len(Imaging_plan)>1:
            ax.plot(*zip(*Imaging_plan),color='orange',linestyle='-.',linewidth=3.5,label=f'{group.capitalize()} CPP Group')
        # best path
        if best and len(best)>1:
            ax.plot(*zip(*best),color='black',linestyle='-',linewidth=2,label='Best CPP')
        ax.legend(); plt.show()
        # print("Ori : \n", original)
        # print("Imaging : \n", Imaging_plan)
        # print("\npath : \n", best)

# 사용 예시
if __name__=='__main__':
    polygon = [(37.878787878787875, 350.75757575757575),(-300, 155),(-140.69264069264068,-104.43722943722946),(121.7532467532468,163.41991341991343)]
    start, goal = (-384.1991341991342, -364.1774891774892, 610),(-346.3203463203464, 423.16017316017314, 610)
    stbf = Standoff_BF(polygon, start, goal)
    group, best, original, Imaging_plan, full_flight_path, full_flight_path_with_yaw, fov, af = stbf.compute()
    # print(fov, af)
    # print("\n")
    # print(Imaging_plan)
    # print("\n")
    print(len(full_flight_path))
    print(full_flight_path)
    print("\n")
    print(len(full_flight_path_with_yaw))
    print(full_flight_path_with_yaw)
    stbf.visualize()
