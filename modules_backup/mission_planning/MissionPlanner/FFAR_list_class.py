import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon as PatchPolygon, Circle as PatchCircle, Arc
from shapely.geometry import Point, LineString, MultiPoint, Polygon as ShapelyPolygon


class FAR:
    def __init__(self, polygon_points, start_point, goal_point, circle_radius=500, Sweep_Avail_Standoff_Dist=400):
        self.polygon_points = polygon_points
        self.start_point = start_point
        self.goal_point = goal_point
        self.circle_radius = circle_radius
        self.Sweep_Avail_Standoff_Dist = Sweep_Avail_Standoff_Dist

        self.Aisle_1 = None
        self.Aisle_2 = None
        self.bounding_box_center = None
        self.unrotated_box = None

        self.Aisle_1_1 = []
        self.Aisle_1_2 = []
        self.Aisle_2_1 = []
        self.Aisle_2_2 = []
        self.max_standoff_point_1 = []
        self.max_standoff_point_2 = []

        self.Mission_start_range_1_1 = None
        self.Mission_start_range_1_2 = None
        self.Mission_start_range_2_1 = None
        self.Mission_start_range_2_2 = None

        self.MSR_1 = []
        self.MSR_2 = []
        self.Aisle__1 = None
        self.Aisle__2 = None

        self.Mission_Start_Point = None
        self.Mission_Goal_Point = None
        self.Flight_Avail_Range = None
        self.Flight_Avail_Range_list = None

        self.arc_infos = {}

    def midpoint(self, p1, p2):
        return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

    def distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def rotate_point(self, p, angle):
        x, y = p
        return (x * math.cos(angle) - y * math.sin(angle),
                x * math.sin(angle) + y * math.cos(angle))

    def rotate_points(self, points, angle):
        return [self.rotate_point(p, angle) for p in points]

    def extend_line(self, p1, p2, length=2000):
        dir_vec = self.unit_vector(p1, p2)
        mid = self.midpoint(p1, p2)
        half = length / 2
        p_start = (mid[0] - dir_vec[0]*half, mid[1] - dir_vec[1]*half)
        p_end = (mid[0] + dir_vec[0]*half, mid[1] + dir_vec[1]*half)
        return LineString([p_start, p_end])

    def unit_vector(self, p1, p2):
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        l = math.hypot(dx, dy)
        return (dx/l, dy/l)

    def unit_normal(self, p1, p2):
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        l = math.hypot(dx, dy)
        return (-dy/l, dx/l)

    def calculate_chord_length_from_center(self, radius, d):
        if d > radius:
            return None
        return 2 * math.sqrt(radius**2 - d**2)

    def arc_to_linestring(self, center, p1, p2, keep_outward=True, resolution=500):
        v1 = np.array([p1[0] - center[0], p1[1] - center[1]])
        v2 = np.array([p2[0] - center[0], p2[1] - center[1]])
        a1 = math.degrees(math.atan2(v1[1], v1[0]))
        a2 = math.degrees(math.atan2(v2[1], v2[0]))
        if not keep_outward:
            a1, a2 = a2, a1
        if a2 < a1:
            a2 += 360
        angles = np.linspace(math.radians(a1), math.radians(a2), resolution)
        return LineString([
            (center[0] + self.circle_radius * math.cos(a), center[1] + self.circle_radius * math.sin(a))
            for a in angles
        ])

    def extract_and_draw_mission_range(self, ax, arc_id, pt1, pt2, label):
        center, p1a, p1b, keep_outward = self.arc_infos[arc_id]
        full_arc = self.arc_to_linestring(center, p1a, p1b, keep_outward, resolution=300)
        coords = list(full_arc.coords)

        idx1 = min(range(len(coords)), key=lambda i: self.distance(coords[i], pt1))
        idx2 = min(range(len(coords)), key=lambda i: self.distance(coords[i], pt2))

        if idx1 > idx2:
            idx1, idx2 = idx2, idx1
            pt1, pt2 = pt2, pt1

        arc_segment_coords = [pt1] + coords[idx1+1:idx2] + [pt2]
        arc_segment = LineString(arc_segment_coords)

        ax.plot(*arc_segment.xy, color='purple', linewidth=2)
        mid = arc_segment.interpolate(0.5, normalized=True)
        ax.text(mid.x, mid.y, label, fontsize=9, color='purple', ha='center')
        return arc_segment


    def precise_circle_line_intersection(self, center, radius, p1, p2):
        x0, y0 = center
        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1
        fx = x1 - x0
        fy = y1 - y0

        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius

        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return []

        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)

        inter_points = []
        for t in [t1, t2]:
            px = x1 + t * dx
            py = y1 + t * dy
            inter_points.append((px, py))

        return inter_points

    def draw_arc_from_intersections(self, ax, arc_id, center, p1, p2, keep_outward=True):
        self.arc_infos[arc_id] = (center, p1, p2, keep_outward)
        v1 = np.array([p1[0] - center[0], p1[1] - center[1]])
        v2 = np.array([p2[0] - center[0], p2[1] - center[1]])
        a1 = math.degrees(math.atan2(v1[1], v1[0]))
        a2 = math.degrees(math.atan2(v2[1], v2[0]))
        start_angle = a1
        sweep_angle = (a2 - a1) % 360
        if not keep_outward:
            start_angle = a2
            sweep_angle = (a1 - a2) % 360
        arc = Arc(center, width=2 * self.circle_radius, height=2 * self.circle_radius,
                  angle=0, theta1=start_angle, theta2=start_angle + sweep_angle,
                  edgecolor='yellow', linewidth=2)
        ax.add_patch(arc)

    def draw_rotated_bounding_box_and_circle(self, ax):
        min_dist = float('inf')
        ref_side = None
        for i in range(len(self.polygon_points)):
            p1, p2 = self.polygon_points[i], self.polygon_points[(i+1)%len(self.polygon_points)]
            d = self.distance(self.midpoint(p1, p2), self.start_point)
            if d < min_dist:
                min_dist, ref_side = d, (p1, p2)

        angle = math.atan2(ref_side[1][1]-ref_side[0][1], ref_side[1][0]-ref_side[0][0])
        rotated = self.rotate_points(self.polygon_points, -angle)
        xs, ys = zip(*rotated)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        box = [(min_x,min_y), (max_x,min_y), (max_x,max_y), (min_x,max_y)]
        self.unrotated_box = self.rotate_points(box, angle)
        self.bounding_box_center = self.midpoint(self.unrotated_box[0], self.unrotated_box[2])
        ax.add_patch(PatchPolygon(self.unrotated_box, closed=True, edgecolor='blue', fill=False, linewidth=2))

        for i in range(4):
            mid = self.midpoint(self.unrotated_box[i], self.unrotated_box[(i+1)%4])
            ax.text(mid[0], mid[1], str(i+1), color='blue', fontsize=12, ha='center', va='center')
            ax.text(self.unrotated_box[i][0], self.unrotated_box[i][1], str(i+1), color='orange', fontsize=12, ha='center')
            ax.add_patch(PatchCircle(self.unrotated_box[i], radius=self.circle_radius, fill=False, edgecolor='purple', linestyle='--'))

        edges = [self.extend_line(self.unrotated_box[0], self.unrotated_box[1], 5000), self.extend_line(self.unrotated_box[2], self.unrotated_box[3], 5000)]

        for i, corner in enumerate(self.unrotated_box):
            edge = edges[0] if i in [0, 1] else edges[1]
            intersections = self.precise_circle_line_intersection(corner, self.circle_radius, *edge.coords)
            if len(intersections) != 2:
                continue
            p1, p2 = intersections
            mid_chord = self.midpoint(p1, p2)
            dir_vec = self.unit_normal(p1, p2)
            check_pt1 = (mid_chord[0] + dir_vec[0]*50, mid_chord[1] + dir_vec[1]*50)
            check_pt2 = (mid_chord[0] - dir_vec[0]*50, mid_chord[1] - dir_vec[1]*50)
            d1 = self.distance(check_pt1, self.bounding_box_center)
            d2 = self.distance(check_pt2, self.bounding_box_center)
            keep_outward = d1 < d2
            self.draw_arc_from_intersections(ax, i+1, corner, p1, p2, keep_outward)

        chord = self.calculate_chord_length_from_center(self.circle_radius, self.Sweep_Avail_Standoff_Dist)
        A, B = self.unrotated_box[0], self.unrotated_box[1]
        dir_vec = self.unit_vector(A, B)
        norm_vec = self.unit_normal(A, B)
        offset = chord / 2
        center1 = (A[0]+dir_vec[0]*offset, A[1]+dir_vec[1]*offset)
        center2 = (B[0]-dir_vec[0]*offset, B[1]-dir_vec[1]*offset)

        def make_line(center):
            length = 5000
            p1 = (center[0]-norm_vec[0]*length/2, center[1]-norm_vec[1]*length/2)
            p2 = (center[0]+norm_vec[0]*length/2, center[1]+norm_vec[1]*length/2)
            return [p1, p2]

        self.Aisle_1 = make_line(center1)
        self.Aisle_2 = make_line(center2)

        ax.plot(*zip(*self.Aisle_1), color='green', linewidth=2)
        ax.plot(*zip(*self.Aisle_2), color='darkgreen', linewidth=2)
        ax.text(*self.midpoint(*self.Aisle_1), "Aisle_1", color='green', fontsize=10, weight='bold')
        ax.text(*self.midpoint(*self.Aisle_2), "Aisle_2", color='darkgreen', fontsize=10, weight='bold')

        for arc_id, (center, p1, p2, keep_outward) in self.arc_infos.items():
            arc_ls = self.arc_to_linestring(center, p1, p2, keep_outward)
            line = LineString(self.Aisle_1) if arc_id in [1, 4] else LineString(self.Aisle_2)
            inter = arc_ls.intersection(line)
            if inter.is_empty:
                continue
            def label_and_store(arc_id, pt):
                if arc_id == 1: self.Aisle_1_1.append(pt); return "Aisle_1_1"
                if arc_id == 4: self.Aisle_1_2.append(pt); return "Aisle_1_2"
                if arc_id == 2: self.Aisle_2_1.append(pt); return "Aisle_2_1"
                if arc_id == 3: self.Aisle_2_2.append(pt); return "Aisle_2_2"
                return ""
            if isinstance(inter, Point):
                pt = (inter.x, inter.y)
                label = label_and_store(arc_id, pt)
                ax.plot(*pt, 'ro')
                ax.text(pt[0], pt[1], label, fontsize=9, color='red', ha='left')
            elif isinstance(inter, MultiPoint):
                for pt in inter.geoms:
                    p = (pt.x, pt.y)
                    label = label_and_store(arc_id, p)
                    ax.plot(*p, 'ro')
                    ax.text(p[0], p[1], label, fontsize=9, color='red', ha='left')

        def compute_arc_intersection(id1, id2, label, store_list):
            c1, p1a, p1b, k1 = self.arc_infos[id1]
            c2, p2a, p2b, k2 = self.arc_infos[id2]
            arc1 = self.arc_to_linestring(c1, p1a, p1b, k1)
            arc2 = self.arc_to_linestring(c2, p2a, p2b, k2)
            inter = arc1.intersection(arc2)
            if inter.is_empty:
                return
            if isinstance(inter, Point):
                pt = (inter.x, inter.y)
                store_list.append(pt)
                ax.plot(*pt, 'mo')
                ax.text(pt[0], pt[1], label, fontsize=9, color='magenta', ha='left')
            elif isinstance(inter, MultiPoint):
                for pt in inter.geoms:
                    p = (pt.x, pt.y)
                    store_list.append(p)
                    ax.plot(*p, 'mo')
                    ax.text(p[0], p[1], label, fontsize=9, color='magenta', ha='left')

        compute_arc_intersection(1, 2, "max_standoff_point_1", self.max_standoff_point_1)
        compute_arc_intersection(3, 4, "max_standoff_point_2", self.max_standoff_point_2)

        if all([self.Aisle_1_1, self.Aisle_2_1, self.Aisle_1_2, self.Aisle_2_2, self.max_standoff_point_1, self.max_standoff_point_2]):
            self.Mission_start_range_1_1 = self.extract_and_draw_mission_range(ax, 1, self.Aisle_1_1[0], self.max_standoff_point_1[0], "MSR_1_1")
            self.Mission_start_range_1_2 = self.extract_and_draw_mission_range(ax, 2, self.Aisle_2_1[0], self.max_standoff_point_1[0], "MSR_1_2")
            self.Mission_start_range_2_1 = self.extract_and_draw_mission_range(ax, 4, self.Aisle_1_2[0], self.max_standoff_point_2[0], "MSR_2_1")
            self.Mission_start_range_2_2 = self.extract_and_draw_mission_range(ax, 3, self.Aisle_2_2[0], self.max_standoff_point_2[0], "MSR_2_2")

            self.MSR_1 = [self.Mission_start_range_1_2, self.Mission_start_range_1_1]
            self.MSR_2 = [self.Mission_start_range_2_1, self.Mission_start_range_2_2]
            self.Aisle__1 = LineString([self.Aisle_1_1[0], self.Aisle_1_2[0]])
            self.Aisle__2 = LineString([self.Aisle_2_2[0], self.Aisle_2_1[0]])

            for segment in self.MSR_1 + self.MSR_2:
                ax.plot(*segment.xy, color='orange', linewidth=2.5)
            ax.plot(*self.Aisle__1.xy, color='orange', linewidth=2.5)
            ax.plot(*self.Aisle__2.xy, color='orange', linewidth=2.5)

            coords = []
            for arc in self.MSR_1:
                coords.extend(arc.coords)
            coords.extend(self.Aisle__1.coords)
            for arc in self.MSR_2:
                coords.extend(arc.coords)
            coords.extend(self.Aisle__2.coords)

            self.Flight_Avail_Range = PatchPolygon(coords, closed=True, edgecolor='black', facecolor='none', linewidth=5)
            ax.add_patch(self.Flight_Avail_Range)
            ax.text(coords[0][0], coords[0][1], "Flight_Avail_Range", color='cyan', fontsize=12, weight='bold')

            self.Flight_Avail_Range_list = coords

            all_msr_points = [pt for arc in self.MSR_1 + self.MSR_2 for pt in arc.coords]
            self.Mission_Start_Point = min(all_msr_points, key=lambda p: self.distance(p, self.start_point))
            ax.plot(*self.Mission_Start_Point, 'bo', markersize=8)
            ax.text(self.Mission_Start_Point[0], self.Mission_Start_Point[1], "Mission_Start_Point", color='blue', fontsize=10, weight='bold')

            self.Mission_Goal_Point = min(all_msr_points, key=lambda p: self.distance(p, self.goal_point))
            ax.plot(*self.Mission_Goal_Point, 'go', markersize=8)
            ax.text(self.Mission_Goal_Point[0], self.Mission_Goal_Point[1], "Mission_Goal_Point", color='green', fontsize=10, weight='bold')


    def slice_flight_avail_range(self, ax=None):
        if not self.Flight_Avail_Range_list or not self.Mission_Start_Point:
            return None, None

        far_polygon = ShapelyPolygon(self.Flight_Avail_Range_list)
        bbox_points = self.unrotated_box
        mission_start_point = self.Mission_Start_Point

        p0, p1, p2, p3 = bbox_points
        height = LineString([p0, p3]).length
        dir_vec = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        dir_vec = dir_vec / np.linalg.norm(dir_vec)

        msp = np.array(mission_start_point)
        cut_center = msp + dir_vec * height

        cut_dir = np.array([p1[0] - p0[0], p1[1] - p0[1]])
        cut_dir = cut_dir / np.linalg.norm(cut_dir)

        line_len = 1e4
        pt1 = cut_center + cut_dir * line_len
        pt2 = cut_center - cut_dir * line_len
        cut_line = LineString([pt1, pt2])

        rect = ShapelyPolygon([tuple(msp), tuple(pt1), tuple(pt2)])
        sliced = far_polygon.intersection(rect)

        FFAR = None
        GOAL_POINT = None

        if sliced.geom_type == 'Polygon':
            FFAR = sliced
            exterior_coords = list(sliced.exterior.coords)
            min_dist = float('inf')
            closest_seg = None
            closest_pt = None
            goal = Point(self.goal_point)
            for i in range(len(exterior_coords) - 1):
                seg = LineString([exterior_coords[i], exterior_coords[i + 1]])
                dist = seg.distance(goal)
                if dist < min_dist:
                    min_dist = dist
                    closest_seg = seg
                    closest_pt = seg.interpolate(seg.project(goal))
            GOAL_POINT = closest_pt

            if ax:
                x, y = FFAR.exterior.xy
                ax.plot(x, y, color='skyblue', linewidth=2, label='FFAR')
                if GOAL_POINT:
                    ax.plot(GOAL_POINT.x, GOAL_POINT.y, 'mo', label='GOAL_POINT')
                    ax.text(GOAL_POINT.x, GOAL_POINT.y, "GOAL_POINT", fontsize=9, color='magenta', ha='left')
                ax.legend()

        return FFAR, GOAL_POINT


    def setup_plot(self):
        fig, ax = plt.subplots()
        ax.set_title("Mission Start Range")
        ax.set_xlim(-800, 800)
        ax.set_ylim(-800, 800)
        ax.set_aspect('equal')
        ax.grid(True)
        return fig, ax

    def compute(self, ax=None):
        if ax is None:
            fig, ax = self.setup_plot()
        self.draw_rotated_bounding_box_and_circle(ax)
        if hasattr(self, 'unrotated_box') and self.unrotated_box:
            return (
                self.MSR_1,
                self.MSR_2,
                self.Flight_Avail_Range_list,
                self.Mission_Start_Point,
                self.Mission_Goal_Point,
                self.unrotated_box
            )
        else:
            return None


polygon = [(228.80490296220637, 341.1449973247727), (-202.3444720073935, -1.2841091492775831), (-135.41514665110185, -118.02130453815846), (322.1946592733109, 194.834379104042)]
start = (-739.3355707962448, -494.69332165961373)
goal = (786.0304489517969, -742.7209494625224)

far = FAR(polygon, start, goal)
fig, ax = far.setup_plot()
# MSR_1, MSR_2, FAR_list, MSP, MGP, bbox = far.compute(ax)
_, _, _, MSP, _, _ = far.compute(ax)


FFAR, GOAL_POINT = far.slice_flight_avail_range(ax) # ✅ 이렇게 호출하면 됨

# print("FFAR:", FFAR)
# print("GOAL_POINT:", GOAL_POINT)
plt.show()
