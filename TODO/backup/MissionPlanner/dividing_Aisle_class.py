import matplotlib.pyplot as plt
import numpy as np

class PolygonProcessor:
    def __init__(self, n, width, points, separation_dist):
        self.n = n
        self.width = width
        self.points = points
        self.separation_dist = separation_dist  # 여기 separation_dist 추가
        self.intersections_left = []  # 왼쪽 교차점 좌표 저장
        self.intersections_right = []  # 오른쪽 교차점 좌표 저장
        self.parallel_lines = []  # 평행 선분 저장
        self.Path_polygon_left = []  # 빨간 점 좌표 저장
        self.Path_polygon_right = []  # 파란 점 좌표 저장
        self.Path_polygon = []  # 전체 경로 저장
        self.sub_polygons = []
        self.separated_lines = []  # 이격된 선분 저장

    def draw_perpendicular_line(self, point, dx, dy, width):
        length = np.sqrt(dx**2 + dy**2)
        ux, uy = -dy / length, dx / length  # 단위 벡터로 만듦
        half_width = width / 2
        midpoint = np.array(point)
        perp_start = midpoint + np.array([ux, uy]) * half_width
        perp_end = midpoint - np.array([ux, uy]) * half_width
        return perp_start, perp_end

    def draw_perpendicular_line_at_midpoint(self, point1, point2, width):
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        length = np.sqrt(dx**2 + dy**2)
        ux, uy = -dy / length, dx / length  # 단위 벡터로 만듦
        half_width = width / 2
        midpoint = (np.array(point1) + np.array(point2)) / 2
        perp_start = midpoint + np.array([ux, uy]) * half_width
        perp_end = midpoint - np.array([ux, uy]) * half_width
        return midpoint, perp_start, perp_end

    def draw_parallel_line_from_point(self, point, dx, dy):
        parallel_end = np.array([point[0] + dx, point[1] + dy])
        return point, parallel_end

    def find_intersection(self, line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])
        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]
        div = det(xdiff, ydiff)
        if div == 0:
            return None
        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

    def process_path_polygon(self):
        sub_polygons = []
        for i in range(len(self.Path_polygon) // 2 - 1):
            sub_polygon = [self.Path_polygon[i], self.Path_polygon[i + 1], self.Path_polygon[-(i + 2)], self.Path_polygon[-(i + 1)]]
            sub_polygons.append(sub_polygon)
        return sub_polygons

    def calculate_polygons(self):
        points = self.points
        for i in range(len(points)-1):
            midpoint, perp_start, perp_end = self.draw_perpendicular_line_at_midpoint(points[i], points[i+1], self.width)
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
            right_point = perp_start if np.cross([dx, dy], [perp_start[0] - midpoint[0], perp_start[1] - midpoint[1]]) < 0 else perp_end
            left_point = perp_end if right_point is perp_start else perp_start
            parallel_start_right, parallel_end_right = self.draw_parallel_line_from_point(right_point, dx, dy)
            parallel_start_left, parallel_end_left = self.draw_parallel_line_from_point(left_point, dx, dy)
            self.parallel_lines.append((parallel_start_right, parallel_end_right, parallel_start_left, parallel_end_left))
            if i > 0:
                prev_parallel_start_right, prev_parallel_end_right, prev_parallel_start_left, prev_parallel_end_left = self.parallel_lines[-2]
                intersection_right = self.find_intersection((prev_parallel_start_right, prev_parallel_end_right), (parallel_start_right, parallel_end_right))
                if intersection_right and not any(np.allclose(intersection_right, x) for x in self.intersections_right):
                    self.intersections_right.append(intersection_right)
                    self.Path_polygon_right.append(intersection_right)
                intersection_left = self.find_intersection((prev_parallel_start_left, prev_parallel_end_left), (parallel_start_left, parallel_end_left))
                if intersection_left and not any(np.allclose(intersection_left, x) for x in self.intersections_left):
                    self.intersections_left.append(intersection_left)
                    self.Path_polygon_left.append(intersection_left)
        dx1 = points[1][0] - points[0][0]
        dy1 = points[1][1] - points[0][1]
        start_perp_start, start_perp_end = self.draw_perpendicular_line(points[0], dx1, dy1, self.width)
        dx2 = points[-1][0] - points[-2][0]
        dy2 = points[-1][1] - points[-2][1]
        end_perp_start, end_perp_end = self.draw_perpendicular_line(points[-1], dx2, dy2, self.width)
        start_left = start_perp_end if np.cross([dx1, dy1], [start_perp_end[0] - points[0][0], start_perp_end[1] - points[0][1]]) > 0 else start_perp_start
        end_left = end_perp_end if np.cross([dx2, dy2], [end_perp_end[0] - points[-1][0], end_perp_end[1] - points[-1][1]]) > 0 else end_perp_start
        start_right = start_perp_start if start_left is start_perp_end else start_perp_end
        end_right = end_perp_start if end_left is end_perp_end else end_perp_end
        if not any(np.allclose(start_left, x) for x in self.Path_polygon_left):
            self.Path_polygon_left.insert(0, start_left)
        if not any(np.allclose(end_left, x) for x in self.Path_polygon_left):
            self.Path_polygon_left.append(end_left)
        if not any(np.allclose(start_right, x) for x in self.Path_polygon_right):
            self.Path_polygon_right.insert(0, start_right)
        if not any(np.allclose(end_right, x) for x in self.Path_polygon_right):
            self.Path_polygon_right.append(end_right)
        self.Path_polygon = self.Path_polygon_left + self.Path_polygon_right[::-1]
        self.sub_polygons = self.process_path_polygon()
        self.draw_separated_lines()
        return self.sub_polygons

    def draw_separated_lines(self):
        separated_lines = []
        for i in range(len(self.points) - 1):
            p1 = np.array(self.points[i])
            p2 = np.array(self.points[i + 1])
            dx, dy = p2 - p1
            length = np.sqrt(dx**2 + dy**2)
            separation_vector = self.separation_dist * np.array([dx, dy]) / length
            separated_p1 = p1 - separation_vector
            separated_p2 = p2 - separation_vector
            separated_lines.append((separated_p1.tolist(), separated_p2.tolist()))
        self.separated_lines = separated_lines

    def get_separated_lines(self):
        return self.separated_lines

    def get_sub_polygons(self):
        return self.sub_polygons

class PolygonVisualizer:
    def __init__(self, polygon_processor):
        self.polygon_processor = polygon_processor

    def draw_polygon(self):
        points = self.polygon_processor.points
        fig, ax = plt.subplots(figsize=(15, 15))
        ax.set_xlim(-5000, 5000)
        ax.set_ylim(-5000, 5000)
        ax.set_aspect('equal')
        for point in points:
            plt.scatter(point[0], point[1], color='black', s=10)
        for i in range(len(points) - 1):
            plt.plot([points[i][0], points[i + 1][0]], [points[i][1], points[i + 1][1]], color='black')
        sub_polygons = self.polygon_processor.calculate_polygons()
        for sub_polygon in sub_polygons:
            sub_polygon = np.array(sub_polygon)
            plt.fill(sub_polygon[:, 0], sub_polygon[:, 1], edgecolor='black', fill=False)
        separated_lines = self.polygon_processor.get_separated_lines()
        for line in separated_lines:
            p1, p2 = line
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'r--')
        plt.show()
