import matplotlib.pyplot as plt
import math
import numpy as np

class RectanglePath:
    def __init__(self, point, rectangle_vertices, separation_dist=850, UAV_height=610):
        self.point = point
        self.rectangle_vertices = rectangle_vertices
        self.separation_dist = separation_dist
        self.UAV_height = UAV_height
        self.bounding_box = self.bounding_box_based_on_parallel_sides(rectangle_vertices)
        self.path = self.calculate_path()

    def horizontal_dist(self):
        mid_dist = math.sqrt(self.separation_dist ** 2 + self.UAV_height ** 2)
        return mid_dist

    def derive_angle(self):
        angle = math.degrees(math.asin(self.UAV_height / self.separation_dist))
        return angle

    def cot(self, angle):
        angle_radian = math.radians(angle)
        return 1 / math.tan(angle_radian)

    def footprint(self, fov1, fov2):
        theta = 0
        h = self.UAV_height
        separation_dist = self.separation_dist

        angle = self.derive_angle()
        af = angle + theta
        # print("af : ",af)
        w1 = (2 * h * math.tan(math.radians(fov2 / 2))) / math.sin(math.radians(af - theta - (fov1 / 2)))
        w2 = (2 * h * math.tan(math.radians(fov2 / 2))) / math.sin(math.radians(af - theta + (fov1 / 2)))
        l = h * (self.cot(af - theta - fov1 / 2) - self.cot(af - theta + fov1 / 2))
        W = ((w1 + w2) / 2) * l

        print(l)
        return W, l, w1, w2
        

    def rotate(self, points, angle):
        rotation_matrix = [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)]
        ]
        rotated_points = []
        for point in points:
            x, y = point
            new_x = x * rotation_matrix[0][0] + y * rotation_matrix[0][1]
            new_y = x * rotation_matrix[1][0] + y * rotation_matrix[1][1]
            rotated_points.append((new_x, new_y))
        return rotated_points

    def bounding_box_based_on_parallel_sides(self, vertices):
        parallel_side_vector = (vertices[1][0] - vertices[0][0], vertices[1][1] - vertices[0][1])
        angle = math.atan2(parallel_side_vector[1], parallel_side_vector[0])
        rotated_rectangle = self.rotate(vertices, -angle)
        
        min_x = min(rotated_rectangle, key=lambda p: p[0])[0]
        max_x = max(rotated_rectangle, key=lambda p: p[0])[0]
        min_y = min(rotated_rectangle, key=lambda p: p[1])[1]
        max_y = max(rotated_rectangle, key=lambda p: p[1])[1]

        bounding_box = [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y)
        ]

        bounding_box = self.rotate(bounding_box, angle)
        return bounding_box

    def Center_find_nearest_side_center_point(self, startpoint, vertices):
        nearest_center_distance = float('inf')
        nearest_rect_center = None
        nearest_rect_side = None

        for i in range(len(vertices)):
            v1, v2 = vertices[i], vertices[(i + 1) % len(vertices)]
            center_of_side = ((v1[0] + v2[0]) / 2, (v1[1] + v2[1]) / 2)
            distance = math.sqrt((startpoint[0] - center_of_side[0]) ** 2 + (startpoint[1] - center_of_side[1]) ** 2)
            if distance < nearest_center_distance:
                nearest_center_distance = distance
                nearest_rect_center = center_of_side
                nearest_rect_side = (v1, v2)
        
        return nearest_rect_side, nearest_rect_center 

    def line_intersection(self, line1, line2, decimals=6):
        def round_point(point, decimals):
            return tuple(round(coord, decimals) for coord in point)

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
        return round_point((x, y), decimals)

    def draw_parallel_lines(self, line_start, line_end, points, bounding_box):
        # fOV, W, l, w1, w2 = self.find_min_fov(target_area=4147.2)  # 간격은 FOV 기반
        fov1 = fov2 = 2.5
        W, l, w1, w2 = self.footprint(fov1, fov2)
        interval = l
        intersection_points = []  # 교차점들을 저장할 리스트

        line_vec = np.array(line_end) - np.array(line_start)  # 녹색선 벡터
        line_vec_norm = line_vec / np.linalg.norm(line_vec)  # 녹색선 단위 벡터
        perpendicular_vec = np.array([-line_vec_norm[1], line_vec_norm[0]])  # 녹색선의 수직인 벡터

        # 평행선의 길이를 bounding box 변의 길이만큼 확장
        line_length1 = math.sqrt((bounding_box[1][0] - bounding_box[0][0]) ** 2 + (bounding_box[1][1] - bounding_box[0][1]) ** 2)
        line_length2 = math.sqrt((bounding_box[2][0] - bounding_box[1][0]) ** 2 + (bounding_box[2][1] - bounding_box[1][1]) ** 2)
        bounding_max_length = max(line_length1, line_length2)

        # Bounding box 내에서만 평행선 그리기
        start_offset = interval / 3
        for offset in np.arange(start_offset, bounding_max_length, interval):
            offset_vec = offset * perpendicular_vec
            new_start = np.array(line_start) + offset_vec - line_vec_norm  # 시작점에서 마진을 둠
            new_end = np.array(line_end) + offset_vec + line_vec_norm  # 끝점에서 마진을 둠

            # 평행선 시작점과 끝점 출력
            # print(f"Parallel line starts at {new_start} and ends at {new_end}")

            # 평행선과 다각형의 변 사이의 교차점 찾기
            current_intersections = []
            for i in range(len(points)):
                edge_start, edge_end = points[i], points[(i + 1) % len(points)]
                intersection = self.line_intersection((new_start, new_end), (edge_start, edge_end))
                if intersection:
                    if min(edge_start[0], edge_end[0]) <= intersection[0] <= max(edge_start[0], edge_end[0]) and \
                    min(edge_start[1], edge_end[1]) <= intersection[1] <= max(edge_start[1], edge_end[1]):
                        current_intersections.append(intersection)
            
            # 평행선의 시작점에서 끝점 순서대로 교차점을 정렬
            current_intersections.sort(key=lambda x: np.linalg.norm(np.array(new_start) - np.array(x)))
            intersection_points.extend(current_intersections)
            # print(f"Current Intersections for line {new_start}-{new_end}: {current_intersections}")
        
        # print("All Intersections: ", intersection_points)
        return intersection_points



    def split_intersections(self, intersection_points):
        even_intersections = intersection_points[::2]
        odd_intersections = intersection_points[1::2]
        return even_intersections, odd_intersections

    def Center_CPP(self, intersection_points, path1, path2):
        path = []
        length = min(len(path1), len(path2))

        for i in range(length):
            if i % 2 == 0:
                path.append(path1[i])
                path.append(path2[i])
            else:
                path.append(path2[i])
                path.append(path1[i])

        return path

    def calculate_path(self):
        nearest_box_side, nearest_box_center = self.Center_find_nearest_side_center_point(self.point, self.bounding_box)
        intersection_points = self.draw_parallel_lines(nearest_box_side[0], nearest_box_side[1], self.rectangle_vertices, self.bounding_box)
        even_intersections, odd_intersection = self.split_intersections(intersection_points)
        path = self.Center_CPP(intersection_points, even_intersections, odd_intersection)
        # print("\npath len : ",len(path)/2)
        # print("\npath : ", path)
        return path

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(*zip(*self.rectangle_vertices, self.rectangle_vertices[0]), 'b-', label='Rectangle')
        ax.plot(*zip(*self.bounding_box, self.bounding_box[0]), 'r--', label='Bounding Box')

        ax.scatter(*zip(*self.rectangle_vertices), color='blue')
        ax.scatter(*zip(*self.bounding_box), color='red')

        # 교차점을 시각화
        nearest_box_side, _ = self.Center_find_nearest_side_center_point(self.point, self.bounding_box)
        intersection_points = self.draw_parallel_lines(nearest_box_side[0], nearest_box_side[1], self.rectangle_vertices, self.bounding_box)
        if intersection_points:
            x_coord, y_coord = zip(*intersection_points)
            ax.scatter(x_coord, y_coord, color='blue', s=30, label='Intersections')

        # 평행선을 시각화
        fov1= fov2 = 2.2
        W, l, w1, w2 = self.footprint(fov1, fov2)  # 간격은 FOV 기반
        interval = l
        line_vec = np.array(nearest_box_side[1]) - np.array(nearest_box_side[0])
        line_vec_norm = line_vec / np.linalg.norm(line_vec)
        perpendicular_vec = np.array([-line_vec_norm[1], line_vec_norm[0]])
        print(interval)

        line_length1 = math.sqrt((self.bounding_box[1][0] - self.bounding_box[0][0]) ** 2 + (self.bounding_box[1][1] - self.bounding_box[0][1]) ** 2)
        line_length2 = math.sqrt((self.bounding_box[2][0] - self.bounding_box[1][0]) ** 2 + (self.bounding_box[2][1] - self.bounding_box[1][1]) ** 2)
        bounding_max_length = max(line_length1, line_length2)
        start_offset = interval / 3

        for offset in np.arange(start_offset, bounding_max_length, interval):
            offset_vec = offset * perpendicular_vec
            new_start = np.array(nearest_box_side[0]) + offset_vec
            new_end = np.array(nearest_box_side[1]) + offset_vec
            ax.plot([new_start[0], new_end[0]], [new_start[1], new_end[1]], color='purple', linestyle='--')

        # 경로를 시각화
        if self.path:
            x_coord, y_coord = zip(*self.path)
            ax.plot(x_coord, y_coord, marker='o', markersize=5, linewidth=2, color='green')

        plt.legend()
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Tilted Rectangle and its Bounding Box Based on Parallel Sides')
        plt.grid(False)
        plt.axis('equal')
        plt.show()



# Usage
# point =[7000, 4500]
# rectangle_vertices = [(6500, 4000),(6500, 6000),(7500, 6000),(7500, 4000)]

# rectangle_path = RectanglePath(point, rectangle_vertices)
# path = rectangle_path.path
# print("Calculated Path: ", path)
# rectangle_path.plot()