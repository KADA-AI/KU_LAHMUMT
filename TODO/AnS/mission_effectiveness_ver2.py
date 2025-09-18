import json
import math

cruise_altitude = 610
velocity = 40
# 상수 설정
TURNING_RADIUS = 3.4  # 선회 반경 (m)
IMAGE_AREA_SIZE = 0.62  # 촬영 영역 크기 (62m x 62m)

from .coord_transform import llh_to_xy, xy_to_llh


"""
개발 필요 내용: 
선형반복주사-BF촬영의 비행 패턴 효과도
직상공순회촬영 임무 효과도  
    
"""


def llhlist_to_xylist(llh_list):
    """
    [{latitude, longitude, Altitude?}, …]  →  [(x,y), …]
    로컬 원점은 리스트 첫 점 사용 (오차 ≤ 수 m)
    """
    lat0, lon0 = llh_list[0]["latitude"], llh_list[0]["longitude"]
    return [llh_to_xy(p["latitude"], p["longitude"], lat0, lon0) for p in llh_list]


# 비행 효과도와 촬영 효과도를 계산하는 함수 수정
def calculate_mission_effectiveness(mission_pattern, current_area):
    """
    개별 임무 패턴에 따른 비행 효과도와 촬영 효과도를 계산합니다.

    :param mission_pattern: 비행 패턴과 촬영 패턴을 포함하는 딕셔너리 (비행 패턴, 촬영 패턴)
    :param current_area: Area data containing 'MissionID', 'coordinateList', and 'meanAltitude'
    :return: 비행 효과도, 촬영 효과도
    """
    # Get the coordinates and mean altitude for the current area
    coordinates = current_area['coordinateList']
    mean_altitude = current_area['meanAltitude']
    altitude_variance = current_area['altitudeVariance']

    # 비행 효과도 계산
    flight_effectiveness = calculate_flight_effectiveness(
        coordinates, mission_pattern['비행 패턴'], mission_pattern['촬영 패턴']
    )
    
    final_flight_effectiveness = 1 - flight_effectiveness

    # 촬영 효과도 계산
    imaging_effectiveness = calculate_imaging_effectiveness(
        coordinates, mission_pattern, mean_altitude, altitude_variance
    )

    return final_flight_effectiveness, imaging_effectiveness


# 비행 효과도 계산 함수
def calculate_flight_effectiveness(coordinates_llh, flight_pattern, imaging_pattern):
    """
    비행 패턴에 따른 비행 효과도를 계산합니다.
    비행 효과도는 선택한 비행 패턴의 경로 길이를 각 영역에서 가능한 최대 비행 경로 길이로 나눈 비율로 정의됩니다.

    :param coordinates: 임무 영역의 좌표 리스트
    :param flight_pattern: 선택된 비행 패턴
    :param imaging_pattern: 촬영 패턴 (필요 시 사용)
    :return: 비행 효과도 (0~1 사이의 값)
    """
    coordinates = llhlist_to_xylist(coordinates_llh)
    # 비행 패턴별 경로 길이 계산
    pattern_distances = {
        "OverFlight": calculate_flight_overflight(coordinates, imaging_pattern),
        "Offset OverFlight": calculate_flight_offset_overflight(coordinates, imaging_pattern),
        "LineLoop": calculate_flight_line_loop(coordinates),
        "SOLS-CentralStandoff": calculate_flight_SOLS_CentralStandoff(coordinates), ##### 선형반복주사-BF촬영의 비행 패턴. 개발필요-250618
        "CentralStandoff": calculate_flight_central_standoff(coordinates),
    }

    # 최대 경로 길이 계산
    max_distance = max(pattern_distances.values())

    # 선택한 비행 패턴의 거리 계산
    selected_distance = pattern_distances[flight_pattern]

    # 비율로 비행 효과도 계산
    flight_effectiveness = selected_distance / max_distance if max_distance > 0 else 0

    return flight_effectiveness


# 비행 거리 계산 함수
def calculate_flight_distance(coordinates_llh, flight_pattern, imaging_pattern=None):
    """
    선택된 비행 패턴에 따른 비행 거리 계산.
    :param coordinates: ROI 좌표 리스트
    :param flight_pattern: 선택된 비행 패턴
    :param imaging_pattern: 선택된 촬영 패턴 (필요 시 사용)
    :return: 비행 거리 (float)
    """
    coordinates = llhlist_to_xylist(coordinates_llh)
    if flight_pattern == "OverFlight":
        return calculate_flight_overflight(coordinates, imaging_pattern)
    elif flight_pattern == "Offset OverFlight":
        return calculate_flight_offset_overflight(coordinates, imaging_pattern)
    elif flight_pattern == "LineLoop":
        return calculate_flight_line_loop(coordinates)
    elif flight_pattern == "SOLS-CentralStandoff": ##### 선형반복주사-BF촬영의 비행 패턴. 개발필요-250618
        return calculate_flight_SOLS_CentralStandoff(coordinates)
    elif flight_pattern == "CentralStandoff":
        return calculate_flight_central_standoff(coordinates)
    else:
        raise ValueError(f"Unknown flight pattern: {flight_pattern}")

def calculate_flight_overflight(coordinates, imaging_pattern):
    """
    OverFlight 패턴의 비행 거리 계산 (Back&Forth 또는 Spiral 패턴).
    """
    imaging_centers = arrange_imaging_areas(coordinates, imaging_pattern)
    total_distance = calculate_total_path_distance(imaging_centers)
    return total_distance

def calculate_flight_offset_overflight(coordinates, imaging_pattern):
    """
    Offset OverFlight 패턴의 비행 거리 계산.
    """
    base_distance = calculate_flight_overflight(coordinates, imaging_pattern)
    return base_distance 

def calculate_flight_line_loop(coordinates):
    """
    LineLoop 패턴의 비행 거리 계산.
    비행 거리 = (선회 거리 * 2) + ROI의 가장 긴 변
    """
    longest_edge = calculate_longest_edge(coordinates)
    turning_circumference = 2 * math.pi * TURNING_RADIUS  # 선회 반경을 기준으로 원 둘레 계산
    flight_distance = (turning_circumference * 2) + longest_edge
    return flight_distance

def calculate_flight_SOLS_CentralStandoff(coordinates):
    """
    SOLS-CentralStandoff 패턴의 비행 거리 계산.
    Back&Forth 주사선 방식으로 촬영 중심점 경로 구성 후, 전체 거리 계산.
    """
    # 1. 촬영 중심점 생성
    imaging_centers = arrange_imaging_centers_sols(coordinates)

    # 2. Back&Forth 주사 경로 구성
    waypoints = []
    num_rows = len(imaging_centers)
    for i, row in enumerate(imaging_centers):
        if i % 2 == 0:
            waypoints.extend(row)
        else:
            waypoints.extend(row[::-1])  # 반대 방향

    # 3. 경로 거리 계산
    return calculate_total_path_distance(waypoints)


def calculate_flight_central_standoff(coordinates):
    """
    CentralStandoff 패턴의 비행 거리 계산.
    비행 거리 = ROI의 가장 긴 변
    """
    return calculate_longest_edge(coordinates) 

# Helper 함수들
def arrange_imaging_areas(coordinates, imaging_pattern):
    """
    ROI 위에 촬영 영역 배치.
    :param coordinates: ROI 좌표
    :param imaging_pattern: 촬영 패턴 (Back&Forth, Spiral 등)
    :return: 촬영 영역 중심 좌표 리스트
    """
    min_x = min(coord[0] for coord in coordinates)
    max_x = max(coord[0] for coord in coordinates)
    min_y = min(coord[1] for coord in coordinates)
    max_y = max(coord[1] for coord in coordinates)

    width = max_x - min_x
    height = max_y - min_y
    num_x = math.ceil(width / IMAGE_AREA_SIZE)
    num_y = math.ceil(height / IMAGE_AREA_SIZE)

    imaging_centers = []

    if imaging_pattern == "Back&Forth":
        for i in range(num_y):
            y = min_y + IMAGE_AREA_SIZE / 2 + i * IMAGE_AREA_SIZE
            row_centers = []
            for j in range(num_x):
                x = min_x + IMAGE_AREA_SIZE / 2 + j * IMAGE_AREA_SIZE
                row_centers.append((x, y))
            if i % 2 == 1:
                row_centers.reverse()  # 행별로 방향 반전
            imaging_centers.extend(row_centers)


    return imaging_centers

def arrange_imaging_centers_sols(coordinates, cell_size=IMAGE_AREA_SIZE):
    """
    ROI 영역 내를 SOLS 방식으로 격자 나누기.
    각 행(row)은 좌→우 촬영 중심점 리스트로 구성.
    return: [[(x1,y1), (x2,y1), ...], [...], ...]
    """
    min_x = min(p[0] for p in coordinates)
    max_x = max(p[0] for p in coordinates)
    min_y = min(p[1] for p in coordinates)
    max_y = max(p[1] for p in coordinates)

    num_x = math.ceil((max_x - min_x) / cell_size)
    num_y = math.ceil((max_y - min_y) / cell_size)

    imaging_grid = []
    for j in range(num_y):
        y = min_y + cell_size / 2 + j * cell_size
        row = []
        for i in range(num_x):
            x = min_x + cell_size / 2 + i * cell_size
            row.append((x, y))
        imaging_grid.append(row)

    return imaging_grid



def calculate_total_path_distance(waypoints):
    """
    경로 거리 계산.
    """
    total_distance = 0
    for i in range(1, len(waypoints)):
        total_distance += math.hypot(waypoints[i][0] - waypoints[i-1][0], waypoints[i][1] - waypoints[i-1][1])
    return total_distance

def calculate_longest_edge(coordinates):
    """
    ROI의 가장 긴 변 계산.
    """
    max_length = 0
    for i in range(len(coordinates)):
        p1 = coordinates[i]
        p2 = coordinates[(i + 1) % len(coordinates)]
        edge_length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if edge_length > max_length:
            max_length = edge_length
    return max_length

def calculate_roi_size_multiplier(coordinates):
    """
    ROI 크기에 따른 배수 계산.
    """
    area = calculate_polygon_area(coordinates)
    base_area = 100000
    multiplier = max(1, area / base_area)
    return multiplier

def calculate_polygon_area(coordinates):
    """
    다각형 면적 계산.
    """
    num_points = len(coordinates)
    area = 0
    for i in range(num_points):
        x1, y1 = coordinates[i]
        x2, y2 = coordinates[(i + 1) % num_points]
        area += (x1 * y2) - (x2 * y1)
    return abs(area) / 2

# 촬영 효과도 계산 함수
def calculate_imaging_effectiveness(coordinates, mission_pattern, mean_altitude, altitude_variance, cruise_altitude=610):
    """
    촬영 패턴 및 비행 패턴에 따른 촬영 효과도를 계산합니다.
    1단계: 2차원 기본 커버리지 비율 계산.
    2단계: 3차원 보정을 적용하여 최종 커버리지 비율 도출.
    
    :param coordinates: 임무 영역의 2D 좌표 (coordinateList)
    :param mission_pattern: 비행 패턴과 촬영 패턴 정보가 포함된 딕셔너리
    :param mean_altitude: 임무 영역의 평균 고도
    :param altitude_variance: 임무 영역의 고도 분산
    :param cruise_altitude: UAV의 순항 고도
    :return: 최종 촬영 효과도
    """
    # 초기값 설정
    flight_pattern = mission_pattern['비행 패턴']
    imaging_pattern = mission_pattern['촬영 패턴']

    base_coverage_ratio = 1.0  # 기본값 설정
    # 촬영 패턴에 따른 기본 커버리지 비율 설정
    if imaging_pattern == "Back&Forth": 
        if flight_pattern == "OverFlight":
            base_coverage_ratio = 1.0  # 100%
        elif flight_pattern == "Offset OverFlight":
            base_coverage_ratio = 0.95  
        elif flight_pattern == "LineLoop":
            base_coverage_ratio = 0.9  
            
    elif imaging_pattern == "LateralSwingSweep":
        if flight_pattern == "CentralStandoff":
            base_coverage_ratio = 0.75  

            
    elif imaging_pattern == "Back&Forth-StepwiseOrthogonalLineSweep":  # 추가
        if flight_pattern == "CentralStandoff":
            base_coverage_ratio = 1.0  # 100%
        elif flight_pattern == "SOLS-CentralStandoff":
            base_coverage_ratio = 1.0  ##### 선형반복주사-BF촬영의 비행 패턴. 개발필요-250618



    # 2단계: 3차원 보정 적용
    # 2-1. 비행 패턴에 따른 고도 보정 (Altitude Correction Factor)
    altitude_correction_factor = 1.0  # 기본값
    if flight_pattern in ["OverFlight"]:
        altitude_correction_factor = 0.97  # 고도 영향 미약
    # elif flight_pattern in ["SOLS-CentralStandoff"]:
    #     altitude_correction_factor = 0.9
    elif flight_pattern in ["LineLoop"]:
        altitude_correction_factor = cruise_altitude / (cruise_altitude + mean_altitude)
    elif flight_pattern in ["CentralStandoff", "Offset OverFlight"]:
        altitude_correction_factor = (0.97 + cruise_altitude / (cruise_altitude + mean_altitude)) / 2

    # 2-2. 고도 분산에 따른 커버리지 감소 보정 (Altitude Variance Factor)
    k = 0.00001  # 경험적 계수
    altitude_variance_factor = max(1 - k * altitude_variance, 0)  # 보정 계수는 0 이상이어야 함

    # 최종 3차원 커버리지 비율 계산
    final_coverage_ratio = (
        base_coverage_ratio
        * altitude_correction_factor
        * altitude_variance_factor
    )

    imaging_effectiveness = final_coverage_ratio

    return imaging_effectiveness



