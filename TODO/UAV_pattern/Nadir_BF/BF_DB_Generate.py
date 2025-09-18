# import math

# def calculate_horizontal_vertical_fov(diagonal_fov_deg):
#     aspect_ratio = 16.0 / 9.0
#     # 라디안 변환
#     diagonal_fov_rad = math.radians(diagonal_fov_deg)
#     tan_half_diag = math.tan(diagonal_fov_rad / 2)
#     denom = math.sqrt(1 + aspect_ratio**2)
    
#     # 수직 화각 (rad)
#     vertical_fov_rad = 2 * math.atan(tan_half_diag / denom)
#     # 수평 화각 (rad)
#     horizontal_fov_rad = 2 * math.atan((aspect_ratio * tan_half_diag) / denom)
    
#     # 도 단위로 변환하여 반환
#     fov_v = math.degrees(vertical_fov_rad)
#     fov_h = math.degrees(horizontal_fov_rad)
#     return fov_h, fov_v

# def cot(angle):
#     angle_rad = math.radians(angle)
#     return 1 / math.tan(angle_rad)

# def Footprint(fov_v, fov_h, af = 90, h = 610):
#     w1 = (2 * h * math.tan(math.radians(fov_h / 2))) / math.sin(math.radians(af - (fov_v / 2)))
#     w2 = (2 * h * math.tan(math.radians(fov_h / 2))) / math.sin(math.radians(af + (fov_v / 2)))
#     l = h * (cot(af - fov_v / 2) - cot(af + fov_v / 2))
#     W = ((w1 + w2) / 2) * l
#     return w1, w2, l, W

# def Detction_RSR():
#     ## 임의값 사용 ##
#     ## 탱크 크기 임의 값, 픽셀 수 임의 값임. ##
#     Detection_Pixels        = 160 * 90 
#     Object_size             = 3.5 * 6.5
#     Total_reolution         = 1920 * 1080
#     Detection_Footprint_Area = Object_size / Detection_Pixels * Total_reolution
#     return Detection_Footprint_Area

# def Identification_RSR():
#     Identification_Pixels   = 54 * 96 
#     Object_size             = 0.6 * 1.1
#     Total_reolution         = 1920 * 1080
#     Identification_Footprint_Area = Object_size / Identification_Pixels * Total_reolution
#     return Identification_Footprint_Area

# def Find_FOV_for_Detection():
#     Detection_Footprint_Area = Detction_RSR()
#     Identification_Footprint_Area = Identification_RSR()

#     print(f"Detection 기준 면적        : {Detection_Footprint_Area:.2f} ㎡")
#     print(f"Identification 기준 면적 : {Identification_Footprint_Area:.2f} ㎡")

#     # Detection용 반복
#     max_fov_under_limit = None
#     max_w1 = None
#     max_w2 = None
#     max_l = None
#     max_W = None

#     diag_fov = 1.2
#     while diag_fov <= 31.2:
#         fov_h, fov_v = calculate_horizontal_vertical_fov(diag_fov)
#         w1, w2, l, W = Footprint(fov_v, fov_h)

#         if W > Detection_Footprint_Area:
#             break

#         # 조건을 만족하는 최대 FOV 저장
#         max_fov_under_limit = diag_fov
#         max_w1 = w1
#         max_w2 = w2
#         max_l = l
#         max_W = W

#         diag_fov += 0.1

#     # Detection 결과 출력 및 Identification 검사
#     if max_fov_under_limit is not None:
#         print(f"\n✅ Detection 조건을 만족하는 최대 대각선 FOV: {max_fov_under_limit:.2f} deg")
#         print(f"Footprint 넓이 W: {max_W:.2f} ㎡")
#         print(f"w1: {max_w1:.3f} m")
#         print(f"w2: {max_w2:.3f} m")
#         print(f"l : {max_l:.3f} m")

#         # Identification 체크 (FOV 고정: 0.61)
#         fixed_iden_fov = 0.61
#         iden_fov_h, iden_fov_v = calculate_horizontal_vertical_fov(fixed_iden_fov)
#         _, _, _, W_iden = Footprint(iden_fov_v, iden_fov_h)

#         if W_iden > Identification_Footprint_Area:
#             print("\n❌ FOV=0.61 : Identification 조건 만족 X")
#         else:
#             print("\n✅ FOV=0.61 : Identification 조건 만족 O")

#     else:
#         print("\n⚠️ Detection 조건 만족 X")
    
# # 실행
# Find_FOV_for_Detection()
import math

class BFDBGenerate:
    """
    Camera footprint and FOV utilities for detection and identification.
    Provides method to compute maximum diagonal FOV under detection limit.
    """
    def __init__(self):
        pass

    def calculate_horizontal_vertical_fov(self, diagonal_fov_deg):
        aspect_ratio = 16.0 / 9.0
        diagonal_rad = math.radians(diagonal_fov_deg)
        tan_half = math.tan(diagonal_rad / 2)
        denom = math.sqrt(1 + aspect_ratio**2)
        vertical = 2 * math.atan(tan_half / denom)
        horizontal = 2 * math.atan((aspect_ratio * tan_half) / denom)
        # return degrees: (fov_h, fov_v)
        return math.degrees(horizontal), math.degrees(vertical)

    def cot(self, angle_deg):
        return 1 / math.tan(math.radians(angle_deg))

    def Footprint(self, fov_v, fov_h, h=610, af=90):
        # footprint half-widths, length, and area
        w1 = (2 * h * math.tan(math.radians(fov_h / 2))) / math.sin(math.radians(af - fov_v / 2))
        w2 = (2 * h * math.tan(math.radians(fov_h / 2))) / math.sin(math.radians(af + fov_v / 2))
        l  = h * (self.cot(af - fov_v / 2) - self.cot(af + fov_v / 2))
        W  = ((w1 + w2) / 2) * l
        return w1, w2, l, W

    def Detction_RSR(self):
        # detection resolution footprint area
        Detection_Pixels = 160 * 90
        Object_size      = 3.5 * 6.5
        Total_resolution = 1920 * 1080
        return Object_size / Detection_Pixels * Total_resolution

    def Identification_RSR(self):
        # identification resolution footprint area
        Identification_Pixels = 54 * 96
        Object_size           = 0.6 * 1.1
        Total_resolution      = 1920 * 1080
        return Object_size / Identification_Pixels * Total_resolution

    def find_max_fov_under_limit(self):
        """
        Computes the maximum diagonal FOV (deg) for which the footprint area W
        does not exceed the detection footprint area limit.
        Returns None if no FOV satisfies the condition.
        """
        Det_Area = self.Detction_RSR()
        # iterate diagonal FOV from 1.2 to 31.2 step 0.1
        max_fov = None
        diag_fov = 1.2
        while diag_fov <= 31.2:
            fov_h, fov_v = self.calculate_horizontal_vertical_fov(diag_fov)
            _, _, _, W = self.Footprint(fov_v, fov_h)
            if W > Det_Area:
                break
            max_fov = diag_fov
            diag_fov += 0.1
            
        max_fov = max_fov - 0.1
        max_fov = math.floor(max_fov * 10) / 10.0
        return max_fov

# Usage example:
calc = BFDBGenerate()
max_fov = calc.find_max_fov_under_limit()
print(max_fov)



