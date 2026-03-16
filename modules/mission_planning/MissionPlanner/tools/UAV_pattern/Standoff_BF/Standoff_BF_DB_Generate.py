import math
import pandas as pd

class BFDBGenerate:
    def __init__(self):
        pass

    def calculate_horizontal_vertical_fov(self, diagonal_fov_deg):
        aspect_ratio = 16.0 / 9.0
        diagonal_rad = math.radians(diagonal_fov_deg)
        tan_half = math.tan(diagonal_rad / 2)
        denom = math.sqrt(1 + aspect_ratio**2)
        vertical = 2 * math.atan(tan_half / denom)
        horizontal = 2 * math.atan((aspect_ratio * tan_half) / denom)
        return math.degrees(horizontal), math.degrees(vertical)

    def cot(self, angle_deg):
        return 1 / math.tan(math.radians(angle_deg))

    def Footprint(self, fov_v, fov_h, h=610, af=90):
        w1 = (2 * h * math.tan(math.radians(fov_h / 2))) / math.sin(math.radians(af - fov_v / 2))
        w2 = (2 * h * math.tan(math.radians(fov_h / 2))) / math.sin(math.radians(af + fov_v / 2))
        l  = h * (self.cot(af - fov_v / 2) - self.cot(af + fov_v / 2))
        W  = ((w1 + w2) / 2) * l
        return w1, w2, l, W

    def Detction_RSR(self):
        Detection_Pixels = 160 * 90
        Object_size      = 3.5 * 6.5
        Total_resolution = 1920 * 1080
        return Object_size / Detection_Pixels * Total_resolution

    def Identification_RSR(self):
        Identification_Pixels = 54 * 96
        Object_size           = 0.6 * 1.1
        Total_resolution      = 1920 * 1080
        return Object_size / Identification_Pixels * Total_resolution

    def find_valid_fov_af_pairs_with_geometry(self):
        Det_Area = self.Detction_RSR()
        Iden_Area = self.Identification_RSR()
        results = []

        diag_fov = 1.2
        while diag_fov <= 6.1:
            fov_h, fov_v = self.calculate_horizontal_vertical_fov(diag_fov)
            iden_h, iden_v = self.calculate_horizontal_vertical_fov(0.61)

            af = 90
            while af >= 0:
                _, _, l, W = self.Footprint(fov_v, fov_h, h=610, af=af)
                _, _, _, W_iden = self.Footprint(iden_v, iden_h, h=610, af=af)

                if W > Det_Area or W_iden > Iden_Area:
                    break

                af -= 1

            af_valid = af + 1
            if af_valid <= 90:
                hypotenuse = 610 / math.sin(math.radians(af_valid))
                base = math.sqrt(hypotenuse**2 - 610**2)
                results.append({
                    'FOV (deg)': round(diag_fov, 2),
                    'AF (deg)': round(af_valid, 1),
                    'Base Length (m)': round(base, 2),
                    'Hypotenuse (m)': round(hypotenuse, 2),
                    'Footprint Length (m)': round(l, 2)
                })

            diag_fov += 0.1

        return pd.DataFrame(results)
    

if __name__ == "__main__":
    calc = BFDBGenerate()
    df = calc.find_valid_fov_af_pairs_with_geometry()
    print("\n=== 유효 FOV & AF 후보 테이블 ===")
    print(df)






