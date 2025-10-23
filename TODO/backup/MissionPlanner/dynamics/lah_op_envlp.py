"""
LAH Operational Envelope
------------------------
전역/부분 전역 경로 추천 모델에서 사용할 고수준(High-level) 운용 한계 정의.

* 작성: 2025-05-26
* 참고 제원: EC155B1 / LCH / LAH (Miron) 공개 자료
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class LAHOperationalEnvelope:
    """
    LAH 헬기의 주요 운용 제한치(고수준).

    ── 속도 및 기동 ─────────────────────────────────────────────
    - max_speed_kmh : VNE (Never-exceed)
    - cruise_speed_kmh : 일반 임무 순항속도 범위 (min, max)
    - min_speed_kmh : 호버 포함 최소 속도
    - climb_rate_mps / descent_rate_mps : 상승/하강률 한계
    - max_turn_rate_dps : 최대 선회율 (deg/s)
    - pitch_roll_limit_deg : 기체 기울기(Pitch/Roll) 제한

    ── 고도 및 통신 ─────────────────────────────────────────────
    - max_altitude_m : 임무 최대 고도(ASL)
    - min_altitude_agl_m : 지면 기준 최소 안전 고도(AGL)
    - max_los_distance_km : UAV 통신 유지 최대 거리
    - min_los_elevation_deg : LOS 확보 최소 고각

    ── 연료·체공 및 위협 ──────────────────────────────────────
    - endurance_hours : 최대 체공 시간(보조연료 포함)
    - threat_avoid_distance_m : 위협 회피 최소 거리
    - max_threat_exposure_s : 위협 영역 최대 노출 시간
    """

    # ⓐ 속도·기동 한계
    max_speed_kmh: float = 265.0
    cruise_speed_kmh: tuple[float, float] = (240.0, 260.0)
    min_speed_kmh: float = 0.0
    climb_rate_mps: float = 8.9
    descent_rate_mps: float = 7.0
    max_turn_rate_dps: float = 20.0
    pitch_roll_limit_deg: float = 30.0

    # ⓑ 고도·통신 한계
    max_altitude_m: float = 2000.0
    min_altitude_agl_m: float = 50.0
    max_los_distance_km: float = 10.0
    min_los_elevation_deg: float = 5.0

    # ⓒ 연료·위협 관련
    endurance_hours: float = 4.75           # ≒ 4 h 45 m
    threat_avoid_distance_m: float = 500.0
    max_threat_exposure_s: float = 5.0

    # ⓓ 추진·가속 한계  ★★ 신규 ★★
    max_accel_mps2: float = 4.0             # ±4 m/s² = 약 0.4 g
    fuel_capacity_kg: float =  920.0        # JP-8 1 ,080 ℓ ≒ 920 kg
    fuel_flow_hover_kgph   = 150.0   # 예) 호버 300 kg/h
    fuel_flow_cruise_kgph  = 100.0   # 새로운 필드로 추가 (V ≈ 240 km/h)

    # ──────────────────────────────────────────────────────────
    # 검증 메서드(필요 시 사용)
    # ──────────────────────────────────────────────────────────
    def in_speed_limits(self, speed_kmh: float) -> bool:
        return self.min_speed_kmh <= speed_kmh <= self.max_speed_kmh

    def in_altitude_limits(self, altitude_m: float, terrain_elev_m: float = 0.0) -> bool:
        agl = altitude_m - terrain_elev_m
        return (
            agl >= self.min_altitude_agl_m
            and altitude_m <= self.max_altitude_m
        )

    def in_turn_rate_limits(self, turn_rate_dps: float) -> bool:
        return 0.0 <= turn_rate_dps <= self.max_turn_rate_dps

    def safe_from_threat(self, distance_m: float, exposure_time_s: float) -> bool:
        return (
            distance_m >= self.threat_avoid_distance_m
            and exposure_time_s <= self.max_threat_exposure_s
        )


# 편의를 위한 기본 인스턴스
DEFAULT_ENVELOPE: LAHOperationalEnvelope = LAHOperationalEnvelope()
