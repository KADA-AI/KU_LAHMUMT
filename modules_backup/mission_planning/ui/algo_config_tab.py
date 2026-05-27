from __future__ import annotations

import json
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict

from PyQt5.QtCore import Qt, QSignalBlocker
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

try:
    from ..MissionPlanner.runtime_settings import (
        ATTACK_WEAPON_TYPE_NAMES,
        DEFAULT_ATTACK_MISSION_VALUES,
        DEFAULT_PRIOR_MISSION_VALUES,
        DEFAULT_RUNTIME_VALUES,
        MANUAL_FOV_DERIVED_KEYS,
        MANUAL_FOV_ROLLBACK_KEY,
        MANUAL_FOV_SYNC_KEYS,
        canonicalize_runtime_payload,
        fov_db_path,
        get_runtime_prior_mission_profile,
        load_fov_db_max_width,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        ATTACK_WEAPON_TYPE_NAMES,
        DEFAULT_ATTACK_MISSION_VALUES,
        DEFAULT_PRIOR_MISSION_VALUES,
        DEFAULT_RUNTIME_VALUES,
        MANUAL_FOV_DERIVED_KEYS,
        MANUAL_FOV_ROLLBACK_KEY,
        MANUAL_FOV_SYNC_KEYS,
        canonicalize_runtime_payload,
        fov_db_path,
        get_runtime_prior_mission_profile,
        load_fov_db_max_width,
    )


@dataclass(frozen=True)
class FieldSpec:
    section: str
    key: str
    label: str
    kind: str = "float"
    minimum: float | int | None = None
    maximum: float | int | None = None
    step: float | int | None = None
    decimals: int = 2
    placeholder: str = ""
    choices: tuple[tuple[str, Any], ...] = ()


WEAPON_TYPE_CHOICES: tuple[tuple[str, Any], ...] = tuple(
    (f"{int(code)} - {str(name)}", int(code))
    for code, name in sorted(ATTACK_WEAPON_TYPE_NAMES.items())
)

CAPTURE_PARAM_MODE_CHOICES: tuple[tuple[str, Any], ...] = (
    ("공통값 사용", "common"),
    ("수동 설정", "manual"),
)

LINE_OVERRIDE_MODE_CHOICES: tuple[tuple[str, Any], ...] = (
    ("공통값 사용", False),
    ("Line 전용값 사용", True),
)

AREA_OVERRIDE_MODE_CHOICES: tuple[tuple[str, Any], ...] = (
    ("공통값 사용", False),
    ("Area 전용값 사용", True),
)

DENSITY_SPEED_WARNING_BASELINE = 1.8


GENERAL_GROUPS: list[tuple[str, str, list[FieldSpec]]] = [
    (
        "Quick Controls",
        "자주 조정하는 전역 카메라/Waypoint 값을 앞쪽에 모았습니다. 카메라 조정은 저장값과 DB 선택은 유지한 채 실제 계획 적용 FOV만 축소합니다.",
        [
            FieldSpec("values", "camera_adjust_enabled", "카메라 조정 사용", "bool"),
            FieldSpec("values", "camera_adjust_percent", "확대/축소 비율(%)", "float", -90.0, 90.0, 1.0, 1),
            FieldSpec("values", "uav_wp_interval_m", "UAV WP 간격(m)", "float", 100.0, 10000.0, 100.0, 1),
            FieldSpec("values", "lah_wp_interval_m", "LAH WP 간격(m)", "float", 100.0, 10000.0, 100.0, 1),
        ],
    ),
    (
        "공통 기본값",
        "기본적으로 전체 임무에 공통 적용되는 값입니다. Line/Area를 따로 조정하지 않으면 이 값을 그대로 사용합니다.",
        [
            FieldSpec("values", "global_manual_fov_deg", "기본 FOV (수동)", "float", 0.1, 120.0, 0.1, 2),
            FieldSpec("values", "post_attack_return_first_fov_scale", "복귀 UAV 첫 촬영 FOV 배수", "float", 0.10, 1.00, 0.05, 2),
            FieldSpec("values", "global_density_scale", "기본 탐색 밀도", "float", 0.2, 10.0, 0.05, 2),
            FieldSpec("values", "global_search_speed_weight", "기본 탐색 속도 가중치", "float", 0.1, 10.0, 0.1, 2),
            FieldSpec("values", "default_sweep_separation_m", "기본 SEP 기준거리(m)", "float", 10.0, 20000.0, 10.0, 1),
            FieldSpec("values", "global_route_offset_scale", "기본 경로 오프셋 배수", "float", 0.0, 2.0, 0.05, 2),
        ],
    ),
    (
        "경로 / 비행",
        "공통 비행/경로 파라미터입니다. 무인기 속도 가중치는 최종 FlightPath 결과 저장 직전에 적용되며 결과 속도는 30~58m/s로 제한됩니다.",
        [
            FieldSpec("values", "uav_wp_interval_m", "UAV WP 간격(m)", "float", 100.0, 10000.0, 100.0, 1),
            FieldSpec("values", "lah_wp_interval_m", "LAH WP 간격(m)", "float", 100.0, 10000.0, 100.0, 1),
            FieldSpec("values", "dubins_turn_radius_m", "공통 Dubins 절대 선회반경(m)", "float", 50.0, 5000.0, 10.0, 1),
            FieldSpec("values", "uav_climb_rate_mps", "UAV 상승률(m/s)", "float", 0.1, 30.0, 0.1, 1),
            FieldSpec("values", "cruise_speed_mps", "UAV 기본 순항 속도(m/s)", "float", 1.0, 100.0, 1.0, 1),
            FieldSpec("values", "uav_speed_weight", "무인기 속도 가중치(DB)", "float", 0.1, 5.0, 0.05, 2),
            FieldSpec("values", "altitude_m", "DEM 기준 AGL 오프셋(m)", "int", 10, 20000, 10),
            FieldSpec("values", "altitude_layer_step_m", "AGL 레이어 간격(m)", "float", 0.0, 1000.0, 1.0, 1),
            FieldSpec("values", "sweep_line_interp_points", "Sweep 점 개수", "int", 2, 15, 1),
            FieldSpec("values", "min_sweep_len_m", "최소 Sweep 길이(m)", "float", 0.0, 1000.0, 1.0, 1),
            FieldSpec("values", "min_route_spacing_m", "최소 경로 간격(m)", "float", 0.0, 5000.0, 10.0, 1),
            FieldSpec("values", "enhanced_area_review_max_segment_m", "Area review 최대 구간(m)", "float", 100.0, 10000.0, 100.0, 1),
            FieldSpec("values", "recon_area_review_max_split_count", "정찰 Area review 최대 분할 수(0=해제)", "int", 0, 50, 1),
            FieldSpec("values", "recon_area_review_min_segment_m", "정찰 Area review 최소 구간(m, 0=해제)", "float", 0.0, 20000.0, 100.0, 1),
            FieldSpec("values", "area_dubins_entry_links_enabled", "Area 전환 Dubins 연결 사용", "bool"),
        ],
    ),
    (
        "Line 전용 덮어쓰기",
        "체크를 켜면 공통 밀도/속도/경로 오프셋 대신 Line 임무 전용값을 씁니다. FOV는 운용 설정의 카메라/FOV 정책을 따릅니다.",
        [
            FieldSpec("values", "line_override_enabled", "Line 전용값 사용", "bool"),
            FieldSpec("values", "line_override_density_scale", "Line 탐색 밀도", "float", 0.2, 10.0, 0.05, 2),
            FieldSpec("values", "line_override_search_speed_weight", "Line 탐색 속도 가중치", "float", 0.1, 10.0, 0.1, 2),
            FieldSpec("values", "line_override_route_offset_scale", "Line 경로 오프셋 배수", "float", 0.0, 2.0, 0.05, 2),
        ],
    ),
    (
        "Area DB / 첫 구간 보정",
        "Area 자동 FOV와 첫 구간 세부 보정입니다. Area 밀도/속도/경로 오프셋 적용 방식은 운용 설정의 촬영 임무 수행 파라미터에서 조정합니다.",
        [
            FieldSpec("values", "area_override_enabled", "Area 전용값 사용", "bool"),
            FieldSpec("values", "area_override_density_scale", "Area 탐색 밀도", "float", 0.2, 10.0, 0.05, 2),
            FieldSpec("values", "area_override_search_speed_weight", "Area 탐색 속도 가중치", "float", 0.1, 10.0, 0.1, 2),
            FieldSpec("values", "area_override_route_offset_scale", "Area 경로 오프셋 배수", "float", 0.0, 2.0, 0.05, 2),
            FieldSpec("values", "area_output_fov_scale", "Area DB 촬영폭 배수(자동 FOV)", "float", 0.1, 10.0, 0.1, 2),
            FieldSpec("values", "area_first_packet_search_speed_scale", "Area 첫 구간 속도 배수", "float", 0.1, 5.0, 0.05, 2),
            FieldSpec("values", "area_first_packet_sweep_group_scale", "Area 첫 구간 스윕 묶음 배수", "float", 0.1, 5.0, 0.05, 2),
        ],
    ),
    (
        "Nadir / Recon Override",
        "Area nadir와 정찰특화는 필요할 때만 전용 FOV/간격을 덮어씁니다. 끄면 공통 또는 Area 값을 따릅니다.",
        [
            FieldSpec("values", "area_nadir_override_enabled", "Area Nadir 사용", "bool"),
            FieldSpec("values", "area_nadir_override_fov_deg", "Area Nadir FOV(deg)", "float", 0.1, 120.0, 0.1, 2),
            FieldSpec("values", "recon_override_enabled", "정찰특화 사용", "bool"),
            FieldSpec("values", "recon_override_split_width_m", "정찰특화 area split width(m)", "float", 50.0, 5000.0, 10.0, 1),
            FieldSpec("values", "recon_override_fixed_fov_deg", "정찰특화 area FOV(deg)", "float", 0.1, 120.0, 0.1, 1),
            FieldSpec("values", "recon_override_sweep_separation_scale", "정찰특화 SEP 배수", "float", 0.05, 1.00, 0.05, 2),
        ],
    ),
    (
        "전문가 / DB",
        "항상 활성인 전문가 설정입니다. DB FOV 반영, 연결 구간 FOV, entry-hold FOV, 기본 탐색 속도 배수를 조정합니다.",
        [
            FieldSpec("values", "db_fov_weight", "DB FOV 반영 배수", "float", 0.1, 5.0, 0.05, 2),
            FieldSpec("values", "point_fov_deg", "Point/연결 FOV(deg)", "float", 0.1, 120.0, 0.1, 2),
            FieldSpec("values", "entry_hold_fov_deg", "Entry Hold FOV(deg)", "float", 0.1, 120.0, 0.1, 2),
            FieldSpec("values", "entry_hold_gimbal_pitch", "Entry Hold Gimbal Pitch", "float", -180.0, 180.0, 1.0, 1),
            FieldSpec("values", "entry_hold_gimbal_yaw", "Entry Hold Gimbal Yaw", "float", -180.0, 180.0, 1.0, 1),
            FieldSpec("values", "loiter_radius_m", "Loiter 반경(m)", "float", 10.0, 5000.0, 10.0, 1),
            FieldSpec("values", "loiter_direction", "Loiter 방향(1/-1)", "int", -1, 1, 1),
            FieldSpec("values", "loiter_time_s", "Loiter 시간(s)", "float", 0.0, 3600.0, 5.0, 1),
            FieldSpec("values", "loiter_speed_mps", "Loiter 속도(m/s)", "float", 1.0, 100.0, 1.0, 1),
            FieldSpec("values", "default_search_speed_multiplier", "기본 탐색 속도 배수", "float", 0.1, 50.0, 0.1, 2),
        ],
    ),
]

_GENERAL_FRONT_KEYS = {"uav_wp_interval_m", "lah_wp_interval_m"}
GENERAL_GROUPS = [
    (
        title,
        note,
        list(specs)
        if title == "Quick Controls"
        else [
            spec
            for spec in specs
            if not (spec.section == "values" and spec.key in _GENERAL_FRONT_KEYS)
        ],
    )
    for title, note, specs in GENERAL_GROUPS
]

NEXT_COLLAB_GROUPS: list[tuple[str, str, list[FieldSpec]]] = [
    (
        "핵심 진입",
        "다음협업기저임무 재계획에서 자주 조정하는 진입 값입니다. 스윕 밀도/탐색 속도와 sweep_points는 운용 설정의 촬영 임무 수행 파라미터를 사용합니다.",
        [
            FieldSpec("values", "next_collab_entry_lead_time_s", "다음 임무 진입 선행시간 (s)", "float", 0.00, 60.00, 0.5, 1),
            FieldSpec(
                "values",
                "next_collab_default_entry_strategy",
                "기본 진입 전략",
                "choice",
                choices=(("0401 current", "current_position"), ("중간점 진입", "midpoint_to_next_start"), ("5초 예측 진입", "turn_projection")),
            ),
            FieldSpec("values", "next_collab_turn_radius_scale", "다음협업 속도기반 선회반경 배수", "float", 0.10, 5.00, 0.05, 2),
            FieldSpec("values", "next_collab_takeover_first_step_ratio", "첫 스윕 추가 촘촘 비율", "float", 0.10, 1.00, 0.05, 2),
        ],
    ),
    (
        "세부 진입 거리",
        "T'와 AREA 첫 시작점 간격을 얼마나 앞당기거나 늦출지 정하는 세부 보정입니다.",
        [
            FieldSpec("values", "next_collab_entry_tprime_target_sep_ratio", "Line/Area T' 진입거리 비율", "float", 0.10, 1.00, 0.05, 2),
            FieldSpec("values", "next_collab_entry_tprime_ratio_scale", "T' 진입거리 미세조정 배수", "float", 0.10, 5.00, 0.05, 2),
            FieldSpec("values", "next_collab_area_path0_trigger_sep_m", "Area 시작 진입 최대 SEP(m)", "float", 0.0, 10000.0, 100.0, 1),
            FieldSpec("values", "next_collab_area_path0_target_sep_ratio", "Area 시작 목표 SEP 비율", "float", 0.05, 1.00, 0.05, 2),
        ],
    ),
    (
        "Line FOV",
        "다음협업기저임무 LINE에서 DB row를 고를 때 width / SEP / FOV를 얼마나 중요하게 볼지 정합니다.",
        [
            FieldSpec("values", "next_collab_line_db_width_weight", "Width 가중치", "float", 0.00, 1.00, 0.05, 2),
            FieldSpec("values", "next_collab_line_db_sep_weight", "SEP 가중치", "float", 0.00, 1.00, 0.05, 2),
            FieldSpec("values", "next_collab_line_db_fov_weight", "FOV 가중치", "float", 0.00, 1.00, 0.05, 2),
        ],
    ),
    (
        "AREA 보정",
        "다음협업기저임무 AREA에서 DB로 고른 FOV와 촬영폭을 추가 배수로 조정합니다. 탐색 속도는 운용 설정의 촬영 임무 수행 파라미터를 사용합니다.",
        [
            FieldSpec("values", "next_collab_area_fov_scale", "Area FOV/촬영폭 배수", "float", 0.10, 5.00, 0.05, 2),
        ],
    ),
    (
        "전문가 / 첫 LINE",
        "다음협업기저임무 첫 LINE 진입 시 첫 waypoint의 FOV를 별도로 키우는 보정입니다.",
        [
            FieldSpec("values", "next_collab_first_line_fov_scale", "첫 LINE FOV 확대 배수", "float", 0.10, 5.00, 0.05, 2),
            FieldSpec("values", "next_collab_first_line_fov_max_deg", "첫 LINE FOV 최대값(deg)", "float", 0.1, 120.0, 0.1, 2),
        ],
    ),
]

PRIOR_GROUPS: list[tuple[str, str, list[FieldSpec]]] = [
    (
        "접근 형상",
        "선행임무 진입 위치와 방향 산출에 직접 들어가는 값입니다.",
        [
            FieldSpec("prior_mission", "approach_base_offset_m", "기본 접근 오프셋(m)", "float", 0.0, 5000.0, 10.0, 1),
            FieldSpec("prior_mission", "approach_far_offset_m", "원거리 접근 오프셋(m)", "float", 0.0, 5000.0, 10.0, 1),
            FieldSpec("prior_mission", "approach_far_trigger_distance_m", "원거리 접근 전환 거리(m)", "float", 0.0, 10000.0, 10.0, 1),
            FieldSpec("prior_mission", "orientation_offset_m", "방향 지정 오프셋(m)", "float", 0.0, 2000.0, 10.0, 1),
        ],
    ),
    (
        "체공 / 추적",
        "목표 감시 체공 시간과 재삽입 체공 시간을 묶었습니다.",
        [
            FieldSpec("prior_mission", "tracking_loiter_seconds", "추적 임무 체공 시간(s)", "int", 0, 3600, 5),
            FieldSpec("prior_mission", "default_loiter_seconds", "일반 선행임무 체공 시간(s)", "int", 0, 3600, 5),
            FieldSpec("prior_mission", "reinsert_loiter_seconds", "재삽입 체공 시간(s)", "int", 0, 3600, 5),
        ],
    ),
    (
        "속도 / 재개",
        "접근 속도, 표적 속도, 재개 탐색 속도 배수를 조정합니다.",
        [
            FieldSpec("prior_mission", "approach_speed_mps", "접근 속도(m/s)", "float", 1.0, 100.0, 1.0, 1),
            FieldSpec("prior_mission", "target_speed_mps", "목표 WP 속도(m/s)", "float", 1.0, 100.0, 1.0, 1),
            FieldSpec("prior_mission", "resume_search_speed_scale", "재개 탐색 속도 배수", "float", 0.1, 5.0, 0.05, 2),
        ],
    ),
]

ATTACK_GROUPS: list[tuple[str, str, list[FieldSpec]]] = [
    (
        "공격기 선택",
        "공격 임무에 투입할 유인기 후보와 fallback 무장을 설정합니다.",
        [
            FieldSpec("attack_mission", "manned_candidate_ids", "공격기 후보 ID", "list", placeholder="2, 3"),
            FieldSpec(
                "attack_mission",
                "weapon_type",
                "Fallback 무기",
                "choice",
                choices=WEAPON_TYPE_CHOICES,
            ),
        ],
    ),
    (
        "표적 우선순위",
        "여러 표적이 동시에 후보일 때 targetType 우선순위를 앞에서부터 적용합니다. 예: 1, 2, 3, 4, 5, 6",
        [
            FieldSpec("attack_mission", "target_type_priority", "표적 우선순위(targetType)", "list", placeholder="1, 2, 3, 4, 5, 6"),
        ],
    ),
    (
        "표적별 무기",
        "표적 type별 기본 weaponType을 설정합니다. 1=전차, 2=장갑차, 3=방사포, 4=곡사포, 5=고정고사포, 6=군인",
        [
            FieldSpec("attack_mission", "weapon_for_target_type_1", "전차 기본 무기", "choice", choices=WEAPON_TYPE_CHOICES),
            FieldSpec("attack_mission", "weapon_for_target_type_2", "장갑차 기본 무기", "choice", choices=WEAPON_TYPE_CHOICES),
            FieldSpec("attack_mission", "weapon_for_target_type_3", "방사포 기본 무기", "choice", choices=WEAPON_TYPE_CHOICES),
            FieldSpec("attack_mission", "weapon_for_target_type_4", "곡사포 기본 무기", "choice", choices=WEAPON_TYPE_CHOICES),
            FieldSpec("attack_mission", "weapon_for_target_type_5", "고정고사포 기본 무기", "choice", choices=WEAPON_TYPE_CHOICES),
            FieldSpec("attack_mission", "weapon_for_target_type_6", "군인 기본 무기", "choice", choices=WEAPON_TYPE_CHOICES),
        ],
    ),
    (
        "진입 / 복귀",
        "공격 진입점, Resume 연결점, 공격점 고도 오프셋을 조정합니다.",
        [
            FieldSpec("attack_mission", "entry_offset_m", "공격 진입 오프셋(m)", "float", 0.0, 5000.0, 10.0, 1),
            FieldSpec("attack_mission", "resume_offset_m", "공격 후 Resume 오프셋(m)", "float", 0.0, 5000.0, 10.0, 1),
            FieldSpec("attack_mission", "collab_replan_lookahead_s", "남은 UAV 재계획 전방예측(s)", "float", 0.0, 30.0, 0.5, 1),
            FieldSpec("attack_mission", "attack_min_standoff_m", "공격 최소 이격거리(m)", "float", 0.0, 5000.0, 10.0, 1),
            FieldSpec("attack_mission", "attack_preferred_standoff_m", "공격 선호 이격거리(m)", "float", 0.0, 10000.0, 10.0, 1),
            FieldSpec("attack_mission", "attack_point_altitude_offset_m", "공격점 고도 오프셋(m)", "float", 0.0, 2000.0, 10.0, 1),
        ],
    ),
    (
        "LAH Hold / Resume",
        "LAH hold 지점과 hold 이후 resume 동작을 조정합니다.",
        [
            FieldSpec("attack_mission", "lah_hold_seconds", "LAH Hold 시간(s)", "int", 0, 3600, 5),
            FieldSpec("attack_mission", "lah_hold_near_resume_offset_m", "Hold 지점 측면 오프셋(m)", "float", 0.0, 5000.0, 10.0, 1),
            FieldSpec("attack_mission", "resume_search_speed_scale", "Resume 탐색 속도 배수", "float", 0.1, 5.0, 0.05, 2),
        ],
    ),
    (
        "공격점 계산",
        "공격점 계산 해상도와 캐시 크기를 조정합니다.",
        [
            FieldSpec("attack_mission", "fast_num_arc_rays", "공격점 계산 ray 수", "int", 180, 1440, 10),
            FieldSpec("attack_mission", "point_cache_max", "공격점 캐시 크기", "int", 1, 256, 1),
        ],
    ),
]

LAH_PATH_MODE_CHOICES: tuple[tuple[str, Any], ...] = (
    ("직선 경로 (기존)", "linear"),
    ("RL 경로계획 (PPO)", "rl"),
)

LAH_GROUPS: list[tuple[str, str, list[FieldSpec]]] = [
    (
        "유인기(LAH) 경로계획",
        "유인기 경로 생성 방식을 선택합니다. RL 모드는 DEM 지형 기반 육각형 그리드에서 PPO 모델로 장애물 회피 경로를 생성합니다.",
        [
            FieldSpec("values", "lah_path_mode", "경로 모드", "choice", choices=LAH_PATH_MODE_CHOICES),
            FieldSpec("values", "lah_rl_hex_step", "RL Hex Step (1칸≈N×30m)", "int", 10, 100, 1),
            FieldSpec("values", "lah_rl_area_km", "RL 영역 크기 (km)", "float", 2.0, 50.0, 1.0, 1),
        ],
    ),
]

OPERATION_GROUPS: list[tuple[str, str, list[FieldSpec]]] = [
    (
        "카메라 / FOV",
        "전체 임무계획 파이프라인에 공통 적용되는 촬영 노브입니다. 수동 FOV 모드에서는 입력 FOV가 Line/Area/Nadir/Recon/Point/Entry Hold에 같이 동기화됩니다.",
        [
            FieldSpec("values", "camera_adjust_enabled", "카메라 확대 보정 사용", "bool"),
            FieldSpec("values", "camera_adjust_percent", "확대/축소 비율(%)", "float", -90.0, 90.0, 1.0, 1),
            FieldSpec("values", "global_manual_fov_deg", "수동 FOV(deg)", "float", 0.1, 120.0, 0.1, 2),
            FieldSpec("values", "default_sweep_separation_m", "수동 SEP 기준거리(m)", "float", 10.0, 20000.0, 10.0, 1),
        ],
    ),
    (
        "WP 간격 / 선회",
        "공통 Dubins 절대 선회반경은 기본/Area/Line/선행 계열에 쓰는 m 단위 반경입니다. 다음협업의 속도기반 배수와는 별도입니다.",
        [
            FieldSpec("values", "uav_wp_interval_m", "UAV WP 간격(m)", "float", 100.0, 10000.0, 100.0, 1),
            FieldSpec("values", "lah_wp_interval_m", "LAH WP 간격(m)", "float", 100.0, 10000.0, 100.0, 1),
            FieldSpec("values", "dubins_turn_radius_m", "공통 Dubins 절대 선회반경(m)", "float", 50.0, 5000.0, 10.0, 1),
        ],
    ),
    (
        "재계획 Sweep 속도 보정",
        "공격/선행/후복귀 재계획에서 새 FlightPath의 첫 번째 lineSearch searchSpeed에 공통으로 곱하는 배수입니다.",
        [
            FieldSpec("values", "replan_sweep_speed_scale", "재계획 sweep 속도 보정 배수", "float", 0.1, 5.0, 0.05, 2),
        ],
    ),
    (
        "촬영 임무 수행 파라미터",
        "FOV 모드와 별도로 Line/Area 촬영 임무의 밀도, 탐색 속도, 경로 오프셋 적용 방식을 고릅니다. 수동 설정일 때만 Line/Area 전용값을 조정합니다.",
        [
            FieldSpec("values", "capture_mission_param_mode", "파라미터 모드", "choice", choices=CAPTURE_PARAM_MODE_CHOICES),
            FieldSpec("values", "global_density_scale", "기본 탐색 밀도", "float", 0.2, 10.0, 0.05, 2),
            FieldSpec("values", "global_search_speed_weight", "기본 탐색 속도 가중치", "float", 0.1, 10.0, 0.1, 2),
            FieldSpec("values", "global_route_offset_scale", "기본 경로 오프셋 배수", "float", 0.0, 2.0, 0.05, 2),
            FieldSpec("values", "line_override_enabled", "Line 적용 방식", "choice", choices=LINE_OVERRIDE_MODE_CHOICES),
            FieldSpec("values", "line_override_density_scale", "Line 탐색 밀도", "float", 0.2, 10.0, 0.05, 2),
            FieldSpec("values", "line_override_search_speed_weight", "Line 탐색 속도 가중치", "float", 0.1, 10.0, 0.1, 2),
            FieldSpec("values", "line_override_route_offset_scale", "Line 경로 오프셋 배수", "float", 0.0, 2.0, 0.05, 2),
            FieldSpec("values", "area_override_enabled", "Area 적용 방식", "choice", choices=AREA_OVERRIDE_MODE_CHOICES),
            FieldSpec("values", "area_override_density_scale", "Area 탐색 밀도", "float", 0.2, 10.0, 0.05, 2),
            FieldSpec("values", "area_override_search_speed_weight", "Area 탐색 속도 가중치", "float", 0.1, 10.0, 0.1, 2),
            FieldSpec("values", "area_override_route_offset_scale", "Area 경로 오프셋 배수", "float", 0.0, 2.0, 0.05, 2),
            FieldSpec("values", "next_collab_auto_sweep_points", "다음협업 Auto sweep_points", "bool"),
            FieldSpec("values", "next_collab_sweep_points_per_leg", "다음협업 스윕 한 줄당 점 개수", "int", 2, 9, 1),
        ],
    ),
    (
        "DEM 고도 / 체공",
        "고도는 절대고도가 아니라 DEM/임무 기준 지면고도에 더하는 AGL 오프셋입니다. 레이어는 오프셋, 오프셋+간격, 오프셋+2*간격으로 적용됩니다.",
        [
            FieldSpec("values", "altitude_m", "DEM 기준 AGL 오프셋(m)", "int", 10, 20000, 10),
            FieldSpec("values", "altitude_layer_step_m", "AGL 레이어 간격(m)", "float", 0.0, 1000.0, 1.0, 1),
            FieldSpec("values", "cruise_speed_mps", "UAV 기본 순항 속도(m/s)", "float", 1.0, 100.0, 1.0, 1),
            FieldSpec("values", "loiter_radius_m", "Loiter 반경(m)", "float", 10.0, 5000.0, 10.0, 1),
            FieldSpec("values", "loiter_time_s", "Loiter 시간(s)", "float", 0.0, 3600.0, 5.0, 1),
            FieldSpec("values", "loiter_speed_mps", "Loiter 속도(m/s)", "float", 1.0, 100.0, 1.0, 1),
        ],
    ),
]


def _unique_field_specs(groups: list[tuple[str, str, list[FieldSpec]]]) -> list[FieldSpec]:
    specs: list[FieldSpec] = []
    seen: set[str] = set()
    for _, _, group_specs in groups:
        for spec in group_specs:
            field_id = f"{spec.section}.{spec.key}"
            if field_id in seen:
                continue
            seen.add(field_id)
            specs.append(spec)
    return specs


OPERATION_FIELD_IDS = {
    f"{spec.section}.{spec.key}"
    for _, _, specs in OPERATION_GROUPS
    for spec in specs
}

ALL_FIELD_SPECS = _unique_field_specs(
    OPERATION_GROUPS + GENERAL_GROUPS + NEXT_COLLAB_GROUPS + PRIOR_GROUPS + ATTACK_GROUPS + LAH_GROUPS
)


class _FocusWheelComboBox(QComboBox):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        if self.hasFocus():
            super().wheelEvent(event)
            return
        event.ignore()


class _FocusWheelSpinBox(QSpinBox):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        if self.hasFocus():
            super().wheelEvent(event)
            return
        event.ignore()


class _FocusWheelDoubleSpinBox(QDoubleSpinBox):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        if self.hasFocus():
            super().wheelEvent(event)
            return
        event.ignore()


class MissionAlgoConfigTab(QWidget):
    def __init__(
        self,
        settings_path: Path,
        on_apply: Callable[[Dict[str, Any]], None] | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("missionAlgoConfigTab")
        self._settings_path = Path(settings_path)
        self._on_apply = on_apply
        self._payload: Dict[str, Any] = {}
        self._building = False
        self._widgets: Dict[str, QWidget] = {}
        self._field_specs: Dict[str, FieldSpec] = {
            self._field_id(spec.section, spec.key): spec for spec in ALL_FIELD_SPECS
        }
        self._area_width_hint: QLabel | None = None
        self._prior_profile_hint: QLabel | None = None
        self._density_speed_warning_label: QLabel | None = None
        self._status_label: QLabel | None = None
        self._path_label: QLabel | None = None
        self._fov_mode: _FocusWheelComboBox | None = None

        self.setStyleSheet(self._build_stylesheet())
        self._build_ui()
        self.reload_from_disk()

    def _build_stylesheet(self) -> str:
        assets_dir = Path(__file__).resolve().parent / "assets"
        spin_up_icon = (assets_dir / "spin_up.svg").as_posix()
        spin_down_icon = (assets_dir / "spin_down.svg").as_posix()

        stylesheet = """
            QWidget#missionAlgoConfigTab { background: #f4f7fb; color: #1b2840; }
            QScrollArea { background: transparent; border: none; }
            QTabWidget#algoConfigTabs::pane { background: transparent; border: none; top: -1px; }
            QTabBar { min-height: 40px; }
            QTabBar::tab { background: #e9f0fa; color: #33445f; border: 1px solid #ced9e8; border-bottom: none; border-top-left-radius: 12px; border-top-right-radius: 12px; min-width: 82px; min-height: 28px; padding: 8px 18px 9px 18px; margin-right: 4px; font-weight: 700; }
            QTabBar::tab:selected { background: #ffffff; color: #12315f; border-color: #c9d7ea; }
            QTabBar::tab:hover:!selected { background: #f5f8fd; }
            QFrame#algoSectionCard { background: #ffffff; border: 1px solid #d8e3ef; border-radius: 18px; }
            QFrame#algoFooterCard { background: #ffffff; border: 1px solid #d8e3ef; border-radius: 16px; }
            QLabel#algoTitle { color: #10203b; font-size: 22px; font-weight: 700; }
            QLabel#algoSubtitle { color: #6c7a8f; font-size: 11px; }
            QLabel#algoBanner { background: #eef4ff; color: #2d4f9b; border: 1px solid #d2defa; border-radius: 10px; padding: 8px 10px; }
            QLabel#algoSectionTitle { color: #10203b; font-size: 16px; font-weight: 700; }
            QLabel#algoSectionNote { color: #66758a; font-size: 11px; }
            QLabel#algoHint { background: #f7faff; color: #49617f; border: 1px solid #dae6f3; border-radius: 10px; padding: 7px 10px; }
            QLabel#algoHint[status="dirty"] { background: #fff1f0; color: #c01818; border-color: #ffb4aa; font-weight: 700; }
            QLabel#densitySpeedWarning { background: #fff1f0; color: #c01818; border: 1px solid #ffb4aa; border-radius: 10px; padding: 8px 10px; font-weight: 700; }
            QGroupBox { background: #fbfdff; border: 1px solid #d9e4ef; border-radius: 14px; margin-top: 14px; font-weight: 700; color: #17325c; padding: 8px 10px 8px 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QLabel#algoGroupNote { color: #62748b; font-size: 11px; }
            QLabel#algoFieldLabel { color: #203047; font-size: 13px; font-weight: 600; }
            QLineEdit { background: #ffffff; color: #172438; border: 1px solid #c8d4e3; border-radius: 11px; min-height: 30px; max-height: 30px; padding: 2px 10px; font-size: 13px; selection-background-color: #2f6df1; }
            QComboBox, QSpinBox, QDoubleSpinBox { background: #ffffff; color: #172438; border: 1px solid #c8d4e3; border-radius: 11px; min-height: 30px; max-height: 30px; padding: 2px 30px 2px 10px; font-size: 13px; selection-background-color: #2f6df1; }
            QLineEdit:hover, QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover { background: #fbfdff; border-color: #afbfd3; }
            QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { background: #ffffff; border-color: #4a7df2; }
            QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled { background: #edf2f8; color: #8ca0b8; border-color: #dce5ef; }
            QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; width: 24px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top-right-radius: 10px; border-bottom-right-radius: 10px; }
            QSpinBox::up-button, QDoubleSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 24px; height: 15px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top-right-radius: 10px; margin: 0px; padding: 0px; }
            QSpinBox::down-button, QDoubleSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 24px; height: 15px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top: 1px solid #d7e2f1; border-bottom-right-radius: 10px; margin: 0px; padding: 0px; }
            QComboBox::drop-down:disabled, QSpinBox::up-button:disabled, QSpinBox::down-button:disabled, QDoubleSpinBox::up-button:disabled, QDoubleSpinBox::down-button:disabled { background: #e3ebf4; border-color: #d4dfeb; }
            QComboBox::down-arrow { image: url("__SPIN_DOWN_ICON__"); width: 10px; height: 10px; }
            QSpinBox::up-arrow, QDoubleSpinBox::up-arrow { image: url("__SPIN_UP_ICON__"); width: 10px; height: 10px; }
            QSpinBox::down-arrow, QDoubleSpinBox::down-arrow { image: url("__SPIN_DOWN_ICON__"); width: 10px; height: 10px; }
            QComboBox::drop-down:hover, QSpinBox::up-button:hover, QSpinBox::down-button:hover, QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover { background: #dfeaff; }
            QCheckBox { color: #203047; font-size: 13px; font-weight: 600; spacing: 8px; }
            QCheckBox:disabled { color: #8ca0b8; }
            QCheckBox::indicator { width: 18px; height: 18px; border: 1px solid #bfd0e4; border-radius: 5px; background: #ffffff; }
            QCheckBox::indicator:checked { background: #2f6df1; border-color: #2f6df1; }
            QPushButton#algoActionButton { background: #2f6df1; color: #ffffff; border: none; border-radius: 12px; padding: 9px 15px; font-weight: 700; }
            QPushButton#algoActionButton:hover { background: #245fe0; }
            QPushButton#algoGhostButton { background: #ffffff; color: #21314a; border: 1px solid #cfd9e7; border-radius: 12px; padding: 9px 15px; font-weight: 700; }
            QPushButton#algoGhostButton:hover { background: #f7faff; }
        """
        return stylesheet.replace("__SPIN_UP_ICON__", spin_up_icon).replace("__SPIN_DOWN_ICON__", spin_down_icon)

    @staticmethod
    def _field_id(section: str, key: str) -> str:
        return f"{section}.{key}"

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        root.addWidget(self._build_header_card(), 0)

        tabs = QTabWidget(self)
        tabs.setObjectName("algoConfigTabs")
        tabs.setElideMode(Qt.ElideNone)
        tabs.setUsesScrollButtons(True)
        tabs.addTab(self._build_tab_page([self._build_operation_section()]), "운용 설정")
        tabs.addTab(self._build_tab_page([self._build_common_detail_section()]), "공통 상세")
        tabs.addTab(self._build_tab_page([self._build_next_collab_section()]), "다음 협업")
        tabs.addTab(self._build_tab_page([self._build_prior_section()]), "선행임무")
        tabs.addTab(self._build_tab_page([self._build_attack_section()]), "공격임무")
        tabs.addTab(self._build_tab_page([self._build_lah_section()]), "LAH")
        tabs.addTab(self._build_tab_page([self._build_flyover_section()]), "출력/Flyover")

        root.addWidget(tabs, 1)
        root.addWidget(self._build_footer_card(), 0)

    def _build_tab_page(self, sections: list[QWidget]) -> QScrollArea:
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(18, 16, 18, 6)
        content_layout.setSpacing(14)
        for section in sections:
            content_layout.addWidget(section)
        content_layout.addStretch(1)
        scroll.setWidget(content)
        return scroll

    def _build_header_card(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("algoSectionCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(8)

        title = QLabel("임무계획 알고리즘 설정")
        title.setObjectName("algoTitle")
        subtitle = QLabel(f"설정 파일: {self._settings_path.name}")
        subtitle.setObjectName("algoSubtitle")
        banner = QLabel(
            "운용 설정 탭은 자주 쓰는 공통 노브만 모았습니다. "
            "세부 조정 탭은 기존 임무별 leaf 파라미터를 유지하므로 저장/적용 경로는 그대로 보존됩니다."
        )
        banner.setObjectName("algoBanner")
        banner.setWordWrap(True)

        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(banner)
        return frame

    def _build_operation_section(self) -> QFrame:
        frame = self._create_section_card(
            "운용 설정",
            "운용자가 평소 조정할 항목입니다. 기존 leaf key를 직접 제어하므로 모든 저장/적용 경로와 호환됩니다.",
        )
        layout = frame.layout()
        assert isinstance(layout, QVBoxLayout)

        operation_groups = {
            title: self._build_group_box(title, note, specs)
            for title, note, specs in OPERATION_GROUPS
        }
        layout.addLayout(
            self._build_column_board(
                [
                    self._build_mode_group(),
                    operation_groups["카메라 / FOV"],
                    operation_groups["WP 간격 / 선회"],
                    operation_groups["재계획 Sweep 속도 보정"],
                ],
                [
                    operation_groups["촬영 임무 수행 파라미터"],
                    operation_groups["DEM 고도 / 체공"],
                ],
            )
        )
        return frame

    def _build_common_detail_section(self) -> QFrame:
        frame = self._create_section_card(
            "공통 상세 조정",
            "Line/Area/Nadir/Recon과 전문가 항목입니다. 운용 설정에 있는 대표 노브는 여기서 중복 노출하지 않습니다.",
        )
        layout = frame.layout()
        assert isinstance(layout, QVBoxLayout)

        widgets: list[QWidget] = []
        for title, note, specs in GENERAL_GROUPS:
            detail_specs = [
                spec
                for spec in specs
                if self._field_id(spec.section, spec.key) not in OPERATION_FIELD_IDS
            ]
            if detail_specs:
                widgets.append(self._build_group_box(title, note, detail_specs))
        layout.addLayout(self._build_column_board(widgets[0::2], widgets[1::2]))
        return frame

    def _build_lah_section(self) -> QFrame:
        frame = self._create_section_card(
            "LAH 상세 조정",
            "LAH WP 간격은 운용 설정에서 관리하고, 여기서는 LAH 경로 생성 방식과 RL 옵션만 조정합니다.",
        )
        layout = frame.layout()
        assert isinstance(layout, QVBoxLayout)
        layout.addLayout(self._build_column_board([self._build_group_box(*LAH_GROUPS[0])], []))
        return frame

    def _build_flyover_section(self) -> QFrame:
        frame = self._create_section_card(
            "출력 / Flyover",
            "WP pass type 후처리 옵션입니다. 생성된 경로 JSON의 waypoint 속성에 반영됩니다.",
        )
        layout = frame.layout()
        assert isinstance(layout, QVBoxLayout)
        layout.addLayout(self._build_column_board([self._build_flyover_group()], []))
        return frame

    def _build_next_collab_section(self) -> QFrame:
        frame = self._create_section_card(
            "다음협업기저임무 재계획",
            "다음협업기저임무 경로를 다시 생성할 때 쓰는 planner 파라미터입니다.",
        )
        layout = frame.layout()
        assert isinstance(layout, QVBoxLayout)

        widgets = {
            title: self._build_group_box(title, note, specs)
            for title, note, specs in NEXT_COLLAB_GROUPS
        }
        layout.addLayout(
            self._build_column_board(
                [
                    widgets["핵심 진입"],
                    widgets["세부 진입 거리"],
                    widgets["AREA 보정"],
                ],
                [
                    widgets["Line FOV"],
                    widgets["전문가 / 첫 LINE"],
                ],
            )
        )
        return frame

    def _build_prior_section(self) -> QFrame:
        frame = self._create_section_card(
            "선행임무",
            "선행임무 진입 위치, 체공, 추적 재개 동작을 관리합니다. FOV는 Dubins 선회반경 기준으로 DB에서 자동 선택됩니다.",
        )
        layout = frame.layout()
        assert isinstance(layout, QVBoxLayout)

        self._prior_profile_hint = QLabel("")
        self._prior_profile_hint.setObjectName("algoHint")
        self._prior_profile_hint.setWordWrap(True)
        layout.addWidget(self._prior_profile_hint)

        widgets = [self._build_group_box(title, note, specs) for title, note, specs in PRIOR_GROUPS]
        layout.addLayout(self._build_column_board(widgets[0::2], widgets[1::2]))

        return frame

    def _build_attack_section(self) -> QFrame:
        frame = self._create_section_card(
            "공격 임무",
            "공격기 선택, 공격 진입/복귀, LAH hold, 공격점 계산 해상도를 조정합니다.",
        )
        layout = frame.layout()
        assert isinstance(layout, QVBoxLayout)

        widgets = [self._build_group_box(title, note, specs) for title, note, specs in ATTACK_GROUPS]
        layout.addLayout(self._build_column_board(widgets[0::2], widgets[1::2]))

        return frame

    def _build_mode_group(self) -> QGroupBox:
        box = QGroupBox("자동 / 수동 FOV")
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(6)

        note = QLabel(
            "자동(DB) 모드에서는 FOV DB row의 FOV/SEP를 기준으로 계산하고, 수동 모드에서는 입력한 FOV와 SEP 기준거리로 라인 간격을 계산합니다."
        )
        note.setObjectName("algoGroupNote")
        note.setWordWrap(True)

        form = QFormLayout()
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(8)
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)

        self._fov_mode = _FocusWheelComboBox()
        self._fov_mode.addItem("자동(DB)", "auto")
        self._fov_mode.addItem("수동", "custom")
        self._fov_mode.currentIndexChanged.connect(self._on_fov_mode_changed)
        self._widgets["fov_mode"] = self._fov_mode
        form.addRow(self._label("FOV 모드"), self._fov_mode)

        self._area_width_hint = QLabel("")
        self._area_width_hint.setObjectName("algoHint")
        self._area_width_hint.setWordWrap(True)

        layout.addWidget(note)
        layout.addLayout(form)
        layout.addWidget(self._area_width_hint)
        return box

    def _build_flyover_group(self) -> QGroupBox:
        box = QGroupBox("Flyover")
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(6)

        note = QLabel("WP pass type 관련 옵션입니다. 체크한 항목만 Over 방식으로 승격합니다.")
        note.setObjectName("algoGroupNote")
        note.setWordWrap(True)
        layout.addWidget(note)

        for key, text in (
            ("flyover.entry_offset", "협업기저임무 시작 점 fly over로 처리"),
            ("flyover.dubins_prefix", "Area 임무 간 Dubins prefix를 Over로 처리"),
            ("flyover.last_point", "각 개별임무 마지막 점 fly over로 처리"),
            ("flyover.all_wps", "모든 WP를 Over로 처리"),
        ):
            checkbox = QCheckBox(text)
            checkbox.stateChanged.connect(self._on_field_changed)
            self._widgets[key] = checkbox
            layout.addWidget(checkbox)
        return box

    def _build_group_box(self, title: str, note: str, specs: list[FieldSpec]) -> QGroupBox:
        box = QGroupBox(title)
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(6)

        note_label = QLabel(note)
        note_label.setObjectName("algoGroupNote")
        note_label.setWordWrap(True)
        layout.addWidget(note_label)

        form = QFormLayout()
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(6)
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        for spec in specs:
            form.addRow(self._label(spec.label), self._create_widget(spec))
        layout.addLayout(form)
        return box

    def _build_footer_card(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("algoFooterCard")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(8)

        btn_reload = QPushButton("불러오기")
        btn_reload.setObjectName("algoGhostButton")
        btn_reload.clicked.connect(self.reload_from_disk)
        btn_save = QPushButton("저장")
        btn_save.setObjectName("algoActionButton")
        btn_save.clicked.connect(self._save_now)
        btn_defaults = QPushButton("기본값")
        btn_defaults.setObjectName("algoGhostButton")
        btn_defaults.clicked.connect(self._load_defaults)

        self._status_label = QLabel("")
        self._status_label.setObjectName("algoHint")
        self._density_speed_warning_label = QLabel("")
        self._density_speed_warning_label.setObjectName("densitySpeedWarning")
        self._density_speed_warning_label.setWordWrap(False)
        self._density_speed_warning_label.setVisible(False)
        self._path_label = QLabel(f"파일: {self._settings_path.name}")
        self._path_label.setObjectName("algoHint")

        layout.addWidget(btn_reload)
        layout.addWidget(btn_save)
        layout.addWidget(btn_defaults)
        layout.addStretch(1)
        layout.addWidget(self._density_speed_warning_label)
        layout.addWidget(self._status_label)
        layout.addWidget(self._path_label)
        return frame

    def _create_section_card(self, title: str, note: str) -> QFrame:
        frame = QFrame()
        frame.setObjectName("algoSectionCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(12)

        head = QHBoxLayout()
        title_label = QLabel(title)
        title_label.setObjectName("algoSectionTitle")
        note_label = QLabel(note)
        note_label.setObjectName("algoSectionNote")
        note_label.setWordWrap(True)
        note_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        head.addWidget(title_label, 1)
        head.addWidget(note_label, 2)
        layout.addLayout(head)
        return frame

    def _label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("algoFieldLabel")
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label.setFixedHeight(30)
        label.setMinimumWidth(136)
        return label

    def _create_widget(self, spec: FieldSpec) -> QWidget:
        widget_id = self._field_id(spec.section, spec.key)
        if spec.kind == "bool":
            widget = QCheckBox()
            widget.stateChanged.connect(self._on_field_changed)
        elif spec.kind == "int":
            widget = _FocusWheelSpinBox()
            widget.setRange(int(spec.minimum or 0), int(spec.maximum or 999999))
            widget.setSingleStep(int(spec.step or 1))
            widget.valueChanged.connect(self._on_field_changed)
        elif spec.kind == "choice":
            widget = _FocusWheelComboBox()
            for label, data in spec.choices or ():
                widget.addItem(str(label), data)
            widget.currentIndexChanged.connect(self._on_field_changed)
        elif spec.kind == "list":
            widget = QLineEdit()
            widget.setPlaceholderText(spec.placeholder)
            widget.textChanged.connect(self._on_field_changed)
        else:
            widget = _FocusWheelDoubleSpinBox()
            widget.setRange(float(spec.minimum or 0.0), float(spec.maximum or 999999.0))
            widget.setSingleStep(float(spec.step or 0.1))
            widget.setDecimals(int(spec.decimals))
            widget.valueChanged.connect(self._on_field_changed)

        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        if isinstance(widget, QCheckBox):
            widget.setFixedHeight(30)
        else:
            widget.setFixedHeight(30)
        self._widgets[widget_id] = widget
        return widget

    @staticmethod
    def _populate_grid(grid: QGridLayout, widgets: list[QWidget], *, columns: int) -> None:
        for index, widget in enumerate(widgets):
            grid.addWidget(widget, index // columns, index % columns, Qt.AlignTop)

    def _build_column_board(self, left_widgets: list[QWidget], right_widgets: list[QWidget]) -> QHBoxLayout:
        board = QHBoxLayout()
        board.setSpacing(12)

        left_col = QVBoxLayout()
        left_col.setSpacing(12)
        for widget in left_widgets:
            left_col.addWidget(widget, 0, Qt.AlignTop)
        left_col.addStretch(1)

        right_col = QVBoxLayout()
        right_col.setSpacing(12)
        for widget in right_widgets:
            right_col.addWidget(widget, 0, Qt.AlignTop)
        right_col.addStretch(1)

        board.addLayout(left_col, 1)
        board.addLayout(right_col, 1)
        return board

    @staticmethod
    def _as_float(value: Any, default: float) -> float:
        try:
            return float(value)
        except Exception:
            return float(default)

    @staticmethod
    def _as_int(value: Any, default: int) -> int:
        try:
            return int(float(value))
        except Exception:
            return int(default)

    @staticmethod
    def _as_int_list(value: Any, default: list[int]) -> list[int]:
        if isinstance(value, str):
            items = value.replace(";", ",").split(",")
        elif isinstance(value, (list, tuple)):
            items = list(value)
        else:
            items = list(default)
        result: list[int] = []
        seen: set[int] = set()
        for item in items:
            try:
                parsed = int(float(str(item).strip()))
            except Exception:
                continue
            if parsed <= 0 or parsed in seen:
                continue
            seen.add(parsed)
            result.append(parsed)
        return result or list(default)

    @staticmethod
    def _format_int_list(values: list[int]) -> str:
        return ", ".join(str(int(value)) for value in values if int(value) > 0)

    @staticmethod
    def _normalize_target_type_priority(value: Any, default: list[int]) -> list[int]:
        items = MissionAlgoConfigTab._as_int_list(value, default)
        result: list[int] = []
        seen: set[int] = set()
        for item in items + list(default):
            if item < 1 or item > 6 or item in seen:
                continue
            seen.add(item)
            result.append(item)
        return result or list(default)

    def _default_payload(self) -> Dict[str, Any]:
        return canonicalize_runtime_payload()

    def _normalize_payload(self, payload: Dict[str, Any] | None) -> Dict[str, Any]:
        base = canonicalize_runtime_payload(deepcopy(payload) if isinstance(payload, dict) else None)

        values = base["values"]
        values["camera_adjust_enabled"] = bool(values.get("camera_adjust_enabled", False))
        values["camera_adjust_percent"] = max(
            -90.0,
            min(90.0, self._as_float(values.get("camera_adjust_percent"), 10.0)),
        )
        values["global_manual_fov_deg"] = max(
            0.1,
            min(120.0, self._as_float(values.get("global_manual_fov_deg"), 2.4)),
        )
        values["global_density_scale"] = max(
            0.2,
            min(10.0, self._as_float(values.get("global_density_scale"), 1.2)),
        )
        values["global_search_speed_weight"] = max(
            0.1,
            min(10.0, self._as_float(values.get("global_search_speed_weight"), 1.0)),
        )
        values["global_route_offset_scale"] = max(
            0.0,
            min(2.0, self._as_float(values.get("global_route_offset_scale"), 1.0)),
        )
        capture_param_mode = str(values.get("capture_mission_param_mode", "common") or "common").strip().lower()
        values["capture_mission_param_mode"] = capture_param_mode if capture_param_mode in {"common", "manual"} else "common"
        values["line_override_enabled"] = bool(values.get("line_override_enabled", False))
        values["area_override_enabled"] = bool(values.get("area_override_enabled", False))
        values["area_nadir_override_enabled"] = bool(values.get("area_nadir_override_enabled", False))
        values["recon_override_enabled"] = bool(values.get("recon_override_enabled", False))
        values["line_override_fov_deg"] = max(
            0.1,
            min(120.0, self._as_float(values.get("line_override_fov_deg"), values["global_manual_fov_deg"])),
        )
        values["area_override_fov_deg"] = max(
            0.1,
            min(120.0, self._as_float(values.get("area_override_fov_deg"), values["global_manual_fov_deg"])),
        )
        values["area_nadir_override_fov_deg"] = max(
            0.1,
            min(120.0, self._as_float(values.get("area_nadir_override_fov_deg"), values["area_override_fov_deg"])),
        )
        values["line_override_density_scale"] = max(
            0.2,
            min(10.0, self._as_float(values.get("line_override_density_scale"), values["global_density_scale"])),
        )
        values["area_override_density_scale"] = max(
            0.2,
            min(10.0, self._as_float(values.get("area_override_density_scale"), values["global_density_scale"])),
        )
        values["line_override_search_speed_weight"] = max(
            0.1,
            min(10.0, self._as_float(values.get("line_override_search_speed_weight"), values["global_search_speed_weight"])),
        )
        values["area_override_search_speed_weight"] = max(
            0.1,
            min(10.0, self._as_float(values.get("area_override_search_speed_weight"), values["global_search_speed_weight"])),
        )
        values["line_override_route_offset_scale"] = max(
            0.0,
            min(2.0, self._as_float(values.get("line_override_route_offset_scale"), values["global_route_offset_scale"])),
        )
        values["area_override_route_offset_scale"] = max(
            0.0,
            min(2.0, self._as_float(values.get("area_override_route_offset_scale"), values["global_route_offset_scale"])),
        )
        values["point_fov_deg"] = max(
            0.1,
            min(120.0, self._as_float(values.get("point_fov_deg"), 31.2)),
        )
        values["entry_hold_fov_deg"] = max(
            0.1,
            min(120.0, self._as_float(values.get("entry_hold_fov_deg"), 10.0)),
        )
        values["area_output_fov_scale"] = self._as_float(values.get("area_output_fov_scale"), 1.0)
        values["recon_override_split_width_m"] = max(
            50.0,
            min(5000.0, self._as_float(values.get("recon_override_split_width_m"), 600.0)),
        )
        values["recon_override_fixed_fov_deg"] = max(
            0.1,
            min(120.0, self._as_float(values.get("recon_override_fixed_fov_deg"), values["area_override_fov_deg"])),
        )
        values["recon_override_sweep_separation_scale"] = max(
            0.05,
            min(1.0, self._as_float(values.get("recon_override_sweep_separation_scale"), 0.50)),
        )
        values["default_sweep_separation_m"] = max(
            10.0,
            min(20000.0, self._as_float(values.get("default_sweep_separation_m"), 1000.0)),
        )
        values["db_fov_weight"] = max(
            0.1,
            min(5.0, self._as_float(values.get("db_fov_weight"), 1.0)),
        )
        values["uav_wp_interval_m"] = self._as_float(values.get("uav_wp_interval_m"), 1200.0)
        values["lah_wp_interval_m"] = self._as_float(values.get("lah_wp_interval_m"), 3000.0)
        values["dubins_turn_radius_m"] = self._as_float(values.get("dubins_turn_radius_m"), 500.0)
        values["cruise_speed_mps"] = self._as_float(values.get("cruise_speed_mps"), 40.0)
        values["turn_step_deg"] = self._as_float(values.get("turn_step_deg"), 15.0)
        values["altitude_m"] = max(10, min(20000, self._as_int(values.get("altitude_m"), 1000)))
        values["altitude_layer_step_m"] = max(
            0.0,
            min(1000.0, self._as_float(values.get("altitude_layer_step_m"), 10.0)),
        )
        values["sweep_merge_heading_deg"] = max(
            0.0,
            min(90.0, self._as_float(values.get("sweep_merge_heading_deg"), 5.0)),
        )
        values["min_sweep_len_m"] = max(0.0, min(1000.0, self._as_float(values.get("min_sweep_len_m"), 3.0)))
        values["min_route_spacing_m"] = max(
            0.0,
            min(5000.0, self._as_float(values.get("min_route_spacing_m"), 200.0)),
        )
        values["default_search_speed_multiplier"] = max(
            0.1,
            min(50.0, self._as_float(values.get("default_search_speed_multiplier"), 16.0)),
        )
        values["sweep_line_interp_points"] = max(2, self._as_int(values.get("sweep_line_interp_points"), 3))
        values["enhanced_area_review_max_segment_m"] = self._as_float(values.get("enhanced_area_review_max_segment_m"), 300.0)
        values["recon_area_review_max_split_count"] = max(
            0,
            min(50, self._as_int(values.get("recon_area_review_max_split_count"), 0)),
        )
        values["recon_area_review_min_segment_m"] = max(
            0.0,
            min(20000.0, self._as_float(values.get("recon_area_review_min_segment_m"), 0.0)),
        )
        values["enhanced_auto_fov_from_db"] = bool(values.get("enhanced_auto_fov_from_db", True))
        values["manual_fov_global_sync_enabled"] = True
        values["area_dubins_entry_links_enabled"] = bool(values.get("area_dubins_entry_links_enabled", True))
        values["entry_hold_gimbal_pitch"] = max(
            -180.0,
            min(180.0, self._as_float(values.get("entry_hold_gimbal_pitch"), -90.0)),
        )
        values["entry_hold_gimbal_yaw"] = max(
            -180.0,
            min(180.0, self._as_float(values.get("entry_hold_gimbal_yaw"), 0.0)),
        )
        values["loiter_radius_m"] = max(10.0, min(5000.0, self._as_float(values.get("loiter_radius_m"), 800.0)))
        loiter_direction = self._as_int(values.get("loiter_direction"), 1)
        values["loiter_direction"] = -1 if loiter_direction < 0 else 1
        values["loiter_time_s"] = max(0.0, min(3600.0, self._as_float(values.get("loiter_time_s"), 30.0)))
        values["loiter_speed_mps"] = max(1.0, min(100.0, self._as_float(values.get("loiter_speed_mps"), 30.0)))
        values["next_collab_sweep_step_ratio"] = max(0.05, min(1.0, self._as_float(values.get("next_collab_sweep_step_ratio"), 0.60)))
        values["next_collab_entry_tprime_target_sep_ratio"] = max(
            0.05,
            min(1.0, self._as_float(values.get("next_collab_entry_tprime_target_sep_ratio"), 0.30)),
        )
        values["next_collab_entry_tprime_ratio_scale"] = max(
            0.10,
            min(5.0, self._as_float(values.get("next_collab_entry_tprime_ratio_scale"), 0.50)),
        )
        values["next_collab_area_path0_trigger_sep_m"] = max(
            0.0,
            min(10000.0, self._as_float(values.get("next_collab_area_path0_trigger_sep_m"), 3000.0)),
        )
        values["next_collab_area_path0_target_sep_ratio"] = max(
            0.05,
            min(1.0, self._as_float(values.get("next_collab_area_path0_target_sep_ratio"), 0.20)),
        )
        values["next_collab_turn_radius_scale"] = max(
            0.10,
            min(5.0, self._as_float(values.get("next_collab_turn_radius_scale"), 1.40)),
        )
        values["next_collab_entry_lead_time_s"] = max(
            0.0,
            min(60.0, self._as_float(values.get("next_collab_entry_lead_time_s"), 5.0)),
        )
        next_collab_entry_strategy = str(values.get("next_collab_default_entry_strategy", "turn_projection") or "turn_projection").strip().lower()
        if next_collab_entry_strategy not in {"current_position", "midpoint_to_next_start", "turn_projection"}:
            next_collab_entry_strategy = "turn_projection"
        values["next_collab_default_entry_strategy"] = next_collab_entry_strategy
        values["next_collab_takeover_first_step_ratio"] = max(
            0.10,
            min(1.0, self._as_float(values.get("next_collab_takeover_first_step_ratio"), 0.40)),
        )
        values["next_collab_auto_sweep_points"] = bool(values.get("next_collab_auto_sweep_points", False))
        values["next_collab_area_fov_scale"] = max(
            0.10,
            min(5.0, self._as_float(values.get("next_collab_area_fov_scale"), 1.00)),
        )
        values["next_collab_area_search_speed_scale"] = max(
            0.10,
            min(5.0, self._as_float(values.get("next_collab_area_search_speed_scale"), 1.00)),
        )
        values["next_collab_line_db_width_weight"] = max(
            0.0,
            min(1.0, self._as_float(values.get("next_collab_line_db_width_weight"), 0.30)),
        )
        values["next_collab_line_db_sep_weight"] = max(
            0.0,
            min(1.0, self._as_float(values.get("next_collab_line_db_sep_weight"), 0.25)),
        )
        values["next_collab_line_db_fov_weight"] = max(
            0.0,
            min(1.0, self._as_float(values.get("next_collab_line_db_fov_weight"), 0.45)),
        )
        values["next_collab_sweep_points_per_leg"] = max(
            2,
            min(9, self._as_int(values.get("next_collab_sweep_points_per_leg"), 3)),
        )
        values["next_collab_first_line_fov_scale"] = max(
            0.10,
            min(5.0, self._as_float(values.get("next_collab_first_line_fov_scale"), 1.35)),
        )
        values["next_collab_first_line_fov_max_deg"] = max(
            0.1,
            min(120.0, self._as_float(values.get("next_collab_first_line_fov_max_deg"), 15.4)),
        )
        values["replan_sweep_speed_scale"] = max(
            0.1,
            min(5.0, self._as_float(values.get("replan_sweep_speed_scale"), 1.3)),
        )
        values["post_attack_return_first_fov_scale"] = max(
            0.10,
            min(1.00, self._as_float(values.get("post_attack_return_first_fov_scale"), 0.70)),
        )

        # LAH 경로계획 설정
        lah_mode = values.get("lah_path_mode", "linear")
        values["lah_path_mode"] = str(lah_mode) if lah_mode in ("linear", "rl") else "linear"
        values["lah_rl_hex_step"] = max(10, min(100, self._as_int(values.get("lah_rl_hex_step"), 50)))
        values["lah_rl_area_km"] = max(2.0, min(50.0, self._as_float(values.get("lah_rl_area_km"), 10.0)))

        prior = base["prior_mission"]
        for key, default in DEFAULT_PRIOR_MISSION_VALUES.items():
            prior[key] = self._as_int(prior.get(key), int(default)) if isinstance(default, int) else self._as_float(prior.get(key), float(default))

        attack = base["attack_mission"]
        attack["manned_candidate_ids"] = self._as_int_list(
            attack.get("manned_candidate_ids"),
            list(DEFAULT_ATTACK_MISSION_VALUES["manned_candidate_ids"]),
        )
        attack["target_type_priority"] = self._normalize_target_type_priority(
            attack.get("target_type_priority"),
            list(DEFAULT_ATTACK_MISSION_VALUES["target_type_priority"]),
        )
        for key, default in DEFAULT_ATTACK_MISSION_VALUES.items():
            if key in ("manned_candidate_ids", "target_type_priority"):
                continue
            attack[key] = self._as_int(attack.get(key), int(default)) if isinstance(default, int) else self._as_float(attack.get(key), float(default))
        attack["weapon_type"] = max(0, min(3, self._as_int(attack.get("weapon_type"), int(DEFAULT_ATTACK_MISSION_VALUES["weapon_type"]))))
        for target_type in range(1, 7):
            key = f"weapon_for_target_type_{target_type}"
            attack[key] = max(0, min(3, self._as_int(attack.get(key), int(DEFAULT_ATTACK_MISSION_VALUES[key]))))

        flyover = base["flyover"]
        for key in ("entry_offset", "dubins_prefix", "last_point", "all_wps"):
            flyover[key] = bool(flyover.get(key, False))
        return canonicalize_runtime_payload(base)

    def _write_widget_value(self, spec: FieldSpec, widget: QWidget, value: Any) -> None:
        if spec.kind == "bool":
            assert isinstance(widget, QCheckBox)
            with QSignalBlocker(widget):
                widget.setChecked(bool(value))
            return
        if spec.kind == "int":
            assert isinstance(widget, QSpinBox)
            default = int(
                DEFAULT_RUNTIME_VALUES.get(
                    spec.key,
                    DEFAULT_PRIOR_MISSION_VALUES.get(spec.key, DEFAULT_ATTACK_MISSION_VALUES.get(spec.key, 0)),
                )
            )
            with QSignalBlocker(widget):
                widget.setValue(self._as_int(value, default))
            return
        if spec.kind == "choice":
            assert isinstance(widget, QComboBox)
            default_value = DEFAULT_RUNTIME_VALUES.get(
                spec.key,
                DEFAULT_ATTACK_MISSION_VALUES.get(spec.key, spec.choices[0][1] if spec.choices else 0),
            )
            target_value = value if value is not None else default_value
            target_index = 0
            for index in range(widget.count()):
                if widget.itemData(index) == target_value:
                    target_index = index
                    break
            with QSignalBlocker(widget):
                widget.setCurrentIndex(target_index)
            return
        if spec.kind == "list":
            assert isinstance(widget, QLineEdit)
            default_list = list(DEFAULT_ATTACK_MISSION_VALUES.get(spec.key, DEFAULT_ATTACK_MISSION_VALUES["manned_candidate_ids"]))
            normalized = self._normalize_target_type_priority(value, default_list) if spec.key == "target_type_priority" else self._as_int_list(value, default_list)
            with QSignalBlocker(widget):
                widget.setText(self._format_int_list(normalized))
            return
        assert isinstance(widget, QDoubleSpinBox)
        default = float(DEFAULT_RUNTIME_VALUES.get(spec.key, DEFAULT_PRIOR_MISSION_VALUES.get(spec.key, DEFAULT_ATTACK_MISSION_VALUES.get(spec.key, 0.0))))
        with QSignalBlocker(widget):
            widget.setValue(self._as_float(value, default))

    def _read_widget_value(self, spec: FieldSpec, widget: QWidget) -> Any:
        if spec.kind == "bool":
            assert isinstance(widget, QCheckBox)
            return bool(widget.isChecked())
        if spec.kind == "int":
            assert isinstance(widget, QSpinBox)
            return int(widget.value())
        if spec.kind == "choice":
            assert isinstance(widget, QComboBox)
            raw = widget.currentData()
            if isinstance(raw, bool):
                return raw
            if isinstance(raw, (int, float)):
                return int(raw)
            return raw
        if spec.kind == "list":
            assert isinstance(widget, QLineEdit)
            default_list = list(DEFAULT_ATTACK_MISSION_VALUES.get(spec.key, DEFAULT_ATTACK_MISSION_VALUES["manned_candidate_ids"]))
            if spec.key == "target_type_priority":
                return self._normalize_target_type_priority(widget.text(), default_list)
            return self._as_int_list(widget.text(), default_list)
        assert isinstance(widget, QDoubleSpinBox)
        return float(widget.value())

    def _is_manual_fov_mode(self) -> bool:
        return bool(self._fov_mode is not None and self._fov_mode.currentData() == "custom")

    def _capture_manual_fov_snapshot(self) -> None:
        values = self._payload.setdefault("values", {})
        snapshot = values.get(MANUAL_FOV_ROLLBACK_KEY)
        if isinstance(snapshot, dict) and snapshot:
            return
        captured: Dict[str, Any] = {}
        for key in tuple(MANUAL_FOV_SYNC_KEYS) + tuple(MANUAL_FOV_DERIVED_KEYS):
            widget_id = self._field_id("values", key)
            widget = self._widgets.get(widget_id)
            spec = self._field_specs.get(widget_id)
            if widget is not None and spec is not None:
                captured[key] = self._read_widget_value(spec, widget)
            elif key in values:
                captured[key] = deepcopy(values.get(key))
        values[MANUAL_FOV_ROLLBACK_KEY] = captured

    def _write_value_widget_by_key(self, key: str, value: Any) -> None:
        widget_id = self._field_id("values", key)
        widget = self._widgets.get(widget_id)
        spec = self._field_specs.get(widget_id)
        if widget is None or spec is None:
            return
        self._write_widget_value(spec, widget, value)

    def _sync_manual_fov_widgets_to_global(self) -> None:
        if not self._is_manual_fov_mode():
            return
        self._capture_manual_fov_snapshot()
        global_widget_id = self._field_id("values", "global_manual_fov_deg")
        global_widget = self._widgets.get(global_widget_id)
        global_spec = self._field_specs.get(global_widget_id)
        if global_widget is None or global_spec is None:
            return
        manual_fov = self._read_widget_value(global_spec, global_widget)
        for key in MANUAL_FOV_SYNC_KEYS:
            self._write_value_widget_by_key(key, manual_fov)

    def _restore_manual_fov_widgets_from_snapshot(self) -> None:
        values = self._payload.setdefault("values", {})
        snapshot = values.get(MANUAL_FOV_ROLLBACK_KEY)
        if not isinstance(snapshot, dict) or not snapshot:
            return
        for key in MANUAL_FOV_SYNC_KEYS:
            if key in snapshot:
                self._write_value_widget_by_key(key, snapshot[key])
        values[MANUAL_FOV_ROLLBACK_KEY] = {}

    def _apply_payload_to_widgets(self, payload: Dict[str, Any]) -> None:
        self._building = True
        try:
            values = payload.get("values") if isinstance(payload.get("values"), dict) else {}
            prior = payload.get("prior_mission") if isinstance(payload.get("prior_mission"), dict) else {}
            attack = payload.get("attack_mission") if isinstance(payload.get("attack_mission"), dict) else {}
            flyover = payload.get("flyover") if isinstance(payload.get("flyover"), dict) else {}

            if self._fov_mode is not None:
                self._fov_mode.setCurrentIndex(0 if bool(values.get("enhanced_auto_fov_from_db", True)) else 1)

            for spec in ALL_FIELD_SPECS:
                widget = self._widgets[self._field_id(spec.section, spec.key)]
                source = values if spec.section == "values" else prior if spec.section == "prior_mission" else attack
                self._write_widget_value(spec, widget, source.get(spec.key))

            for key in ("entry_offset", "dubins_prefix", "last_point", "all_wps"):
                widget = self._widgets[f"flyover.{key}"]
                assert isinstance(widget, QCheckBox)
                with QSignalBlocker(widget):
                    widget.setChecked(bool(flyover.get(key, False)))
        finally:
            self._building = False
        self._sync_enabled_state(payload)

    def _current_payload(self) -> Dict[str, Any]:
        payload = deepcopy(self._payload if isinstance(self._payload, dict) else self._default_payload())
        values = payload.setdefault("values", {})
        prior = payload.setdefault("prior_mission", {})
        attack = payload.setdefault("attack_mission", {})
        flyover = payload.setdefault("flyover", {})

        for spec in ALL_FIELD_SPECS:
            widget = self._widgets[self._field_id(spec.section, spec.key)]
            target = values if spec.section == "values" else prior if spec.section == "prior_mission" else attack
            target[spec.key] = self._read_widget_value(spec, widget)

        values["enhanced_auto_fov_from_db"] = bool(self._fov_mode.currentData() == "auto") if self._fov_mode is not None else True
        values["manual_fov_global_sync_enabled"] = True
        if not bool(values["enhanced_auto_fov_from_db"]):
            values["camera_adjust_enabled"] = False
        for key in ("entry_offset", "dubins_prefix", "last_point", "all_wps"):
            widget = self._widgets[f"flyover.{key}"]
            assert isinstance(widget, QCheckBox)
            flyover[key] = bool(widget.isChecked())
        return self._normalize_payload(payload)

    def reload_from_disk(self) -> None:
        try:
            payload = json.loads(self._settings_path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
        self._payload = self._normalize_payload(payload)
        self._apply_payload_to_widgets(self._payload)
        self._set_status("현재 값을 불러왔습니다.")

    def _save_now(self) -> None:
        payload = self._current_payload()
        self._settings_path.parent.mkdir(parents=True, exist_ok=True)
        self._settings_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self._payload = payload
        self._set_status("저장했습니다.")
        if callable(self._on_apply):
            self._on_apply(deepcopy(payload))

    def _load_defaults(self) -> None:
        self._payload = self._normalize_payload(self._default_payload())
        self._apply_payload_to_widgets(self._payload)
        self._set_status("기본값을 불러왔습니다.")

    def _set_status(self, text: str) -> None:
        if self._status_label is not None:
            self._status_label.setText(text)
            self._status_label.setProperty("status", "dirty" if str(text).strip() == "수정 중" else "normal")
            style = self._status_label.style()
            style.unpolish(self._status_label)
            style.polish(self._status_label)
            self._status_label.update()

    def _on_fov_mode_changed(self) -> None:
        if self._building:
            return
        if self._is_manual_fov_mode():
            self._sync_manual_fov_widgets_to_global()
        else:
            self._restore_manual_fov_widgets_from_snapshot()
        self._sync_enabled_state()
        self._set_status("수정 중")

    def _on_field_changed(self) -> None:
        if self._building:
            return
        if self._is_manual_fov_mode():
            self._sync_manual_fov_widgets_to_global()
        self._sync_enabled_state()
        self._set_status("수정 중")

    def _sync_enabled_state(self, payload: Dict[str, Any] | None = None) -> None:
        custom_enabled = bool(self._fov_mode is not None and self._fov_mode.currentData() == "custom")

        def _bool_control_value(key: str) -> bool:
            widget = self._widgets.get(key)
            if isinstance(widget, QCheckBox):
                return bool(widget.isChecked())
            if isinstance(widget, QComboBox):
                return bool(widget.currentData())
            return False

        camera_adjust_enabled = bool(
            getattr(self._widgets.get("values.camera_adjust_enabled"), "isChecked", lambda: False)()
        )
        capture_mode_widget = self._widgets.get("values.capture_mission_param_mode")
        capture_param_manual = bool(
            isinstance(capture_mode_widget, QComboBox)
            and str(capture_mode_widget.currentData()) == "manual"
        )
        line_override_enabled = _bool_control_value("values.line_override_enabled")
        area_override_enabled = _bool_control_value("values.area_override_enabled")
        area_nadir_override_enabled = bool(
            getattr(self._widgets.get("values.area_nadir_override_enabled"), "isChecked", lambda: False)()
        )
        recon_override_enabled = bool(getattr(self._widgets.get("values.recon_override_enabled"), "isChecked", lambda: False)())

        camera_adjust_widget = self._widgets.get("values.camera_adjust_enabled")
        if isinstance(camera_adjust_widget, QCheckBox):
            if custom_enabled and camera_adjust_widget.isChecked():
                with QSignalBlocker(camera_adjust_widget):
                    camera_adjust_widget.setChecked(False)
                camera_adjust_enabled = False
            camera_adjust_widget.setEnabled(not custom_enabled)

        widget = self._widgets.get("values.camera_adjust_percent")
        if widget is not None:
            widget.setEnabled((not custom_enabled) and camera_adjust_enabled)

        for key in ("values.global_manual_fov_deg", "values.default_sweep_separation_m"):
            widget = self._widgets.get(key)
            if widget is not None:
                widget.setEnabled(custom_enabled)
        for key in ("values.point_fov_deg", "values.entry_hold_fov_deg"):
            widget = self._widgets.get(key)
            if widget is not None:
                widget.setEnabled(custom_enabled)

        for key in (
            "values.global_density_scale",
            "values.global_search_speed_weight",
            "values.global_route_offset_scale",
        ):
            widget = self._widgets.get(key)
            if widget is not None:
                widget.setEnabled(not capture_param_manual)

        next_collab_auto_sweep_points = bool(
            getattr(self._widgets.get("values.next_collab_auto_sweep_points"), "isChecked", lambda: False)()
        )
        widget = self._widgets.get("values.next_collab_sweep_points_per_leg")
        if widget is not None:
            widget.setEnabled(not next_collab_auto_sweep_points)

        for key in ("values.line_override_enabled", "values.area_override_enabled"):
            widget = self._widgets.get(key)
            if widget is not None:
                widget.setEnabled(capture_param_manual)

        for key in (
            "values.line_override_density_scale",
            "values.line_override_search_speed_weight",
            "values.line_override_route_offset_scale",
        ):
            widget = self._widgets.get(key)
            if widget is not None:
                widget.setEnabled(capture_param_manual and line_override_enabled)
        widget = self._widgets.get("values.line_override_fov_deg")
        if widget is not None:
            widget.setEnabled(custom_enabled and line_override_enabled)

        for key in (
            "values.area_override_density_scale",
            "values.area_override_search_speed_weight",
            "values.area_override_route_offset_scale",
            "values.area_first_packet_search_speed_scale",
            "values.area_first_packet_sweep_group_scale",
        ):
            widget = self._widgets.get(key)
            if widget is not None:
                if key.startswith("values.area_override_"):
                    widget.setEnabled(capture_param_manual and area_override_enabled)
                else:
                    widget.setEnabled(True)
        widget = self._widgets.get("values.area_output_fov_scale")
        if widget is not None:
            widget.setEnabled(not custom_enabled)
        widget = self._widgets.get("values.area_override_fov_deg")
        if widget is not None:
            widget.setEnabled(custom_enabled and area_override_enabled)

        widget = self._widgets.get("values.area_nadir_override_fov_deg")
        if widget is not None:
            widget.setEnabled(custom_enabled and area_nadir_override_enabled)

        for key in (
            "values.recon_override_split_width_m",
            "values.recon_override_sweep_separation_scale",
        ):
            widget = self._widgets.get(key)
            if widget is not None:
                widget.setEnabled(recon_override_enabled)
        widget = self._widgets.get("values.recon_override_fixed_fov_deg")
        if widget is not None:
            widget.setEnabled(custom_enabled and recon_override_enabled)

        for key in ("values.db_fov_weight",):
            widget = self._widgets.get(key)
            if widget is not None:
                widget.setEnabled(not custom_enabled)
        self._update_density_speed_warning(payload)
        self._update_area_width_hint(custom_enabled)
        self._update_prior_profile_hint(payload)

    def _update_density_speed_warning(self, payload: Dict[str, Any] | None = None) -> None:
        label = self._density_speed_warning_label
        if label is None:
            return
        try:
            source = payload if isinstance(payload, dict) else self._current_payload()
            values = source.get("values") if isinstance(source.get("values"), dict) else {}
            line_density = self._as_float(values.get("line_density_scale"), DENSITY_SPEED_WARNING_BASELINE)
            area_density = self._as_float(values.get("area_density_scale"), DENSITY_SPEED_WARNING_BASELINE)
        except Exception:
            label.setVisible(False)
            return

        baseline = float(DENSITY_SPEED_WARNING_BASELINE)
        changed = abs(line_density - baseline) > 1e-6 or abs(area_density - baseline) > 1e-6
        if not changed:
            label.setVisible(False)
            return

        label.setText(
            f"탐색 밀도가 기준값 {baseline:.1f}와 다릅니다 "
            f"(Line {line_density:.2f}, Area {area_density:.2f}). "
            "Search speed도 같이 조정해주세요."
        )
        label.setVisible(True)

    def _update_area_width_hint(self, custom_enabled: bool) -> None:
        if self._area_width_hint is None:
            return
        if custom_enabled:
            self._area_width_hint.setText("수동 모드에서는 입력한 FOV와 Area review 최대 구간 값을 그대로 사용합니다.")
            return
        auto_width = self._as_float(load_fov_db_max_width(0.0), 0.0)
        if auto_width > 0.0:
            db_name = fov_db_path().name
            self._area_width_hint.setText(
                f"자동(DB) 모드에서는 선택된 FOV DB({db_name})의 최대 width 값 {auto_width:.1f} m를 Area review 기준으로 사용합니다."
            )
        else:
            self._area_width_hint.setText("자동(DB) 모드에서는 FOV DB width를 읽지 못해 기본 fallback 값을 사용합니다.")

    def _update_prior_profile_hint(self, payload: Dict[str, Any] | None = None) -> None:
        if self._prior_profile_hint is None:
            return
        effective_payload = payload if isinstance(payload, dict) else self._current_payload()
        profile = get_runtime_prior_mission_profile(
            default_turn_radius_m=400.0,
            default_fov_deg=5.0,
            payload=effective_payload,
        )
        turn_radius = self._as_float(profile.get("turn_radius_m"), 400.0)
        fov_deg = self._as_float(profile.get("fov_deg"), 5.0)
        sep_m = self._as_float(profile.get("sep_m"), 0.0)
        width_m = self._as_float(profile.get("width_m"), 0.0)
        if sep_m > 0.0:
            self._prior_profile_hint.setText(
                "선행임무 FOV는 DB에서 sep > Dubins 선회반경 조건으로 자동 선택됩니다. "
                f"현재 반경 {turn_radius:.1f} m -> FOV {fov_deg:.1f}°, SEP {sep_m:.1f} m, width {width_m:.1f} m"
            )
        else:
            self._prior_profile_hint.setText(
                "선행임무 FOV는 DB 기반 자동 선택입니다. 현재는 DB 선택값이 없어 fallback FOV를 사용합니다."
            )
