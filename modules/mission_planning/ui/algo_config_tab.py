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
    QVBoxLayout,
    QWidget,
)

try:
    from ..MissionPlanner.runtime_settings import (
        DEFAULT_ATTACK_MISSION_VALUES,
        DEFAULT_PRIOR_MISSION_VALUES,
        DEFAULT_RUNTIME_VALUES,
        canonicalize_runtime_payload,
        get_runtime_prior_mission_profile,
        load_fov_db_max_width,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        DEFAULT_ATTACK_MISSION_VALUES,
        DEFAULT_PRIOR_MISSION_VALUES,
        DEFAULT_RUNTIME_VALUES,
        canonicalize_runtime_payload,
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


GENERAL_GROUPS: list[tuple[str, str, list[FieldSpec]]] = [
    (
        "FOV",
        "Line/Area FOV와 자동 DB 사용 여부를 조정합니다.",
        [
            FieldSpec("values", "line_custom_fov_deg", "Line FOV (수동)", "float", 0.1, 120.0, 0.1, 2),
            FieldSpec("values", "area_custom_fov_deg", "Area FOV (수동)", "float", 0.1, 120.0, 0.1, 2),
            FieldSpec("values", "area_output_fov_scale", "Area FOV 배수", "float", 0.1, 10.0, 0.1, 2),
        ],
    ),
    (
        "탐색 밀도 / 속도",
        "Line/Area 탐색 경로의 밀도와 속도 가중치를 조정합니다.",
        [
            FieldSpec("values", "line_density_scale", "Line 밀도 배수", "float", 0.2, 3.0, 0.05, 2),
            FieldSpec("values", "area_density_scale", "Area 밀도 배수", "float", 0.2, 3.0, 0.05, 2),
            FieldSpec("values", "search_speed_weight", "Line 탐색 속도 가중치", "float", 0.1, 10.0, 0.1, 2),
            FieldSpec("values", "area_search_speed_weight", "Area 탐색 속도 가중치", "float", 0.1, 10.0, 0.1, 2),
        ],
    ),
    (
        "영역 / SEP",
        "Area sweep 간격과 Area review 기준을 조정합니다.",
        [
            FieldSpec("values", "area_route_offset_scale", "Area SEP 배수", "float", 0.0, 2.0, 0.05, 2),
            FieldSpec("values", "enhanced_area_review_max_segment_m", "Area review 최대 구간(m)", "float", 100.0, 10000.0, 100.0, 1),
        ],
    ),
    (
        "Sweep",
        "Sweep 라인 분할 점 개수를 조정합니다.",
        [
            FieldSpec("values", "sweep_line_interp_points", "Sweep 점 개수", "int", 2, 15, 1),
        ],
    ),
    (
        "WP / 선회",
        "UAV/LAH WP 간격과 Dubins 선회반경을 조정합니다.",
        [
            FieldSpec("values", "uav_wp_interval_m", "UAV WP 간격(m)", "float", 100.0, 10000.0, 100.0, 1),
            FieldSpec("values", "lah_wp_interval_m", "LAH WP 간격(m)", "float", 100.0, 10000.0, 100.0, 1),
            FieldSpec("values", "dubins_turn_radius_m", "Dubins 선회반경(m)", "float", 50.0, 5000.0, 10.0, 1),
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
        "공격 임무에 투입할 유인기 후보와 무장 타입을 설정합니다.",
        [
            FieldSpec("attack_mission", "manned_candidate_ids", "공격기 후보 ID", "list", placeholder="2, 3"),
            FieldSpec("attack_mission", "weapon_type", "무장 타입", "int", 0, 99, 1),
        ],
    ),
    (
        "진입 / 복귀",
        "공격 진입점, Resume 연결점, 공격점 고도 오프셋을 조정합니다.",
        [
            FieldSpec("attack_mission", "entry_offset_m", "공격 진입 오프셋(m)", "float", 0.0, 5000.0, 10.0, 1),
            FieldSpec("attack_mission", "resume_offset_m", "공격 후 Resume 오프셋(m)", "float", 0.0, 5000.0, 10.0, 1),
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

ALL_FIELD_SPECS = [spec for _, _, specs in GENERAL_GROUPS + PRIOR_GROUPS + ATTACK_GROUPS for spec in specs]


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
        self._status_label: QLabel | None = None
        self._path_label: QLabel | None = None
        self._fov_mode: _FocusWheelComboBox | None = None

        self.setStyleSheet(self._build_stylesheet())
        self._build_ui()
        self.reload_from_disk()

    def _build_stylesheet(self) -> str:
        assets_dir = Path(__file__).resolve().parents[2] / "monitoring" / "gui" / "tabs" / "assets"
        spin_up_icon = (assets_dir / "spin_up.svg").as_posix()
        spin_down_icon = (assets_dir / "spin_down.svg").as_posix()

        stylesheet = """
            QWidget#missionAlgoConfigTab { background: #f4f7fb; color: #1b2840; }
            QScrollArea { background: transparent; border: none; }
            QFrame#algoSectionCard { background: #ffffff; border: 1px solid #d8e3ef; border-radius: 18px; }
            QFrame#algoFooterCard { background: #ffffff; border: 1px solid #d8e3ef; border-radius: 16px; }
            QLabel#algoTitle { color: #10203b; font-size: 22px; font-weight: 700; }
            QLabel#algoSubtitle { color: #6c7a8f; font-size: 11px; }
            QLabel#algoBanner { background: #eef4ff; color: #2d4f9b; border: 1px solid #d2defa; border-radius: 10px; padding: 8px 10px; }
            QLabel#algoSectionTitle { color: #10203b; font-size: 16px; font-weight: 700; }
            QLabel#algoSectionNote { color: #66758a; font-size: 11px; }
            QLabel#algoHint { background: #f7faff; color: #49617f; border: 1px solid #dae6f3; border-radius: 10px; padding: 7px 10px; }
            QGroupBox { background: #fbfdff; border: 1px solid #d9e4ef; border-radius: 14px; margin-top: 14px; font-weight: 700; color: #17325c; padding: 8px 10px 8px 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QLabel#algoGroupNote { color: #62748b; font-size: 11px; }
            QLabel#algoFieldLabel { color: #203047; font-size: 13px; font-weight: 600; }
            QLineEdit { background: #ffffff; color: #172438; border: 1px solid #c8d4e3; border-radius: 11px; min-height: 30px; max-height: 30px; padding: 2px 10px; font-size: 13px; selection-background-color: #2f6df1; }
            QComboBox, QSpinBox, QDoubleSpinBox { background: #ffffff; color: #172438; border: 1px solid #c8d4e3; border-radius: 11px; min-height: 30px; max-height: 30px; padding: 2px 30px 2px 10px; font-size: 13px; selection-background-color: #2f6df1; }
            QLineEdit:hover, QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover { background: #fbfdff; border-color: #afbfd3; }
            QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { background: #ffffff; border-color: #4a7df2; }
            QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; width: 24px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top-right-radius: 10px; border-bottom-right-radius: 10px; }
            QSpinBox::up-button, QDoubleSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 24px; height: 14px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top-right-radius: 10px; }
            QSpinBox::down-button, QDoubleSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 24px; height: 14px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top: 1px solid #d7e2f1; border-bottom-right-radius: 10px; }
            QComboBox::down-arrow { image: url("__SPIN_DOWN_ICON__"); width: 12px; height: 12px; }
            QSpinBox::up-arrow, QDoubleSpinBox::up-arrow { image: url("__SPIN_UP_ICON__"); width: 12px; height: 12px; }
            QSpinBox::down-arrow, QDoubleSpinBox::down-arrow { image: url("__SPIN_DOWN_ICON__"); width: 12px; height: 12px; }
            QComboBox::drop-down:hover, QSpinBox::up-button:hover, QSpinBox::down-button:hover, QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover { background: #dfeaff; }
            QCheckBox { color: #203047; font-size: 13px; font-weight: 600; spacing: 8px; }
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

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(18, 18, 18, 6)
        content_layout.setSpacing(14)
        content_layout.addWidget(self._build_header_card())
        content_layout.addWidget(self._build_general_section())
        content_layout.addWidget(self._build_prior_section())
        content_layout.addWidget(self._build_attack_section())
        content_layout.addStretch(1)
        scroll.setWidget(content)

        root.addWidget(scroll, 1)
        root.addWidget(self._build_footer_card(), 0)

    def _build_header_card(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("algoSectionCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(8)

        title = QLabel("임무 계획 파라미터")
        title.setObjectName("algoTitle")
        subtitle = QLabel(f"설정 파일: {self._settings_path.name}")
        subtitle.setObjectName("algoSubtitle")
        banner = QLabel(
            "기본 임무 계획, 선행임무, 공격 임무 파라미터를 한 곳에서 조정합니다. "
            "저장 시 JSON과 런타임 설정에 바로 반영됩니다."
        )
        banner.setObjectName("algoBanner")
        banner.setWordWrap(True)

        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(banner)
        return frame

    def _build_general_section(self) -> QFrame:
        frame = self._create_section_card(
            "기본 임무 계획",
            "일반 임무계획과 일반 재계획에서 공통으로 쓰는 파라미터입니다.",
        )
        layout = frame.layout()
        assert isinstance(layout, QVBoxLayout)

        left_widgets = [
            self._build_mode_group(),
            self._build_group_box(*GENERAL_GROUPS[1]),
            self._build_group_box(*GENERAL_GROUPS[3]),
            self._build_flyover_group(),
        ]
        right_widgets = [
            self._build_group_box(*GENERAL_GROUPS[0]),
            self._build_group_box(*GENERAL_GROUPS[2]),
            self._build_group_box(*GENERAL_GROUPS[4]),
        ]
        layout.addLayout(self._build_column_board(left_widgets, right_widgets))
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

        note = QLabel("자동(DB) 모드에서는 FOV DB를 기준으로 폭을 계산하고, 수동 모드에서는 입력한 값을 그대로 씁니다.")
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
            ("flyover.entry_offset", "진입 오프셋 WP를 Over로 처리"),
            ("flyover.dubins_prefix", "Area 임무 간 Dubins prefix를 Over로 처리"),
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
        self._path_label = QLabel(f"파일: {self._settings_path.name}")
        self._path_label.setObjectName("algoHint")

        layout.addWidget(btn_reload)
        layout.addWidget(btn_save)
        layout.addWidget(btn_defaults)
        layout.addStretch(1)
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
        if spec.kind == "int":
            widget = _FocusWheelSpinBox()
            widget.setRange(int(spec.minimum or 0), int(spec.maximum or 999999))
            widget.setSingleStep(int(spec.step or 1))
            widget.valueChanged.connect(self._on_field_changed)
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

    def _default_payload(self) -> Dict[str, Any]:
        return canonicalize_runtime_payload()

    def _normalize_payload(self, payload: Dict[str, Any] | None) -> Dict[str, Any]:
        base = canonicalize_runtime_payload(deepcopy(payload) if isinstance(payload, dict) else None)

        values = base["values"]
        values["search_speed_weight"] = self._as_float(values.get("search_speed_weight"), 1.0)
        values["area_search_speed_weight"] = self._as_float(values.get("area_search_speed_weight"), 1.0)
        line_fov = self._as_float(values.get("line_custom_fov_deg", values.get("fov_deg")), 2.4)
        area_fov = self._as_float(values.get("area_custom_fov_deg", values.get("fov_deg")), line_fov)
        values["line_custom_fov_deg"] = line_fov
        values["area_custom_fov_deg"] = area_fov
        values["fov_deg"] = line_fov
        values["area_output_fov_scale"] = self._as_float(values.get("area_output_fov_scale"), 1.0)
        values["line_density_scale"] = self._as_float(values.get("line_density_scale"), 1.2)
        values["area_density_scale"] = self._as_float(values.get("area_density_scale"), 1.2)
        values["area_route_offset_scale"] = self._as_float(values.get("area_route_offset_scale"), 1.0)
        values["uav_wp_interval_m"] = self._as_float(values.get("uav_wp_interval_m"), 1200.0)
        values["lah_wp_interval_m"] = self._as_float(values.get("lah_wp_interval_m"), 3000.0)
        values["dubins_turn_radius_m"] = self._as_float(values.get("dubins_turn_radius_m"), 500.0)
        values["sweep_line_interp_points"] = max(2, self._as_int(values.get("sweep_line_interp_points"), 3))
        values["enhanced_area_review_max_segment_m"] = self._as_float(values.get("enhanced_area_review_max_segment_m"), 300.0)
        values["enhanced_auto_fov_from_db"] = bool(values.get("enhanced_auto_fov_from_db", True))

        prior = base["prior_mission"]
        for key, default in DEFAULT_PRIOR_MISSION_VALUES.items():
            prior[key] = self._as_int(prior.get(key), int(default)) if isinstance(default, int) else self._as_float(prior.get(key), float(default))

        attack = base["attack_mission"]
        attack["manned_candidate_ids"] = self._as_int_list(
            attack.get("manned_candidate_ids"),
            list(DEFAULT_ATTACK_MISSION_VALUES["manned_candidate_ids"]),
        )
        for key, default in DEFAULT_ATTACK_MISSION_VALUES.items():
            if key == "manned_candidate_ids":
                continue
            attack[key] = self._as_int(attack.get(key), int(default)) if isinstance(default, int) else self._as_float(attack.get(key), float(default))

        flyover = base["flyover"]
        for key in ("entry_offset", "dubins_prefix", "all_wps"):
            flyover[key] = bool(flyover.get(key, False))
        return base

    def _write_widget_value(self, spec: FieldSpec, widget: QWidget, value: Any) -> None:
        if spec.kind == "int":
            assert isinstance(widget, QSpinBox)
            default = int(DEFAULT_PRIOR_MISSION_VALUES.get(spec.key, DEFAULT_ATTACK_MISSION_VALUES.get(spec.key, 0)))
            with QSignalBlocker(widget):
                widget.setValue(self._as_int(value, default))
            return
        if spec.kind == "list":
            assert isinstance(widget, QLineEdit)
            with QSignalBlocker(widget):
                widget.setText(self._format_int_list(self._as_int_list(value, list(DEFAULT_ATTACK_MISSION_VALUES["manned_candidate_ids"]))))
            return
        assert isinstance(widget, QDoubleSpinBox)
        default = float(DEFAULT_RUNTIME_VALUES.get(spec.key, DEFAULT_PRIOR_MISSION_VALUES.get(spec.key, DEFAULT_ATTACK_MISSION_VALUES.get(spec.key, 0.0))))
        with QSignalBlocker(widget):
            widget.setValue(self._as_float(value, default))

    def _read_widget_value(self, spec: FieldSpec, widget: QWidget) -> Any:
        if spec.kind == "int":
            assert isinstance(widget, QSpinBox)
            return int(widget.value())
        if spec.kind == "list":
            assert isinstance(widget, QLineEdit)
            return self._as_int_list(widget.text(), list(DEFAULT_ATTACK_MISSION_VALUES["manned_candidate_ids"]))
        assert isinstance(widget, QDoubleSpinBox)
        return float(widget.value())

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

            for key in ("entry_offset", "dubins_prefix", "all_wps"):
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
        values["fov_deg"] = float(values.get("line_custom_fov_deg", DEFAULT_RUNTIME_VALUES["line_custom_fov_deg"]))
        for key in ("entry_offset", "dubins_prefix", "all_wps"):
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

    def _on_fov_mode_changed(self) -> None:
        if self._building:
            return
        self._sync_enabled_state()
        self._set_status("수정 중")

    def _on_field_changed(self) -> None:
        if self._building:
            return
        self._sync_enabled_state()
        self._set_status("수정 중")

    def _sync_enabled_state(self, payload: Dict[str, Any] | None = None) -> None:
        custom_enabled = bool(self._fov_mode is not None and self._fov_mode.currentData() == "custom")
        for key in (
            "values.line_custom_fov_deg",
            "values.area_custom_fov_deg",
            "values.enhanced_area_review_max_segment_m",
        ):
            widget = self._widgets.get(key)
            if widget is not None:
                widget.setEnabled(custom_enabled)
        self._update_area_width_hint(custom_enabled)
        self._update_prior_profile_hint(payload)

    def _update_area_width_hint(self, custom_enabled: bool) -> None:
        if self._area_width_hint is None:
            return
        if custom_enabled:
            self._area_width_hint.setText("수동 모드에서는 입력한 FOV와 Area review 최대 구간 값을 그대로 사용합니다.")
            return
        auto_width = self._as_float(load_fov_db_max_width(0.0), 0.0)
        if auto_width > 0.0:
            self._area_width_hint.setText(
                f"자동(DB) 모드에서는 resource/db/fov_db.csv의 최대 width 값 {auto_width:.1f} m를 Area review 기준으로 사용합니다."
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
