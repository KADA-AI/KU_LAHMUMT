from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable, Dict

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class MissionAlgoConfigTab(QWidget):
    def __init__(self, settings_path: Path, on_apply: Callable[[Dict[str, Any]], None] | None = None, parent=None):
        super().__init__(parent)
        self._settings_path = Path(settings_path)
        self._on_apply = on_apply
        self._save_timer = QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.timeout.connect(self._save_now)
        self._widgets: Dict[str, Any] = {}
        self._field_labels: Dict[str, QLabel] = {}
        self._status = QLabel()
        self._db_usage_hint = QLabel()
        self._building = False
        self._applying_preset = False

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        root.addWidget(self._build_preset_group())
        root.addWidget(self._build_flight_group())
        root.addWidget(self._build_sweep_group())
        root.addWidget(self._build_enhanced_group())
        root.addWidget(self._build_flyover_group())
        root.addLayout(self._build_footer())
        root.addStretch(1)

        self.reload_from_disk()

    def _build_preset_group(self) -> QGroupBox:
        box = QGroupBox("알고리즘 세트")
        form = QFormLayout(box)
        form.addRow("프리셋", self._combo("preset_key", self._preset_options()))
        hint = QLabel(
            "Bearing_Par_Sweep Mode: bearing 평행 sweep 세트 / "
            "Bearing_Ver_Sweep Mode: bearing 수직 sweep 세트 / "
            "Nadir Mode: 직하방 경로 세트"
        )
        hint.setWordWrap(True)
        form.addRow("", hint)
        return box

    def _build_flight_group(self) -> QGroupBox:
        box = QGroupBox("비행 기본")
        form = QFormLayout(box)
        form.addRow("경로 알고리즘", self._combo("algo_key", {
            "dtatrim": "DTA Trim",
            "algo2": "Linear",
            "algo3": "Algo3",
        }))
        form.addRow("순항 속도(m/s)", self._double("cruise_speed_mps", 1.0, 120.0, 0.1))
        form.addRow("회전 스텝(°)", self._double("turn_step_deg", 1.0, 90.0, 0.5))
        form.addRow("기준 고도(m)", self._spin("altitude_m", 100, 5000))
        form.addRow("탐색 속도 가중치", self._double("search_speed_weight", 0.1, 20.0, 0.1))
        form.addRow("유인기 계획 모드", self._combo("manned_plan_mode", {
            "normal": "Normal",
            "ops_sim": "작전모사",
            "capstone": "종합과제 전용",
        }))
        form.addRow("UAV plan mode", self._combo("uav_plan_mode", {
            "normal": "Normal",
            "dub_path": "Dub Path",
        }))
        return box

    def _build_sweep_group(self) -> QGroupBox:
        box = QGroupBox("라인/영역")
        form = QFormLayout(box)
        self._db_usage_hint.setWordWrap(True)
        self._db_usage_hint.setObjectName("InfoBadge")
        form.addRow(self._db_usage_hint)
        form.addRow(
            self._label("area_sweep_mode", "Area sweep 기준"),
            self._combo("area_sweep_mode", {
                "parallel": "Bearing 평행",
                "vertical": "Bearing 수직",
                "nadir": "Nadir",
            }),
        )
        form.addRow(self._label("default_sweep_separation_m", "기본 이격거리(m)"), self._double("default_sweep_separation_m", 10.0, 5000.0, 10.0))
        form.addRow(self._label("fov_deg", "기본 FOV(°)"), self._double("fov_deg", 0.1, 90.0, 0.1))
        form.addRow(self._label("area_nadir_fov_deg", "직하방 FOV(°)"), self._double("area_nadir_fov_deg", 0.1, 120.0, 0.1))
        form.addRow(self._label("db_fov_weight", "DB FOV 가중치"), self._double("db_fov_weight", 1.0, 5.0, 0.05))
        form.addRow(self._label("sweep_entry_offset_m", "Entry 오프셋(m)"), self._double("sweep_entry_offset_m", 0.0, 5000.0, 10.0))
        form.addRow(self._label("sweep_merge_heading_deg", "병합 허용 각도(°)"), self._double("sweep_merge_heading_deg", 0.0, 180.0, 1.0))
        form.addRow(self._label("min_sweep_len_m", "최소 스윕 길이(m)"), self._double("min_sweep_len_m", 0.0, 1000.0, 1.0))
        form.addRow(self._label("min_route_spacing_m", "최소 경로 간격(m)"), self._double("min_route_spacing_m", 0.0, 5000.0, 10.0))
        form.addRow(self._label("enhanced_auto_fov_from_db", "자동 FOV/SEP/VEL 선택(DB)"), self._check("enhanced_auto_fov_from_db"))
        form.addRow(self._label("area_dubins_entry_links_enabled", "Area sweep Dubins link [1][2]"), self._check("area_dubins_entry_links_enabled"))
        return box

    def _build_enhanced_group(self) -> QGroupBox:
        box = QGroupBox("고급 분할")
        form = QFormLayout(box)
        form.addRow(
            self._label("area_split_mode", "Area 분할 모드"),
            self._combo("area_split_mode", {
                "two_stage": "2단 분할(기존)",
                "single_stage": "1단 분할만",
            }),
        )
        form.addRow(self._label("enhanced_area_review_enabled", "Area 재분할 사용"), self._check("enhanced_area_review_enabled"))
        form.addRow(self._label("enhanced_area_review_max_segment_m", "Area 최대 세그먼트(m)"), self._double("enhanced_area_review_max_segment_m", 50.0, 5000.0, 10.0))
        form.addRow(self._label("sweep_line_interp_points", "스윕 보간 개수"), self._spin("sweep_line_interp_points", 2, 20))
        form.addRow(self._label("point_fov_deg", "좌표 임무 FOV(°)"), self._double("point_fov_deg", 0.1, 120.0, 0.1))
        return box

    def _build_flyover_group(self) -> QGroupBox:
        box = QGroupBox("Fly-over 옵션")
        form = QFormLayout(box)
        form.addRow(self._label("flyover_entry_offset", "Entry offset 사용"), self._check("flyover_entry_offset"))
        form.addRow(self._label("flyover_all_wps", "전체 WP Fly-over"), self._check("flyover_all_wps"))
        return box

    def _build_footer(self):
        row = QHBoxLayout()
        btn_reload = QPushButton("다시 불러오기")
        btn_reload.clicked.connect(self.reload_from_disk)
        btn_save = QPushButton("즉시 저장")
        btn_save.clicked.connect(self._save_now)
        btn_defaults = QPushButton("기본값")
        btn_defaults.clicked.connect(self._load_defaults)
        btn_preset = QPushButton("프리셋 적용")
        btn_preset.clicked.connect(self._apply_selected_preset)
        self._status.setObjectName("InfoBadge")
        row.addWidget(btn_reload)
        row.addWidget(btn_save)
        row.addWidget(btn_preset)
        row.addWidget(btn_defaults)
        row.addStretch(1)
        row.addWidget(self._status)
        return row

    def _combo(self, key: str, options: Dict[str, str]) -> QComboBox:
        w = QComboBox()
        for value, label in options.items():
            w.addItem(label, value)
        if key == "preset_key":
            w.currentIndexChanged.connect(self._on_preset_changed)
        else:
            w.currentIndexChanged.connect(self._schedule_save)
        self._widgets[key] = w
        return w

    def _label(self, key: str, text: str) -> QLabel:
        label = QLabel(text)
        self._field_labels[key] = label
        return label

    def _double(self, key: str, min_v: float, max_v: float, step: float) -> QDoubleSpinBox:
        w = QDoubleSpinBox()
        w.setRange(min_v, max_v)
        w.setSingleStep(step)
        w.setDecimals(2 if step < 1.0 else 1)
        w.valueChanged.connect(self._schedule_save)
        self._widgets[key] = w
        return w

    def _spin(self, key: str, min_v: int, max_v: int) -> QSpinBox:
        w = QSpinBox()
        w.setRange(min_v, max_v)
        w.valueChanged.connect(self._schedule_save)
        self._widgets[key] = w
        return w

    def _check(self, key: str) -> QCheckBox:
        w = QCheckBox()
        w.stateChanged.connect(self._schedule_save)
        self._widgets[key] = w
        return w

    def _default_payload(self) -> Dict[str, Any]:
        return {
            "preset_key": "bearing_par_sweep",
            "algo_key": "algo2",
            "values": {
                "cruise_speed_mps": 40.0,
                "turn_step_deg": 15.0,
                "default_sweep_separation_m": 600.0,
                "area_sweep_mode": "parallel",
                "area_split_mode": "two_stage",
                "manned_plan_mode": "normal",
                "uav_plan_mode": "normal",
                "search_speed_weight": 1.2,
                "fov_deg": 2.4,
                "db_fov_weight": 1.0,
                "altitude_m": 610,
                "sweep_entry_offset_m": 500.0,
                "sweep_merge_heading_deg": 5.0,
                "sweep_line_interp_points": 3,
                "min_sweep_len_m": 3.0,
                "min_route_spacing_m": 200.0,
                "default_search_speed_multiplier": 16.0,
                "point_fov_deg": 66.638654,
                "area_nadir_fov_deg": 31.2,
                "entry_hold_fov_deg": 10.0,
                "entry_hold_gimbal_pitch": -90.0,
                "entry_hold_gimbal_yaw": 0.0,
                "loiter_radius_m": 800.0,
                "loiter_direction": 1,
                "loiter_time_s": 30.0,
                "loiter_speed_mps": 30.0,
                "enhanced_area_review_enabled": True,
                "enhanced_area_review_max_segment_m": 550.0,
                "enhanced_auto_fov_from_db": True,
                "area_dubins_entry_links_enabled": False,
            },
            "flyover": {
                "entry_offset": False,
                "dubins_prefix": False,
                "all_wps": False,
            },
        }

    def _preset_options(self) -> Dict[str, str]:
        return {
            "bearing_par_sweep": "Bearing_Par_Sweep Mode",
            "bearing_ver_sweep": "Bearing_Ver_Sweep Mode",
            "nadir_mode": "Nadir Mode",
            "dubins_mode": "Dubins Mode",
            "custom": "Custom",
        }

    def _preset_payload(self, preset_key: str) -> Dict[str, Any]:
        presets = {
            "bearing_par_sweep": {
                "preset_key": "bearing_par_sweep",
                "algo_key": "algo2",
                "values": {
                    "cruise_speed_mps": 40.0,
                    "turn_step_deg": 15.0,
                    "default_sweep_separation_m": 1000.0,
                    "area_sweep_mode": "parallel",
                    "area_split_mode": "two_stage",
                    "manned_plan_mode": "normal",
                    "uav_plan_mode": "normal",
                    "search_speed_weight": 1.2,
                    "fov_deg": 2.4,
                    "db_fov_weight": 1.0,
                    "altitude_m": 610,
                    "sweep_entry_offset_m": 1500.0,
                    "sweep_merge_heading_deg": 5.0,
                    "sweep_line_interp_points": 3,
                    "min_sweep_len_m": 3.0,
                    "min_route_spacing_m": 200.0,
                    "default_search_speed_multiplier": 16.0,
                    "point_fov_deg": 66.638654,
                    "area_nadir_fov_deg": 31.2,
                    "entry_hold_fov_deg": 10.0,
                    "entry_hold_gimbal_pitch": -90.0,
                    "entry_hold_gimbal_yaw": 0.0,
                    "loiter_radius_m": 800.0,
                    "loiter_direction": 1,
                    "loiter_time_s": 30.0,
                    "loiter_speed_mps": 30.0,
                    "enhanced_area_review_enabled": True,
                    "enhanced_area_review_max_segment_m": 550.0,
                    "enhanced_auto_fov_from_db": True,
                    "area_dubins_entry_links_enabled": False,
                },
                "flyover": {
                    "entry_offset": False,
                    "dubins_prefix": False,
                    "all_wps": True,
                },
            },
            "bearing_ver_sweep": {
                "preset_key": "bearing_ver_sweep",
                "algo_key": "algo2",
                "values": {
                    "cruise_speed_mps": 40.0,
                    "turn_step_deg": 15.0,
                    "default_sweep_separation_m": 1000.0,
                    "area_sweep_mode": "vertical",
                    "area_split_mode": "two_stage",
                    "manned_plan_mode": "normal",
                    "uav_plan_mode": "normal",
                    "search_speed_weight": 1.2,
                    "fov_deg": 2.4,
                    "db_fov_weight": 1.0,
                    "altitude_m": 610,
                    "sweep_entry_offset_m": 1500.0,
                    "sweep_merge_heading_deg": 5.0,
                    "sweep_line_interp_points": 3,
                    "min_sweep_len_m": 3.0,
                    "min_route_spacing_m": 200.0,
                    "default_search_speed_multiplier": 16.0,
                    "point_fov_deg": 66.638654,
                    "area_nadir_fov_deg": 31.2,
                    "entry_hold_fov_deg": 10.0,
                    "entry_hold_gimbal_pitch": -90.0,
                    "entry_hold_gimbal_yaw": 0.0,
                    "loiter_radius_m": 800.0,
                    "loiter_direction": 1,
                    "loiter_time_s": 30.0,
                    "loiter_speed_mps": 30.0,
                    "enhanced_area_review_enabled": True,
                    "enhanced_area_review_max_segment_m": 550.0,
                    "enhanced_auto_fov_from_db": True,
                    "area_dubins_entry_links_enabled": False,
                },
                "flyover": {
                    "entry_offset": False,
                    "dubins_prefix": False,
                    "all_wps": False,
                },
            },
            "nadir_mode": {
                "preset_key": "nadir_mode",
                "algo_key": "algo2",
                "values": {
                    "cruise_speed_mps": 40.0,
                    "turn_step_deg": 15.0,
                    "default_sweep_separation_m": 1000.0,
                    "area_sweep_mode": "nadir",
                    "area_split_mode": "two_stage",
                    "manned_plan_mode": "normal",
                    "uav_plan_mode": "normal",
                    "search_speed_weight": 1.2,
                    "fov_deg": 2.4,
                    "db_fov_weight": 1.0,
                    "altitude_m": 610,
                    "sweep_entry_offset_m": 1500.0,
                    "sweep_merge_heading_deg": 5.0,
                    "sweep_line_interp_points": 3,
                    "min_sweep_len_m": 3.0,
                    "min_route_spacing_m": 200.0,
                    "default_search_speed_multiplier": 16.0,
                    "point_fov_deg": 66.638654,
                    "area_nadir_fov_deg": 31.2,
                    "entry_hold_fov_deg": 10.0,
                    "entry_hold_gimbal_pitch": -90.0,
                    "entry_hold_gimbal_yaw": 0.0,
                    "loiter_radius_m": 800.0,
                    "loiter_direction": 1,
                    "loiter_time_s": 30.0,
                    "loiter_speed_mps": 30.0,
                    "enhanced_area_review_enabled": True,
                    "enhanced_area_review_max_segment_m": 550.0,
                    "enhanced_auto_fov_from_db": True,
                    "area_dubins_entry_links_enabled": False,
                },
                "flyover": {
                    "entry_offset": False,
                    "dubins_prefix": False,
                    "all_wps": False,
                },
            },
            "dubins_mode": {
                "preset_key": "dubins_mode",
                "algo_key": "algo2",
                "values": {
                    "cruise_speed_mps": 40.0,
                    "turn_step_deg": 15.0,
                    "default_sweep_separation_m": 1000.0,
                    "area_sweep_mode": "vertical",
                    "area_split_mode": "two_stage",
                    "manned_plan_mode": "normal",
                    "uav_plan_mode": "dub_path",
                    "search_speed_weight": 1.2,
                    "fov_deg": 2.4,
                    "db_fov_weight": 1.0,
                    "altitude_m": 610,
                    "sweep_entry_offset_m": 1500.0,
                    "sweep_merge_heading_deg": 5.0,
                    "sweep_line_interp_points": 3,
                    "min_sweep_len_m": 3.0,
                    "min_route_spacing_m": 200.0,
                    "default_search_speed_multiplier": 16.0,
                    "point_fov_deg": 66.638654,
                    "area_nadir_fov_deg": 31.2,
                    "entry_hold_fov_deg": 10.0,
                    "entry_hold_gimbal_pitch": -90.0,
                    "entry_hold_gimbal_yaw": 0.0,
                    "loiter_radius_m": 800.0,
                    "loiter_direction": 1,
                    "loiter_time_s": 30.0,
                    "loiter_speed_mps": 30.0,
                    "enhanced_area_review_enabled": True,
                    "enhanced_area_review_max_segment_m": 550.0,
                    "enhanced_auto_fov_from_db": True,
                    "area_dubins_entry_links_enabled": False,
                },
                "flyover": {
                    "entry_offset": False,
                    "dubins_prefix": False,
                    "all_wps": False,
                },
            },
        }
        if preset_key == "custom":
            payload = self._collect_payload()
            payload["preset_key"] = "custom"
            return payload
        base = presets.get(preset_key)
        if base is None:
            return deepcopy(self._default_payload())
        return deepcopy(base)

    def _load_defaults(self) -> None:
        self._apply_payload(self._default_payload())
        self._schedule_save()

    def reload_from_disk(self) -> None:
        payload = self._default_payload()
        if self._settings_path.exists():
            try:
                loaded = json.loads(self._settings_path.read_text(encoding="utf-8"))
                if isinstance(loaded, dict):
                    payload.update({k: v for k, v in loaded.items() if k in ("preset_key", "algo_key", "values", "flyover")})
                    if isinstance(payload.get("values"), dict) and isinstance(loaded.get("values"), dict):
                        payload["values"].update(loaded["values"])
                    if isinstance(payload.get("flyover"), dict) and isinstance(loaded.get("flyover"), dict):
                        payload["flyover"].update(loaded["flyover"])
            except Exception:
                pass
        self._apply_payload(payload)
        self._status.setText(f"파일: {self._settings_path.name}")

    def _apply_payload(self, payload: Dict[str, Any]) -> None:
        self._building = True
        try:
            values = payload.get("values") if isinstance(payload.get("values"), dict) else {}
            flyover = payload.get("flyover") if isinstance(payload.get("flyover"), dict) else {}
            preset_key = str(payload.get("preset_key", "bearing_par_sweep") or "bearing_par_sweep")
            preset_combo: QComboBox = self._widgets["preset_key"]
            preset_idx = preset_combo.findData(preset_key)
            preset_combo.setCurrentIndex(max(preset_idx, 0))
            combo: QComboBox = self._widgets["algo_key"]
            idx = combo.findData(payload.get("algo_key", "algo2"))
            combo.setCurrentIndex(max(idx, 0))
            area_sweep_combo: QComboBox = self._widgets["area_sweep_mode"]
            area_mode_value = values.get("area_sweep_mode", "parallel")
            area_sweep_idx = area_sweep_combo.findData(area_mode_value)
            area_sweep_combo.setCurrentIndex(max(area_sweep_idx, 0))
            area_split_combo: QComboBox = self._widgets["area_split_mode"]
            area_split_idx = area_split_combo.findData(values.get("area_split_mode", "two_stage"))
            area_split_combo.setCurrentIndex(max(area_split_idx, 0))
            manned_mode_combo: QComboBox = self._widgets["manned_plan_mode"]
            manned_mode_idx = manned_mode_combo.findData(values.get("manned_plan_mode", "normal"))
            manned_mode_combo.setCurrentIndex(max(manned_mode_idx, 0))
            uav_mode_combo: QComboBox = self._widgets["uav_plan_mode"]
            uav_mode_value = values.get("uav_plan_mode", "normal")
            uav_mode_idx = uav_mode_combo.findData(uav_mode_value)
            uav_mode_combo.setCurrentIndex(max(uav_mode_idx, 0))
            self._widgets["cruise_speed_mps"].setValue(float(values.get("cruise_speed_mps", 40.0)))
            self._widgets["turn_step_deg"].setValue(float(values.get("turn_step_deg", 15.0)))
            self._widgets["default_sweep_separation_m"].setValue(float(values.get("default_sweep_separation_m", 600.0)))
            self._widgets["search_speed_weight"].setValue(float(values.get("search_speed_weight", 1.2)))
            self._widgets["fov_deg"].setValue(float(values.get("fov_deg", 2.4)))
            self._widgets["db_fov_weight"].setValue(float(values.get("db_fov_weight", 1.0)))
            self._widgets["altitude_m"].setValue(int(float(values.get("altitude_m", 610))))
            self._widgets["sweep_entry_offset_m"].setValue(float(values.get("sweep_entry_offset_m", 500.0)))
            self._widgets["sweep_merge_heading_deg"].setValue(float(values.get("sweep_merge_heading_deg", 5.0)))
            self._widgets["min_sweep_len_m"].setValue(float(values.get("min_sweep_len_m", 3.0)))
            self._widgets["min_route_spacing_m"].setValue(float(values.get("min_route_spacing_m", 200.0)))
            self._widgets["area_nadir_fov_deg"].setValue(float(values.get("area_nadir_fov_deg", 31.2)))
            self._widgets["enhanced_area_review_enabled"].setChecked(bool(values.get("enhanced_area_review_enabled", True)))
            self._widgets["enhanced_area_review_max_segment_m"].setValue(float(values.get("enhanced_area_review_max_segment_m", 550.0)))
            self._widgets["enhanced_auto_fov_from_db"].setChecked(
                False if preset_key == "custom" else bool(values.get("enhanced_auto_fov_from_db", True))
            )
            self._widgets["area_dubins_entry_links_enabled"].setChecked(bool(values.get("area_dubins_entry_links_enabled", False)))
            self._widgets["sweep_line_interp_points"].setValue(int(float(values.get("sweep_line_interp_points", 3))))
            self._widgets["point_fov_deg"].setValue(float(values.get("point_fov_deg", 66.638654)))
            self._widgets["flyover_entry_offset"].setChecked(bool(flyover.get("entry_offset", False)))
            self._widgets["flyover_all_wps"].setChecked(bool(flyover.get("all_wps", False)))
        finally:
            self._building = False
        self._refresh_mode_hints()

    def _collect_payload(self) -> Dict[str, Any]:
        preset_combo: QComboBox = self._widgets["preset_key"]
        combo: QComboBox = self._widgets["algo_key"]
        preset_key = str(preset_combo.currentData() or "custom")
        values = self._default_payload()["values"]
        values.update(
            {
                "cruise_speed_mps": float(self._widgets["cruise_speed_mps"].value()),
                "turn_step_deg": float(self._widgets["turn_step_deg"].value()),
                "default_sweep_separation_m": float(self._widgets["default_sweep_separation_m"].value()),
                "area_sweep_mode": str(self._widgets["area_sweep_mode"].currentData() or "parallel"),
                "area_split_mode": str(self._widgets["area_split_mode"].currentData() or "two_stage"),
                "manned_plan_mode": str(self._widgets["manned_plan_mode"].currentData() or "normal"),
                "uav_plan_mode": str(self._widgets["uav_plan_mode"].currentData() or "normal"),
                "search_speed_weight": float(self._widgets["search_speed_weight"].value()),
                "fov_deg": float(self._widgets["fov_deg"].value()),
                "db_fov_weight": float(self._widgets["db_fov_weight"].value()),
                "altitude_m": int(self._widgets["altitude_m"].value()),
                "sweep_entry_offset_m": float(self._widgets["sweep_entry_offset_m"].value()),
                "sweep_merge_heading_deg": float(self._widgets["sweep_merge_heading_deg"].value()),
                "min_sweep_len_m": float(self._widgets["min_sweep_len_m"].value()),
                "min_route_spacing_m": float(self._widgets["min_route_spacing_m"].value()),
                "area_nadir_fov_deg": float(self._widgets["area_nadir_fov_deg"].value()),
                "enhanced_area_review_enabled": bool(self._widgets["enhanced_area_review_enabled"].isChecked()),
                "enhanced_area_review_max_segment_m": float(self._widgets["enhanced_area_review_max_segment_m"].value()),
                "enhanced_auto_fov_from_db": False if preset_key == "custom" else bool(self._widgets["enhanced_auto_fov_from_db"].isChecked()),
                "area_dubins_entry_links_enabled": bool(self._widgets["area_dubins_entry_links_enabled"].isChecked()),
                "sweep_line_interp_points": int(self._widgets["sweep_line_interp_points"].value()),
                "point_fov_deg": float(self._widgets["point_fov_deg"].value()),
            }
        )
        return {
            "preset_key": preset_key,
            "algo_key": combo.currentData(),
            "values": values,
            "flyover": {
                "entry_offset": bool(self._widgets["flyover_entry_offset"].isChecked()),
                "dubins_prefix": False,
                "all_wps": bool(self._widgets["flyover_all_wps"].isChecked()),
            },
        }

    def _schedule_save(self, *args) -> None:
        if self._building:
            return
        self._save_timer.start(250)
        self._refresh_mode_hints()

    def _save_now(self) -> None:
        payload = self._collect_payload()
        self._settings_path.parent.mkdir(parents=True, exist_ok=True)
        self._settings_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self._status.setText(f"저장됨: {self._settings_path.name}")
        if self._on_apply is not None:
            self._on_apply(payload)

    def _apply_selected_preset(self) -> None:
        preset_combo: QComboBox = self._widgets["preset_key"]
        preset_key = str(preset_combo.currentData() or "custom")
        if preset_key == "custom":
            self._status.setText("Custom은 현재 값 유지")
            self._save_now()
            return
        self._applying_preset = True
        try:
            self._apply_payload(self._preset_payload(preset_key))
        finally:
            self._applying_preset = False
        self._save_now()
        self._status.setText(f"프리셋 적용: {preset_combo.currentText()}")
        self._refresh_mode_hints()

    def _on_preset_changed(self, *args) -> None:
        if self._building:
            return
        preset_combo: QComboBox = self._widgets["preset_key"]
        preset_key = str(preset_combo.currentData() or "custom")
        if preset_key == "custom":
            db_check = self._widgets.get("enhanced_auto_fov_from_db")
            if isinstance(db_check, QCheckBox) and db_check.isChecked():
                self._building = True
                try:
                    db_check.setChecked(False)
                finally:
                    self._building = False
            self._schedule_save()
            self._refresh_mode_hints()
            return
        self._apply_selected_preset()

    def _refresh_mode_hints(self) -> None:
        preset_combo = self._widgets.get("preset_key")
        db_check = self._widgets.get("enhanced_auto_fov_from_db")
        preset_key = str(preset_combo.currentData()) if isinstance(preset_combo, QComboBox) else "custom"
        is_custom = preset_key == "custom"
        if isinstance(db_check, QCheckBox):
            db_check.setEnabled(not is_custom)
        use_db = bool(db_check.isChecked()) if isinstance(db_check, QCheckBox) else False
        if is_custom:
            use_db = False

        area_mode_widget = self._widgets.get("area_sweep_mode")
        sep_widget = self._widgets.get("default_sweep_separation_m")
        fov_widget = self._widgets.get("fov_deg")
        db_fov_weight_widget = self._widgets.get("db_fov_weight")
        if area_mode_widget is not None:
            area_mode_widget.setEnabled(True)
        if sep_widget is not None:
            sep_widget.setEnabled(True)
        if fov_widget is not None:
            fov_widget.setEnabled(True)
        if db_fov_weight_widget is not None:
            db_fov_weight_widget.setEnabled(use_db)

        sep_label = self._field_labels.get("default_sweep_separation_m")
        fov_label = self._field_labels.get("fov_deg")
        if use_db:
            if sep_label is not None:
                sep_label.setText("기본 이격거리(m) [DB 미사용 시 fallback]")
            if fov_label is not None:
                fov_label.setText("기본 FOV(°) [DB 미사용 시 fallback]")
            self._db_usage_hint.setText(
                "현재 일반 Line/Area는 DB에서 width 기준으로 SEP/FOV/VEL을 자동 선택한다. "
                "아래 값은 DB를 쓰지 않을 때만 fallback으로 사용된다."
            )
        else:
            if sep_label is not None:
                sep_label.setText("기본 이격거리(m) [직접 사용]")
            if fov_label is not None:
                fov_label.setText("기본 FOV(°) [직접 사용]")
            if is_custom:
                self._db_usage_hint.setText(
                    "Custom 모드는 DB를 사용하지 않는다. 현재 일반 Line/Area는 아래 기본 이격거리/FOV 값을 직접 사용한다."
                )
            else:
                self._db_usage_hint.setText(
                    "현재 일반 Line/Area는 아래 기본 이격거리/FOV 값을 직접 사용한다."
                )

        area_mode_label = self._field_labels.get("area_sweep_mode")
        if area_mode_label is not None and isinstance(area_mode_widget, QComboBox):
            mode_text = area_mode_widget.currentText()
            area_mode_label.setText(f"Area sweep 기준 [{mode_text}]")
        area_split_widget = self._widgets.get("area_split_mode")
        area_split_label = self._field_labels.get("area_split_mode")
        if area_split_label is not None and isinstance(area_split_widget, QComboBox):
            area_split_label.setText(f"Area 분할 모드 [{area_split_widget.currentText()}]")

        if not is_custom:
            mode_name = preset_combo.currentText() if isinstance(preset_combo, QComboBox) else preset_key
            self._status.setText(f"프리셋 기반 편집 중: {mode_name}")
