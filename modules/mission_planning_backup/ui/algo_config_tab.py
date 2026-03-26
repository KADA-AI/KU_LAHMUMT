from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable, Dict

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

try:
    from ..MissionPlanner.runtime_settings import load_fov_db_max_width
except Exception:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import load_fov_db_max_width  # type: ignore
    except Exception:
        def load_fov_db_max_width(default: float = 0.0) -> float:
            return float(default)


class MissionAlgoConfigTab(QWidget):
    def __init__(
        self,
        settings_path: Path,
        on_apply: Callable[[Dict[str, Any]], None] | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._settings_path = Path(settings_path)
        self._on_apply = on_apply
        self._payload: Dict[str, Any] = {}
        self._building = False
        self._widgets: Dict[str, Any] = {}
        self._area_width_hint: QLabel | None = None

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        self._summary = QLabel(
            "기본 임무 계획 프리셋 값입니다. "
            "여기서 런타임 설정을 수정한 뒤 바로 저장할 수 있습니다."
        )
        self._summary.setWordWrap(True)
        self._summary.setObjectName("InfoBadge")
        root.addWidget(self._summary)
        root.addWidget(self._build_mode_group())
        root.addWidget(self._build_search_speed_group())
        root.addWidget(self._build_fov_group())
        root.addWidget(self._build_area_group())
        root.addWidget(self._build_sweep_group())
        self._area_width_hint = QLabel("")
        self._area_width_hint.setWordWrap(True)
        self._area_width_hint.setObjectName("InfoBadge")
        root.addWidget(self._area_width_hint)
        root.addWidget(self._build_wp_group())
        root.addWidget(self._build_flyover_group())
        root.addLayout(self._build_footer())
        root.addStretch(1)

        self.reload_from_disk()

    def _build_mode_group(self) -> QGroupBox:
        box = QGroupBox("계획 모드")
        form = QFormLayout(box)

        mode = QComboBox()
        mode.addItem("기본 임무 계획", "dubins_mode")
        mode.currentIndexChanged.connect(self._on_field_changed)
        self._widgets["preset_key"] = mode

        fov_mode = QComboBox()
        fov_mode.addItem("자동(DB)", "auto")
        fov_mode.addItem("수동", "custom")
        fov_mode.currentIndexChanged.connect(self._on_fov_mode_changed)
        self._widgets["fov_mode"] = fov_mode

        note = QLabel(
            "현재는 기본 임무 계획 프리셋만 노출되어 있습니다. "
            "자동 모드에서는 DB/기본 FOV를 사용하고, 수동 모드에서는 아래 Line/Area FOV 값을 적용합니다."
        )
        note.setWordWrap(True)
        form.addRow("프리셋", mode)
        form.addRow("FOV 모드", fov_mode)
        form.addRow("", note)
        return box

    def _build_fov_group(self) -> QGroupBox:
        box = QGroupBox("FOV")
        form = QFormLayout(box)
        form.addRow("Line FOV (수동)", self._double("line_custom_fov_deg", 0.1, 120.0, 0.1, 2))
        form.addRow("Area FOV (수동)", self._double("area_custom_fov_deg", 0.1, 120.0, 0.1, 2))
        form.addRow("Area FOV 배율", self._double("area_output_fov_scale", 0.1, 10.0, 0.1, 2))
        return box

    def _build_search_speed_group(self) -> QGroupBox:
        box = QGroupBox("탐색 속도")
        form = QFormLayout(box)
        form.addRow("Line 밀도 배율", self._double("line_density_scale", 0.2, 3.0, 0.05, 2))
        form.addRow("Area 밀도 배율", self._double("area_density_scale", 0.2, 3.0, 0.05, 2))
        form.addRow("Line 탐색 가중치", self._double("search_speed_weight", 0.1, 10.0, 0.1, 2))
        form.addRow("Area 탐색 가중치", self._double("area_search_speed_weight", 0.1, 10.0, 0.1, 2))
        return box

    def _build_area_group(self) -> QGroupBox:
        box = QGroupBox("영역 / SEP")
        form = QFormLayout(box)
        form.addRow("Area SEP 배율", self._double("area_route_offset_scale", 0.0, 2.0, 0.05, 2))
        form.addRow("Area 검토 최대 구간(m)", self._double("enhanced_area_review_max_segment_m", 100.0, 10000.0, 100.0, 1))
        return box

    def _build_sweep_group(self) -> QGroupBox:
        box = QGroupBox("스윕")
        form = QFormLayout(box)
        form.addRow("Sweep 점 개수", self._int("sweep_line_interp_points", 2, 15, 1))
        return box

    def _build_wp_group(self) -> QGroupBox:
        box = QGroupBox("WP 간격")
        form = QFormLayout(box)
        form.addRow("UAV WP 간격(m)", self._double("uav_wp_interval_m", 100.0, 10000.0, 100.0, 1))
        form.addRow("LAH WP 간격(m)", self._double("lah_wp_interval_m", 100.0, 10000.0, 100.0, 1))
        form.addRow("Dubins 선회 반경(m)", self._double("dubins_turn_radius_m", 50.0, 5000.0, 10.0, 1))
        return box

    def _build_flyover_group(self) -> QGroupBox:
        box = QGroupBox("Flyover")
        form = QFormLayout(box)
        form.addRow(self._check("flyover_entry_offset", "진입 오프셋 WP를 Over로 처리"))
        form.addRow(self._check("flyover_dubins_prefix", "Area 임무 간 Dubins prefix를 Over로 처리"))
        form.addRow(self._check("flyover_all_wps", "모든 WP를 Over로 처리"))
        return box

    def _build_footer(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(8)

        btn_reload = QPushButton("불러오기")
        btn_reload.clicked.connect(self.reload_from_disk)
        btn_save = QPushButton("저장")
        btn_save.clicked.connect(self._save_now)
        btn_defaults = QPushButton("기본값")
        btn_defaults.clicked.connect(self._load_defaults)

        self._status = QLabel("")
        self._status.setObjectName("InfoBadge")
        self._path_label = QLabel(f"파일: {self._settings_path.name}")
        self._path_label.setObjectName("InfoBadge")

        row.addWidget(btn_reload)
        row.addWidget(btn_save)
        row.addWidget(btn_defaults)
        row.addStretch(1)
        row.addWidget(self._status)
        row.addWidget(self._path_label)
        return row

    def _double(self, key: str, min_v: float, max_v: float, step: float, decimals: int) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(min_v, max_v)
        widget.setSingleStep(step)
        widget.setDecimals(decimals)
        widget.valueChanged.connect(self._on_field_changed)
        self._widgets[key] = widget
        return widget

    def _int(self, key: str, min_v: int, max_v: int, step: int) -> QSpinBox:
        widget = QSpinBox()
        widget.setRange(min_v, max_v)
        widget.setSingleStep(step)
        widget.valueChanged.connect(self._on_field_changed)
        self._widgets[key] = widget
        return widget

    def _check(self, key: str, label: str) -> QCheckBox:
        widget = QCheckBox(label)
        widget.stateChanged.connect(self._on_field_changed)
        self._widgets[key] = widget
        return widget

    def _default_payload(self) -> Dict[str, Any]:
        return {
            "preset_key": "dubins_mode",
            "algo_key": "algo2",
            "values": {
                "search_speed_weight": 1.0,
                "area_search_speed_weight": 1.2,
                "line_density_scale": 1.18,
                "area_density_scale": 1.5,
                "fov_deg": 2.4,
                "line_custom_fov_deg": 2.4,
                "area_custom_fov_deg": 2.4,
                "area_output_fov_scale": 3.0,
                "enhanced_auto_fov_from_db": True,
                "area_route_offset_scale": 0.5,
                "enhanced_area_review_max_segment_m": 3000.0,
                "sweep_line_interp_points": 3,
                "min_sweep_len_m": 3.0,
                "uav_wp_interval_m": 2000.0,
                "lah_wp_interval_m": 3000.0,
                "dubins_turn_radius_m": 450.0,
            },
            "flyover": {
                "entry_offset": False,
                "dubins_prefix": False,
                "all_wps": False,
            },
        }

    def _normalize_payload(self, payload: Dict[str, Any] | None) -> Dict[str, Any]:
        base = deepcopy(self._default_payload())
        data = deepcopy(payload) if isinstance(payload, dict) else {}

        if isinstance(data.get("values"), dict):
            base["values"].update(data["values"])
        if isinstance(data.get("flyover"), dict):
            base["flyover"].update(data["flyover"])

        for key in ("preset_key", "algo_key"):
            if key in data:
                base[key] = data[key]

        values = base["values"]
        line_fov = self._as_float(
            values.get("line_custom_fov_deg", values.get("fov_deg", 2.4)),
            2.4,
        )
        area_fov = self._as_float(
            values.get("area_custom_fov_deg", values.get("fov_deg", line_fov)),
            line_fov,
        )
        values["line_custom_fov_deg"] = line_fov
        values["area_custom_fov_deg"] = area_fov
        values["fov_deg"] = line_fov
        values["search_speed_weight"] = self._as_float(values.get("search_speed_weight", 1.0), 1.0)
        values["area_search_speed_weight"] = self._as_float(values.get("area_search_speed_weight", 1.2), 1.2)
        values["line_density_scale"] = self._as_float(values.get("line_density_scale", 1.18), 1.18)
        values["area_density_scale"] = self._as_float(values.get("area_density_scale", 1.5), 1.5)
        values["area_output_fov_scale"] = self._as_float(values.get("area_output_fov_scale", 3.0), 3.0)
        values["area_route_offset_scale"] = self._as_float(values.get("area_route_offset_scale", 0.5), 0.5)
        values["enhanced_area_review_max_segment_m"] = self._as_float(
            values.get("enhanced_area_review_max_segment_m", 3000.0),
            3000.0,
        )
        try:
            values["sweep_line_interp_points"] = max(2, int(float(values.get("sweep_line_interp_points", 3))))
        except Exception:
            values["sweep_line_interp_points"] = 3
        values["min_sweep_len_m"] = self._as_float(values.get("min_sweep_len_m", 3.0), 3.0)
        values["uav_wp_interval_m"] = self._as_float(values.get("uav_wp_interval_m", 2000.0), 2000.0)
        values["lah_wp_interval_m"] = self._as_float(values.get("lah_wp_interval_m", 3000.0), 3000.0)
        values["dubins_turn_radius_m"] = self._as_float(values.get("dubins_turn_radius_m", 450.0), 450.0)
        values["enhanced_auto_fov_from_db"] = bool(values.get("enhanced_auto_fov_from_db", True))
        flyover = base["flyover"]
        flyover["entry_offset"] = bool(flyover.get("entry_offset", False))
        flyover["dubins_prefix"] = bool(flyover.get("dubins_prefix", False))
        flyover["all_wps"] = bool(flyover.get("all_wps", False))
        base["preset_key"] = "dubins_mode"
        return base

    @staticmethod
    def _as_float(value: Any, default: float) -> float:
        try:
            return float(value)
        except Exception:
            return float(default)

    def reload_from_disk(self) -> None:
        try:
            payload = json.loads(self._settings_path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
        self._payload = self._normalize_payload(payload)
        self._apply_payload_to_widgets(self._payload)
        self._status.setText("불러옴")

    def _apply_payload_to_widgets(self, payload: Dict[str, Any]) -> None:
        self._building = True
        try:
            values = payload.get("values") if isinstance(payload.get("values"), dict) else {}
            flyover = payload.get("flyover") if isinstance(payload.get("flyover"), dict) else {}

            self._widgets["preset_key"].setCurrentIndex(0)
            self._widgets["fov_mode"].setCurrentIndex(0 if bool(values.get("enhanced_auto_fov_from_db", True)) else 1)
            self._widgets["line_custom_fov_deg"].setValue(self._as_float(values.get("line_custom_fov_deg", values.get("fov_deg", 2.4)), 2.4))
            self._widgets["area_custom_fov_deg"].setValue(self._as_float(values.get("area_custom_fov_deg", values.get("fov_deg", 2.4)), 2.4))
            self._widgets["search_speed_weight"].setValue(self._as_float(values.get("search_speed_weight", 1.0), 1.0))
            self._widgets["area_search_speed_weight"].setValue(self._as_float(values.get("area_search_speed_weight", 1.2), 1.2))
            self._widgets["line_density_scale"].setValue(self._as_float(values.get("line_density_scale", 1.18), 1.18))
            self._widgets["area_density_scale"].setValue(self._as_float(values.get("area_density_scale", 1.5), 1.5))
            self._widgets["area_output_fov_scale"].setValue(self._as_float(values.get("area_output_fov_scale", 3.0), 3.0))
            self._widgets["area_route_offset_scale"].setValue(self._as_float(values.get("area_route_offset_scale", 0.5), 0.5))
            self._widgets["enhanced_area_review_max_segment_m"].setValue(
                self._as_float(values.get("enhanced_area_review_max_segment_m", 3000.0), 3000.0)
            )
            self._widgets["sweep_line_interp_points"].setValue(
                max(2, int(self._as_float(values.get("sweep_line_interp_points", 3), 3.0)))
            )
            self._widgets["uav_wp_interval_m"].setValue(self._as_float(values.get("uav_wp_interval_m", 2000.0), 2000.0))
            self._widgets["lah_wp_interval_m"].setValue(self._as_float(values.get("lah_wp_interval_m", 3000.0), 3000.0))
            self._widgets["dubins_turn_radius_m"].setValue(self._as_float(values.get("dubins_turn_radius_m", 450.0), 450.0))
            self._widgets["flyover_entry_offset"].setChecked(bool(flyover.get("entry_offset", False)))
            self._widgets["flyover_dubins_prefix"].setChecked(bool(flyover.get("dubins_prefix", False)))
            self._widgets["flyover_all_wps"].setChecked(bool(flyover.get("all_wps", False)))
            self._sync_enabled_state()
        finally:
            self._building = False

    def _current_payload(self) -> Dict[str, Any]:
        payload = deepcopy(self._payload if isinstance(self._payload, dict) else self._default_payload())
        values = payload.get("values")
        if not isinstance(values, dict):
            values = {}
            payload["values"] = values

        line_custom_fov = float(self._widgets["line_custom_fov_deg"].value())
        area_custom_fov = float(self._widgets["area_custom_fov_deg"].value())
        values["search_speed_weight"] = float(self._widgets["search_speed_weight"].value())
        values["area_search_speed_weight"] = float(self._widgets["area_search_speed_weight"].value())
        values["line_density_scale"] = float(self._widgets["line_density_scale"].value())
        values["area_density_scale"] = float(self._widgets["area_density_scale"].value())
        values["line_custom_fov_deg"] = line_custom_fov
        values["area_custom_fov_deg"] = area_custom_fov
        values["fov_deg"] = line_custom_fov
        values["area_output_fov_scale"] = float(self._widgets["area_output_fov_scale"].value())
        values["area_route_offset_scale"] = float(self._widgets["area_route_offset_scale"].value())
        values["enhanced_area_review_max_segment_m"] = float(self._widgets["enhanced_area_review_max_segment_m"].value())
        values["sweep_line_interp_points"] = int(self._widgets["sweep_line_interp_points"].value())
        values["uav_wp_interval_m"] = float(self._widgets["uav_wp_interval_m"].value())
        values["lah_wp_interval_m"] = float(self._widgets["lah_wp_interval_m"].value())
        values["dubins_turn_radius_m"] = float(self._widgets["dubins_turn_radius_m"].value())
        values["enhanced_auto_fov_from_db"] = self._widgets["fov_mode"].currentData() == "auto"

        payload["preset_key"] = "dubins_mode"
        if not isinstance(payload.get("flyover"), dict):
            payload["flyover"] = {"entry_offset": False, "dubins_prefix": False, "all_wps": False}
        flyover = payload["flyover"]
        flyover["entry_offset"] = bool(self._widgets["flyover_entry_offset"].isChecked())
        flyover["dubins_prefix"] = bool(self._widgets["flyover_dubins_prefix"].isChecked())
        flyover["all_wps"] = bool(self._widgets["flyover_all_wps"].isChecked())
        if not payload.get("algo_key"):
            payload["algo_key"] = "algo2"
        return self._normalize_payload(payload)

    def _save_now(self) -> None:
        payload = self._current_payload()
        self._settings_path.parent.mkdir(parents=True, exist_ok=True)
        self._settings_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self._payload = payload
        self._status.setText("저장됨")
        if callable(self._on_apply):
            self._on_apply(deepcopy(payload))

    def _load_defaults(self) -> None:
        self._payload = self._normalize_payload(self._default_payload())
        self._apply_payload_to_widgets(self._payload)
        self._status.setText("기본값 불러옴")

    def _on_fov_mode_changed(self) -> None:
        if self._building:
            return
        self._sync_enabled_state()
        self._status.setText("수정됨")

    def _on_field_changed(self) -> None:
        if self._building:
            return
        self._status.setText("수정됨")

    def _auto_area_review_width_m(self) -> float:
        try:
            return float(load_fov_db_max_width(0.0))
        except Exception:
            return 0.0

    def _update_area_width_hint(self, custom_enabled: bool) -> None:
        if self._area_width_hint is None:
            return
        if custom_enabled:
            self._area_width_hint.setText(
                "수동 모드에서는 위에 입력한 Area 검토 최대 구간 값을 사용합니다."
            )
            return

        auto_width = self._auto_area_review_width_m()
        if auto_width > 0.0:
            self._area_width_hint.setText(
                f"자동(DB) 모드에서는 resource/db/fov_db.csv 의 최대 width 값 {auto_width:.1f} m 를 사용합니다."
            )
        else:
            self._area_width_hint.setText(
                "자동(DB) 모드에서 DB width 를 읽지 못해 fallback 런타임 값을 사용합니다."
            )

    def _sync_enabled_state(self) -> None:
        custom_enabled = self._widgets["fov_mode"].currentData() == "custom"
        self._widgets["line_custom_fov_deg"].setEnabled(custom_enabled)
        self._widgets["area_custom_fov_deg"].setEnabled(custom_enabled)
        self._widgets["enhanced_area_review_max_segment_m"].setEnabled(custom_enabled)
        self._widgets["enhanced_area_review_max_segment_m"].setToolTip(
            "이 항목은 수동 모드에서만 수정할 수 있습니다. 자동(DB) 모드에서는 resource/db/fov_db.csv 의 최대 width 값을 사용합니다."
        )
        self._update_area_width_hint(custom_enabled)
