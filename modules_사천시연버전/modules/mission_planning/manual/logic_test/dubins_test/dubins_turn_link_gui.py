from __future__ import annotations

import math
import re
import sys
from pathlib import Path

from modules.mission_planning._paths import project_root

THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = project_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from modules.common.qt_env import ensure_qt_platform

ensure_qt_platform()

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.patches import Circle
from matplotlib.ticker import MultipleLocator
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSizePolicy,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from modules.common.gui_style import load_shared_stylesheet, position_window_from_env
from modules.common.turn_radius import (
    REFERENCE_TURN_RADIUS_TABLE_MPS,
    interpolate_reference_turn_radius,
)
from modules.mission_planning.manual.logic_test.dubins_test.dubins_turn_link_logic import (
    DubinsTurnLinkResult,
    Point2D,
    compute_turn_link,
    format_result,
)


VIEW_SIZE_M = 3000.0
REF_SPEED_RADIUS_TABLE = REFERENCE_TURN_RADIUS_TABLE_MPS


def _parse_point_text(raw: str) -> Point2D:
    parts = [part for part in re.split(r"[\s,;/]+", raw.strip()) if part]
    if len(parts) != 2:
        raise ValueError("Point format must be 'x, y'.")
    try:
        return Point2D(float(parts[0]), float(parts[1]))
    except ValueError as exc:
        raise ValueError("Coordinates must be numeric.") from exc


def _format_point_text(point: Point2D) -> str:
    return f"{point.x:.1f}, {point.y:.1f}"


def _interpolate_reference_radius(speed_mps: float) -> float:
    return float(interpolate_reference_turn_radius(speed_mps))


class DubinsTurnLinkWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Dubins Turn Link Tester")
        self.resize(1540, 940)

        self.input_mode = "outbound"
        self.current_result: DubinsTurnLinkResult | None = None

        self.prev_start_edit = QLineEdit()
        self.prev_end_edit = QLineEdit()
        self.next_start_edit = QLineEdit()
        self.next_end_edit = QLineEdit()
        self.result_box = QPlainTextEdit()
        self.result_box.setReadOnly(True)

        for edit in (
            self.prev_start_edit,
            self.prev_end_edit,
            self.next_start_edit,
            self.next_end_edit,
        ):
            edit.setPlaceholderText("x, y")
            edit.returnPressed.connect(self.handle_enter_action)
            edit.textChanged.connect(self._on_inputs_changed)

        self.radius_mode_combo = QComboBox()
        self.radius_mode_combo.addItems(
            [
                "Reference table",
                "Formula  R = V^2 / (g tan|phi|)",
                "Manual radius",
            ]
        )
        self.speed_spin = self._make_spinbox(10.0, 90.0, 40.0, 1.0, " m/s")
        self.bank_spin = self._make_spinbox(1.0, 89.0, 30.0, 1.0, " deg")
        self.radius_spin = self._make_spinbox(1.0, 100000.0, 450.0, 10.0, " m")
        self.sample_step_spin = self._make_spinbox(1.0, 1000.0, 5.0, 1.0, " m")

        self.mode_label = QLabel()
        self.mode_label.setWordWrap(True)
        self.radius_preview_label = QLabel()
        self.radius_preview_label.setWordWrap(True)

        for widget in (
            self.radius_mode_combo,
            self.speed_spin,
            self.bank_spin,
            self.radius_spin,
            self.sample_step_spin,
        ):
            if isinstance(widget, QComboBox):
                widget.currentIndexChanged.connect(self._on_config_changed)
            else:
                widget.valueChanged.connect(self._on_config_changed)

        self.figure = Figure(figsize=(8, 6), tight_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.canvas.setFocusPolicy(Qt.ClickFocus)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.ax = self.figure.add_subplot(111)

        self._mpl_click_cid = self.canvas.mpl_connect("button_press_event", self._on_plot_click)
        self._mpl_key_cid = self.canvas.mpl_connect("key_press_event", self._on_plot_key_press)

        self._build_ui()
        self._set_input_mode("outbound")
        self._update_radius_preview()
        self._draw_scene()
        self.statusBar().showMessage("Left-click P1/P2, press Enter, then click P3/P4.", 6000)

    @staticmethod
    def _make_spinbox(
        minimum: float,
        maximum: float,
        value: float,
        step: float,
        suffix: str,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setDecimals(3)
        spin.setRange(minimum, maximum)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _build_ui(self) -> None:
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(12, 12, 12, 12)
        left_layout.setSpacing(10)

        mode_box = QGroupBox("Mouse Input")
        mode_layout = QVBoxLayout(mode_box)
        mode_layout.addWidget(self.mode_label)
        left_layout.addWidget(mode_box)

        segment_box = QGroupBox("Points")
        segment_form = QFormLayout(segment_box)
        segment_form.addRow("Outbound P1", self.prev_start_edit)
        segment_form.addRow("Outbound P2", self.prev_end_edit)
        segment_form.addRow("Inbound P3", self.next_start_edit)
        segment_form.addRow("Inbound P4", self.next_end_edit)
        left_layout.addWidget(segment_box)

        option_box = QGroupBox("Turn Radius")
        option_form = QFormLayout(option_box)
        option_form.addRow("Radius source", self.radius_mode_combo)
        option_form.addRow("Path family", QLabel("All Dubins shortest"))
        option_form.addRow("Speed", self.speed_spin)
        option_form.addRow("Bank angle |phi|", self.bank_spin)
        option_form.addRow("Manual radius", self.radius_spin)
        option_form.addRow("Radius preview", self.radius_preview_label)
        left_layout.addWidget(option_box)

        sample_box = QGroupBox("Sampling")
        sample_form = QFormLayout(sample_box)
        sample_form.addRow("Sample step", self.sample_step_spin)
        left_layout.addWidget(sample_box)

        button_row = QHBoxLayout()
        sample_button = QPushButton("Load example")
        sample_button.clicked.connect(self.fill_example)
        reset_button = QPushButton("Clear")
        reset_button.clicked.connect(self.clear_inputs)
        enter_button = QPushButton("Enter / Next")
        enter_button.clicked.connect(self.handle_enter_action)
        compute_button = QPushButton("Compute now")
        compute_button.clicked.connect(self.compute_result)
        compute_button.setDefault(True)
        button_row.addWidget(sample_button)
        button_row.addWidget(reset_button)
        button_row.addWidget(enter_button)
        button_row.addWidget(compute_button)
        left_layout.addLayout(button_row)

        result_box = QGroupBox("Result")
        result_layout = QVBoxLayout(result_box)
        result_layout.addWidget(self.result_box)
        left_layout.addWidget(result_box, 1)

        plot_panel = QWidget()
        plot_layout = QVBoxLayout(plot_panel)
        plot_layout.setContentsMargins(12, 12, 12, 12)
        plot_layout.setSpacing(8)
        plot_layout.addWidget(self.toolbar)
        plot_layout.addWidget(self.canvas, 1)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(plot_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([440, 1020])

        central = QWidget()
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.addWidget(splitter)
        self.setCentralWidget(central)

    def fill_example(self) -> None:
        self.prev_start_edit.setText("-900, -700")
        self.prev_end_edit.setText("-250, -700")
        self.next_start_edit.setText("650, -250")
        self.next_end_edit.setText("650, 950")
        self.speed_spin.setValue(40.0)
        self.radius_mode_combo.setCurrentIndex(0)
        self.sample_step_spin.setValue(5.0)
        self._set_input_mode("inbound")
        self.compute_result()

    def clear_inputs(self) -> None:
        for edit in (
            self.prev_start_edit,
            self.prev_end_edit,
            self.next_start_edit,
            self.next_end_edit,
        ):
            edit.blockSignals(True)
            edit.clear()
            edit.blockSignals(False)
        self.result_box.clear()
        self.current_result = None
        self._set_input_mode("outbound")
        self._draw_scene()
        self.statusBar().showMessage("Left-click P1/P2, press Enter, then click P3/P4.", 5000)

    def handle_enter_action(self) -> None:
        if self.input_mode == "outbound":
            if not self._has_segment_points("outbound"):
                self.statusBar().showMessage("Outbound mode needs two points: P1, P2.", 4000)
                return
            self._set_input_mode("inbound")
            self._draw_scene()
            self.statusBar().showMessage("Inbound mode active. Click P3/P4, then press Enter.", 5000)
            return

        if not self._has_segment_points("inbound"):
            self.statusBar().showMessage("Inbound mode needs two points: P3, P4.", 4000)
            return

        self.compute_result()

    def compute_result(self) -> None:
        try:
            prev_start = _parse_point_text(self.prev_start_edit.text())
            prev_end = _parse_point_text(self.prev_end_edit.text())
            next_start = _parse_point_text(self.next_start_edit.text())
            next_end = _parse_point_text(self.next_end_edit.text())
            radius_m, radius_desc = self._resolve_turn_radius()
            result = compute_turn_link(
                prev_start=prev_start,
                prev_end=prev_end,
                next_start=next_start,
                next_end=next_end,
                radius_m=radius_m,
                sample_step_m=self.sample_step_spin.value(),
                allow_ccc=True,
                path_policy="all_shortest",
            )
        except Exception as exc:
            QMessageBox.critical(self, "Compute failed", str(exc))
            return

        self.current_result = result
        path_mode_desc = "Path policy: All Dubins shortest"
        self.result_box.setPlainText(f"{radius_desc}\n{path_mode_desc}\n\n{format_result(result)}")
        self._draw_scene()
        self.statusBar().showMessage(
            f"Computed {result.dubins_type} | total={result.total_length_m:.2f} m | radius={radius_m:.1f} m",
            6000,
        )

    def _resolve_turn_radius(self) -> tuple[float, str]:
        mode = self.radius_mode_combo.currentText()
        speed_mps = self.speed_spin.value()
        bank_deg = abs(self.bank_spin.value())

        if mode.startswith("Reference"):
            radius_m = _interpolate_reference_radius(speed_mps)
            desc = (
                "Radius source: reference table "
                f"(30 m/s -> 340 m, 40 m/s -> 450 m, 50 m/s -> 560 m), "
                f"speed={speed_mps:.1f} m/s => radius={radius_m:.1f} m"
            )
            return radius_m, desc

        if mode.startswith("Formula"):
            tan_phi = math.tan(math.radians(bank_deg))
            if abs(tan_phi) <= 1e-9:
                raise ValueError("Bank angle must be non-zero in formula mode.")
            radius_m = speed_mps * speed_mps / (9.81 * tan_phi)
            desc = (
                "Radius source: formula "
                f"R = V^2 / (g tan|phi|), V={speed_mps:.1f} m/s, |phi|={bank_deg:.1f} deg "
                f"=> radius={radius_m:.1f} m"
            )
            return radius_m, desc

        radius_m = self.radius_spin.value()
        desc = f"Radius source: manual radius = {radius_m:.1f} m"
        return radius_m, desc

    def _update_radius_preview(self) -> None:
        try:
            radius_m, desc = self._resolve_turn_radius()
        except Exception as exc:
            self.radius_preview_label.setText(str(exc))
            return
        self.radius_spin.setEnabled(self.radius_mode_combo.currentText().startswith("Manual"))
        self.radius_preview_label.setText(f"{radius_m:.1f} m\n{desc}")

    def _has_segment_points(self, mode: str) -> bool:
        first, second = self._segment_edits(mode)
        return bool(first.text().strip()) and bool(second.text().strip())

    def _segment_edits(self, mode: str) -> tuple[QLineEdit, QLineEdit]:
        if mode == "outbound":
            return self.prev_start_edit, self.prev_end_edit
        return self.next_start_edit, self.next_end_edit

    def _set_input_mode(self, mode: str) -> None:
        self.input_mode = mode
        if mode == "outbound":
            text = (
                "Mode 1: click P1 and P2 on the map, then press Enter.\n"
                "Left click adds a point. Right click removes the last point."
            )
        else:
            text = (
                "Mode 2: click P3 and P4 on the map, then press Enter to compute.\n"
                "Left click adds a point. Right click removes the last point."
            )
        self.mode_label.setText(text)

    def _on_inputs_changed(self) -> None:
        self.current_result = None
        self.result_box.clear()
        self._draw_scene()

    def _on_config_changed(self) -> None:
        self.current_result = None
        self.result_box.clear()
        self._update_radius_preview()
        self._draw_scene()

    def _on_plot_key_press(self, event) -> None:
        if str(getattr(event, "key", "")).lower() in {"enter", "return"}:
            self.handle_enter_action()

    def _on_plot_click(self, event) -> None:
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        if getattr(self.toolbar, "mode", ""):
            return

        self.canvas.setFocus()
        if event.button == 1:
            self._append_point(Point2D(float(event.xdata), float(event.ydata)))
        elif event.button == 3:
            self._remove_last_point()
        else:
            return

        self.current_result = None
        self.result_box.clear()
        self._draw_scene()

    def _append_point(self, point: Point2D) -> None:
        first_edit, second_edit = self._segment_edits(self.input_mode)
        if not first_edit.text().strip():
            first_edit.blockSignals(True)
            first_edit.setText(_format_point_text(point))
            first_edit.blockSignals(False)
            self.statusBar().showMessage(f"{self.input_mode.capitalize()} point 1 set. Click the next point.", 3000)
            return
        if not second_edit.text().strip():
            second_edit.blockSignals(True)
            second_edit.setText(_format_point_text(point))
            second_edit.blockSignals(False)
            if self.input_mode == "outbound":
                self.statusBar().showMessage("Outbound points ready. Press Enter for inbound mode.", 4000)
            else:
                self.statusBar().showMessage("Inbound points ready. Press Enter to compute.", 4000)
            return

        if self.input_mode == "outbound":
            self.statusBar().showMessage("Outbound already has P1/P2. Press Enter to switch mode.", 4000)
        else:
            self.statusBar().showMessage("Inbound already has P3/P4. Press Enter to compute.", 4000)

    def _remove_last_point(self) -> None:
        ordered_edits = [
            (self.next_end_edit, "inbound"),
            (self.next_start_edit, "inbound"),
            (self.prev_end_edit, "outbound"),
            (self.prev_start_edit, "outbound"),
        ]
        for edit, mode in ordered_edits:
            if edit.text().strip():
                edit.blockSignals(True)
                edit.clear()
                edit.blockSignals(False)
                self._set_input_mode(mode)
                self.statusBar().showMessage("Removed the last point.", 2500)
                return

    def _safe_point(self, edit: QLineEdit) -> Point2D | None:
        text = edit.text().strip()
        if not text:
            return None
        try:
            return _parse_point_text(text)
        except Exception:
            return None

    def _current_points(self) -> dict[str, Point2D]:
        points: dict[str, Point2D] = {}
        mapping = {
            "P1": self.prev_start_edit,
            "P2": self.prev_end_edit,
            "P3": self.next_start_edit,
            "P4": self.next_end_edit,
        }
        for label, edit in mapping.items():
            point = self._safe_point(edit)
            if point is not None:
                points[label] = point
        return points

    def _draw_scene(self) -> None:
        points = self._current_points()
        result = self.current_result

        self.ax.clear()
        self._style_axes(result)
        self._draw_instruction_overlay()

        if "P1" in points and "P2" in points:
            self.ax.plot(
                [points["P1"].x, points["P2"].x],
                [points["P1"].y, points["P2"].y],
                color="#2563eb",
                linewidth=2.6,
                marker="o",
                label="Outbound segment",
            )
            self._draw_heading_arrow(points["P2"], points["P1"], points["P2"], "#2563eb")
        elif "P1" in points:
            self.ax.scatter([points["P1"].x], [points["P1"].y], color="#2563eb", s=70, zorder=4)

        if "P3" in points and "P4" in points:
            self.ax.plot(
                [points["P3"].x, points["P4"].x],
                [points["P3"].y, points["P4"].y],
                color="#ea580c",
                linewidth=2.6,
                marker="o",
                label="Inbound segment",
            )
            self._draw_heading_arrow(points["P3"], points["P3"], points["P4"], "#ea580c")
        elif "P3" in points:
            self.ax.scatter([points["P3"].x], [points["P3"].y], color="#ea580c", s=70, zorder=4)

        for label, point in points.items():
            self.ax.annotate(
                label,
                (point.x, point.y),
                textcoords="offset points",
                xytext=(8, 8),
                fontsize=10,
                color="#111827",
            )

        if result is not None and {"P1", "P2", "P3", "P4"} <= set(points.keys()):
            self._draw_turn_circles(result)
            path_x = [point.x for point in result.path_points]
            path_y = [point.y for point in result.path_points]
            self.ax.plot(path_x, path_y, color="#16a34a", linewidth=2.8, label="Transition path")
            self.ax.scatter(
                [result.exit_point.x, result.entry_point.x],
                [result.exit_point.y, result.entry_point.y],
                color="#16a34a",
                s=90,
                marker="s",
                zorder=5,
                label="Link points",
            )
            self.ax.annotate("[1]", (result.exit_point.x, result.exit_point.y), textcoords="offset points", xytext=(8, 8))
            self.ax.annotate("[2]", (result.entry_point.x, result.entry_point.y), textcoords="offset points", xytext=(8, 8))

        all_points = list(points.values())
        if result is not None:
            all_points.extend(result.path_points)
            all_points.append(result.exit_point)
            all_points.append(result.entry_point)
        self._apply_fixed_view(all_points)

        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(loc="upper right")
        self.canvas.draw_idle()

    def _style_axes(self, result: DubinsTurnLinkResult | None) -> None:
        title = "Dubins Turn Link | 3 km x 3 km view"
        if result is not None:
            title += f" | {result.dubins_type}"
        self.ax.set_title(title)
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, which="major", linestyle="--", alpha=0.45)
        self.ax.grid(True, which="minor", linestyle=":", alpha=0.18)
        self.ax.axhline(0.0, color="#d1d5db", linewidth=1.0)
        self.ax.axvline(0.0, color="#d1d5db", linewidth=1.0)
        self.ax.xaxis.set_major_locator(MultipleLocator(500.0))
        self.ax.yaxis.set_major_locator(MultipleLocator(500.0))
        self.ax.xaxis.set_minor_locator(MultipleLocator(250.0))
        self.ax.yaxis.set_minor_locator(MultipleLocator(250.0))

    def _draw_instruction_overlay(self) -> None:
        if self.input_mode == "outbound":
            mode_text = "Click P1/P2, then press Enter"
        else:
            mode_text = "Click P3/P4, then press Enter to compute"
        self.ax.text(
            0.02,
            0.98,
            f"{mode_text}\nLeft click: add point | Right click: remove last point",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=10,
            bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "#d1d5db"},
        )

    def _apply_fixed_view(self, points: list[Point2D]) -> None:
        half_span = VIEW_SIZE_M / 2.0
        if points:
            min_x = min(point.x for point in points)
            max_x = max(point.x for point in points)
            min_y = min(point.y for point in points)
            max_y = max(point.y for point in points)
            center_x = 0.5 * (min_x + max_x)
            center_y = 0.5 * (min_y + max_y)
        else:
            center_x = 0.0
            center_y = 0.0
        self.ax.set_xlim(center_x - half_span, center_x + half_span)
        self.ax.set_ylim(center_y - half_span, center_y + half_span)

    def _draw_turn_circles(self, result: DubinsTurnLinkResult) -> None:
        for idx, (seg_type, center) in enumerate(zip(result.curve_types, result.curve_centers), start=1):
            color = "#2563eb" if seg_type == "L" else "#dc2626"
            patch = Circle(
                (center.x, center.y),
                result.turn_radius_m,
                fill=False,
                linestyle=":",
                linewidth=1.5,
                edgecolor=color,
                alpha=0.65,
            )
            self.ax.add_patch(patch)
            self.ax.annotate(
                f"{seg_type}{idx}",
                (center.x, center.y),
                textcoords="offset points",
                xytext=(6, 6),
                fontsize=9,
                color=color,
            )

    def _draw_heading_arrow(
        self,
        anchor: Point2D,
        line_start: Point2D,
        line_end: Point2D,
        color: str,
    ) -> None:
        dx = line_end.x - line_start.x
        dy = line_end.y - line_start.y
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return
        scale = min(120.0, max(60.0, length * 0.18)) / length
        self.ax.arrow(
            anchor.x,
            anchor.y,
            dx * scale,
            dy * scale,
            width=0.0,
            head_width=24.0,
            head_length=36.0,
            fc=color,
            ec=color,
            length_includes_head=True,
            alpha=0.88,
            zorder=6,
        )

    def keyPressEvent(self, event) -> None:
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            self.handle_enter_action()
            event.accept()
            return
        super().keyPressEvent(event)


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    load_shared_stylesheet(app, PROJECT_ROOT)
    window = DubinsTurnLinkWindow()
    window.show()
    position_window_from_env(app, window)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
