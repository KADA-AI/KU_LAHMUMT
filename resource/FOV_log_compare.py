from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
import re

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
except Exception:
    tk = None
    filedialog = None
    messagebox = None
    ttk = None

import numpy as np
import tifffile
from pyproj import CRS, Transformer

from FOV_compare import (
    FOOTPRINT_SPEC_CORNER_DEFINITION,
    FOV_INTERPRETATIONS,
    GeoPoint,
    GeoPointInput,
    LocalFrame,
    build_camera_axes,
    build_polygon_result,
    calculate_fov_components,
    ground_distance_m,
    intersect_ray_with_dem,
    normalize,
    order_corners_by_camera_axes,
    resolve_ground_point,
    resolve_existing_path,
    resolve_ray_step_m,
)

import matplotlib.pyplot as plt
from matplotlib import font_manager


SCRIPT_DIR = Path(__file__).resolve().parent
LOG_PATH = SCRIPT_DIR / "log_temp" / "0401.json"
OUTPUT_DIR = SCRIPT_DIR / "log_temp" / "fov_compare_outputs"
AIRCRAFT_ID = 4
DEFAULT_LOG_PATHS = [
    SCRIPT_DIR / "log_temp" / "0401.json",
    SCRIPT_DIR / "log_temp" / "log_0401_agent_status_sim.jsonl",
]
ASPECT_RATIO = 16.0 / 9.0
ROLL_DEG = 0.0
RAY_STEP_M = 20.0
MAX_DISTANCE_M = 50000.0
FOV_INTERPRETATION = "auto"
PREDICTED_GROUND_ALTITUDE_MODE = "dem"
ACTUAL_GROUND_ALTITUDE_MODE = "auto"
SHOW_PLOTS = True
SAVE_FIGURES = True
RANDOM_SNAPSHOT_SAMPLE_COUNT = 10
RANDOM_SNAPSHOT_SEED = 42
RANDOM_SNAPSHOT_DEM_NAME: str | None = None
FOV_DIAGNOSTIC_SAMPLE_LIMIT = 80
LOG_OPTION_NONE = "없음"

DEM_PATHS = [
    Path("n37_e126_1arc_v3.tif"),
    Path("n37_e127_1arc_v3.tif"),
    Path("n37_e128_1arc_v3.tif"),
    Path("n38_e126_1arc_v3.tif"),
    Path("n38_e127_1arc_v3.tif"),
    Path("n38_e128_1arc_v3.tif"),
    Path("Hongik_48km.tif"),
    Path("Inje_48km.tif"),
    Path("Jipo_48km.tif"),
]
CORNER_LABELS_KO = ("C1 좌상", "C2 우상", "C3 우하", "C4 좌하")


def configure_matplotlib_for_korean() -> str:
    candidate_fonts = [
        "Malgun Gothic",
        "NanumGothic",
        "Noto Sans CJK KR",
        "AppleGothic",
        "Arial Unicode MS",
    ]
    available_fonts = {font.name for font in font_manager.fontManager.ttflist}
    selected_font = ""
    for candidate in candidate_fonts:
        if candidate in available_fonts:
            selected_font = candidate
            break

    if selected_font:
        plt.rcParams["font.family"] = selected_font
    plt.rcParams["axes.unicode_minus"] = False
    return selected_font or "default"


@dataclass(frozen=True)
class AnalysisConfig:
    log_path: Path
    aircraft_id: int
    output_prefix: str

    @property
    def run_label(self) -> str:
        return f"UAV {self.aircraft_id}"


@dataclass(frozen=True)
class AircraftLogOption:
    aircraft_id: int
    valid_frame_count: int

    @property
    def display_label(self) -> str:
        return f"UAV {self.aircraft_id} ({self.valid_frame_count} valid frames)"


@dataclass(frozen=True)
class LoggedFootprintFrame:
    sample_index: int
    log_frame_index: int
    timestamp: int
    time_s: float
    uav: GeoPoint
    center: GeoPointInput
    actual_corners: tuple[GeoPointInput, GeoPointInput, GeoPointInput, GeoPointInput]
    fov_deg: float


@dataclass(frozen=True)
class FootprintComparisonSample:
    sample_index: int
    log_frame_index: int
    timestamp: int
    time_s: float
    fov_deg: float
    uav: GeoPoint
    actual_area_m2: float
    predicted_area_m2: float
    area_error_m2: float
    area_error_pct: float
    centroid_error_m: float
    corner_errors_m: tuple[float, float, float, float]
    mean_corner_error_m: float
    max_corner_error_m: float
    actual_center: GeoPoint
    predicted_center: GeoPoint
    actual_corners: tuple[GeoPoint, GeoPoint, GeoPoint, GeoPoint]
    predicted_corners: tuple[GeoPoint, GeoPoint, GeoPoint, GeoPoint]


@dataclass
class DemComparisonResult:
    dem_name: str
    dem_path: Path
    total_frames: int
    compared_samples: list[FootprintComparisonSample]
    skipped_frames: int
    skip_reasons: dict[str, int]

    def coverage_ratio(self) -> float:
        if self.total_frames <= 0:
            return 0.0
        return len(self.compared_samples) / self.total_frames

    def summary_row(self) -> dict[str, float | int | str]:
        if not self.compared_samples:
            return {
                "dem_name": self.dem_name,
                "dem_path": str(self.dem_path),
                "total_frames": self.total_frames,
                "compared_frames": 0,
                "coverage_ratio": 0.0,
                "area_mae_m2": float("nan"),
                "area_mape_pct": float("nan"),
                "area_mean_error_pct": float("nan"),
                "area_ratio_pct": float("nan"),
                "corner_mae_m": float("nan"),
                "centroid_mae_m": float("nan"),
                "max_corner_error_m": float("nan"),
            }

        area_abs = np.array([abs(sample.area_error_m2) for sample in self.compared_samples], dtype=float)
        area_pct = np.array([sample.area_error_pct for sample in self.compared_samples], dtype=float)
        area_pct_abs = np.array([abs(sample.area_error_pct) for sample in self.compared_samples], dtype=float)
        area_ratio_pct = np.array(
            [
                (sample.predicted_area_m2 / sample.actual_area_m2 * 100.0)
                if sample.actual_area_m2 > 0.0
                else float("nan")
                for sample in self.compared_samples
            ],
            dtype=float,
        )
        corner_mean = np.array([sample.mean_corner_error_m for sample in self.compared_samples], dtype=float)
        centroid = np.array([sample.centroid_error_m for sample in self.compared_samples], dtype=float)
        max_corner = np.array([sample.max_corner_error_m for sample in self.compared_samples], dtype=float)
        return {
            "dem_name": self.dem_name,
            "dem_path": str(self.dem_path),
            "total_frames": self.total_frames,
            "compared_frames": len(self.compared_samples),
            "coverage_ratio": self.coverage_ratio(),
            "area_mae_m2": float(np.mean(area_abs)),
            "area_mape_pct": float(np.mean(area_pct_abs)),
            "area_mean_error_pct": float(np.mean(area_pct)),
            "area_ratio_pct": float(np.mean(area_ratio_pct)),
            "corner_mae_m": float(np.mean(corner_mean)),
            "centroid_mae_m": float(np.mean(centroid)),
            "max_corner_error_m": float(np.max(max_corner)),
        }


class GeoTiffDemAnyCrs:
    def __init__(self, path: Path) -> None:
        self.path = resolve_existing_path(path)
        with tifffile.TiffFile(self.path) as tif:
            page = tif.pages[0]
            tags = page.tags
            self.data = page.asarray().astype(np.float64)
            scale = tags[33550].value
            tie = tags[33922].value
            self.pixel_width = float(scale[0])
            self.pixel_height = float(scale[1])
            self.x_min = float(tie[3])
            self.y_max = float(tie[4])
            self.nodata = float(tags[42113].value) if 42113 in tags else None
            self.crs = self._extract_crs(tags)

        if self.nodata is not None:
            self.data[self.data == self.nodata] = np.nan

        self.height, self.width = self.data.shape
        self.x_max = self.x_min + self.pixel_width * (self.width - 1)
        self.y_min = self.y_max - self.pixel_height * (self.height - 1)
        self.is_geographic = self.crs is None or self.crs.is_geographic
        self._to_raster = None
        if not self.is_geographic:
            self._to_raster = Transformer.from_crs(CRS.from_epsg(4326), self.crs, always_xy=True)

    def _extract_crs(self, tags) -> CRS | None:
        if 34735 not in tags:
            if abs(self.x_min) <= 180.0 and abs(self.y_max) <= 90.0:
                return CRS.from_epsg(4326)
            return None

        key_dir = tags[34735].value
        epsg_code = None
        for index in range(4, len(key_dir), 4):
            key_id = key_dir[index]
            value = key_dir[index + 3]
            if key_id == 3072:
                epsg_code = value
                break
            if key_id == 2048:
                epsg_code = value
        if epsg_code is None:
            if abs(self.x_min) <= 180.0 and abs(self.y_max) <= 90.0:
                return CRS.from_epsg(4326)
            return None
        return CRS.from_epsg(int(epsg_code))

    def _latlon_to_xy(self, lat: float, lon: float) -> tuple[float, float]:
        if self.is_geographic:
            return float(lon), float(lat)
        assert self._to_raster is not None
        x, y = self._to_raster.transform(float(lon), float(lat))
        return float(x), float(y)

    def contains(self, lat: float, lon: float) -> bool:
        x, y = self._latlon_to_xy(lat, lon)
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def latlon_to_rowcol(self, lat: float, lon: float) -> tuple[float, float]:
        x, y = self._latlon_to_xy(lat, lon)
        row = (self.y_max - y) / self.pixel_height
        col = (x - self.x_min) / self.pixel_width
        return row, col

    def sample_elevation(self, lat: float, lon: float) -> float:
        if not self.contains(lat, lon):
            raise ValueError(f"Point ({lat:.6f}, {lon:.6f}) is outside {self.path.name}")

        row, col = self.latlon_to_rowcol(lat, lon)
        row0 = int(np.floor(row))
        col0 = int(np.floor(col))
        row1 = min(row0 + 1, self.height - 1)
        col1 = min(col0 + 1, self.width - 1)
        row0 = max(row0, 0)
        col0 = max(col0, 0)

        fr = row - row0
        fc = col - col0

        values = np.array(
            [
                self.data[row0, col0],
                self.data[row0, col1],
                self.data[row1, col0],
                self.data[row1, col1],
            ],
            dtype=float,
        )
        weights = np.array(
            [
                (1.0 - fr) * (1.0 - fc),
                (1.0 - fr) * fc,
                fr * (1.0 - fc),
                fr * fc,
            ],
            dtype=float,
        )

        mask = np.isfinite(values) & (weights > 0.0)
        if not np.any(mask):
            raise ValueError(f"No valid DEM value around ({lat:.6f}, {lon:.6f})")
        return float(np.dot(values[mask], weights[mask]) / np.sum(weights[mask]))


def resolve_log_path(log_path: Path) -> Path:
    return resolve_existing_path(log_path)


def load_raw_log_frames(log_path: Path) -> list[dict]:
    resolved_path = resolve_log_path(log_path)
    suffix = resolved_path.suffix.lower()

    if suffix == ".jsonl":
        frames: list[dict] = []
        with resolved_path.open("r", encoding="utf-8") as handle:
            for line_index, line in enumerate(handle, start=1):
                text = line.strip()
                if not text:
                    continue
                payload = json.loads(text)
                if not isinstance(payload, dict):
                    raise ValueError(f"Expected object in {resolved_path.name}:{line_index}")
                frames.append(payload)
        return frames

    if suffix == ".json":
        with resolved_path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        if isinstance(payload, list):
            return payload
        if isinstance(payload, dict):
            for key in ("frames", "items", "records"):
                value = payload.get(key)
                if isinstance(value, list):
                    return value
            return [payload]
        raise ValueError(f"Unsupported JSON payload in {resolved_path.name}")

    raise ValueError(f"Unsupported log format: {resolved_path.suffix}")


def frame_timestamp(raw_frame: dict) -> int | None:
    if raw_frame.get("timestamp") is not None:
        return int(raw_frame["timestamp"])
    raw_payload = raw_frame.get("raw")
    if isinstance(raw_payload, dict) and raw_payload.get("timestamp") is not None:
        return int(raw_payload["timestamp"])
    return None


def frame_agent_states(raw_frame: dict) -> list[dict]:
    agent_state_list = raw_frame.get("agentStateList")
    if isinstance(agent_state_list, list):
        return agent_state_list
    raw_payload = raw_frame.get("raw")
    if isinstance(raw_payload, dict):
        nested = raw_payload.get("agentStateList")
        if isinstance(nested, list):
            return nested
    return []


def iter_valid_sensor_records(raw_frames: list[dict]):
    for frame_index, raw_frame in enumerate(raw_frames):
        timestamp = frame_timestamp(raw_frame)
        if timestamp is None:
            continue
        for agent in frame_agent_states(raw_frame):
            aircraft_id = agent.get("aircraftID")
            if aircraft_id is None:
                continue
            sensor_info = ((agent.get("unmannedInfo") or {}).get("sensorInfo") or {})
            center = sensor_info.get("centerCoordinate")
            corners = sensor_info.get("footprintCornerList") or []
            fov_deg = sensor_info.get("fov")
            if center is None or len(corners) != 4 or fov_deg is None:
                continue
            yield frame_index, timestamp, agent, sensor_info


def collect_aircraft_options(log_path: Path) -> list[AircraftLogOption]:
    counts: dict[int, int] = {}
    for _, _, agent, _ in iter_valid_sensor_records(load_raw_log_frames(log_path)):
        aircraft_id = int(agent["aircraftID"])
        counts[aircraft_id] = counts.get(aircraft_id, 0) + 1
    return [
        AircraftLogOption(aircraft_id=aircraft_id, valid_frame_count=counts[aircraft_id])
        for aircraft_id in sorted(counts)
    ]


def sanitize_output_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", value).strip("_") or "analysis"


def build_analysis_config(log_path: Path, aircraft_id: int) -> AnalysisConfig:
    resolved_log_path = resolve_log_path(log_path)
    output_prefix = sanitize_output_name(f"{resolved_log_path.stem}_uav{aircraft_id}")
    return AnalysisConfig(
        log_path=resolved_log_path,
        aircraft_id=int(aircraft_id),
        output_prefix=output_prefix,
    )


def resolve_selected_log_paths(raw_values: list[str | Path]) -> list[Path]:
    resolved_paths: list[Path] = []
    seen: set[str] = set()
    for raw_value in raw_values:
        candidate_text = str(raw_value).strip()
        if candidate_text in ("", LOG_OPTION_NONE, "None", "none", "null", "Null"):
            continue
        resolved_path = resolve_log_path(Path(candidate_text))
        key = str(resolved_path).casefold()
        if key in seen:
            continue
        seen.add(key)
        resolved_paths.append(resolved_path)
    return resolved_paths


def build_analysis_configs(log_paths: list[Path], aircraft_id: int) -> list[AnalysisConfig]:
    resolved_paths = resolve_selected_log_paths(log_paths)
    if not resolved_paths:
        raise ValueError("Select at least one log file.")
    return [build_analysis_config(path, aircraft_id) for path in resolved_paths]


def collect_aircraft_options_for_logs(log_paths: list[Path]) -> list[AircraftLogOption]:
    resolved_paths = resolve_selected_log_paths(log_paths)
    if not resolved_paths:
        return []

    per_log_counts: list[dict[int, int]] = []
    for resolved_path in resolved_paths:
        options = collect_aircraft_options(resolved_path)
        if not options:
            raise ValueError(f"No valid UAV footprint frames found in {resolved_path.name}.")
        per_log_counts.append({option.aircraft_id: option.valid_frame_count for option in options})

    common_aircraft_ids = set(per_log_counts[0])
    for counts in per_log_counts[1:]:
        common_aircraft_ids &= set(counts)

    if not common_aircraft_ids:
        raise ValueError("No common UAV with valid footprint frames across the selected logs.")

    return [
        AircraftLogOption(
            aircraft_id=aircraft_id,
            valid_frame_count=sum(counts[aircraft_id] for counts in per_log_counts),
        )
        for aircraft_id in sorted(common_aircraft_ids)
    ]


def prompt_analysis_configs(default_log_paths: list[Path], default_aircraft_id: int) -> list[AnalysisConfig] | None:
    existing_defaults = [path for path in default_log_paths if Path(path).exists()]
    fallback_defaults = existing_defaults or default_log_paths[:1]
    if tk is None or ttk is None or filedialog is None or messagebox is None:
        return build_analysis_configs(fallback_defaults, default_aircraft_id)

    selected: dict[str, list[AnalysisConfig]] = {}
    option_lookup: dict[str, AircraftLogOption] = {}
    try:
        root = tk.Tk()
    except Exception:
        return build_analysis_configs(fallback_defaults, default_aircraft_id)
    root.title("FOV Log Compare")
    root.resizable(False, False)
    root.columnconfigure(1, weight=1)

    initial_paths = resolve_selected_log_paths(existing_defaults[:2]) if existing_defaults else []
    if not initial_paths and fallback_defaults:
        try:
            initial_paths = resolve_selected_log_paths(fallback_defaults[:2])
        except Exception:
            initial_paths = []

    path_var_1 = tk.StringVar(value=str(initial_paths[0]) if initial_paths else str(default_log_paths[0]))
    path_var_2 = tk.StringVar(value=str(initial_paths[1]) if len(initial_paths) > 1 else LOG_OPTION_NONE)
    aircraft_var = tk.StringVar(value="")
    status_var = tk.StringVar(value="Select one or two log files.")

    def refresh_aircraft_options(*_args) -> None:
        option_lookup.clear()
        aircraft_combo["values"] = ()
        aircraft_var.set("")

        try:
            resolved_paths = resolve_selected_log_paths([path_var_1.get(), path_var_2.get()])
        except Exception as exc:
            status_var.set(str(exc))
            return

        if not resolved_paths:
            status_var.set("Select one or two log files.")
            return

        try:
            options = collect_aircraft_options_for_logs(resolved_paths)
            if not options:
                raise ValueError("No valid UAV footprint frames found.")

            labels = [option.display_label for option in options]
            option_lookup.update({option.display_label: option for option in options})
            aircraft_combo["values"] = labels

            preferred = next(
                (label for label, option in option_lookup.items() if option.aircraft_id == default_aircraft_id),
                labels[0],
            )
            aircraft_var.set(preferred)
            joined_names = ", ".join(path.name for path in resolved_paths)
            status_var.set(
                f"{joined_names}: {len(options)} aircraft with valid footprint frames across {len(resolved_paths)} log(s)"
            )
        except Exception as exc:
            status_var.set(str(exc))

    def browse_log_file(target_var: "tk.StringVar") -> None:
        selected_path = filedialog.askopenfilename(
            title="Select a log file",
            initialdir=str(SCRIPT_DIR / "log_temp"),
            filetypes=[
                ("Log files", "*.json *.jsonl"),
                ("JSON files", "*.json"),
                ("JSONL files", "*.jsonl"),
                ("All files", "*.*"),
            ],
        )
        if selected_path:
            target_var.set(selected_path)
            refresh_aircraft_options()

    def confirm_selection() -> None:
        refresh_aircraft_options()
        try:
            selected_paths = resolve_selected_log_paths([path_var_1.get(), path_var_2.get()])
        except Exception as exc:
            messagebox.showerror("Input Error", str(exc))
            return
        selected_aircraft_label = aircraft_var.get().strip()
        if not selected_paths:
            messagebox.showerror("Input Error", "Select at least one log file.")
            return
        option = option_lookup.get(selected_aircraft_label)
        if option is None:
            messagebox.showerror("Input Error", "Select a UAV.")
            return
        selected["configs"] = build_analysis_configs(selected_paths, option.aircraft_id)
        root.destroy()

    def cancel_selection() -> None:
        root.destroy()

    default_values = [str(path) for path in resolve_selected_log_paths(existing_defaults[:2])]
    optional_values = [LOG_OPTION_NONE, *default_values]

    ttk.Label(root, text="Log file 1").grid(row=0, column=0, padx=10, pady=(12, 6), sticky="w")
    file_combo_1 = ttk.Combobox(
        root,
        textvariable=path_var_1,
        values=default_values,
        width=64,
    )
    file_combo_1.grid(row=0, column=1, padx=(0, 6), pady=(12, 6), sticky="ew")
    ttk.Button(root, text="Browse...", command=lambda: browse_log_file(path_var_1)).grid(
        row=0,
        column=2,
        padx=(0, 10),
        pady=(12, 6),
    )

    ttk.Label(root, text="Log file 2").grid(row=1, column=0, padx=10, pady=6, sticky="w")
    file_combo_2 = ttk.Combobox(
        root,
        textvariable=path_var_2,
        values=optional_values,
        width=64,
    )
    file_combo_2.grid(row=1, column=1, padx=(0, 6), pady=6, sticky="ew")
    ttk.Button(root, text="Browse...", command=lambda: browse_log_file(path_var_2)).grid(
        row=1,
        column=2,
        padx=(0, 10),
        pady=6,
    )

    ttk.Label(root, text="UAV").grid(row=2, column=0, padx=10, pady=6, sticky="w")
    aircraft_combo = ttk.Combobox(root, textvariable=aircraft_var, state="readonly", width=30)
    aircraft_combo.grid(row=2, column=1, padx=(0, 6), pady=6, sticky="w")

    ttk.Label(root, textvariable=status_var, foreground="#444444").grid(
        row=3,
        column=0,
        columnspan=3,
        padx=10,
        pady=(4, 10),
        sticky="w",
    )

    button_frame = ttk.Frame(root)
    button_frame.grid(row=4, column=0, columnspan=3, padx=10, pady=(0, 12), sticky="e")
    ttk.Button(button_frame, text="Cancel", command=cancel_selection).pack(side="right")
    ttk.Button(button_frame, text="Run", command=confirm_selection).pack(side="right", padx=(0, 8))

    file_combo_1.bind("<<ComboboxSelected>>", refresh_aircraft_options)
    file_combo_2.bind("<<ComboboxSelected>>", refresh_aircraft_options)
    root.bind("<Return>", lambda _event: confirm_selection())
    root.protocol("WM_DELETE_WINDOW", cancel_selection)

    refresh_aircraft_options()
    root.mainloop()
    return selected.get("configs")


def load_aircraft_frames(log_path: Path, aircraft_id: int) -> list[LoggedFootprintFrame]:
    extracted: list[tuple[int, int, dict, dict]] = []
    for frame_index, timestamp, agent, sensor_info in iter_valid_sensor_records(load_raw_log_frames(log_path)):
        if int(agent.get("aircraftID")) != int(aircraft_id):
            continue
        extracted.append((frame_index, timestamp, agent, sensor_info))

    if not extracted:
        raise ValueError(f"No valid sensor frames found for aircraft {aircraft_id} in {log_path}")

    time0 = extracted[0][1]
    output: list[LoggedFootprintFrame] = []
    for sample_index, (frame_index, timestamp, agent, sensor_info) in enumerate(extracted):
        center = sensor_info["centerCoordinate"]
        corners = sensor_info["footprintCornerList"]
        output.append(
            LoggedFootprintFrame(
                sample_index=sample_index,
                log_frame_index=frame_index,
                timestamp=timestamp,
                time_s=(timestamp - time0) / 1000.0,
                uav=GeoPoint(
                    lat=float(agent["coordinate"]["latitude"]),
                    lon=float(agent["coordinate"]["longitude"]),
                    alt=float(agent["coordinate"]["altitude"]),
                ),
                center=GeoPointInput(
                    lat=float(center["latitude"]),
                    lon=float(center["longitude"]),
                    alt=float(center.get("altitude")) if center.get("altitude") is not None else None,
                ),
                actual_corners=tuple(
                    GeoPointInput(
                        lat=float(corner["latitude"]),
                        lon=float(corner["longitude"]),
                        alt=float(corner.get("altitude")) if corner.get("altitude") is not None else None,
                    )
                    for corner in corners
                ),
                fov_deg=float(sensor_info["fov"]),
            )
        )
    return output


def compute_predicted_polygon(
    dem: GeoTiffDemAnyCrs,
    logged_frame: LoggedFootprintFrame,
    aspect_ratio: float,
    roll_deg: float,
    fov_interpretation: str,
    predicted_altitude_mode: str,
) -> tuple[
    LocalFrame,
    GeoPoint,
    tuple[GeoPoint, GeoPoint, GeoPoint, GeoPoint],
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    if not dem.contains(logged_frame.uav.lat, logged_frame.uav.lon):
        raise ValueError("uav_out_of_coverage")
    if not dem.contains(logged_frame.center.lat, logged_frame.center.lon):
        raise ValueError("center_out_of_coverage")

    center_geo = resolve_ground_point(
        dem,
        logged_frame.center,
        uav_alt=logged_frame.uav.alt,
        altitude_mode=predicted_altitude_mode,
    )
    frame = LocalFrame(lat0_deg=center_geo.lat, lon0_deg=center_geo.lon)
    uav_xyz = frame.to_enu(logged_frame.uav)
    center_xyz = frame.to_enu(center_geo)
    forward, right, up = build_camera_axes(uav_xyz, center_xyz, roll_deg)
    ray_step_m = resolve_ray_step_m(dem, center_geo.lat, RAY_STEP_M)

    horizontal_fov_deg, vertical_fov_deg = calculate_fov_components(
        fov_value_deg=logged_frame.fov_deg,
        aspect_ratio=aspect_ratio,
        interpretation=fov_interpretation,
    )
    tan_x = math.tan(math.radians(horizontal_fov_deg) / 2.0)
    tan_y = math.tan(math.radians(vertical_fov_deg) / 2.0)

    predicted: list[GeoPoint] = []
    for sx, sy in FOOTPRINT_SPEC_CORNER_DEFINITION:
        ray = normalize(forward + sx * tan_x * right + sy * tan_y * up)
        _, hit_geo = intersect_ray_with_dem(
            dem=dem,
            frame=frame,
            origin_xyz=uav_xyz,
            direction_xyz=ray,
            max_distance_m=MAX_DISTANCE_M,
            step_m=ray_step_m,
        )
        predicted.append(hit_geo)

    _, predicted_center_geo = intersect_ray_with_dem(
        dem=dem,
        frame=frame,
        origin_xyz=uav_xyz,
        direction_xyz=forward,
        max_distance_m=MAX_DISTANCE_M,
        step_m=ray_step_m,
    )
    predicted = order_corners_by_camera_axes(
        frame=frame,
        origin_xyz=uav_xyz,
        forward=forward,
        right=right,
        up=up,
        corner_points=predicted,
    )
    return frame, predicted_center_geo, tuple(predicted), uav_xyz, forward, right, up


def build_actual_polygon(
    dem: GeoTiffDemAnyCrs,
    frame: LocalFrame,
    uav_xyz: np.ndarray,
    forward: np.ndarray,
    right: np.ndarray,
    up: np.ndarray,
    logged_frame: LoggedFootprintFrame,
    actual_altitude_mode: str,
) -> tuple[GeoPoint, tuple[GeoPoint, GeoPoint, GeoPoint, GeoPoint]]:
    center_geo = resolve_ground_point(
        dem,
        logged_frame.center,
        uav_alt=logged_frame.uav.alt,
        altitude_mode=actual_altitude_mode,
    )
    actual_corners = [
        resolve_ground_point(
            dem,
            GeoPointInput(lat=point.lat, lon=point.lon, alt=point.alt),
            uav_alt=logged_frame.uav.alt,
            altitude_mode=actual_altitude_mode,
        )
        for point in logged_frame.actual_corners
    ]
    actual_corners = order_corners_by_camera_axes(
        frame=frame,
        origin_xyz=uav_xyz,
        forward=forward,
        right=right,
        up=up,
        corner_points=actual_corners,
    )
    return center_geo, tuple(actual_corners)


def point_xy(frame: LocalFrame, point: GeoPoint) -> np.ndarray:
    xyz = frame.to_enu(point)
    return xyz[:2]


def polygon_centroid_xy(frame: LocalFrame, points: tuple[GeoPoint, GeoPoint, GeoPoint, GeoPoint]) -> np.ndarray:
    xy = np.vstack([point_xy(frame, point) for point in points])
    return np.mean(xy, axis=0)


def compare_dem_against_log(
    dem_path: Path,
    frames: list[LoggedFootprintFrame],
    aspect_ratio: float,
    roll_deg: float,
    fov_interpretation: str,
    predicted_altitude_mode: str,
    actual_altitude_mode: str,
) -> DemComparisonResult:
    dem = GeoTiffDemAnyCrs(dem_path)
    samples: list[FootprintComparisonSample] = []
    skip_reasons: dict[str, int] = {}

    for logged_frame in frames:
        try:
            frame, predicted_center, predicted_corners, uav_xyz, forward, right, up = compute_predicted_polygon(
                dem=dem,
                logged_frame=logged_frame,
                aspect_ratio=aspect_ratio,
                roll_deg=roll_deg,
                fov_interpretation=fov_interpretation,
                predicted_altitude_mode=predicted_altitude_mode,
            )
            actual_center, actual_corners_raw = build_actual_polygon(
                dem,
                frame,
                uav_xyz,
                forward,
                right,
                up,
                logged_frame,
                actual_altitude_mode,
            )

            actual_polygon = build_polygon_result(
                "Actual",
                frame,
                list(actual_corners_raw),
                preserve_input_order=True,
            )
            predicted_polygon = build_polygon_result(
                "Predicted",
                frame,
                list(predicted_corners),
                preserve_input_order=True,
            )
            actual_corners = tuple(actual_polygon.corner_points)
            predicted_ordered = tuple(predicted_polygon.corner_points)
            corner_errors = tuple(
                ground_distance_m(frame, actual_point, predicted_point)
                for actual_point, predicted_point in zip(actual_corners, predicted_ordered)
            )

            actual_area_m2 = float(actual_polygon.area_m2)
            predicted_area_m2 = float(predicted_polygon.area_m2)
            area_error_m2 = predicted_area_m2 - actual_area_m2
            area_error_pct = (area_error_m2 / actual_area_m2 * 100.0) if actual_area_m2 > 0.0 else float("nan")
            centroid_error_m = float(
                np.linalg.norm(
                    polygon_centroid_xy(frame, actual_corners) - polygon_centroid_xy(frame, predicted_ordered)
                )
            )
            samples.append(
                FootprintComparisonSample(
                    sample_index=logged_frame.sample_index,
                    log_frame_index=logged_frame.log_frame_index,
                    timestamp=logged_frame.timestamp,
                    time_s=logged_frame.time_s,
                    fov_deg=logged_frame.fov_deg,
                    uav=logged_frame.uav,
                    actual_area_m2=actual_area_m2,
                    predicted_area_m2=predicted_area_m2,
                    area_error_m2=area_error_m2,
                    area_error_pct=area_error_pct,
                    centroid_error_m=centroid_error_m,
                    corner_errors_m=corner_errors,
                    mean_corner_error_m=float(np.mean(corner_errors)),
                    max_corner_error_m=float(np.max(corner_errors)),
                    actual_center=actual_center,
                    predicted_center=predicted_center,
                    actual_corners=actual_corners,
                    predicted_corners=predicted_ordered,
                )
            )
        except Exception as exc:
            reason = str(exc) or exc.__class__.__name__
            skip_reasons[reason] = skip_reasons.get(reason, 0) + 1

    return DemComparisonResult(
        dem_name=dem.path.name,
        dem_path=dem.path,
        total_frames=len(frames),
        compared_samples=samples,
        skipped_frames=len(frames) - len(samples),
        skip_reasons=skip_reasons,
    )


def sample_frames_for_diagnostics(
    frames: list[LoggedFootprintFrame],
    sample_limit: int,
) -> list[LoggedFootprintFrame]:
    if sample_limit <= 0 or len(frames) <= sample_limit:
        return list(frames)
    sample_indices = np.linspace(0, len(frames) - 1, num=sample_limit, dtype=int)
    ordered_unique_indices = list(dict.fromkeys(int(index) for index in sample_indices))
    return [frames[index] for index in ordered_unique_indices]


def evaluate_fov_interpretations(
    dem_path: Path,
    frames: list[LoggedFootprintFrame],
    aspect_ratio: float,
    roll_deg: float,
    predicted_altitude_mode: str,
    actual_altitude_mode: str,
    sample_limit: int = FOV_DIAGNOSTIC_SAMPLE_LIMIT,
) -> list[dict[str, float | int | str]]:
    sampled_frames = sample_frames_for_diagnostics(frames, sample_limit)
    diagnostics: list[dict[str, float | int | str]] = []
    for interpretation in FOV_INTERPRETATIONS:
        result = compare_dem_against_log(
            dem_path=dem_path,
            frames=sampled_frames,
            aspect_ratio=aspect_ratio,
            roll_deg=roll_deg,
            fov_interpretation=interpretation,
            predicted_altitude_mode=predicted_altitude_mode,
            actual_altitude_mode=actual_altitude_mode,
        )
        summary = result.summary_row()
        diagnostics.append(
            {
                "interpretation": interpretation,
                "sampled_frames": len(sampled_frames),
                "compared_frames": int(summary["compared_frames"]),
                "corner_mae_m": float(summary["corner_mae_m"]),
                "centroid_mae_m": float(summary["centroid_mae_m"]),
                "area_mape_pct": float(summary["area_mape_pct"]),
            }
        )

    diagnostics.sort(
        key=lambda row: (
            math.inf if not math.isfinite(float(row["corner_mae_m"])) else float(row["corner_mae_m"]),
            math.inf if not math.isfinite(float(row["area_mape_pct"])) else float(row["area_mape_pct"]),
        )
    )
    return diagnostics


def print_fov_diagnostics(dem_name: str, diagnostics: list[dict[str, float | int | str]], top_k: int = 3) -> None:
    if not diagnostics:
        return
    print("")
    print(f"FOV diagnostics   : {dem_name}")
    for row in diagnostics[:top_k]:
        print(
            f"  {row['interpretation']}: "
            f"corner_MAE={float(row['corner_mae_m']):.2f}m, "
            f"centroid_MAE={float(row['centroid_mae_m']):.2f}m, "
            f"area_MAPE={float(row['area_mape_pct']):.2f}%, "
            f"frames={int(row['compared_frames'])}/{int(row['sampled_frames'])}"
        )


def select_fov_diagnostic_dem_path(
    frames: list[LoggedFootprintFrame],
    sample_limit: int = FOV_DIAGNOSTIC_SAMPLE_LIMIT,
) -> Path | None:
    sampled_frames = sample_frames_for_diagnostics(frames, sample_limit)
    best_path: Path | None = None
    best_count = -1
    for dem_path in DEM_PATHS:
        try:
            dem = GeoTiffDemAnyCrs(dem_path)
        except Exception:
            continue
        count = sum(
            1
            for frame in sampled_frames
            if dem.contains(frame.uav.lat, frame.uav.lon) and dem.contains(frame.center.lat, frame.center.lon)
        )
        if count > best_count:
            best_count = count
            best_path = dem.path
    if best_count <= 0:
        return None
    return best_path


def resolve_active_fov_interpretation(
    frames: list[LoggedFootprintFrame],
    aspect_ratio: float,
    roll_deg: float,
    predicted_altitude_mode: str,
    actual_altitude_mode: str,
    configured_interpretation: str,
) -> tuple[str, Path | None, list[dict[str, float | int | str]]]:
    if configured_interpretation != "auto":
        return configured_interpretation, None, []

    diagnostic_dem_path = select_fov_diagnostic_dem_path(frames, FOV_DIAGNOSTIC_SAMPLE_LIMIT)
    if diagnostic_dem_path is None:
        return "diagonal_full", None, []

    diagnostics = evaluate_fov_interpretations(
        dem_path=diagnostic_dem_path,
        frames=frames,
        aspect_ratio=aspect_ratio,
        roll_deg=roll_deg,
        predicted_altitude_mode=predicted_altitude_mode,
        actual_altitude_mode=actual_altitude_mode,
        sample_limit=FOV_DIAGNOSTIC_SAMPLE_LIMIT,
    )
    if not diagnostics:
        return "diagonal_full", diagnostic_dem_path, diagnostics
    return str(diagnostics[0]["interpretation"]), diagnostic_dem_path, diagnostics


def build_axes_grid(count: int, max_cols: int = 3, base_width: float = 6.0, base_height: float = 3.6):
    cols = min(max_cols, max(1, count))
    rows = math.ceil(count / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(base_width * cols, base_height * rows), squeeze=False)
    return fig, axes, rows, cols


def format_skip_reasons(skip_reasons: dict[str, int]) -> str:
    labels = {
        "uav_out_of_coverage": "UAV가 DEM 범위 밖",
        "center_out_of_coverage": "중심점이 DEM 범위 밖",
        "A camera ray left the DEM tile before hitting terrain.": "광선이 DEM 범위를 벗어남",
        "UAV altitude is not above the DEM surface.": "UAV 고도가 DEM 표면 이하",
    }
    items = []
    for key, value in sorted(skip_reasons.items()):
        items.append(f"{labels.get(key, key)}: {value}")
    return ", ".join(items) or "비교 불가"


def style_empty_axis(ax, title: str, reason: str) -> None:
    ax.set_title(title)
    ax.text(0.5, 0.5, reason, ha="center", va="center", fontsize=11, transform=ax.transAxes)
    ax.set_xticks([])
    ax.set_yticks([])


def select_random_snapshot_dem_result(
    results: list[DemComparisonResult], preferred_dem_name: str | None = None
) -> DemComparisonResult | None:
    valid_results = [result for result in results if result.compared_samples]
    if not valid_results:
        return None

    if preferred_dem_name:
        preferred = preferred_dem_name.strip().lower()
        preferred_tokens = {
            preferred,
            Path(preferred_dem_name).name.lower(),
            Path(preferred_dem_name).stem.lower(),
        }
        for result in valid_results:
            if (
                result.dem_name.lower() in preferred_tokens
                or result.dem_path.name.lower() in preferred_tokens
                or result.dem_path.stem.lower() in preferred_tokens
            ):
                return result

    return valid_results[0]


def select_random_footprint_samples(
    samples: list[FootprintComparisonSample], sample_count: int, random_seed: int | None = None
) -> list[FootprintComparisonSample]:
    if not samples:
        return []

    selected_count = min(sample_count, len(samples))
    rng = np.random.default_rng(random_seed)
    selected_indices = rng.choice(len(samples), size=selected_count, replace=False)
    selected_samples = [samples[int(index)] for index in selected_indices]
    selected_samples.sort(key=lambda sample: sample.time_s)
    return selected_samples


def plot_footprint_sample_overlay(ax, sample: FootprintComparisonSample, show_legend: bool = False) -> None:
    frame = LocalFrame(lat0_deg=sample.actual_center.lat, lon0_deg=sample.actual_center.lon)
    uav_xy = point_xy(frame, sample.uav)
    actual_center_xy = point_xy(frame, sample.actual_center)
    predicted_center_xy = point_xy(frame, sample.predicted_center)
    actual_xy = np.vstack([point_xy(frame, point) for point in sample.actual_corners])
    predicted_xy = np.vstack([point_xy(frame, point) for point in sample.predicted_corners])
    actual_ring = np.vstack([actual_xy, actual_xy[0]])
    predicted_ring = np.vstack([predicted_xy, predicted_xy[0]])

    ax.plot(actual_ring[:, 0], actual_ring[:, 1], color="black", linewidth=1.8, label="actual")
    ax.fill(actual_ring[:, 0], actual_ring[:, 1], color="black", alpha=0.10)
    ax.plot(predicted_ring[:, 0], predicted_ring[:, 1], color="crimson", linewidth=1.8, label="predicted")
    ax.fill(predicted_ring[:, 0], predicted_ring[:, 1], color="crimson", alpha=0.12)
    ax.scatter(actual_xy[:, 0], actual_xy[:, 1], color="black", s=18)
    ax.scatter(predicted_xy[:, 0], predicted_xy[:, 1], color="crimson", s=18)
    ax.scatter(uav_xy[0], uav_xy[1], color="navy", marker="^", s=55, label="uav")
    ax.scatter(actual_center_xy[0], actual_center_xy[1], color="black", marker="x", s=35, label="actual center")
    ax.scatter(
        predicted_center_xy[0],
        predicted_center_xy[1],
        color="crimson",
        marker="x",
        s=35,
        label="predicted center",
    )
    ax.set_title(
        f"sample #{sample.sample_index}  t={sample.time_s:.1f}s\n"
        f"area err={sample.area_error_m2:+.1f} m^2, centroid={sample.centroid_error_m:.1f} m"
    )
    ax.set_xlabel("east [m]")
    ax.set_ylabel("north [m]")
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    if show_legend:
        ax.legend(loc="upper right", fontsize=8)


def plot_area_comparison_grid(results: list[DemComparisonResult], run_label: str) -> plt.Figure:
    fig, axes, rows, cols = build_axes_grid(len(results), max_cols=3, base_width=6.2, base_height=3.8)
    for axis in axes.ravel():
        axis.set_visible(False)

    for index, result in enumerate(results):
        ax = axes[index // cols][index % cols]
        ax.set_visible(True)
        if not result.compared_samples:
            style_empty_axis(ax, result.dem_name, format_skip_reasons(result.skip_reasons))
            continue

        x = np.array([sample.time_s for sample in result.compared_samples], dtype=float)
        actual = np.array([sample.actual_area_m2 for sample in result.compared_samples], dtype=float)
        predicted = np.array([sample.predicted_area_m2 for sample in result.compared_samples], dtype=float)
        summary = result.summary_row()
        ax.plot(x, actual, color="black", linewidth=1.8, label="실제 면적")
        ax.plot(x, predicted, color="crimson", linewidth=1.4, alpha=0.9, label="예상 면적")
        ax.set_title(
            f"{result.dem_name}\n"
            f"비교={len(result.compared_samples)}/{result.total_frames}, "
            f"면적 MAE={summary['area_mae_m2']:.1f} m^2\n"
            f"평균 오차율={summary['area_mape_pct']:.2f}%, "
            f"평균 비율={summary['area_ratio_pct']:.2f}%"
        )
        ax.set_xlabel("시간 [s]")
        ax.set_ylabel("면적 [m^2]")
        ax.grid(alpha=0.25)
        ax.legend(loc="upper right", fontsize=8)

    fig.suptitle(f"{run_label} 실제 Footprint와 DEM 기반 예상 면적 비교", fontsize=15)
    fig.tight_layout()
    return fig


def plot_corner_error_grid(results: list[DemComparisonResult], run_label: str) -> plt.Figure:
    fig, axes, rows, cols = build_axes_grid(len(results), max_cols=3, base_width=6.2, base_height=3.8)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd"]
    for axis in axes.ravel():
        axis.set_visible(False)

    for index, result in enumerate(results):
        ax = axes[index // cols][index % cols]
        ax.set_visible(True)
        if not result.compared_samples:
            style_empty_axis(ax, result.dem_name, format_skip_reasons(result.skip_reasons))
            continue

        x = np.array([sample.time_s for sample in result.compared_samples], dtype=float)
        corner_errors = np.array([sample.corner_errors_m for sample in result.compared_samples], dtype=float)
        mean_errors = np.array([sample.mean_corner_error_m for sample in result.compared_samples], dtype=float)
        summary = result.summary_row()
        for corner_index in range(4):
            ax.plot(
                x,
                corner_errors[:, corner_index],
                color=colors[corner_index],
                linewidth=1.0,
                alpha=0.9,
                label=CORNER_LABELS_KO[corner_index],
            )
        ax.plot(x, mean_errors, color="black", linewidth=2.0, label="평균")
        ax.set_title(
            f"{result.dem_name}\n코너 MAE={summary['corner_mae_m']:.1f} m, "
            f"최대={summary['max_corner_error_m']:.1f} m"
        )
        ax.set_xlabel("시간 [s]")
        ax.set_ylabel("코너 오차 [m]")
        ax.grid(alpha=0.25)
        ax.legend(loc="upper right", fontsize=8, ncol=3)

    fig.suptitle(f"{run_label} DEM별 코너 오차", fontsize=15)
    fig.tight_layout()
    return fig


def plot_worst_snapshot_grid(results: list[DemComparisonResult]) -> plt.Figure:
    fig, axes, rows, cols = build_axes_grid(len(results), max_cols=3, base_width=6.2, base_height=4.2)
    for axis in axes.ravel():
        axis.set_visible(False)

    for index, result in enumerate(results):
        ax = axes[index // cols][index % cols]
        ax.set_visible(True)
        if not result.compared_samples:
            style_empty_axis(ax, result.dem_name, format_skip_reasons(result.skip_reasons))
            continue

        worst = max(result.compared_samples, key=lambda sample: abs(sample.area_error_m2))
        frame = LocalFrame(lat0_deg=worst.actual_center.lat, lon0_deg=worst.actual_center.lon)
        uav_xy = point_xy(frame, worst.uav)
        actual_center_xy = point_xy(frame, worst.actual_center)
        predicted_center_xy = point_xy(frame, worst.predicted_center)
        actual_xy = np.vstack([point_xy(frame, point) for point in worst.actual_corners])
        predicted_xy = np.vstack([point_xy(frame, point) for point in worst.predicted_corners])
        actual_ring = np.vstack([actual_xy, actual_xy[0]])
        predicted_ring = np.vstack([predicted_xy, predicted_xy[0]])
        actual_center_line_x = [uav_xy[0], actual_center_xy[0]]
        actual_center_line_y = [uav_xy[1], actual_center_xy[1]]
        predicted_center_line_x = [uav_xy[0], predicted_center_xy[0]]
        predicted_center_line_y = [uav_xy[1], predicted_center_xy[1]]
        ax.plot(actual_ring[:, 0], actual_ring[:, 1], color="black", linewidth=2.0, label="실제")
        ax.fill(actual_ring[:, 0], actual_ring[:, 1], color="black", alpha=0.12)
        ax.plot(predicted_ring[:, 0], predicted_ring[:, 1], color="crimson", linewidth=2.0, label="예상")
        ax.fill(predicted_ring[:, 0], predicted_ring[:, 1], color="crimson", alpha=0.15)
        ax.scatter(actual_xy[:, 0], actual_xy[:, 1], color="black", s=25)
        ax.scatter(predicted_xy[:, 0], predicted_xy[:, 1], color="crimson", s=25)
        for label, actual_point, predicted_point in zip(CORNER_LABELS_KO, actual_xy, predicted_xy):
            ax.text(actual_point[0] + 6.0, actual_point[1] + 6.0, label, color="black", fontsize=8)
            ax.text(predicted_point[0] + 6.0, predicted_point[1] - 10.0, label, color="crimson", fontsize=8)
        ax.set_title(
            f"{result.dem_name}\n최대 면적 오차 시점 t={worst.time_s:.1f}s, "
            f"{worst.area_error_m2:+.1f} m^2"
        )
        ax.set_xlabel("동쪽 [m]")
        ax.set_ylabel("북쪽 [m]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.legend(loc="upper right", fontsize=8)

    fig.suptitle("DEM별 최대 오차 Footprint 중첩", fontsize=15)
    fig.tight_layout()
    return fig


def plot_random_snapshot_grid_with_uav(results: list[DemComparisonResult]) -> plt.Figure:
    selected_result = select_random_snapshot_dem_result(results, RANDOM_SNAPSHOT_DEM_NAME)
    selected_samples = []
    if selected_result is not None:
        selected_samples = select_random_footprint_samples(
            selected_result.compared_samples,
            sample_count=RANDOM_SNAPSHOT_SAMPLE_COUNT,
            random_seed=RANDOM_SNAPSHOT_SEED,
        )

    fig, axes, rows, cols = build_axes_grid(
        max(1, len(selected_samples)),
        max_cols=5,
        base_width=4.6,
        base_height=4.0,
    )
    for axis in axes.ravel():
        axis.set_visible(False)

    if selected_result is None or not selected_samples:
        ax = axes[0][0]
        ax.set_visible(True)
        style_empty_axis(ax, "Random footprint samples", "No valid footprint samples.")
        fig.tight_layout()
        return fig

    for index, sample in enumerate(selected_samples):
        ax = axes[index // cols][index % cols]
        ax.set_visible(True)
        plot_footprint_sample_overlay(ax, sample, show_legend=index == 0)

    fig.suptitle(
        f"{selected_result.dem_name} random footprint samples "
        f"({len(selected_samples)}, seed={RANDOM_SNAPSHOT_SEED})",
        fontsize=15,
    )
    fig.tight_layout()
    return fig


def plot_position_context_grid(results: list[DemComparisonResult], run_label: str) -> plt.Figure:
    fig, axes, rows, cols = build_axes_grid(len(results), max_cols=3, base_width=6.2, base_height=4.4)
    for axis in axes.ravel():
        axis.set_visible(False)

    for index, result in enumerate(results):
        ax = axes[index // cols][index % cols]
        ax.set_visible(True)
        if not result.compared_samples:
            style_empty_axis(ax, result.dem_name, format_skip_reasons(result.skip_reasons))
            continue

        worst = max(result.compared_samples, key=lambda sample: abs(sample.area_error_m2))
        frame = LocalFrame(lat0_deg=worst.actual_center.lat, lon0_deg=worst.actual_center.lon)
        worst_uav_xy = point_xy(frame, worst.uav)
        worst_actual_xy = np.vstack([point_xy(frame, point) for point in worst.actual_corners])
        worst_predicted_xy = np.vstack([point_xy(frame, point) for point in worst.predicted_corners])
        worst_actual_ring = np.vstack([worst_actual_xy, worst_actual_xy[0]])
        worst_predicted_ring = np.vstack([worst_predicted_xy, worst_predicted_xy[0]])

        ax.scatter(worst_uav_xy[0], worst_uav_xy[1], color="navy", s=90, marker="^", label="worst-frame UAV")
        ax.plot(
            worst_actual_ring[:, 0],
            worst_actual_ring[:, 1],
            color="black",
            linewidth=1.8,
            alpha=0.85,
            label="worst actual footprint",
        )
        ax.fill(worst_actual_ring[:, 0], worst_actual_ring[:, 1], color="black", alpha=0.08)
        ax.plot(
            worst_predicted_ring[:, 0],
            worst_predicted_ring[:, 1],
            color="crimson",
            linewidth=1.8,
            alpha=0.85,
            label="worst predicted footprint",
        )
        ax.fill(worst_predicted_ring[:, 0], worst_predicted_ring[:, 1], color="crimson", alpha=0.10)

        ax.text(worst_uav_xy[0] + 8.0, worst_uav_xy[1] + 8.0, "worst UAV", color="navy", fontsize=8)

        summary = result.summary_row()
        ax.set_title(
            f"{result.dem_name}\n"
            f"worst-frame UAV + footprint, centroid MAE={summary['centroid_mae_m']:.1f} m"
        )
        ax.set_xlabel("east [m]")
        ax.set_ylabel("north [m]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.legend(loc="best", fontsize=7)

    fig.suptitle(f"{run_label} position context by DEM", fontsize=15)
    fig.tight_layout()
    return fig


def write_summary_csv(results: list[DemComparisonResult], output_dir: Path, output_prefix: str) -> Path:
    output_path = output_dir / f"{output_prefix}_dem_summary.csv"
    rows = [result.summary_row() for result in results]
    fieldnames = list(rows[0].keys())
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return output_path


def write_detail_csv(result: DemComparisonResult, output_dir: Path, output_prefix: str) -> Path | None:
    if not result.compared_samples:
        return None

    output_path = output_dir / f"{sanitize_output_name(result.dem_name)}_{output_prefix}_detail.csv"
    fieldnames = [
        "sample_index",
        "log_frame_index",
        "timestamp",
        "time_s",
        "fov_deg",
        "uav_lat",
        "uav_lon",
        "uav_alt",
        "actual_area_m2",
        "predicted_area_m2",
        "area_error_m2",
        "area_error_pct",
        "centroid_error_m",
        "corner_error_c1_m",
        "corner_error_c2_m",
        "corner_error_c3_m",
        "corner_error_c4_m",
        "mean_corner_error_m",
        "max_corner_error_m",
        "actual_center_lat",
        "actual_center_lon",
        "predicted_center_lat",
        "predicted_center_lon",
        "actual_c1_lat",
        "actual_c1_lon",
        "predicted_c1_lat",
        "predicted_c1_lon",
        "actual_c2_lat",
        "actual_c2_lon",
        "predicted_c2_lat",
        "predicted_c2_lon",
        "actual_c3_lat",
        "actual_c3_lon",
        "predicted_c3_lat",
        "predicted_c3_lon",
        "actual_c4_lat",
        "actual_c4_lon",
        "predicted_c4_lat",
        "predicted_c4_lon",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample in result.compared_samples:
            writer.writerow(
                {
                    "sample_index": sample.sample_index,
                    "log_frame_index": sample.log_frame_index,
                    "timestamp": sample.timestamp,
                    "time_s": f"{sample.time_s:.3f}",
                    "fov_deg": f"{sample.fov_deg:.6f}",
                    "uav_lat": f"{sample.uav.lat:.9f}",
                    "uav_lon": f"{sample.uav.lon:.9f}",
                    "uav_alt": f"{sample.uav.alt:.6f}",
                    "actual_area_m2": f"{sample.actual_area_m2:.6f}",
                    "predicted_area_m2": f"{sample.predicted_area_m2:.6f}",
                    "area_error_m2": f"{sample.area_error_m2:.6f}",
                    "area_error_pct": f"{sample.area_error_pct:.6f}",
                    "centroid_error_m": f"{sample.centroid_error_m:.6f}",
                    "corner_error_c1_m": f"{sample.corner_errors_m[0]:.6f}",
                    "corner_error_c2_m": f"{sample.corner_errors_m[1]:.6f}",
                    "corner_error_c3_m": f"{sample.corner_errors_m[2]:.6f}",
                    "corner_error_c4_m": f"{sample.corner_errors_m[3]:.6f}",
                    "mean_corner_error_m": f"{sample.mean_corner_error_m:.6f}",
                    "max_corner_error_m": f"{sample.max_corner_error_m:.6f}",
                    "actual_center_lat": f"{sample.actual_center.lat:.9f}",
                    "actual_center_lon": f"{sample.actual_center.lon:.9f}",
                    "predicted_center_lat": f"{sample.predicted_center.lat:.9f}",
                    "predicted_center_lon": f"{sample.predicted_center.lon:.9f}",
                    "actual_c1_lat": f"{sample.actual_corners[0].lat:.9f}",
                    "actual_c1_lon": f"{sample.actual_corners[0].lon:.9f}",
                    "predicted_c1_lat": f"{sample.predicted_corners[0].lat:.9f}",
                    "predicted_c1_lon": f"{sample.predicted_corners[0].lon:.9f}",
                    "actual_c2_lat": f"{sample.actual_corners[1].lat:.9f}",
                    "actual_c2_lon": f"{sample.actual_corners[1].lon:.9f}",
                    "predicted_c2_lat": f"{sample.predicted_corners[1].lat:.9f}",
                    "predicted_c2_lon": f"{sample.predicted_corners[1].lon:.9f}",
                    "actual_c3_lat": f"{sample.actual_corners[2].lat:.9f}",
                    "actual_c3_lon": f"{sample.actual_corners[2].lon:.9f}",
                    "predicted_c3_lat": f"{sample.predicted_corners[2].lat:.9f}",
                    "predicted_c3_lon": f"{sample.predicted_corners[2].lon:.9f}",
                    "actual_c4_lat": f"{sample.actual_corners[3].lat:.9f}",
                    "actual_c4_lon": f"{sample.actual_corners[3].lon:.9f}",
                    "predicted_c4_lat": f"{sample.predicted_corners[3].lat:.9f}",
                    "predicted_c4_lon": f"{sample.predicted_corners[3].lon:.9f}",
                }
            )
    return output_path


def save_figure(fig: plt.Figure, output_dir: Path, filename: str) -> Path:
    output_path = output_dir / filename
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    return output_path


def print_summary(results: list[DemComparisonResult], config: AnalysisConfig, active_fov_interpretation: str) -> None:
    print(f"코너 규칙        : {', '.join(CORNER_LABELS_KO)}")
    print(f"로그 경로        : {config.log_path}")
    print(f"항공기 ID        : {config.aircraft_id}")
    print(f"추출 프레임 수   : {results[0].total_frames if results else 0}")
    if FOV_INTERPRETATION == "auto":
        print(f"FOV 해석         : auto -> {active_fov_interpretation}")
    else:
        print(f"FOV 해석         : {active_fov_interpretation}")
    print(f"예측 고도 모드   : {PREDICTED_GROUND_ALTITUDE_MODE}")
    print(f"실측 고도 모드   : {ACTUAL_GROUND_ALTITUDE_MODE}")
    print("")
    for result in results:
        summary = result.summary_row()
        if not result.compared_samples:
            reason = format_skip_reasons(result.skip_reasons)
            print(f"[{result.dem_name}] 비교=0/{result.total_frames} 건너뜀={result.skipped_frames} 사유={reason}")
            continue
        print(
            f"[{result.dem_name}] 비교={summary['compared_frames']}/{summary['total_frames']} "
            f"coverage={summary['coverage_ratio']:.3f} "
            f"면적_MAE={summary['area_mae_m2']:.2f}m^2 "
            f"면적_MAPE={summary['area_mape_pct']:.2f}% "
            f"코너_MAE={summary['corner_mae_m']:.2f}m "
            f"중심_MAE={summary['centroid_mae_m']:.2f}m "
            f"최대_코너={summary['max_corner_error_m']:.2f}m"
        )


def run_analysis(config: AnalysisConfig, selected_font: str) -> None:
    frames = load_aircraft_frames(config.log_path, config.aircraft_id)
    active_fov_interpretation, diagnostic_dem_path, fov_diagnostics = resolve_active_fov_interpretation(
        frames=frames,
        aspect_ratio=ASPECT_RATIO,
        roll_deg=ROLL_DEG,
        predicted_altitude_mode=PREDICTED_GROUND_ALTITUDE_MODE,
        actual_altitude_mode=ACTUAL_GROUND_ALTITUDE_MODE,
        configured_interpretation=FOV_INTERPRETATION,
    )
    results = [
        compare_dem_against_log(
            dem_path=dem_path,
            frames=frames,
            aspect_ratio=ASPECT_RATIO,
            roll_deg=ROLL_DEG,
            fov_interpretation=active_fov_interpretation,
            predicted_altitude_mode=PREDICTED_GROUND_ALTITUDE_MODE,
            actual_altitude_mode=ACTUAL_GROUND_ALTITUDE_MODE,
        )
        for dem_path in DEM_PATHS
    ]

    print(f"Matplotlib 폰트 : {selected_font}")
    print_summary(results, config, active_fov_interpretation)
    summary_csv = write_summary_csv(results, OUTPUT_DIR, config.output_prefix)
    detail_csv_paths = [write_detail_csv(result, OUTPUT_DIR, config.output_prefix) for result in results]
    if diagnostic_dem_path is not None:
        print_fov_diagnostics(diagnostic_dem_path.name, fov_diagnostics)

    area_fig = plot_area_comparison_grid(results, config.run_label)
    corner_fig = plot_corner_error_grid(results, config.run_label)
    snapshot_fig = plot_random_snapshot_grid_with_uav(results)
    position_fig = plot_position_context_grid(results, config.run_label)

    if SAVE_FIGURES:
        area_png = save_figure(area_fig, OUTPUT_DIR, f"{config.output_prefix}_area_comparison.png")
        corner_png = save_figure(corner_fig, OUTPUT_DIR, f"{config.output_prefix}_corner_errors.png")
        snapshot_result = select_random_snapshot_dem_result(results, RANDOM_SNAPSHOT_DEM_NAME)
        snapshot_dem_label = sanitize_output_name(snapshot_result.dem_name) if snapshot_result is not None else "no_dem"
        snapshot_png = save_figure(
            snapshot_fig,
            OUTPUT_DIR,
            f"{config.output_prefix}_{snapshot_dem_label}_random_snapshots.png",
        )
        position_png = save_figure(position_fig, OUTPUT_DIR, f"{config.output_prefix}_position_context.png")
        print("")
        print(f"요약 CSV         : {summary_csv}")
        print(f"면적 그래프      : {area_png}")
        print(f"코너 그래프      : {corner_png}")
        print(f"스냅샷 그래프    : {snapshot_png}")
        print(f"Position graph      : {position_png}")
        if snapshot_result is not None:
            sampled_count = min(RANDOM_SNAPSHOT_SAMPLE_COUNT, len(snapshot_result.compared_samples))
            print(f"Random snapshot DEM : {snapshot_result.dem_name} ({sampled_count} samples, seed={RANDOM_SNAPSHOT_SEED})")
        for detail_csv_path in detail_csv_paths:
            if detail_csv_path is not None:
                print(f"상세 CSV         : {detail_csv_path}")

    if SHOW_PLOTS:
        plt.show()

    plt.close(area_fig)
    plt.close(corner_fig)
    plt.close(snapshot_fig)
    plt.close(position_fig)


def main(config: AnalysisConfig | list[AnalysisConfig] | None = None) -> None:
    if config is None:
        configs = prompt_analysis_configs(DEFAULT_LOG_PATHS, AIRCRAFT_ID)
        if configs is None:
            print("Analysis cancelled.")
            return
    elif isinstance(config, AnalysisConfig):
        configs = [config]
    else:
        configs = list(config)

    if not configs:
        print("No analysis config provided.")
        return

    selected_font = configure_matplotlib_for_korean()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    for index, current_config in enumerate(configs, start=1):
        if len(configs) > 1:
            if index > 1:
                print("")
            print(f"[{index}/{len(configs)}] Running analysis for {current_config.log_path.name}")
        run_analysis(current_config, selected_font)


if __name__ == "__main__":
    main()
