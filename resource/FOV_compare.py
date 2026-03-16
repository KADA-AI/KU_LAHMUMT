from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib


def select_matplotlib_backend() -> str:
    try:
        import tkinter  # noqa: F401
    except Exception:
        matplotlib.use("Agg")
        return "Agg"

    matplotlib.use("TkAgg")
    return "TkAgg"


MATPLOTLIB_BACKEND = select_matplotlib_backend()
import matplotlib.pyplot as plt
import numpy as np
import tifffile
from matplotlib.colors import LightSource
from matplotlib.path import Path as MplPath

EARTH_RADIUS_M = 6_378_137.0
SCRIPT_DIR = Path(__file__).resolve().parent
FOOTPRINT_SPEC_CORNER_DEFINITION = [
    (-1.0, 1.0),   # C1: top-left
    (1.0, 1.0),    # C2: top-right
    (1.0, -1.0),   # C3: bottom-right
    (-1.0, -1.0),  # C4: bottom-left
]
FOOTPRINT_SPEC_CORNER_LABELS = ("C1(TL)", "C2(TR)", "C3(BR)", "C4(BL)")


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float
    alt: float


@dataclass(frozen=True)
class GeoPointInput:
    lat: float
    lon: float
    alt: float | None = None


@dataclass
class PolygonResult:
    name: str
    corner_points: list[GeoPoint] = field(default_factory=list)
    corner_xyz: np.ndarray | None = None
    area_m2: float = 0.0


@dataclass
class FootprintOutputs:
    fov_interpretation: str
    diagonal_fov_deg: float
    horizontal_fov_deg: float
    vertical_fov_deg: float
    aspect_ratio: float
    roll_deg: float
    ground_altitude_mode: str
    ray_step_m: float
    uav: GeoPoint
    center_input: GeoPoint
    center_hit: GeoPoint
    center_error_horizontal_m: float
    center_error_vertical_m: float
    uav_xyz: np.ndarray | None = None
    center_hit_xyz: np.ndarray | None = None
    computed_footprint: PolygonResult | None = None
    comparison_footprint: PolygonResult | None = None
    area_difference_m2: float | None = None


# ===== Input Variables =====
INPUT_DEM_PATH = Path("n37_e127_1arc_v3.tif")
INPUT_DIAG_FOV_DEG = 36.632125120620415
INPUT_ASPECT_RATIO = 16.0 / 9.0
INPUT_ROLL_DEG = 0.0
INPUT_UAV_POSITION = GeoPoint(lat=37.8535, lon=127.4465, alt=1550.0)
INPUT_FOOTPRINT_CENTER = GeoPointInput(lat=37.8495, lon=127.4515, alt=None)
# Sample comparison footprint corners from another source.
# Replace these four points with the external 4-corner footprint when available.
INPUT_COMPARISON_FOOTPRINT_CORNERS: list[GeoPointInput] | None = [
    GeoPointInput(lat=37.848620, lon=127.447780, alt=None),
    GeoPointInput(lat=37.844080, lon=127.451620, alt=None),
    GeoPointInput(lat=37.849980, lon=127.457620, alt=None),
    GeoPointInput(lat=37.852460, lon=127.452040, alt=None),
]
INPUT_RAY_STEP_M = 20.0
INPUT_MAX_DISTANCE_M = 50000.0
INPUT_SHOW_PLOTS = True
INPUT_FOV_INTERPRETATION = "diagonal_full"
INPUT_GROUND_ALTITUDE_MODE = "auto"

FOV_INTERPRETATIONS = (
    "diagonal_full",
    "diagonal_half",
    "horizontal_full",
    "horizontal_half",
    "vertical_full",
    "vertical_half",
)
GROUND_ALTITUDE_MODES = ("auto", "dem", "logged")
MAX_AUTO_GROUND_ALT_DELTA_M = 200.0
MIN_GROUND_CLEARANCE_M = 1.0
MIN_RAY_STEP_M = 1.0


def resolve_existing_path(path: Path) -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        if candidate.exists():
            return candidate.resolve()
        raise FileNotFoundError(f"Input file not found: {candidate}")

    search_order = [
        Path.cwd() / candidate,
        SCRIPT_DIR / candidate,
    ]
    for resolved in search_order:
        if resolved.exists():
            return resolved.resolve()

    searched = ", ".join(str(path_item) for path_item in search_order)
    raise FileNotFoundError(f"Input file not found: {candidate} (searched: {searched})")


class BFDBGenerate:
    """
    Camera footprint and FOV utilities.
    Diagonal FOV is converted to horizontal/vertical FOV with the given aspect ratio.
    """

    def calculate_horizontal_vertical_fov(
        self,
        diagonal_fov_deg: float,
        aspect_ratio: float = 16.0 / 9.0,
    ) -> tuple[float, float]:
        diagonal_rad = math.radians(diagonal_fov_deg)
        tan_half = math.tan(diagonal_rad / 2.0)
        denom = math.sqrt(1.0 + aspect_ratio ** 2)
        vertical = 2.0 * math.atan(tan_half / denom)
        horizontal = 2.0 * math.atan((aspect_ratio * tan_half) / denom)
        return math.degrees(horizontal), math.degrees(vertical)

    def cot(self, angle_deg: float) -> float:
        return 1.0 / math.tan(math.radians(angle_deg))

    def Footprint(self, fov_v: float, fov_h: float, h: float = 610.0, af: float = 90.0) -> tuple[float, float, float, float]:
        w1 = (2.0 * h * math.tan(math.radians(fov_h / 2.0))) / math.sin(math.radians(af - fov_v / 2.0))
        w2 = (2.0 * h * math.tan(math.radians(fov_h / 2.0))) / math.sin(math.radians(af + fov_v / 2.0))
        length = h * (self.cot(af - fov_v / 2.0) - self.cot(af + fov_v / 2.0))
        area = ((w1 + w2) / 2.0) * length
        return w1, w2, length, area


def calculate_fov_components(
    fov_value_deg: float,
    aspect_ratio: float,
    interpretation: str = "diagonal_full",
) -> tuple[float, float]:
    if interpretation not in FOV_INTERPRETATIONS:
        raise ValueError(f"Unsupported FOV interpretation: {interpretation}")

    fov_util = BFDBGenerate()
    if interpretation == "diagonal_full":
        return fov_util.calculate_horizontal_vertical_fov(fov_value_deg, aspect_ratio)
    if interpretation == "diagonal_half":
        return fov_util.calculate_horizontal_vertical_fov(fov_value_deg * 2.0, aspect_ratio)
    if interpretation == "horizontal_full":
        horizontal_fov_deg = float(fov_value_deg)
        vertical_fov_deg = math.degrees(
            2.0 * math.atan(math.tan(math.radians(horizontal_fov_deg) / 2.0) / aspect_ratio)
        )
        return horizontal_fov_deg, vertical_fov_deg
    if interpretation == "horizontal_half":
        horizontal_fov_deg = float(fov_value_deg) * 2.0
        vertical_fov_deg = math.degrees(
            2.0 * math.atan(math.tan(math.radians(horizontal_fov_deg) / 2.0) / aspect_ratio)
        )
        return horizontal_fov_deg, vertical_fov_deg
    if interpretation == "vertical_full":
        vertical_fov_deg = float(fov_value_deg)
        horizontal_fov_deg = math.degrees(
            2.0 * math.atan(math.tan(math.radians(vertical_fov_deg) / 2.0) * aspect_ratio)
        )
        return horizontal_fov_deg, vertical_fov_deg

    vertical_fov_deg = float(fov_value_deg) * 2.0
    horizontal_fov_deg = math.degrees(
        2.0 * math.atan(math.tan(math.radians(vertical_fov_deg) / 2.0) * aspect_ratio)
    )
    return horizontal_fov_deg, vertical_fov_deg


def should_use_logged_ground_altitude(
    logged_alt: float | None,
    dem_alt: float,
    uav_alt: float | None = None,
    max_dem_delta_m: float = MAX_AUTO_GROUND_ALT_DELTA_M,
    min_ground_clearance_m: float = MIN_GROUND_CLEARANCE_M,
) -> bool:
    if logged_alt is None or not math.isfinite(logged_alt):
        return False
    if uav_alt is not None and logged_alt >= (uav_alt - min_ground_clearance_m):
        return False
    return abs(logged_alt - dem_alt) <= max_dem_delta_m


def resolve_ground_altitude(
    dem,
    lat: float,
    lon: float,
    logged_alt: float | None = None,
    uav_alt: float | None = None,
    altitude_mode: str = "auto",
) -> float:
    if altitude_mode not in GROUND_ALTITUDE_MODES:
        raise ValueError(f"Unsupported ground altitude mode: {altitude_mode}")

    dem_alt = float(dem.sample_elevation(lat, lon))
    if altitude_mode == "dem":
        return dem_alt
    if altitude_mode == "logged":
        return float(logged_alt) if logged_alt is not None else dem_alt
    if should_use_logged_ground_altitude(logged_alt, dem_alt, uav_alt=uav_alt):
        return float(logged_alt)
    return dem_alt


def resolve_ground_point(
    dem,
    point: GeoPointInput,
    uav_alt: float | None = None,
    altitude_mode: str = "auto",
) -> GeoPoint:
    altitude = resolve_ground_altitude(
        dem,
        lat=point.lat,
        lon=point.lon,
        logged_alt=point.alt,
        uav_alt=uav_alt,
        altitude_mode=altitude_mode,
    )
    return GeoPoint(lat=point.lat, lon=point.lon, alt=altitude)


def dem_ground_resolution_m(dem, lat: float) -> tuple[float, float]:
    if hasattr(dem, "pixel_width_deg") and hasattr(dem, "pixel_height_deg"):
        dx = EARTH_RADIUS_M * math.cos(math.radians(lat)) * math.radians(abs(float(dem.pixel_width_deg)))
        dy = EARTH_RADIUS_M * math.radians(abs(float(dem.pixel_height_deg)))
        return dx, dy

    if getattr(dem, "is_geographic", False):
        dx = EARTH_RADIUS_M * math.cos(math.radians(lat)) * math.radians(abs(float(dem.pixel_width)))
        dy = EARTH_RADIUS_M * math.radians(abs(float(dem.pixel_height)))
        return dx, dy

    return abs(float(dem.pixel_width)), abs(float(dem.pixel_height))


def resolve_ray_step_m(dem, reference_lat: float, configured_step_m: float | None) -> float:
    dx, dy = dem_ground_resolution_m(dem, reference_lat)
    recommended = max(MIN_RAY_STEP_M, 0.5 * min(dx, dy))
    if configured_step_m is None or configured_step_m <= 0.0:
        return recommended
    return max(MIN_RAY_STEP_M, min(float(configured_step_m), recommended))


class LocalFrame:
    def __init__(self, lat0_deg: float, lon0_deg: float) -> None:
        self.lat0_deg = lat0_deg
        self.lon0_deg = lon0_deg
        self.lat0_rad = math.radians(lat0_deg)
        self.cos_lat0 = math.cos(self.lat0_rad)

    def to_enu(self, point: GeoPoint) -> np.ndarray:
        east = EARTH_RADIUS_M * math.radians(point.lon - self.lon0_deg) * self.cos_lat0
        north = EARTH_RADIUS_M * math.radians(point.lat - self.lat0_deg)
        return np.array([east, north, point.alt], dtype=float)

    def to_geo(self, xyz: np.ndarray) -> GeoPoint:
        east, north, up = (float(v) for v in xyz)
        lat = self.lat0_deg + math.degrees(north / EARTH_RADIUS_M)
        lon = self.lon0_deg + math.degrees(east / (EARTH_RADIUS_M * self.cos_lat0))
        return GeoPoint(lat=lat, lon=lon, alt=up)

    def east_from_lon(self, lon: np.ndarray) -> np.ndarray:
        return EARTH_RADIUS_M * np.deg2rad(np.asarray(lon) - self.lon0_deg) * self.cos_lat0

    def north_from_lat(self, lat: np.ndarray) -> np.ndarray:
        return EARTH_RADIUS_M * np.deg2rad(np.asarray(lat) - self.lat0_deg)


class GeoTiffDem:
    def __init__(self, path: Path) -> None:
        self.path = resolve_existing_path(path)
        with tifffile.TiffFile(self.path) as tif:
            page = tif.pages[0]
            tags = page.tags
            self.data = page.asarray().astype(np.float64)
            scale = tags[33550].value
            tie = tags[33922].value
            self.pixel_width_deg = float(scale[0])
            self.pixel_height_deg = float(scale[1])
            self.lon_min = float(tie[3])
            self.lat_max = float(tie[4])
            self.nodata = float(tags[42113].value) if 42113 in tags else None

        if self.nodata is not None:
            self.data[self.data == self.nodata] = np.nan

        self.height, self.width = self.data.shape
        self.lon_max = self.lon_min + self.pixel_width_deg * (self.width - 1)
        self.lat_min = self.lat_max - self.pixel_height_deg * (self.height - 1)

    def contains(self, lat: float, lon: float) -> bool:
        return self.lat_min <= lat <= self.lat_max and self.lon_min <= lon <= self.lon_max

    def latlon_to_rowcol(self, lat: float, lon: float) -> tuple[float, float]:
        row = (self.lat_max - lat) / self.pixel_height_deg
        col = (lon - self.lon_min) / self.pixel_width_deg
        return row, col

    def rowcol_to_latlon(self, row: np.ndarray, col: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        lat = self.lat_max - np.asarray(row) * self.pixel_height_deg
        lon = self.lon_min + np.asarray(col) * self.pixel_width_deg
        return lat, lon

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

    def extract_window(
        self,
        lat_min: float,
        lat_max: float,
        lon_min: float,
        lon_max: float,
        margin_pixels: int = 80,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        row_top, col_left = self.latlon_to_rowcol(lat_max, lon_min)
        row_bottom, col_right = self.latlon_to_rowcol(lat_min, lon_max)

        row0 = max(int(math.floor(min(row_top, row_bottom))) - margin_pixels, 0)
        row1 = min(int(math.ceil(max(row_top, row_bottom))) + margin_pixels, self.height - 1)
        col0 = max(int(math.floor(min(col_left, col_right))) - margin_pixels, 0)
        col1 = min(int(math.ceil(max(col_left, col_right))) + margin_pixels, self.width - 1)

        rows = np.arange(row0, row1 + 1)
        cols = np.arange(col0, col1 + 1)
        subset = self.data[row0 : row1 + 1, col0 : col1 + 1]
        lats, _ = self.rowcol_to_latlon(rows, np.zeros_like(rows))
        _, lons = self.rowcol_to_latlon(np.zeros_like(cols), cols)
        return subset, lats, lons


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm == 0.0:
        raise ValueError("Zero-length vector")
    return vector / norm


def rotate_basis(right: np.ndarray, up: np.ndarray, roll_deg: float) -> tuple[np.ndarray, np.ndarray]:
    if roll_deg == 0.0:
        return right, up
    angle = math.radians(roll_deg)
    new_right = right * math.cos(angle) + up * math.sin(angle)
    new_up = -right * math.sin(angle) + up * math.cos(angle)
    return normalize(new_right), normalize(new_up)


def build_camera_axes(uav_xyz: np.ndarray, center_xyz: np.ndarray, roll_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = normalize(center_xyz - uav_xyz)
    world_up = np.array([0.0, 0.0, 1.0], dtype=float)

    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-8:
        right = np.array([1.0, 0.0, 0.0], dtype=float)
    right = normalize(right)
    up = normalize(np.cross(right, forward))
    right, up = rotate_basis(right, up, roll_deg)
    return forward, right, up


def order_corners_by_camera_axes(
    frame: LocalFrame,
    origin_xyz: np.ndarray,
    forward: np.ndarray,
    right: np.ndarray,
    up: np.ndarray,
    corner_points: list[GeoPoint],
) -> list[GeoPoint]:
    if len(corner_points) != 4:
        raise ValueError("Exactly 4 corner points are required.")

    projected: list[tuple[int, float, float]] = []
    for index, point in enumerate(corner_points):
        ray_xyz = frame.to_enu(point) - origin_xyz
        forward_component = float(np.dot(ray_xyz, forward))
        if forward_component <= 0.0:
            raise ValueError("Corner lies behind the camera reference frame.")
        image_x = float(np.dot(ray_xyz, right) / forward_component)
        image_y = float(np.dot(ray_xyz, up) / forward_component)
        projected.append((index, image_x, image_y))

    projected.sort(key=lambda item: item[2], reverse=True)
    top = sorted(projected[:2], key=lambda item: item[1])
    bottom = sorted(projected[2:], key=lambda item: item[1])
    order = [top[0][0], top[1][0], bottom[1][0], bottom[0][0]]
    return [corner_points[index] for index in order]


def sort_corners(points_xyz: np.ndarray, points_geo: list[GeoPoint]) -> tuple[np.ndarray, list[GeoPoint]]:
    center_xy = np.mean(points_xyz[:, :2], axis=0)
    angles = np.arctan2(points_xyz[:, 1] - center_xy[1], points_xyz[:, 0] - center_xy[0])
    order = np.argsort(angles)
    sorted_xyz = points_xyz[order]
    sorted_geo = [points_geo[int(i)] for i in order]
    return sorted_xyz, sorted_geo


def intersect_ray_with_dem(
    dem: GeoTiffDem,
    frame: LocalFrame,
    origin_xyz: np.ndarray,
    direction_xyz: np.ndarray,
    max_distance_m: float,
    step_m: float,
) -> tuple[np.ndarray, GeoPoint]:
    direction_xyz = normalize(direction_xyz)

    origin_geo = frame.to_geo(origin_xyz)
    if not dem.contains(origin_geo.lat, origin_geo.lon):
        raise ValueError("UAV horizontal position is outside the selected DEM tile.")

    previous_t = 0.0
    previous_ground = dem.sample_elevation(origin_geo.lat, origin_geo.lon)
    previous_diff = origin_xyz[2] - previous_ground
    if previous_diff <= 0.0:
        raise ValueError("UAV altitude is not above the DEM surface.")

    t = step_m
    while t <= max_distance_m:
        point_xyz = origin_xyz + direction_xyz * t
        point_geo = frame.to_geo(point_xyz)

        if not dem.contains(point_geo.lat, point_geo.lon):
            break

        ground = dem.sample_elevation(point_geo.lat, point_geo.lon)
        diff = point_xyz[2] - ground

        if diff <= 0.0:
            lower = previous_t
            upper = t
            for _ in range(45):
                middle = 0.5 * (lower + upper)
                mid_xyz = origin_xyz + direction_xyz * middle
                mid_geo = frame.to_geo(mid_xyz)
                mid_ground = dem.sample_elevation(mid_geo.lat, mid_geo.lon)
                mid_diff = mid_xyz[2] - mid_ground
                if mid_diff > 0.0:
                    lower = middle
                else:
                    upper = middle

            hit_xyz = origin_xyz + direction_xyz * upper
            hit_geo = frame.to_geo(hit_xyz)
            hit_ground = dem.sample_elevation(hit_geo.lat, hit_geo.lon)
            hit_xyz[2] = hit_ground
            return hit_xyz, GeoPoint(lat=hit_geo.lat, lon=hit_geo.lon, alt=hit_ground)

        previous_t = t
        t += step_m

    raise ValueError("A camera ray left the DEM tile before hitting terrain.")


def polygon_area_2d(points_xy: np.ndarray) -> float:
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def polygon_area_3d(points_xyz: np.ndarray) -> float:
    tri1 = 0.5 * np.linalg.norm(np.cross(points_xyz[1] - points_xyz[0], points_xyz[2] - points_xyz[0]))
    tri2 = 0.5 * np.linalg.norm(np.cross(points_xyz[2] - points_xyz[0], points_xyz[3] - points_xyz[0]))
    return float(tri1 + tri2)


def estimate_surface_area(
    dem: GeoTiffDem,
    frame: LocalFrame,
    polygon_geo: list[GeoPoint],
    polygon_xy: np.ndarray,
) -> float:
    lat_values = np.array([point.lat for point in polygon_geo], dtype=float)
    lon_values = np.array([point.lon for point in polygon_geo], dtype=float)
    subset, lats, lons = dem.extract_window(
        lat_min=float(np.min(lat_values)),
        lat_max=float(np.max(lat_values)),
        lon_min=float(np.min(lon_values)),
        lon_max=float(np.max(lon_values)),
        margin_pixels=8,
    )

    east = frame.east_from_lon(lons)
    north = frame.north_from_lat(lats)
    xx, yy = np.meshgrid(east, north)

    closed_polygon = np.vstack([polygon_xy[:, :2], polygon_xy[0, :2]])
    path = MplPath(closed_polygon)
    center_x = 0.25 * (xx[:-1, :-1] + xx[:-1, 1:] + xx[1:, :-1] + xx[1:, 1:])
    center_y = 0.25 * (yy[:-1, :-1] + yy[:-1, 1:] + yy[1:, :-1] + yy[1:, 1:])
    mask = path.contains_points(np.column_stack([center_x.ravel(), center_y.ravel()]), radius=1e-9).reshape(center_x.shape)

    p00 = np.stack([xx[:-1, :-1], yy[:-1, :-1], subset[:-1, :-1]], axis=-1)
    p01 = np.stack([xx[:-1, 1:], yy[:-1, 1:], subset[:-1, 1:]], axis=-1)
    p10 = np.stack([xx[1:, :-1], yy[1:, :-1], subset[1:, :-1]], axis=-1)
    p11 = np.stack([xx[1:, 1:], yy[1:, 1:], subset[1:, 1:]], axis=-1)

    area1 = 0.5 * np.linalg.norm(np.cross(p01 - p00, p11 - p00), axis=-1)
    area2 = 0.5 * np.linalg.norm(np.cross(p11 - p00, p10 - p00), axis=-1)
    cell_area = area1 + area2

    valid = mask
    valid &= np.all(np.isfinite(p00), axis=-1)
    valid &= np.all(np.isfinite(p01), axis=-1)
    valid &= np.all(np.isfinite(p10), axis=-1)
    valid &= np.all(np.isfinite(p11), axis=-1)
    return float(np.sum(cell_area[valid]))


def ground_distance_m(frame: LocalFrame, p1: GeoPoint, p2: GeoPoint) -> float:
    xyz1 = frame.to_enu(p1)
    xyz2 = frame.to_enu(p2)
    delta = xyz2 - xyz1
    return float(np.hypot(delta[0], delta[1]))


def resolve_input_point(dem: GeoTiffDem, point: GeoPointInput) -> GeoPoint:
    altitude = point.alt
    if altitude is None:
        altitude = dem.sample_elevation(point.lat, point.lon)
    return GeoPoint(lat=point.lat, lon=point.lon, alt=float(altitude))


def build_polygon_result(
    name: str,
    frame: LocalFrame,
    corner_points: list[GeoPoint],
    preserve_input_order: bool = False,
) -> PolygonResult:
    corner_xyz = np.vstack([frame.to_enu(point) for point in corner_points])
    if preserve_input_order:
        ordered_xyz = corner_xyz
        ordered_points = corner_points
    else:
        ordered_xyz, ordered_points = sort_corners(corner_xyz, corner_points)
    area_m2 = polygon_area_2d(ordered_xyz[:, :2])
    return PolygonResult(
        name=name,
        corner_points=ordered_points,
        corner_xyz=ordered_xyz,
        area_m2=area_m2,
    )


def make_plot_window(
    dem: GeoTiffDem,
    polygon_geo: list[GeoPoint],
    center_geo: GeoPoint,
    extra_polygon_geo: list[GeoPoint] | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lat_values = [point.lat for point in polygon_geo] + [center_geo.lat]
    lon_values = [point.lon for point in polygon_geo] + [center_geo.lon]
    if extra_polygon_geo:
        lat_values.extend(point.lat for point in extra_polygon_geo)
        lon_values.extend(point.lon for point in extra_polygon_geo)
    lat_span = max(lat_values) - min(lat_values)
    lon_span = max(lon_values) - min(lon_values)
    lat_pad = max(0.002, lat_span * 0.4)
    lon_pad = max(0.002, lon_span * 0.4)

    return dem.extract_window(
        lat_min=max(dem.lat_min, min(lat_values) - lat_pad),
        lat_max=min(dem.lat_max, max(lat_values) + lat_pad),
        lon_min=max(dem.lon_min, min(lon_values) - lon_pad),
        lon_max=min(dem.lon_max, max(lon_values) + lon_pad),
        margin_pixels=0,
    )


def plot_2d(
    dem: GeoTiffDem,
    frame: LocalFrame,
    computed_polygon: PolygonResult,
    uav_xyz: np.ndarray,
    center_xyz: np.ndarray,
    comparison_polygon: PolygonResult | None = None,
) -> plt.Figure:
    subset, lats, lons = make_plot_window(
        dem,
        computed_polygon.corner_points,
        frame.to_geo(center_xyz),
        extra_polygon_geo=None if comparison_polygon is None else comparison_polygon.corner_points,
    )
    east = frame.east_from_lon(lons)
    north = frame.north_from_lat(lats)
    extent = [float(east.min()), float(east.max()), float(north.min()), float(north.max())]

    lat_mean = float(np.mean([point.lat for point in computed_polygon.corner_points]))
    dx = EARTH_RADIUS_M * math.cos(math.radians(lat_mean)) * math.radians(dem.pixel_width_deg)
    dy = EARTH_RADIUS_M * math.radians(dem.pixel_height_deg)

    light = LightSource(azdeg=315, altdeg=45)
    shaded = light.shade(subset, cmap=plt.cm.terrain, vert_exag=0.8, dx=dx, dy=dy, blend_mode="overlay")

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(shaded, extent=extent, origin="upper")

    ring = np.vstack([computed_polygon.corner_xyz[:, :2], computed_polygon.corner_xyz[0, :2]])
    ax.plot(ring[:, 0], ring[:, 1], color="crimson", linewidth=2.5, label=computed_polygon.name)
    ax.fill(ring[:, 0], ring[:, 1], color="crimson", alpha=0.18)
    ax.scatter(
        computed_polygon.corner_xyz[:, 0],
        computed_polygon.corner_xyz[:, 1],
        c="white",
        edgecolors="crimson",
        s=34,
        zorder=6,
    )
    if comparison_polygon is not None:
        comp_ring = np.vstack([comparison_polygon.corner_xyz[:, :2], comparison_polygon.corner_xyz[0, :2]])
        ax.plot(
            comp_ring[:, 0],
            comp_ring[:, 1],
            color="dodgerblue",
            linewidth=2.2,
            linestyle="--",
            label=comparison_polygon.name,
        )
        ax.fill(comp_ring[:, 0], comp_ring[:, 1], color="dodgerblue", alpha=0.20)
        ax.scatter(
            comparison_polygon.corner_xyz[:, 0],
            comparison_polygon.corner_xyz[:, 1],
            c="white",
            edgecolors="dodgerblue",
            marker="s",
            s=34,
            zorder=6,
        )
    ax.plot(
        [uav_xyz[0], center_xyz[0]],
        [uav_xyz[1], center_xyz[1]],
        color="gold",
        linewidth=2.0,
        linestyle="-",
        label="Center line",
    )

    ax.scatter(uav_xyz[0], uav_xyz[1], c="black", s=70, marker="^", label="UAV")
    ax.scatter(center_xyz[0], center_xyz[1], c="deepskyblue", s=60, marker="x", label="Center")

    for label, point in zip(FOOTPRINT_SPEC_CORNER_LABELS, computed_polygon.corner_xyz):
        ax.scatter(point[0], point[1], c="white", edgecolors="black", s=42, zorder=5)
        ax.text(point[0] + 10.0, point[1] + 10.0, label, fontsize=9, color="black")
    if comparison_polygon is not None:
        reference_labels = [f"R{idx + 1}" for idx in range(len(FOOTPRINT_SPEC_CORNER_LABELS))]
        for label, point in zip(reference_labels, comparison_polygon.corner_xyz):
            ax.text(point[0] + 10.0, point[1] - 14.0, label, fontsize=9, color="dodgerblue")

    ax.set_title("DEM Footprint Projection")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    fig.tight_layout()
    return fig


def plot_3d(
    dem: GeoTiffDem,
    frame: LocalFrame,
    polygon_xyz: np.ndarray,
    polygon_geo: list[GeoPoint],
    uav_xyz: np.ndarray,
    center_xyz: np.ndarray,
) -> plt.Figure:
    subset, lats, lons = make_plot_window(dem, polygon_geo, frame.to_geo(center_xyz))
    stride = max(1, int(max(subset.shape) / 120))
    subset_small = subset[::stride, ::stride]
    lats_small = lats[::stride]
    lons_small = lons[::stride]

    east = frame.east_from_lon(lons_small)
    north = frame.north_from_lat(lats_small)
    xx, yy = np.meshgrid(east, north)

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(xx, yy, subset_small, cmap="terrain", linewidth=0, antialiased=False, alpha=0.95)

    ring = np.vstack([polygon_xyz, polygon_xyz[0]])
    ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color="crimson", linewidth=2.5)
    ax.plot(
        [uav_xyz[0], center_xyz[0]],
        [uav_xyz[1], center_xyz[1]],
        [uav_xyz[2], center_xyz[2]],
        color="gold",
        linewidth=2.0,
    )
    ax.scatter(uav_xyz[0], uav_xyz[1], uav_xyz[2], color="black", s=55, marker="^")
    ax.scatter(center_xyz[0], center_xyz[1], center_xyz[2], color="deepskyblue", s=45, marker="x")

    for point in polygon_xyz:
        ax.plot(
            [uav_xyz[0], point[0]],
            [uav_xyz[1], point[1]],
            [uav_xyz[2], point[2]],
            color="gray",
            linestyle="--",
            linewidth=1.0,
        )

    ax.set_title("3D Footprint on DEM")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_zlabel("Elevation [m]")
    ax.view_init(elev=35, azim=-55)
    fig.tight_layout()
    return fig


def run_footprint_analysis() -> tuple[GeoTiffDem, LocalFrame, FootprintOutputs]:
    horizontal_fov_deg, vertical_fov_deg = calculate_fov_components(
        fov_value_deg=INPUT_DIAG_FOV_DEG,
        aspect_ratio=INPUT_ASPECT_RATIO,
        interpretation=INPUT_FOV_INTERPRETATION,
    )

    dem = GeoTiffDem(INPUT_DEM_PATH)
    if not dem.contains(INPUT_UAV_POSITION.lat, INPUT_UAV_POSITION.lon):
        raise ValueError("The UAV latitude/longitude must be inside the selected DEM tile.")
    if not dem.contains(INPUT_FOOTPRINT_CENTER.lat, INPUT_FOOTPRINT_CENTER.lon):
        raise ValueError("The footprint center latitude/longitude must be inside the selected DEM tile.")

    uav_geo = INPUT_UAV_POSITION
    center_geo = resolve_ground_point(
        dem,
        INPUT_FOOTPRINT_CENTER,
        uav_alt=uav_geo.alt,
        altitude_mode=INPUT_GROUND_ALTITUDE_MODE,
    )
    frame = LocalFrame(lat0_deg=center_geo.lat, lon0_deg=center_geo.lon)

    uav_xyz = frame.to_enu(uav_geo)
    center_xyz = frame.to_enu(center_geo)
    forward, right, up = build_camera_axes(uav_xyz, center_xyz, INPUT_ROLL_DEG)
    ray_step_m = resolve_ray_step_m(dem, center_geo.lat, INPUT_RAY_STEP_M)

    tan_x = math.tan(math.radians(horizontal_fov_deg) / 2.0)
    tan_y = math.tan(math.radians(vertical_fov_deg) / 2.0)
    corner_definition = FOOTPRINT_SPEC_CORNER_DEFINITION

    corner_xyz = []
    corner_geo = []
    for sx, sy in corner_definition:
        ray = normalize(forward + sx * tan_x * right + sy * tan_y * up)
        hit_xyz, hit_geo = intersect_ray_with_dem(
            dem=dem,
            frame=frame,
            origin_xyz=uav_xyz,
            direction_xyz=ray,
            max_distance_m=INPUT_MAX_DISTANCE_M,
            step_m=ray_step_m,
        )
        corner_xyz.append(hit_xyz)
        corner_geo.append(hit_geo)

    corner_geo = order_corners_by_camera_axes(
        frame=frame,
        origin_xyz=uav_xyz,
        forward=forward,
        right=right,
        up=up,
        corner_points=corner_geo,
    )

    center_hit_xyz, center_hit_geo = intersect_ray_with_dem(
        dem=dem,
        frame=frame,
        origin_xyz=uav_xyz,
        direction_xyz=forward,
        max_distance_m=INPUT_MAX_DISTANCE_M,
        step_m=ray_step_m,
    )

    center_horizontal_error_m = ground_distance_m(frame, center_geo, center_hit_geo)
    center_vertical_error_m = abs(center_geo.alt - center_hit_geo.alt)
    computed_footprint = build_polygon_result(
        "Computed footprint",
        frame,
        corner_geo,
        preserve_input_order=True,
    )

    comparison_footprint = None
    area_difference_m2 = None
    if INPUT_COMPARISON_FOOTPRINT_CORNERS:
        if len(INPUT_COMPARISON_FOOTPRINT_CORNERS) != 4:
            raise ValueError("INPUT_COMPARISON_FOOTPRINT_CORNERS must contain exactly 4 points.")
        comparison_points = [resolve_input_point(dem, point) for point in INPUT_COMPARISON_FOOTPRINT_CORNERS]
        comparison_points = order_corners_by_camera_axes(
            frame=frame,
            origin_xyz=uav_xyz,
            forward=forward,
            right=right,
            up=up,
            corner_points=comparison_points,
        )
        comparison_footprint = build_polygon_result(
            "Reference footprint",
            frame,
            comparison_points,
            preserve_input_order=True,
        )
        area_difference_m2 = abs(computed_footprint.area_m2 - comparison_footprint.area_m2)

    outputs = FootprintOutputs(
        fov_interpretation=INPUT_FOV_INTERPRETATION,
        diagonal_fov_deg=INPUT_DIAG_FOV_DEG,
        horizontal_fov_deg=horizontal_fov_deg,
        vertical_fov_deg=vertical_fov_deg,
        aspect_ratio=INPUT_ASPECT_RATIO,
        roll_deg=INPUT_ROLL_DEG,
        ground_altitude_mode=INPUT_GROUND_ALTITUDE_MODE,
        ray_step_m=ray_step_m,
        uav=uav_geo,
        center_input=center_geo,
        center_hit=center_hit_geo,
        center_error_horizontal_m=center_horizontal_error_m,
        center_error_vertical_m=center_vertical_error_m,
        uav_xyz=uav_xyz,
        center_hit_xyz=center_hit_xyz,
        computed_footprint=computed_footprint,
        comparison_footprint=comparison_footprint,
        area_difference_m2=area_difference_m2,
    )
    return dem, frame, outputs


def print_outputs(outputs: FootprintOutputs) -> None:
    print(f"DEM              : {INPUT_DEM_PATH}")
    print(f"Matplotlib       : {MATPLOTLIB_BACKEND}")
    print(
        "FOV              : "
        f"mode={outputs.fov_interpretation}, "
        f"diag={outputs.diagonal_fov_deg:.3f} deg, "
        f"h={outputs.horizontal_fov_deg:.3f} deg, "
        f"v={outputs.vertical_fov_deg:.3f} deg, "
        f"aspect={outputs.aspect_ratio:.6f}"
    )
    print(f"Ground alt mode  : {outputs.ground_altitude_mode}")
    print(f"Ray step         : {outputs.ray_step_m:.3f} m")
    print(f"UAV              : lat={outputs.uav.lat:.6f}, lon={outputs.uav.lon:.6f}, alt={outputs.uav.alt:.2f} m")
    print(
        "Center input     : "
        f"lat={outputs.center_input.lat:.6f}, lon={outputs.center_input.lon:.6f}, alt={outputs.center_input.alt:.2f} m"
    )
    print(
        "Center hit       : "
        f"lat={outputs.center_hit.lat:.6f}, lon={outputs.center_hit.lon:.6f}, alt={outputs.center_hit.alt:.2f} m"
    )
    print(
        "Center error     : "
        f"horizontal={outputs.center_error_horizontal_m:.3f} m, "
        f"vertical={outputs.center_error_vertical_m:.3f} m"
    )
    print("Computed footprint corners:")
    for label, point in zip(FOOTPRINT_SPEC_CORNER_LABELS, outputs.computed_footprint.corner_points):
        print(f"  {label}: lat={point.lat:.6f}, lon={point.lon:.6f}, alt={point.alt:.2f} m")
    print(f"Computed footprint area : {outputs.computed_footprint.area_m2:.2f} m^2")

    if outputs.comparison_footprint is not None:
        print("Reference footprint corners:")
        for label, point in zip(FOOTPRINT_SPEC_CORNER_LABELS, outputs.comparison_footprint.corner_points):
            print(f"  {label}: lat={point.lat:.6f}, lon={point.lon:.6f}, alt={point.alt:.2f} m")
        print(f"Reference footprint area: {outputs.comparison_footprint.area_m2:.2f} m^2")
        print(f"Area difference         : {outputs.area_difference_m2:.2f} m^2")


def main() -> None:
    dem, frame, outputs = run_footprint_analysis()

    fig_2d = plot_2d(
        dem,
        frame,
        outputs.computed_footprint,
        outputs.uav_xyz,
        outputs.center_hit_xyz,
        comparison_polygon=outputs.comparison_footprint,
    )
    print_outputs(outputs)

    if INPUT_SHOW_PLOTS:
        if MATPLOTLIB_BACKEND == "Agg":
            print("Interactive view : unavailable (Tk backend not found).")
        else:
            plt.show()

    plt.close(fig_2d)


if __name__ == "__main__":
    main()
