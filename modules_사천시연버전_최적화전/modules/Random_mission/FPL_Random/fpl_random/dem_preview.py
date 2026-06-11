from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

from .areas import LatLon

try:
    import numpy as np
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import Affine
    from rasterio.windows import from_bounds
    from rasterio.warp import transform
except Exception:
    rasterio = None
    transform = None
    Resampling = None
    from_bounds = None
    np = None


@dataclass(frozen=True)
class MissionGeometry:
    kind: str
    points: List[LatLon]
    altitudes: Optional[List[Optional[float]]] = None


def _load_mission_geometry(imp_path: Path, mission_id: int) -> Optional[MissionGeometry]:
    if not imp_path.exists():
        return None
    data = json.loads(imp_path.read_text(encoding="utf-8"))
    for mission in data.get("inputMissionList", []) or []:
        if int(mission.get("inputMissionID", 0) or 0) != mission_id:
            continue
        detail = mission.get("missionDetail") or {}
        line_list = detail.get("lineList") or []
        if line_list:
            coords = line_list[0].get("coordinateList") or []
            points = [LatLon(c["latitude"], c["longitude"]) for c in coords]
            altitudes: List[Optional[float]] = []
            for c in coords:
                alt = c.get("altitude")
                altitudes.append(float(alt) if alt is not None else None)
            if not any(a is not None for a in altitudes):
                altitudes = None
            return MissionGeometry(kind="line", points=points, altitudes=altitudes)
        area_list = detail.get("areaList") or []
        if area_list:
            coords = area_list[0].get("coordinateList") or []
            points = [LatLon(c["latitude"], c["longitude"]) for c in coords]
            if points and (points[0] != points[-1]):
                points.append(points[0])
            return MissionGeometry(kind="area", points=points)
        coord_list = detail.get("coordinateList") or []
        if coord_list:
            point = coord_list[0]
            alt = point.get("altitude")
            altitudes = [float(alt)] if alt is not None else None
            return MissionGeometry(kind="point", points=[LatLon(point["latitude"], point["longitude"])], altitudes=altitudes)
    return None


def _utm_points(ds, points: List[LatLon]) -> List[Tuple[float, float]]:
    lats = [p.latitude for p in points]
    lons = [p.longitude for p in points]
    xs, ys = transform("EPSG:4326", ds.crs, lons, lats)
    return list(zip(xs, ys))


def _bounds_with_buffer(points: List[Tuple[float, float]], buffer_m: float) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    minx = min(xs) - buffer_m
    maxx = max(xs) + buffer_m
    miny = min(ys) - buffer_m
    maxy = max(ys) + buffer_m
    return minx, miny, maxx, maxy


def build_mission_3d_figure(
    imp_path: Path,
    mission_id: int,
    dem_path: Path,
    *,
    buffer_m: float = 1500.0,
    grid_size: int = 180,
    view_elev: float = 38.0,
    view_azim: float = 130.0,
    z_max: float = 1500.0,
    line_offset_m: float = 200.0,
    surface_alpha: float = 0.75,
) -> Optional[Tuple[object, object]]:
    if rasterio is None or transform is None or np is None:
        return None
    if not dem_path.exists():
        return None

    geom = _load_mission_geometry(imp_path, mission_id)
    if not geom or not geom.points:
        return None

    with rasterio.open(dem_path) as ds:
        if ds.crs is None:
            return None
        utm_pts = _utm_points(ds, geom.points)
        minx, miny, maxx, maxy = _bounds_with_buffer(utm_pts, buffer_m)
        window = from_bounds(minx, miny, maxx, maxy, ds.transform)
        data = ds.read(
            1,
            window=window,
            out_shape=(grid_size, grid_size),
            resampling=Resampling.bilinear,
        )
        if ds.nodata is not None:
            data = np.where(data == ds.nodata, np.nan, data)
        terrain_min = float(np.nanmin(data)) if np.isfinite(np.nanmin(data)) else 0.0
        terrain_max = float(np.nanmax(data)) if np.isfinite(np.nanmax(data)) else terrain_min + 1.0
        terrain_range = max(1.0, terrain_max - terrain_min)
        transform_win = ds.window_transform(window)
        scale_x = window.width / data.shape[1]
        scale_y = window.height / data.shape[0]
        transform_scaled = transform_win * Affine.scale(scale_x, scale_y)
        cols = np.arange(data.shape[1])
        rows = np.arange(data.shape[0])
        xs = transform_scaled.c + (cols + 0.5) * transform_scaled.a
        ys = transform_scaled.f + (rows + 0.5) * transform_scaled.e
        X, Y = np.meshgrid(xs, ys)

        sample_z = [val[0] for val in ds.sample(utm_pts)]
        z_offset = max(line_offset_m, terrain_range * 0.18)
        xs_line = [p[0] for p in utm_pts]
        ys_line = [p[1] for p in utm_pts]
        ground_zs = [z if z is not None else terrain_min for z in sample_z]
        altitudes = None
        if geom.kind in ("line", "point") and geom.altitudes and len(geom.altitudes) == len(utm_pts):
            if any(a is not None for a in geom.altitudes):
                altitudes = [float(a) if a is not None else 0.0 for a in geom.altitudes]
        if altitudes is not None:
            zs_line = altitudes
            z_offset = 0.0
        else:
            zs_line = [z + z_offset if z is not None else terrain_min + z_offset for z in sample_z]
        line_peak = max(zs_line) if zs_line else terrain_min

        try:
            import matplotlib.pyplot as plt
            from matplotlib.colors import Normalize
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
            from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
        except Exception:
            return None

        fig = plt.figure(figsize=(7.2, 6.2))
        ax = fig.add_subplot(111, projection="3d")
        surface = ax.plot_surface(
            X,
            Y,
            data,
            cmap="terrain",
            linewidth=0,
            antialiased=True,
            alpha=surface_alpha,
            zorder=0,
        )
        try:
            fig.colorbar(surface, ax=ax, shrink=0.6, pad=0.08, label="Elevation (m)")
        except Exception:
            pass

        try:
            levels = np.linspace(terrain_min, terrain_max, 10)
            ax.contour(
                X,
                Y,
                data,
                levels=levels,
                colors="k",
                linewidths=0.4,
                alpha=0.25,
                zorder=1,
            )
        except Exception:
            pass

        ground_trace_offset = max(8.0, terrain_range * 0.01)
        if geom.kind == "line" and len(xs_line) >= 2:
            segments = [
                (
                    (xs_line[i], ys_line[i], ground_zs[i] + ground_trace_offset),
                    (xs_line[i + 1], ys_line[i + 1], ground_zs[i + 1] + ground_trace_offset),
                )
                for i in range(len(xs_line) - 1)
            ]
            norm = Normalize(vmin=terrain_min, vmax=terrain_max)
            ground_lc = Line3DCollection(segments, cmap="viridis", norm=norm)
            ground_lc.set_array(np.array(ground_zs[:-1]))
            ground_lc.set_linewidth(5.0)
            ground_lc.set_alpha(0.95)
            ground_lc.set_zorder(8)
            ax.add_collection3d(ground_lc)

        if geom.kind == "point":
            ax.scatter(xs_line, ys_line, zs_line, color="#ff8f00", s=70, edgecolor="#111827")
        elif geom.kind == "area":
            area_offset = max(20.0, terrain_range * 0.02)
            area_zs = [z + area_offset if z is not None else terrain_min + area_offset for z in ground_zs]
            poly = list(zip(xs_line, ys_line, area_zs))
            poly3d = Poly3DCollection([poly], alpha=0.75)
            poly3d.set_facecolor("#f97316")
            poly3d.set_edgecolor("#7c2d12")
            poly3d.set_linewidth(3.0)
            poly3d.set_zorder(11)
            ax.add_collection3d(poly3d)
            if len(xs_line) >= 2:
                outline = [
                    (
                        (xs_line[i], ys_line[i], area_zs[i]),
                        (xs_line[i + 1], ys_line[i + 1], area_zs[i + 1]),
                    )
                    for i in range(len(xs_line) - 1)
                ]
                outline_lc = Line3DCollection(outline)
                outline_lc.set_color("#1f2937")
                outline_lc.set_linewidth(2.2)
                outline_lc.set_zorder(12)
                ax.add_collection3d(outline_lc)
        else:
            ax.plot(xs_line, ys_line, zs_line, color="#111827", linewidth=7.0, alpha=0.6, zorder=10)
            ax.plot(xs_line, ys_line, zs_line, color="#f59e0b", linewidth=4.2, zorder=11)
            if len(xs_line) >= 2:
                ax.scatter(xs_line[0], ys_line[0], zs_line[0], color="#10b981", s=60, zorder=12)
                ax.scatter(xs_line[-1], ys_line[-1], zs_line[-1], color="#ef4444", s=60, zorder=12)
                label_offset = max(10.0, (line_peak if line_peak > 0 else 1.0) * 0.02)
                ax.text(xs_line[0], ys_line[0], zs_line[0] + label_offset, "S", color="#10b981")
                ax.text(xs_line[-1], ys_line[-1], zs_line[-1] + label_offset, "E", color="#ef4444")

        ax.set_title(f"Mission {mission_id} 3D")
        ax.set_xlabel("Easting (m)")
        ax.set_ylabel("Northing (m)")
        ax.set_zlabel("Elevation (m)")
        ax.set_zlim(0, max(z_max, line_peak + z_offset * 0.2))
        ax.view_init(elev=view_elev, azim=view_azim)
        fig.tight_layout()

        return fig, ax


def render_mission_3d(
    imp_path: Path,
    mission_id: int,
    dem_path: Path,
    out_dir: Path,
    *,
    buffer_m: float = 1500.0,
    grid_size: int = 180,
    view_elev: float = 38.0,
    view_azim: float = 130.0,
    z_max: float = 1500.0,
    line_offset_m: float = 200.0,
    surface_alpha: float = 0.75,
) -> Optional[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"mission_{mission_id:04d}.png"
    fig_ax = build_mission_3d_figure(
        imp_path,
        mission_id,
        dem_path,
        buffer_m=buffer_m,
        grid_size=grid_size,
        view_elev=view_elev,
        view_azim=view_azim,
        z_max=z_max,
        line_offset_m=line_offset_m,
        surface_alpha=surface_alpha,
    )
    if not fig_ax:
        return None
    fig, _ax = fig_ax
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return None
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return out_path
