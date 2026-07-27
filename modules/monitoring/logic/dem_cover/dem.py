"""DEM loading and coordinate helpers.

``DemGrid`` reads a GeoTIFF once, down-samples it to a coarse analysis grid
(so the whole pipeline stays inside the 5-second budget), and exposes:

* elevation array with nodata as NaN plus a boolean ``valid`` mask,
* native <-> lat/lon transforms (via pyproj),
* per-cell native-coordinate meshgrids ``X`` / ``Y`` (cell centres),
* fast native<->grid index conversion,
* bilinear point sampling and a hillshade for display.

The analysis works entirely in the DEM's native projected metres, so distances
and the weapon range need no conversion.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.transform import Affine
from pyproj import CRS, Transformer


@dataclass(frozen=True)
class DemMeta:
    name: str
    epsg: int
    width: int
    height: int
    cell_x_m: float
    cell_y_m: float
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    elev_min: float
    elev_max: float


class DemGrid:
    """A down-sampled DEM ready for vectorised terrain analysis."""

    def __init__(self, path: str | Path, *, max_dim: int = 340, fallback_epsg: int = 32652) -> None:
        self.path = Path(path).resolve()
        if not self.path.exists():
            raise FileNotFoundError(f"DEM not found: {self.path}")

        # Clamp the analysis resolution: too small is useless.  The upper bound
        # allows the native grid, because a down-sampled read disagrees with the
        # planner's own full-resolution terrain lookup by tens of metres on a
        # ridge - enough to certify a hide point the enemy can see, or a firing
        # point that cannot fire.  Per-ray cost is bounded by ``los_max_steps``
        # rather than by the grid size.
        max_dim = max(64, int(max_dim))

        with rasterio.open(self.path) as ds:
            factor = max(1, int(np.ceil(max(ds.width, ds.height) / float(max_dim))))
            out_w = max(1, ds.width // factor)
            out_h = max(1, ds.height // factor)
            data = ds.read(
                1,
                out_shape=(1, out_h, out_w),
                resampling=Resampling.bilinear,
            ).astype(np.float32)
            # Transform for the down-sampled grid.
            self.transform: Affine = ds.transform * Affine.scale(
                ds.width / out_w, ds.height / out_h
            )
            self._inv_transform: Affine = ~self.transform
            crs = ds.crs
            nodata = ds.nodata

        # nodata -> NaN. Bilinear resampling blends real values with the nodata
        # sentinel (e.g. -9999) near the DEM edge, producing large-negative
        # garbage; treat anything implausibly low as invalid too.
        if nodata is not None:
            data[data == np.float32(nodata)] = np.nan
        data[data < -1000.0] = np.nan

        self.elev = data
        self.valid = np.isfinite(data)
        if not np.any(self.valid):
            raise ValueError(f"DEM has no valid elevation samples: {self.path}")

        self.height, self.width = data.shape
        self.cell_x_m = float(abs(self.transform.a))
        self.cell_y_m = float(abs(self.transform.e))
        self.cell_m = 0.5 * (self.cell_x_m + self.cell_y_m)

        # Elevation used for terrain blocking: nodata never blocks a sightline.
        fill_low = float(np.nanmin(data)) - 10000.0
        self._block_elev = np.where(self.valid, data, fill_low).astype(np.float32)
        # Elevation used for gradients/hillshade: fill with the local minimum.
        self._display_elev = np.where(self.valid, data, float(np.nanmin(data))).astype(np.float32)

        # Native cell-centre coordinate meshgrids.
        cols = np.arange(self.width)
        rows = np.arange(self.height)
        cc, rr = np.meshgrid(cols, rows)
        xs, ys = self.transform * (cc + 0.5, rr + 0.5)
        self.X = np.asarray(xs, dtype=np.float64)
        self.Y = np.asarray(ys, dtype=np.float64)

        # Bounds (outer edges of the grid).
        x0, y0 = self.transform * (0, 0)
        x1, y1 = self.transform * (self.width, self.height)
        self.x_min, self.x_max = float(min(x0, x1)), float(max(x0, x1))
        self.y_min, self.y_max = float(min(y0, y1)), float(max(y0, y1))

        self.crs = crs if crs is not None else CRS.from_epsg(fallback_epsg)
        self._to_lonlat = Transformer.from_crs(self.crs, CRS.from_epsg(4326), always_xy=True)
        self._to_native = Transformer.from_crs(CRS.from_epsg(4326), self.crs, always_xy=True)

    # -- coordinate conversion --------------------------------------------
    def extent(self) -> tuple[float, float, float, float]:
        """(xmin, xmax, ymin, ymax) for ``imshow(extent=...)`` with origin='upper'."""
        return (self.x_min, self.x_max, self.y_min, self.y_max)

    def native_to_rowcol(self, x: float, y: float) -> tuple[float, float]:
        col, row = self._inv_transform * (float(x), float(y))
        return float(row), float(col)

    def rowcol_to_native(self, row: float, col: float) -> tuple[float, float]:
        x, y = self.transform * (float(col) + 0.5, float(row) + 0.5)
        return float(x), float(y)

    def nearest_index(self, x: float, y: float) -> tuple[int, int]:
        row, col = self.native_to_rowcol(x, y)
        ri = int(np.clip(round(row - 0.5), 0, self.height - 1))
        ci = int(np.clip(round(col - 0.5), 0, self.width - 1))
        return ri, ci

    def contains_native(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def native_to_latlon(self, x: float, y: float) -> tuple[float, float]:
        lon, lat = self._to_lonlat.transform(float(x), float(y))
        return float(lat), float(lon)

    def latlon_to_native(self, lat: float, lon: float) -> tuple[float, float]:
        x, y = self._to_native.transform(float(lon), float(lat))
        return float(x), float(y)

    # -- sampling ----------------------------------------------------------
    def sample_native(self, x: float, y: float) -> float:
        """Bilinear ground elevation at a native coordinate (NaN if unavailable)."""
        row, col = self.native_to_rowcol(x, y)
        row -= 0.5
        col -= 0.5
        r0 = int(np.clip(np.floor(row), 0, self.height - 1))
        c0 = int(np.clip(np.floor(col), 0, self.width - 1))
        r1 = min(r0 + 1, self.height - 1)
        c1 = min(c0 + 1, self.width - 1)
        fr = float(np.clip(row - r0, 0.0, 1.0))
        fc = float(np.clip(col - c0, 0.0, 1.0))
        vals = np.array(
            [self.elev[r0, c0], self.elev[r0, c1], self.elev[r1, c0], self.elev[r1, c1]],
            dtype=np.float64,
        )
        wts = np.array(
            [(1 - fr) * (1 - fc), (1 - fr) * fc, fr * (1 - fc), fr * fc],
            dtype=np.float64,
        )
        ok = np.isfinite(vals) & (wts > 0)
        if not np.any(ok):
            return float("nan")
        return float(np.sum(vals[ok] * wts[ok]) / np.sum(wts[ok]))

    # -- display -----------------------------------------------------------
    def hillshade(self, azimuth_deg: float = 315.0, altitude_deg: float = 45.0) -> np.ndarray:
        z = self._display_elev.astype(np.float64)
        gy, gx = np.gradient(z, self.cell_y_m, self.cell_x_m)
        slope = np.pi / 2.0 - np.arctan(np.hypot(gx, gy))
        aspect = np.arctan2(-gx, gy)
        az = np.radians(azimuth_deg)
        alt = np.radians(altitude_deg)
        shaded = np.sin(alt) * np.sin(slope) + np.cos(alt) * np.cos(slope) * np.cos(
            (az - np.pi / 2.0) - aspect
        )
        return np.clip(shaded, 0.0, 1.0)

    def metadata(self) -> DemMeta:
        return DemMeta(
            name=self.path.name,
            epsg=int(self.crs.to_epsg() or 0),
            width=self.width,
            height=self.height,
            cell_x_m=self.cell_x_m,
            cell_y_m=self.cell_y_m,
            x_min=self.x_min,
            x_max=self.x_max,
            y_min=self.y_min,
            y_max=self.y_max,
            elev_min=float(np.nanmin(self.elev)),
            elev_max=float(np.nanmax(self.elev)),
        )

    @property
    def block_elev(self) -> np.ndarray:
        """Elevation array where nodata is a large-negative fill (never blocks LOS)."""
        return self._block_elev
