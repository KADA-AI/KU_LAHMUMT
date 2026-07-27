"""Memory-conscious access to a DEM at its native raster resolution.

``NativeDemRaster`` is intended for the small-window second pass after the
coarse terrain search.  Unlike :class:`dem.DemGrid`, it deliberately does not
build full-size floating-point ``X``/``Y`` meshes (or retain a full validity
mask).  A typical 10 m ``int16`` GeoTIFF therefore costs roughly one compact
band in memory.

Invalid cells follow the same LOS convention as ``DemGrid``: elevations below
-1000 m are unavailable for point sampling and are represented by a very low
value in ``block_elev`` so they never create an artificial terrain obstacle.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rasterio
from pyproj import CRS, Transformer
from rasterio.transform import Affine


class NativeDemRaster:
    """Load band 1 at source resolution without allocating coordinate meshes."""

    def __init__(self, path: str | Path, *, fallback_epsg: int = 32652) -> None:
        self.path = Path(path).resolve()
        if not self.path.exists():
            raise FileNotFoundError(f"DEM not found: {self.path}")

        with rasterio.open(self.path) as ds:
            data = ds.read(1, masked=False)
            self.transform: Affine = ds.transform
            self._inv_transform: Affine = ~self.transform
            # Public alias for callers that need the affine inverse directly.
            self.inverse_transform: Affine = self._inv_transform
            self.height = int(ds.height)
            self.width = int(ds.width)
            self.crs = (
                CRS.from_user_input(ds.crs)
                if ds.crs is not None
                else CRS.from_epsg(fallback_epsg)
            )
            self.nodata = ds.nodata
            bounds = ds.bounds

        if data.ndim != 2 or data.shape != (self.height, self.width):
            raise ValueError(
                f"Unexpected DEM band shape {data.shape}; expected "
                f"({self.height}, {self.width})"
            )

        # Work in place where the source dtype can represent a safely-low LOS
        # sentinel.  Inje_10m.tif is int16, so this retains its ~46 MiB band
        # rather than expanding it to a ~92 MiB float32 band plus masks.
        keep_integer = (
            np.issubdtype(data.dtype, np.signedinteger)
            and int(np.iinfo(data.dtype).min) < -1000
        )
        if not keep_integer and data.dtype != np.dtype(np.float32):
            data = data.astype(np.float32)

        invalid = ~np.isfinite(data) if np.issubdtype(data.dtype, np.floating) else np.zeros(
            data.shape, dtype=bool
        )
        if self.nodata is not None and np.isfinite(self.nodata):
            nodata_value = float(self.nodata)
            if np.issubdtype(data.dtype, np.integer):
                dtype_info = np.iinfo(data.dtype)
                if (
                    nodata_value.is_integer()
                    and float(dtype_info.min) <= nodata_value <= float(dtype_info.max)
                ):
                    invalid |= data == data.dtype.type(int(nodata_value))
            else:
                invalid |= data == data.dtype.type(nodata_value)
        # Match DemGrid's protection against resampled/encoded nodata garbage.
        invalid |= data < -1000

        if np.all(invalid):
            raise ValueError(f"DEM has no valid elevation samples: {self.path}")

        if keep_integer:
            sentinel: int | float = int(np.iinfo(data.dtype).min)
        else:
            # All valid samples are >= -1000 by definition, so this remains a
            # finite, non-blocking value even for originally unsigned rasters.
            sentinel = float(np.min(data[~invalid]).astype(np.float64)) - 10000.0
        data[invalid] = sentinel
        # Do not retain ``invalid``: local-window callers can test > -1000 and
        # the temporary full-size boolean array is released after construction.
        self._invalid_sentinel = sentinel
        self._block_elev = np.ascontiguousarray(data)

        self.cell_x_m = float(np.hypot(self.transform.a, self.transform.d))
        self.cell_y_m = float(np.hypot(self.transform.b, self.transform.e))
        self.cell_m = 0.5 * (self.cell_x_m + self.cell_y_m)

        self.x_min = float(bounds.left)
        self.x_max = float(bounds.right)
        self.y_min = float(bounds.bottom)
        self.y_max = float(bounds.top)

        self._to_lonlat = Transformer.from_crs(
            self.crs, CRS.from_epsg(4326), always_xy=True
        )
        self._to_native = Transformer.from_crs(
            CRS.from_epsg(4326), self.crs, always_xy=True
        )

        if not bool(self.crs.is_projected):
            raise ValueError(
                f"Native DEM refinement requires a projected metre CRS: {self.crs}"
            )
        unit_scales = [
            float(axis.unit_conversion_factor)
            for axis in self.crs.axis_info[:2]
            if axis.unit_conversion_factor is not None
        ]
        if unit_scales and not all(np.isclose(scale, 1.0) for scale in unit_scales):
            raise ValueError(
                "Native DEM refinement requires projected coordinates in metres."
            )

    @property
    def block_elev(self) -> np.ndarray:
        """Compact elevation band with invalid cells replaced by a low sentinel."""
        return self._block_elev

    def extent(self) -> tuple[float, float, float, float]:
        """Return ``(xmin, xmax, ymin, ymax)`` in native projected metres."""
        return (self.x_min, self.x_max, self.y_min, self.y_max)

    # -- coordinate conversion -----------------------------------------
    def native_to_rowcol(self, x: float, y: float) -> tuple[float, float]:
        col, row = self._inv_transform * (float(x), float(y))
        return float(row), float(col)

    def rowcol_to_native(self, row: float, col: float) -> tuple[float, float]:
        """Return the native coordinate at the centre of a raster cell."""
        x, y = self.transform * (float(col) + 0.5, float(row) + 0.5)
        return float(x), float(y)

    def nearest_index(self, x: float, y: float) -> tuple[int, int]:
        row, col = self.native_to_rowcol(x, y)
        ri = int(np.clip(round(row - 0.5), 0, self.height - 1))
        ci = int(np.clip(round(col - 0.5), 0, self.width - 1))
        return ri, ci

    def contains_native(self, x: float, y: float) -> bool:
        if not (np.isfinite(x) and np.isfinite(y)):
            return False
        row, col = self.native_to_rowcol(float(x), float(y))
        # Affine-space testing also works for a rotated raster. Raster bounds
        # are half-open: row==height or col==width is already outside.
        return (
            0.0 <= row < self.height
            and 0.0 <= col < self.width
        )

    def native_to_latlon(self, x: float, y: float) -> tuple[float, float]:
        lon, lat = self._to_lonlat.transform(float(x), float(y))
        return float(lat), float(lon)

    def latlon_to_native(self, lat: float, lon: float) -> tuple[float, float]:
        x, y = self._to_native.transform(float(lon), float(lat))
        return float(x), float(y)

    # -- sampling -------------------------------------------------------
    def sample_native(self, x: float, y: float) -> float:
        """Bilinearly sample ground elevation; return NaN outside/nodata."""
        if not self.contains_native(x, y):
            return float("nan")

        row, col = self.native_to_rowcol(x, y)
        # Convert raster-edge coordinates to a cell-centre coordinate system.
        row -= 0.5
        col -= 0.5
        r0 = int(np.clip(np.floor(row), 0, self.height - 1))
        c0 = int(np.clip(np.floor(col), 0, self.width - 1))
        r1 = min(r0 + 1, self.height - 1)
        c1 = min(c0 + 1, self.width - 1)
        fr = float(np.clip(row - r0, 0.0, 1.0))
        fc = float(np.clip(col - c0, 0.0, 1.0))

        vals = np.asarray(
            (
                self._block_elev[r0, c0],
                self._block_elev[r0, c1],
                self._block_elev[r1, c0],
                self._block_elev[r1, c1],
            ),
            dtype=np.float64,
        )
        wts = np.asarray(
            ((1.0 - fr) * (1.0 - fc), (1.0 - fr) * fc, fr * (1.0 - fc), fr * fc),
            dtype=np.float64,
        )
        valid = np.isfinite(vals) & (vals >= -1000.0) & (wts > 0.0)
        if not np.any(valid):
            return float("nan")
        return float(np.sum(vals[valid] * wts[valid]) / np.sum(wts[valid]))

