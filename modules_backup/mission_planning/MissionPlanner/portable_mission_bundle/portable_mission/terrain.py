from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Tuple
import uuid

import numpy as np
import rasterio
from rasterio.windows import Window
from werkzeug.datastructures import FileStorage
from werkzeug.utils import secure_filename


def load_dem(dem_path: str | Path):
    dem_path = Path(dem_path)
    if not dem_path.exists():
        raise FileNotFoundError(f"DEM not found: {dem_path}")
    with rasterio.open(dem_path) as dataset:
        dem = dataset.read(1, masked=True)
        bounds = dataset.bounds
    return dem, bounds


def list_dem_files(inputs_dir: Path) -> list[str]:
    if not inputs_dir.exists():
        return []
    items = [
        path.name
        for path in inputs_dir.iterdir()
        if path.is_file() and path.suffix.lower() in {".tif", ".tiff"}
    ]
    return sorted(items, key=str.lower)


def resolve_dem_path(inputs_dir: Path, name: str) -> Path:
    candidate = (inputs_dir / name).resolve()
    if inputs_dir.resolve() not in candidate.parents or not candidate.exists():
        raise FileNotFoundError(f"DEM not found: {name}")
    return candidate


def save_uploaded_dem(file_storage: FileStorage, inputs_dir: Path) -> Path:
    filename = secure_filename(file_storage.filename or "")
    if not filename:
        raise ValueError("No file name was provided.")
    if Path(filename).suffix.lower() not in {".tif", ".tiff"}:
        raise ValueError("Only .tif or .tiff files are supported.")

    stem = Path(filename).stem
    suffix = Path(filename).suffix.lower()
    output_name = f"{stem}_{uuid.uuid4().hex[:8]}{suffix}"
    output_path = inputs_dir / output_name
    file_storage.save(output_path)
    return output_path


def describe_dem(dem_path: Path) -> Dict[str, Any]:
    with rasterio.open(dem_path) as dataset:
        crs_text = dataset.crs.to_string() if dataset.crs else None
        is_geographic = bool(dataset.crs and dataset.crs.is_geographic)
        return {
            "name": dem_path.name,
            "path": str(dem_path),
            "width": int(dataset.width),
            "height": int(dataset.height),
            "bounds": {
                "left": float(dataset.bounds.left),
                "right": float(dataset.bounds.right),
                "bottom": float(dataset.bounds.bottom),
                "top": float(dataset.bounds.top),
            },
            "crs": crs_text,
            "is_geographic": is_geographic,
            "nodata": None if dataset.nodata is None else float(dataset.nodata),
        }


def build_dem_preview(dem_path: Path, max_side: int = 256) -> Dict[str, Any]:
    with rasterio.open(dem_path) as dataset:
        data = dataset.read(1, masked=True)
        height = int(dataset.height)
        width = int(dataset.width)

    preview_h = max(1, min(max_side, height))
    preview_w = max(1, min(max_side, width))
    row_idx = np.linspace(0, height - 1, preview_h).astype(int)
    col_idx = np.linspace(0, width - 1, preview_w).astype(int)
    sampled = data[np.ix_(row_idx, col_idx)]

    if np.ma.isMaskedArray(sampled):
        raw = np.ma.asarray(sampled, dtype=float).filled(np.nan)
    else:
        raw = np.asarray(sampled, dtype=float)

    finite = raw[np.isfinite(raw)]
    if finite.size:
        lo = float(np.percentile(finite, 2))
        hi = float(np.percentile(finite, 98))
        if hi <= lo:
            hi = lo + 1.0
        scaled = np.clip((raw - lo) / (hi - lo), 0.0, 1.0)
    else:
        lo = 0.0
        hi = 1.0
        scaled = np.zeros_like(raw)

    pixels = np.round(np.nan_to_num(scaled, nan=0.0) * 255.0).astype(np.uint8)
    return {
        "width": preview_w,
        "height": preview_h,
        "pixels": pixels.tolist(),
        "stats": {
            "min": lo,
            "max": hi,
        },
    }


def crop_dem_by_normalized_roi(
    dem_path: Path,
    roi: Dict[str, Any],
    output_path: Path,
) -> Dict[str, Any]:
    x0, y0, x1, y1 = _normalize_roi(roi)
    with rasterio.open(dem_path) as dataset:
        width = int(dataset.width)
        height = int(dataset.height)

        col_min = max(0, min(width - 1, int(np.floor(x0 * width))))
        col_max = max(col_min + 1, min(width, int(np.ceil(x1 * width))))
        row_min = max(0, min(height - 1, int(np.floor(y0 * height))))
        row_max = max(row_min + 1, min(height, int(np.ceil(y1 * height))))

        if col_max - col_min < 8 or row_max - row_min < 8:
            raise ValueError("ROI is too small. Select a larger area.")

        window = Window(col_off=col_min, row_off=row_min, width=col_max - col_min, height=row_max - row_min)
        data = dataset.read(1, window=window, masked=True)
        transform = dataset.window_transform(window)
        bounds = rasterio.windows.bounds(window, dataset.transform)
        nodata = dataset.nodata if dataset.nodata is not None else -9999.0

        meta = dataset.meta.copy()
        meta.update(
            {
                "height": int(window.height),
                "width": int(window.width),
                "transform": transform,
                "nodata": nodata,
            }
        )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with rasterio.open(output_path, "w", **meta) as out_ds:
            out_ds.write(data.filled(nodata), 1)

        return {
            "pixel_window": {
                "row_min": row_min,
                "row_max": row_max,
                "col_min": col_min,
                "col_max": col_max,
            },
            "bounds": {
                "left": float(bounds[0]),
                "bottom": float(bounds[1]),
                "right": float(bounds[2]),
                "top": float(bounds[3]),
            },
            "shape": {
                "width": int(window.width),
                "height": int(window.height),
            },
        }


def _normalize_roi(roi: Dict[str, Any]) -> Tuple[float, float, float, float]:
    try:
        x0 = float(roi["x0"])
        y0 = float(roi["y0"])
        x1 = float(roi["x1"])
        y1 = float(roi["y1"])
    except Exception as exc:  # noqa: BLE001
        raise ValueError("ROI must contain x0, y0, x1, y1.") from exc

    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    x0 = float(np.clip(x0, 0.0, 1.0))
    x1 = float(np.clip(x1, 0.0, 1.0))
    y0 = float(np.clip(y0, 0.0, 1.0))
    y1 = float(np.clip(y1, 0.0, 1.0))
    if x1 - x0 < 0.01 or y1 - y0 < 0.01:
        raise ValueError("ROI must cover at least 1% of the preview in both axes.")
    return x0, y0, x1, y1
