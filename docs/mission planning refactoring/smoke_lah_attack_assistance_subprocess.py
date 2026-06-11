from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = PROJECT_ROOT / "modules/mission_planning/MissionPlanner/data_def/lah_attack_assistance.py"


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def expect_true(label: str, condition: bool) -> None:
    if not condition:
        fail(f"{label} expected true")


def assert_source_contains(rel_path: str, *snippets: str) -> None:
    path = PROJECT_ROOT / rel_path
    text = path.read_text(encoding="utf-8", errors="ignore")
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing attack assistance subprocess markers: {missing!r}")


def write_gdal_shim(tmp_path: Path) -> Path:
    package_dir = tmp_path / "osgeo"
    package_dir.mkdir(parents=True, exist_ok=True)
    (package_dir / "__init__.py").write_text("from . import gdal\n", encoding="utf-8")
    (package_dir / "gdal.py").write_text(
        textwrap.dedent(
            """
            from __future__ import annotations

            import rasterio
            from rasterio.windows import Window

            GA_ReadOnly = 0
            GRA_NearestNeighbour = 0


            class Band:
                def __init__(self, dataset):
                    self._dataset = dataset

                def GetNoDataValue(self):
                    return self._dataset.nodata

                def ReadAsArray(self, xoff=0, yoff=0, xsize=None, ysize=None):
                    if xsize is None or ysize is None:
                        return self._dataset.read(1)
                    window = Window(int(xoff), int(yoff), int(xsize), int(ysize))
                    return self._dataset.read(1, window=window)


            class Dataset:
                def __init__(self, path):
                    self._src = rasterio.open(path)
                    self.RasterXSize = self._src.width
                    self.RasterYSize = self._src.height

                def GetGeoTransform(self, can_return_null=False):
                    transform = self._src.transform
                    return (
                        transform.c,
                        transform.a,
                        transform.b,
                        transform.f,
                        transform.d,
                        transform.e,
                    )

                def GetProjection(self):
                    return self._src.crs.to_wkt() if self._src.crs else ""

                def GetRasterBand(self, index):
                    return Band(self._src)


            def Open(path, mode=GA_ReadOnly):
                try:
                    return Dataset(path)
                except Exception:
                    return None


            def WarpOptions(**kwargs):
                return kwargs


            def Warp(destNameOrDestDS, srcDSOrSrcDSTab, options=None):
                path = srcDSOrSrcDSTab[0] if isinstance(srcDSOrSrcDSTab, (list, tuple)) else srcDSOrSrcDSTab
                return Dataset(path)
            """
        ).strip()
        + "\n",
        encoding="utf-8",
    )
    return tmp_path


def write_attack_fixture_raster(tmp_path: Path) -> tuple[Path, float, float, float, float]:
    try:
        import rasterio
        from rasterio.transform import from_origin
    except Exception as exc:
        raise SmokeFailure(f"rasterio is required for this smoke: {exc}") from exc

    raster_path = tmp_path / "attack_fixture.tif"
    width = height = 160
    pixel_deg = 0.00015
    min_lon = 126.988
    max_lat = 37.012
    enemy_lon = min_lon + width * pixel_deg / 2.0
    enemy_lat = max_lat - height * pixel_deg / 2.0
    friendly_lon = enemy_lon - 0.006
    friendly_lat = enemy_lat

    rows, cols = np.indices((height, width))
    cx = width / 2.0
    cy = height / 2.0
    dist_px = np.sqrt((cols - cx) ** 2 + (rows - cy) ** 2)
    elevation = (100.0 + dist_px * 4.0).astype("float32")
    transform = from_origin(min_lon, max_lat, pixel_deg, pixel_deg)

    with rasterio.open(
        raster_path,
        "w",
        driver="GTiff",
        width=width,
        height=height,
        count=1,
        dtype="float32",
        crs="EPSG:4326",
        transform=transform,
        nodata=-9999.0,
    ) as dataset:
        dataset.write(elevation, 1)

    return raster_path, friendly_lat, friendly_lon, enemy_lat, enemy_lon


def run_attack_assistance_subprocess() -> dict:
    if not SCRIPT_PATH.exists():
        fail(f"missing attack assistance script: {SCRIPT_PATH}")

    with tempfile.TemporaryDirectory(prefix="mp_lah_attack_subprocess_") as tmp:
        tmp_path = Path(tmp)
        shim_root = write_gdal_shim(tmp_path)
        raster_path, friendly_lat, friendly_lon, enemy_lat, enemy_lon = write_attack_fixture_raster(tmp_path)

        cmd = [
            sys.executable,
            str(SCRIPT_PATH),
            "--friendly-lat",
            str(friendly_lat),
            "--friendly-lon",
            str(friendly_lon),
            "--enemy-lat",
            str(enemy_lat),
            "--enemy-lon",
            str(enemy_lon),
            "--raster-path",
            str(raster_path),
            "--radius-m",
            "600",
            "--num-rays",
            "36",
            "--candidate-count",
            "2",
            "--output-json",
        ]
        env = dict(os.environ)
        env["PYTHONPATH"] = str(shim_root) + os.pathsep + env.get("PYTHONPATH", "")
        result = subprocess.run(
            cmd,
            cwd=PROJECT_ROOT,
            env=env,
            capture_output=True,
            text=True,
            check=False,
            timeout=60,
        )
        if result.returncode != 0:
            fail(
                "lah_attack_assistance.py subprocess failed: "
                f"exit={result.returncode}, stdout={result.stdout!r}, stderr={result.stderr!r}"
            )
        try:
            return json.loads(result.stdout)
        except Exception as exc:
            fail(f"subprocess stdout is not JSON: {exc}; stdout={result.stdout!r}")
    fail("unreachable")
    return {}


def check_payload(payload: dict) -> None:
    attack_point = payload.get("attack_point")
    expect_true("attack point object", isinstance(attack_point, dict))
    for key in ("lat", "lon", "alt_m"):
        expect_true(f"attack point has {key}", key in attack_point)
    expect_true("attack latitude finite", isinstance(attack_point.get("lat"), (int, float)))
    expect_true("attack longitude finite", isinstance(attack_point.get("lon"), (int, float)))
    expect_true("attack altitude integer", isinstance(attack_point.get("alt_m"), int))
    expect_true("friendly distance positive", float(payload.get("distance_friendly_m", 0.0)) > 0.0)
    expect_true("enemy distance positive", float(payload.get("distance_enemy_m", 0.0)) > 0.0)
    raster_sources = payload.get("raster_sources")
    expect_true("raster sources list", isinstance(raster_sources, list) and len(raster_sources) == 1)
    expect_true("raster path present", isinstance(payload.get("raster_path"), str))
    expect_true("no PNG in JSON-only smoke", "visualization_png" not in payload)


def check_source_markers() -> None:
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/data_def/lah_attack_assistance.py",
        'parser.add_argument("--friendly-lat"',
        'parser.add_argument("--friendly-lon"',
        'parser.add_argument("--enemy-lat"',
        'parser.add_argument("--enemy-lon"',
        '"--raster-path"',
        'parser.add_argument("--output-json"',
        '"attack_point": {',
        '"raster_sources": raster_sources_abs',
        "print(json.dumps(result, indent=2, ensure_ascii=False))",
    )
    assert_source_contains(
        "modules/mission_planning/replanning/triggers/attack/pipeline.py",
        "def _compute_attack_point_subprocess(",
        '"--friendly-lat"',
        '"--enemy-lat"',
        '"--output-json"',
        "json.loads(result.stdout or \"{}\")",
    )
    assert_source_contains(
        "modules/mission_planning/pipelines/mission_planning_attack_helpers.py",
        '"lah_attack_assistance.py"',
        '"--friendly-lat"',
        '"--enemy-lat"',
        '"--output-json"',
        "subprocess.run(cmd, capture_output=True, text=True, check=False)",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke lah_attack_assistance.py subprocess JSON CLI.")
    parser.parse_args()

    try:
        check_source_markers()
        payload = run_attack_assistance_subprocess()
        check_payload(payload)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("lah attack assistance subprocess smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
