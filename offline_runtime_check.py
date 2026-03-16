from __future__ import annotations

import importlib
import json
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List


@dataclass
class CheckResult:
    name: str
    ok: bool
    required: bool
    detail: str
    data: Dict[str, Any]


def _result(
    name: str,
    ok: bool,
    required: bool,
    detail: str,
    data: Dict[str, Any] | None = None,
) -> CheckResult:
    return CheckResult(name=name, ok=ok, required=required, detail=detail, data=data or {})


def _run_check(
    name: str,
    func: Callable[[], Dict[str, Any]],
    *,
    required: bool = True,
) -> CheckResult:
    try:
        data = func()
        return _result(name, True, required, "ok", data)
    except Exception as exc:
        return _result(name, False, required, f"{type(exc).__name__}: {exc}", {})


def _module_version(mod: Any) -> str:
    version = getattr(mod, "__version__", None)
    return str(version) if version is not None else "unknown"


def _check_base_imports() -> Dict[str, Any]:
    modules = [
        "numpy",
        "PIL",
        "affine",
        "shapely",
        "rasterio",
        "folium",
        "PyQt5",
        "pythonnet",
    ]
    imported: Dict[str, str] = {}
    for name in modules:
        mod = importlib.import_module(name)
        imported[name] = _module_version(mod)
    return {"imports": imported}


def _check_pythonnet_coreclr() -> Dict[str, Any]:
    from pythonnet import load

    load("coreclr")
    import clr  # noqa: F401

    return {"runtime": "coreclr loaded"}


def _check_osgeo_gdal() -> Dict[str, Any]:
    from osgeo import gdal

    return {"gdal_version_info": gdal.VersionInfo()}


def _check_resource_geotiff(project_root: Path) -> Dict[str, Any]:
    from osgeo import gdal

    resource_dir = project_root / "resource"
    tif_files = sorted(resource_dir.glob("*.tif"))
    if not tif_files:
        raise RuntimeError(f"No GeoTIFF found in {resource_dir}")

    first_tif = tif_files[0]
    ds = gdal.Open(str(first_tif), gdal.GA_ReadOnly)
    if ds is None:
        raise RuntimeError(f"GDAL failed to open {first_tif}")

    width = int(ds.RasterXSize or 0)
    height = int(ds.RasterYSize or 0)
    band_count = int(ds.RasterCount or 0)
    ds = None
    return {
        "tif_count": len(tif_files),
        "sample_file": str(first_tif),
        "sample_size": [width, height],
        "sample_band_count": band_count,
    }


def _check_attack_assistance(project_root: Path) -> Dict[str, Any]:
    script = (
        project_root
        / "modules"
        / "mission_planning"
        / "MissionPlanner"
        / "data_def"
        / "lah_attack_assistance.py"
    )
    if not script.exists():
        raise RuntimeError(f"Missing script: {script}")

    cmd = [
        sys.executable,
        str(script),
        "--friendly-lat",
        "37.56",
        "--friendly-lon",
        "126.97",
        "--enemy-lat",
        "37.57",
        "--enemy-lon",
        "126.99",
        "--output-json",
    ]
    result = subprocess.run(
        cmd,
        cwd=str(project_root),
        capture_output=True,
        text=True,
        timeout=90,
        check=False,
    )
    if result.returncode != 0:
        stderr = (result.stderr or "").strip()
        raise RuntimeError(f"exit={result.returncode}, stderr={stderr}")

    payload = json.loads(result.stdout or "{}")
    attack = payload.get("attack_point") or {}
    lat = attack.get("lat") or attack.get("latitude")
    lon = attack.get("lon") or attack.get("longitude")
    if lat is None or lon is None:
        raise RuntimeError("attack_point is missing latitude/longitude")

    return {
        "attack_point": {
            "lat": lat,
            "lon": lon,
            "alt_m": attack.get("alt_m") or attack.get("altitude"),
        },
        "distance_friendly_m": payload.get("distance_friendly_m"),
        "distance_enemy_m": payload.get("distance_enemy_m"),
        "raster_sources_count": len(payload.get("raster_sources") or []),
    }


def _check_sim_web_assets(project_root: Path) -> Dict[str, Any]:
    web_dir = project_root / "modules" / "sim" / "web"
    index_path = web_dir / "index.html"
    map_style_path = web_dir / "js" / "map_style.js"
    vendor_js = web_dir / "vendor" / "maplibre-gl.js"
    vendor_css = web_dir / "vendor" / "maplibre-gl.css"

    if not index_path.exists():
        raise RuntimeError(f"Missing file: {index_path}")
    if not map_style_path.exists():
        raise RuntimeError(f"Missing file: {map_style_path}")
    if not vendor_js.exists() or not vendor_css.exists():
        raise RuntimeError("Missing local MapLibre vendor files")

    index_text = index_path.read_text(encoding="utf-8", errors="ignore")
    style_text = map_style_path.read_text(encoding="utf-8", errors="ignore")

    if "/vendor/maplibre-gl.js" not in index_text or "/vendor/maplibre-gl.css" not in index_text:
        raise RuntimeError("index.html is not using local MapLibre assets")
    if "https://unpkg.com/maplibre-gl" in index_text:
        raise RuntimeError("index.html still contains unpkg MapLibre URL")
    if "const useLabelLayers = false;" not in style_text:
        raise RuntimeError("map_style.js offline label guard is missing")

    return {
        "index_file": str(index_path),
        "vendor_js_bytes": vendor_js.stat().st_size,
        "vendor_css_bytes": vendor_css.stat().st_size,
    }


def _check_nfusion_settings(project_root: Path) -> Dict[str, Any]:
    settings = project_root / "nFusionSettings.json"
    if not settings.exists():
        raise RuntimeError(f"Missing file: {settings}")

    payload = json.loads(settings.read_text(encoding="utf-8"))
    middleware = payload.get("Middleware") if isinstance(payload, dict) else None
    if not isinstance(middleware, dict):
        raise RuntimeError("Invalid nFusionSettings.json format")

    network_address = str(middleware.get("NetworkAddress") or "")
    if not network_address:
        raise RuntimeError("Middleware.NetworkAddress is empty")
    if not network_address.endswith("."):
        raise RuntimeError("Middleware.NetworkAddress should be prefix format (example: 192.168.20.)")

    return {
        "network_address": network_address,
        "local_domain": middleware.get("LocalDomain"),
        "external_domain": middleware.get("ExternalDomain"),
    }


def _print_console(results: List[CheckResult], report_path: Path) -> None:
    print("=" * 72)
    print("DSS Offline Runtime Self-Test")
    print(f"Python: {sys.executable}")
    print("=" * 72)
    for res in results:
        status = "PASS" if res.ok else ("WARN" if not res.required else "FAIL")
        req = "required" if res.required else "optional"
        print(f"[{status:4}] {res.name} ({req})")
        if res.ok:
            print(f"       detail: {res.detail}")
        else:
            print(f"       reason: {res.detail}")
    print("-" * 72)
    required_failures = [r for r in results if r.required and not r.ok]
    if required_failures:
        print(f"OVERALL: FAIL ({len(required_failures)} required checks failed)")
    else:
        print("OVERALL: PASS")
    print(f"Report: {report_path}")


def main() -> int:
    project_root = Path(__file__).resolve().parent
    checks: List[CheckResult] = []

    checks.append(_run_check("Base imports", _check_base_imports, required=True))
    checks.append(_run_check("pythonnet coreclr load", _check_pythonnet_coreclr, required=True))
    checks.append(_run_check("osgeo.gdal import", _check_osgeo_gdal, required=True))
    checks.append(
        _run_check(
            "GeoTIFF resource open",
            lambda: _check_resource_geotiff(project_root),
            required=True,
        )
    )
    checks.append(
        _run_check(
            "Attack assistance subprocess",
            lambda: _check_attack_assistance(project_root),
            required=True,
        )
    )
    checks.append(
        _run_check(
            "Simulation web offline assets",
            lambda: _check_sim_web_assets(project_root),
            required=True,
        )
    )
    checks.append(
        _run_check(
            "nFusion settings sanity",
            lambda: _check_nfusion_settings(project_root),
            required=True,
        )
    )

    required_failures = [r for r in checks if r.required and not r.ok]
    report = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "python_executable": sys.executable,
        "project_root": str(project_root),
        "overall_pass": len(required_failures) == 0,
        "results": [
            {
                "name": r.name,
                "ok": r.ok,
                "required": r.required,
                "detail": r.detail,
                "data": r.data,
            }
            for r in checks
        ],
    }
    report_path = project_root / "offline_runtime_check_report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    _print_console(checks, report_path)
    return 0 if len(required_failures) == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
