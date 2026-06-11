# LAH Attack Assistance Subprocess Progress

## Scope

This checkpoint verifies the `lah_attack_assistance.py` CLI as a real subprocess contract for attack-point calculation.

## Added

- `smoke_lah_attack_assistance_subprocess.py`

## Contract Captured

- `lah_attack_assistance.py` accepts:
  - `--friendly-lat`
  - `--friendly-lon`
  - `--enemy-lat`
  - `--enemy-lon`
  - `--raster-path`
  - `--radius-m`
  - `--num-rays`
  - `--candidate-count`
  - `--output-json`
- JSON output contains:
  - `attack_point.lon`
  - `attack_point.lat`
  - `attack_point.alt_m`
  - `friendly_point`
  - `enemy_point`
  - `distance_friendly_m`
  - `distance_enemy_m`
  - `raster_path`
  - `raster_sources`
- Active subprocess call sites keep building the same JSON CLI command:
  - `replanning/triggers/attack/pipeline.py`
  - `pipelines/mission_planning_attack_helpers.py`

## Boundary

This smoke does not use production terrain resources and does not write a PNG. It generates a temporary synthetic GeoTIFF and a temporary `osgeo.gdal` compatibility shim backed by `rasterio`, because the current dev Python environment has `rasterio` but not `osgeo`.

The real production script remains unchanged and still imports `osgeo.gdal` when run normally.

## Why This Is Safe

No runtime code changed. The subprocess runs in a temporary directory with a temporary raster and temporary `PYTHONPATH` shim, then all generated files are removed by `TemporaryDirectory`.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_lah_attack_assistance_subprocess.py"
python "docs\mission planning refactoring\smoke_lah_attack_assistance_subprocess.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
lah attack assistance subprocess smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `portable bundle python app.py/run_portable.bat smoke`.
