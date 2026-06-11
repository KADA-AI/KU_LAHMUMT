from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

ROOT_SETTINGS_FILES = (
    "current_scenario.json",
    "nFusionSettings.json",
    "nFusionLicense.lic",
    "replan_settings.json",
    "replan_settings_defaults.json",
    "uav_params.json",
)


def fail(message: str) -> None:
    raise AssertionError(message)


def add_project_root_to_syspath() -> None:
    project_root = str(PROJECT_ROOT)
    if project_root not in sys.path:
        sys.path.insert(0, project_root)


def check_root_has_no_migrated_settings() -> None:
    offenders = [name for name in ROOT_SETTINGS_FILES if (PROJECT_ROOT / name).exists()]
    if offenders:
        fail("migrated settings still exist in project root: " + ", ".join(offenders))


def check_settings_files_exist() -> None:
    missing = [name for name in ROOT_SETTINGS_FILES if not (PROJECT_ROOT / "settings" / name).exists()]
    if missing:
        fail("canonical settings files are missing: " + ", ".join(missing))


def check_settings_path_contract() -> None:
    add_project_root_to_syspath()
    from modules.common import settings_paths

    expected = PROJECT_ROOT / "settings"
    if settings_paths.settings_dir() != expected:
        fail(f"settings_dir mismatch: {settings_paths.settings_dir()} != {expected}")

    contract = {
        "current_scenario.json": settings_paths.scenario_info_path(),
        "nFusionSettings.json": settings_paths.nfusion_settings_path(),
        "nFusionLicense.lic": settings_paths.nfusion_license_path(),
        "replan_settings.json": settings_paths.replan_settings_path(),
        "replan_settings_defaults.json": settings_paths.replan_defaults_path(),
        "uav_params.json": settings_paths.uav_params_path(),
    }
    mismatches = [
        f"{name}: {path}"
        for name, path in contract.items()
        if path != PROJECT_ROOT / "settings" / name
    ]
    if mismatches:
        fail("settings path contract mismatch: " + "; ".join(mismatches))


def check_root_cache_absent() -> None:
    if (PROJECT_ROOT / "__pycache__").exists():
        fail("root __pycache__ exists; remove generated cache from the project surface")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke project root settings cleanup.")
    parser.parse_args()

    try:
        check_root_has_no_migrated_settings()
        check_settings_files_exist()
        check_settings_path_contract()
        check_root_cache_absent()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("project root settings surface smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
