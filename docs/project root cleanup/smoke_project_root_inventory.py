from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]

ROOT_SURFACE = {
    ".gitattributes": "repo-metadata",
    ".gitignore": "repo-metadata",
    ".vscode": "workspace-config",
    "app": "core-source",
    "docs": "documentation",
    "log_main.py": "launcher",
    "Logs": "runtime-data",
    "modules": "core-source",
    "modules copy": "cleanup-candidate",
    "modules_bkup": "user-managed-backup",
    "resource": "active-resource",
    "run.py": "launcher",
    "settings": "project-settings",
    "sim_main.py": "launcher",
}

OPTIONAL_ROOT_ITEMS = {
    "modules copy",
    "modules_bkup",
}

MIGRATED_ROOT_SETTINGS = {
    "current_scenario.json",
    "nFusionLicense.lic",
    "nFusionSettings.json",
    "replan_settings.json",
    "replan_settings_defaults.json",
    "uav_params.json",
}

REQUIRED_GITIGNORE_MARKERS = (
    "/Logs/",
    "/ref/",
    "/temp/",
    "/modules copy/",
    "/modules_bkup/",
)


def fail(message: str) -> None:
    raise AssertionError(message)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def git_ls_files(*patterns: str) -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "--", *patterns],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode != 0:
        fail(f"git ls-files failed: {result.stderr}")
    return [line.strip().replace("\\", "/") for line in result.stdout.splitlines() if line.strip()]


def check_every_root_item_classified() -> None:
    existing = {path.name for path in PROJECT_ROOT.iterdir() if path.name != ".git"}
    missing = sorted(existing.difference(ROOT_SURFACE))
    if missing:
        fail("unclassified project root items: " + ", ".join(missing))

    missing_required = sorted(
        item
        for item in ROOT_SURFACE
        if item not in OPTIONAL_ROOT_ITEMS and not (PROJECT_ROOT / item).exists()
    )
    if missing_required:
        fail("classified required root items are missing: " + ", ".join(missing_required))


def check_no_migrated_settings_at_root() -> None:
    offenders = sorted(name for name in MIGRATED_ROOT_SETTINGS if (PROJECT_ROOT / name).exists())
    if offenders:
        fail("migrated settings found at project root: " + ", ".join(offenders))


def check_gitignore_blocks_new_runtime_surface() -> None:
    gitignore = read_text(PROJECT_ROOT / ".gitignore")
    missing = [marker for marker in REQUIRED_GITIGNORE_MARKERS if marker not in gitignore]
    if missing:
        fail(".gitignore missing root cleanup markers: " + ", ".join(missing))


def check_backup_roots_are_untracked() -> None:
    offenders: list[str] = []
    for name in ("modules copy", "modules_bkup"):
        if not (PROJECT_ROOT / name).exists():
            continue
        tracked = git_ls_files(name)
        if tracked:
            offenders.append(f"{name}: " + ", ".join(tracked[:10]))
    if offenders:
        fail("backup source roots must stay untracked: " + "; ".join(offenders))


def check_reference_docs_moved() -> None:
    if (PROJECT_ROOT / "ref").exists():
        fail("legacy root ref/ exists; reference documents belong under docs/reference/")
    reference_dir = PROJECT_ROOT / "docs" / "reference"
    if not reference_dir.exists():
        fail("docs/reference is missing")
    if not any(reference_dir.iterdir()):
        fail("docs/reference is empty")


def check_legacy_temp_absent() -> None:
    if (PROJECT_ROOT / "temp").exists():
        fail("legacy root temp/ exists; scratch files should not live on the project surface")


def check_active_resource_contract_still_present() -> None:
    required = (
        PROJECT_ROOT / "resource" / "db",
        PROJECT_ROOT / "resource" / "korea.mbtiles",
    )
    missing = [str(path.relative_to(PROJECT_ROOT)) for path in required if not path.exists()]
    if missing:
        fail("active resource paths are missing: " + ", ".join(missing))

    dem_tiles = list((PROJECT_ROOT / "resource").glob("*.tif"))
    if not dem_tiles:
        fail("resource/*.tif DEM tiles are missing")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke project root surface inventory.")
    parser.parse_args()

    try:
        check_every_root_item_classified()
        check_no_migrated_settings_at_root()
        check_gitignore_blocks_new_runtime_surface()
        check_backup_roots_are_untracked()
        check_reference_docs_moved()
        check_legacy_temp_absent()
        check_active_resource_contract_still_present()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("project root inventory smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
