from __future__ import annotations

import os
import shutil
from contextlib import contextmanager
from pathlib import Path
from typing import Iterable, Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[2]
COMMON_DIR = Path(__file__).resolve().parent
SETTINGS_DIR = PROJECT_ROOT / "settings"


def settings_dir() -> Path:
    return SETTINGS_DIR


def settings_file(name: str) -> Path:
    return SETTINGS_DIR / name


def legacy_project_file(name: str) -> Path:
    return PROJECT_ROOT / name


def existing_settings_file(name: str) -> Path:
    canonical = settings_file(name)
    if canonical.exists():
        return canonical
    legacy = legacy_project_file(name)
    if legacy.exists():
        return legacy
    return canonical


def writable_settings_file(name: str) -> Path:
    SETTINGS_DIR.mkdir(parents=True, exist_ok=True)
    return settings_file(name)


def scenario_info_path() -> Path:
    return settings_file("current_scenario.json")


def existing_scenario_info_path() -> Path:
    return existing_settings_file("current_scenario.json")


def replan_settings_path() -> Path:
    return settings_file("replan_settings.json")


def replan_defaults_path() -> Path:
    return settings_file("replan_settings_defaults.json")


def uav_params_path() -> Path:
    return settings_file("uav_params.json")


def nfusion_settings_path() -> Path:
    return settings_file("nFusionSettings.json")


def nfusion_license_path() -> Path:
    return settings_file("nFusionLicense.lic")


def fusion_settings_candidates(
    *,
    project_root: Path = PROJECT_ROOT,
    common_dir: Path = COMMON_DIR,
    ds_dir: Path | None = None,
) -> tuple[Path, ...]:
    candidates: list[Path] = [
        project_root / "settings" / "nFusionSettings.json",
    ]
    if ds_dir is not None:
        module_dir = _module_owned_fusion_dir(project_root, ds_dir)
        candidates.append(module_dir / "nFusionSettings.json")
    candidates.extend(
        [
            common_dir / "nFusionSettings.json",
            project_root / "settings" / "FusionSettings.json",
        ]
    )
    if ds_dir is not None:
        candidates.append(module_dir / "FusionSettings.json")
    candidates.extend(
        [
            common_dir / "FusionSettings.json",
            project_root / "nFusion" / "FusionSettings.json",
        ]
    )
    return tuple(candidates)


def fusion_license_candidates(
    *,
    project_root: Path = PROJECT_ROOT,
    common_dir: Path = COMMON_DIR,
) -> tuple[Path, ...]:
    return (
        project_root / "settings" / "nFusionLicense.lic",
        project_root / "nFusionLicense.lic",
        common_dir / "nFusionLicense.lic",
        project_root / "nFusion" / "nFusionLicense.lic",
    )


def first_existing_path(candidates: Iterable[Path]) -> Path | None:
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def _module_owned_fusion_dir(project_root: Path, directory: Path) -> Path:
    """Redirect the removed root ``app/ui`` runtime path into ``modules``."""
    legacy_app_ui = project_root / "app" / "ui"
    try:
        is_legacy_app_ui = directory.resolve() == legacy_app_ui.resolve()
    except Exception:
        is_legacy_app_ui = directory == legacy_app_ui
    if is_legacy_app_ui:
        return project_root / "modules" / "app" / "ui"
    return directory


def fusion_settings_runtime_targets(
    *,
    project_root: Path = PROJECT_ROOT,
    common_dir: Path = COMMON_DIR,
    ds_dir: Path | None = None,
) -> tuple[Path, ...]:
    """Canonical and module-owned nFusion runtime settings locations."""
    candidates: list[Path] = [
        project_root / "settings" / "nFusionSettings.json",
    ]
    if ds_dir is not None:
        module_dir = _module_owned_fusion_dir(project_root, ds_dir)
        candidates.append(module_dir / "nFusionSettings.json")
    candidates.append(common_dir / "nFusionSettings.json")

    # Do not derive this from ``common_dir``: a stale caller can still pass
    # ``<root>/common`` and accidentally resurrect ``<root>/app/ui``.
    app_ui_dir = project_root / "modules" / "app" / "ui"
    candidates.extend(
        [
            app_ui_dir / "nFusionSettings.json",
        ]
    )

    targets: list[Path] = []
    seen: set[str] = set()
    for candidate in candidates:
        key = str(candidate).lower()
        if key in seen:
            continue
        seen.add(key)
        targets.append(candidate)
    return tuple(targets)


def ensure_fusion_settings_file(
    *,
    project_root: Path = PROJECT_ROOT,
    common_dir: Path = COMMON_DIR,
    ds_dir: Path | None = None,
) -> Path | None:
    src = first_existing_path(
        fusion_settings_candidates(project_root=project_root, common_dir=common_dir, ds_dir=ds_dir)
    )
    if src is None:
        return None
    dst = project_root / "settings" / "nFusionSettings.json"
    text = src.read_text(encoding="utf-8")
    try:
        dst_resolved = dst.resolve()
    except Exception:
        dst_resolved = dst

    for target in fusion_settings_runtime_targets(
        project_root=project_root,
        common_dir=common_dir,
        ds_dir=ds_dir,
    ):
        target.parent.mkdir(parents=True, exist_ok=True)
        try:
            if target.resolve() == src.resolve() and target.read_text(encoding="utf-8") == text:
                continue
        except Exception:
            if target == src:
                continue
        target.write_text(text, encoding="utf-8")
        try:
            if target.resolve() == dst_resolved:
                dst = target
        except Exception:
            if target == dst:
                dst = target
    return dst


def ensure_fusion_license_file(
    *,
    project_root: Path = PROJECT_ROOT,
    common_dir: Path = COMMON_DIR,
) -> Path | None:
    src = first_existing_path(fusion_license_candidates(project_root=project_root, common_dir=common_dir))
    if src is None:
        return None
    dst = project_root / "settings" / "nFusionLicense.lic"
    dst.parent.mkdir(parents=True, exist_ok=True)
    try:
        if src.resolve() != dst.resolve():
            shutil.copyfile(src, dst)
    except Exception:
        if src != dst:
            shutil.copyfile(src, dst)
    return dst


@contextmanager
def fusion_runtime_working_dir(
    *,
    project_root: Path = PROJECT_ROOT,
) -> Iterator[Path]:
    runtime_dir = project_root / "settings"
    previous = Path.cwd()
    try:
        os.chdir(runtime_dir)
        yield runtime_dir
    finally:
        try:
            os.chdir(previous)
        except Exception:
            pass
