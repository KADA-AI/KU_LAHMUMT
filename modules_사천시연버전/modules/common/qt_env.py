from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path
from typing import Iterable, Optional

_INITIALIZED = False


def _find_pyqt5_base() -> Optional[Path]:
    """Locate the PyQt5 package directory without importing heavy submodules."""
    spec = importlib.util.find_spec("PyQt5")
    if spec is None or not spec.origin:
        return None
    return Path(spec.origin).resolve().parent


def _prepend_env_path(key: str, value: str) -> None:
    """Prepend `value` to os.environ[key] if it is not already present."""
    current = os.environ.get(key, "")
    parts = [p for p in current.split(os.pathsep) if p]
    if value in parts:
        return
    if parts:
        os.environ[key] = os.pathsep.join([value, *parts])
    else:
        os.environ[key] = value


def _first_existing(paths: Iterable[Path]) -> Optional[Path]:
    for candidate in paths:
        if candidate.exists():
            return candidate
    return None


def _qt_root_candidates(base: Optional[Path]) -> list[Path]:
    roots: list[Path] = []

    conda_prefix = os.environ.get("CONDA_PREFIX") or sys.prefix
    if conda_prefix:
        conda_library = Path(conda_prefix) / "Library"
        if conda_library.exists():
            roots.append(conda_library)

    if base is not None:
        roots.extend([base / "Qt", base / "Qt5", base / "Qt6"])

    unique: list[Path] = []
    seen: set[str] = set()
    for root in roots:
        key = str(root.resolve()) if root.exists() else str(root)
        if key in seen:
            continue
        seen.add(key)
        unique.append(root)
    return unique


def ensure_qt_platform(force: bool = False) -> None:
    """
    Guarantee Qt platform plugins (qwindows, etc.) are discoverable at runtime.

    On some Windows setups the PyQt5 wheels do not automatically register the
    plugin directory, which leads to the "Could not load the Qt platform plugin
    \"windows\"" error. We resolve the PyQt5 installation folder and expose both
    the plugin directory and the Qt binaries on PATH so that Qt can load its DLLs.
    """
    global _INITIALIZED
    if _INITIALIZED and not force:
        return

    base = _find_pyqt5_base()
    if base is None:
        return

    qt_roots = _qt_root_candidates(base)
    plugin_dir = _first_existing(root / "plugins" for root in qt_roots)
    platform_plugin_dir = _first_existing(root / "plugins" / "platforms" for root in qt_roots)
    if plugin_dir:
        os.environ.setdefault("QT_PLUGIN_PATH", str(plugin_dir))
        # QT_QPA_PLATFORM_PLUGIN_PATH overrides Qt's internal search order, so
        # only override it when not already provided by the user (or force=True).
        if platform_plugin_dir and (force or not os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH")):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(platform_plugin_dir)

    bin_dir = _first_existing(root / "bin" for root in qt_roots)
    if bin_dir:
        _prepend_env_path("PATH", str(bin_dir))

    os.environ.setdefault("QT_QPA_PLATFORM", "windows")
    _INITIALIZED = True
