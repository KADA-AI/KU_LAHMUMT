from __future__ import annotations

import sys
from importlib import import_module
from pathlib import Path
from typing import Any


def import_runtime_compat_module(canonical_name: str, wrapper_file: str) -> Any:
    runtime_dir = Path(wrapper_file).resolve().parent
    project_root = next(
        (
            parent
            for parent in Path(wrapper_file).resolve().parents
            if (parent / "modules" / "common").exists()
        ),
        Path(wrapper_file).resolve().parents[3],
    )
    project_root_str = str(project_root)
    if project_root_str not in sys.path:
        sys.path.insert(0, project_root_str)

    removed_entries: list[tuple[int, str]] = []
    try:
        cwd = Path.cwd().resolve()
    except Exception:
        cwd = None
    if cwd == runtime_dir:
        kept: list[str] = []
        for index, entry in enumerate(sys.path):
            try:
                entry_path = Path(entry or cwd).resolve()
            except Exception:
                entry_path = None
            if entry in ("", str(runtime_dir), str(runtime_dir.resolve())) or entry_path == runtime_dir:
                removed_entries.append((index, entry))
                continue
            kept.append(entry)
        sys.path[:] = kept

    logging_module = sys.modules.get("logging")
    logging_file = getattr(logging_module, "__file__", None)
    if logging_file:
        try:
            if Path(logging_file).resolve().is_relative_to(runtime_dir / "logging"):
                sys.modules.pop("logging", None)
        except Exception:
            pass

    try:
        return import_module(canonical_name)
    finally:
        for index, entry in sorted(removed_entries, key=lambda item: item[0]):
            sys.path.insert(min(index, len(sys.path)), entry)
