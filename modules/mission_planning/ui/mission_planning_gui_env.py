from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Any, Tuple

_EPOCH2000_MS = 946_684_800_000


def _now_ms_since_2000() -> int:
    return int(time.time() * 1000) - _EPOCH2000_MS


def _sanitize_reason(value: Any, fallback: str = "init-plan") -> str:
    if isinstance(value, bytes):
        try:
            text_val = value.decode("utf-8", "ignore")
        except Exception:
            text_val = ""
    else:
        text_val = "" if value is None else str(value)
    text_val = text_val.replace("\x00", "").strip()
    return text_val or fallback


def _z4(s: str) -> str:
    s = str(s).strip()
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s



def _bootstrap_paths(script_path: Path) -> Tuple[Path, Path]:
    modules_dir = script_path.resolve().parents[1]
    root = modules_dir.parent
    common_dir = modules_dir / "common"
    for p in (modules_dir / "mission_planning", common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try:
        os.chdir(root)
    except Exception:
        pass
    return root, common_dir


def _ensure_fusion_configs(project_root: Path, common_dir: Path) -> str:
    from modules.common.fusion_files import copy_file_with_retry

    cands = [
        project_root / "nFusionSettings.json",
        common_dir / "nFusionSettings.json",
        project_root / "FusionSettings.json",
        common_dir / "FusionSettings.json",
        project_root / "nFusion" / "FusionSettings.json",
    ]
    src = next((p for p in cands if p.exists()), None)
    if src is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json 이 없습니다.")
    dst = project_root / "nFusionSettings.json"
    if src != dst:
        copy_file_with_retry(src, dst)

    lcands = [
        project_root / "nFusionLicense.lic",
        common_dir / "nFusionLicense.lic",
        project_root / "nFusion" / "nFusionLicense.lic",
    ]
    lsrc = next((p for p in lcands if p.exists()), None)
    if lsrc:
        ldst = project_root / "nFusionLicense.lic"
        if lsrc != ldst:
            copy_file_with_retry(lsrc, ldst)
    return str(dst)


def _load_msglib_and_deps(common_dir: Path) -> None:
    try:
        from dll_files.nFusionImports import clr as _clr  # type: ignore
    except Exception:
        import clr as _clr  # type: ignore

    msg_dir = common_dir / "msg_files"
    stem = msg_dir / "MessageLibrary"
    try:
        _clr.AddReference(str(stem))
    except Exception:
        _clr.AddReference(str(stem.with_suffix(".dll")))
    for s in ("K4586Model", "K4586Model.Assist", "MiscUtil"):
        dll = msg_dir / (s + ".dll")
        if dll.exists():
            try:
                _clr.AddReference(str(dll.with_suffix("")))
            except Exception:
                try:
                    _clr.AddReference(str(dll))
                except Exception:
                    pass
