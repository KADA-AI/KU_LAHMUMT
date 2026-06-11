# -*- coding: utf-8 -*-
"""
Utility helpers for copying nFusion configuration files safely.

These files are touched by multiple GUI processes during auto boot.
When several processes try to overwrite the same destination simultaneously,
Windows can raise PermissionError because the file handle is locked.
The helper below retries the copy operation briefly and also short-circuits
if another process already wrote identical contents.
"""
from __future__ import annotations

import time
from pathlib import Path


def copy_file_with_retry(src: Path, dst: Path, *, attempts: int = 5, delay: float = 0.2) -> None:
    """
    Copy a file while tolerating transient PermissionError on Windows.

    The function copies the file contents eagerly and then retries the write
    a few times if the destination is temporarily locked by another process.
    If the destination already matches the source, it exits early.
    """
    if not src.exists():
        raise FileNotFoundError(f"source not found: {src}")

    data = src.read_bytes()
    dst.parent.mkdir(parents=True, exist_ok=True)

    for attempt in range(max(1, attempts)):
        try:
            dst.write_bytes(data)
            return
        except PermissionError:
            # Another process is writing the same file. If the current file
            # already has the desired contents we can stop retrying.
            try:
                if dst.exists() and dst.read_bytes() == data:
                    return
            except Exception:
                pass

            if attempt == attempts - 1:
                raise
            time.sleep(max(0.01, delay))
