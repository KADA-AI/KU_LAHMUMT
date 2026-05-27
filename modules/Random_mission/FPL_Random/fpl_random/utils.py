"""
공통 유틸리티: 시간, 좌표 오프셋, 상태 저장/로드.
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Tuple

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)


def now_ms_2000() -> int:
    """2000-01-01 기준 UTC ms."""
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def offset_lat_lon(lat: float, lon: float, east_m: float, north_m: float) -> Tuple[float, float]:
    """
    위경도에 동/북 오프셋(m)을 더한 좌표를 반환.
    equirectangular 근사 사용.
    """
    lat_rad = math.radians(lat)
    dlat = north_m / 111_320.0
    dlon = east_m / (111_320.0 * math.cos(lat_rad))
    return lat + dlat, lon + dlon


def save_state(path: Path, data: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2, sort_keys=True)


def load_state(path: Path) -> Dict:
    if not path.exists():
        return {}
    try:
        with path.open("r", encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:
        return {}
