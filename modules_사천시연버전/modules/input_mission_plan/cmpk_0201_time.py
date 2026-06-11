# cmpk_0201_time.py
# timestamp = ms since 2000-01-01T00:00:00Z (입력 시점)
from __future__ import annotations
from datetime import datetime, timezone
from typing import Dict, Any, List

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

def now_ms_since_2000() -> int:
    """
    2000-01-01T00:00:00Z 기준 경과 밀리초를 정수로 반환.
    """
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def assign_timestamp_inplace(cmpk: Dict[str, Any], force: bool = False) -> int:
    """
    cmpk['timestamp']를 보장한다.
    - force=False: 유효한 값이 없거나 잘못된 경우에만 현재 시각(ms since 2000)으로 세팅
    - force=True : 무조건 현재 시각으로 덮어씀
    반환: 최종 timestamp(int)
    """
    ts = cmpk.get("timestamp")
    if force or not isinstance(ts, (int, float)) or ts < 0:
        ts = now_ms_since_2000()
        cmpk["timestamp"] = int(ts)
    else:
        # 입력이 float이면 정수화
        cmpk["timestamp"] = int(ts)
    return cmpk["timestamp"]

def validate_timestamp(cmpk: Dict[str, Any], allow_future_ms: int = 5000) -> List[str]:
    """
    timestamp 유효성 검사:
    - 정수/0이상
    - 현재 시각보다 과도하게 미래값 금지(기본 5초 허용)
    """
    errs: List[str] = []
    ts = cmpk.get("timestamp")
    if not isinstance(ts, (int, float)):
        errs.append("timestamp는 2000-01-01 UTC 기준 ms 정수여야 합니다.")
        return errs
    ts = int(ts)
    if ts < 0:
        errs.append("timestamp는 음수일 수 없습니다.")
        return errs
    now_ms = now_ms_since_2000()
    drift = ts - now_ms
    if drift > allow_future_ms:
        errs.append(f"timestamp가 현재 시각보다 {drift}ms 앞섭니다.")
    return errs
