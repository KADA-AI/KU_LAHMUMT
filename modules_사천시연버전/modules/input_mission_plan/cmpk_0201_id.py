# cmpk_0201_id.py
# 0201(CMPK) 전용 ID 부여/검증 유틸
# - inputMissionPackageID: 1..N 전역 오름차순, 중복 방지(스토어 파일로 관리)
# - inputMissionID: 패키지 내부 1..M 오름차순, 중복 방지(리스트 순서 기준 재번호)

from __future__ import annotations
from pathlib import Path
from typing import Dict, Any, List, Optional, Iterable
import json
import threading
from datetime import datetime, timezone

# 전역 패키지 ID 저장소 (마지막 발급값 유지)
_STORE_DEFAULT = Path(__file__).resolve().parent / "cmpk_0201_id_tracker.json"
_LOCK = threading.Lock()

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

def _load_store(store_path: Optional[str] = None) -> Dict[str, int]:
    p = Path(store_path) if store_path else _STORE_DEFAULT
    if p.exists():
        try:
            return json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            return {}
    return {}

def _save_store(data: Dict[str, int], store_path: Optional[str] = None) -> None:
    p = Path(store_path) if store_path else _STORE_DEFAULT
    p.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

def next_input_mission_package_id(store_path: Optional[str] = None) -> int:
    """
    전역적으로 다음 inputMissionPackageID를 발급한다. (1부터 시작, 1씩 증가)
    - 스토어 파일에 마지막 발급값을 저장해 중복 방지
    """
    with _LOCK:
        st = _load_store(store_path)
        last = int(st.get("lastPackageID", 0))
        nxt = last + 1
        st["lastPackageID"] = nxt
        _save_store(st, store_path)
        return nxt

def assign_package_id_inplace(cmpk: Dict[str, Any],
                              store_path: Optional[str] = None) -> int:
    """
    cmpk 객체에 inputMissionPackageID가 없거나 1 미만이면 새 ID를 발급하여 설정.
    이미 1 이상의 값이 있으면 그대로 둔다. 반환: 최종 ID
    """
    key = "inputMissionPackageID"
    cur = cmpk.get(key)
    if not isinstance(cur, int) or cur < 1:
        cur = next_input_mission_package_id(store_path)
        cmpk[key] = cur
    return cur

def assign_mission_ids_inplace(cmpk: Dict[str, Any]) -> None:
    """
    패키지 내부 inputMissionList의 각 inputMissionID를
    리스트의 현재 순서대로 1..N으로 재부여한다.
    (중복/누락 방지, 오름차순 보장)
    """
    missions = cmpk.get("inputMissionList") or []
    for i, m in enumerate(missions, start=1):
        if not isinstance(m, dict):
            raise TypeError("inputMissionList 원소는 객체여야 합니다.")
        m["inputMissionID"] = i

def validate_sequential_ids(cmpk: Dict[str, Any],
                            existing_package_ids: Optional[Iterable[int]] = None
                            ) -> List[str]:
    """
    ID 규칙 검증:
    - inputMissionPackageID: 1 이상 정수, (옵션) existing_package_ids와 중복 금지
    - inputMissionList[].inputMissionID: 1..N 오름차순, 중복/누락 없음
    반환: 에러 메시지 리스트(비어있으면 통과)
    """
    errors: List[str] = []

    # 패키지 ID 검증
    pkg_id = cmpk.get("inputMissionPackageID")
    if not isinstance(pkg_id, int) or pkg_id < 1:
        errors.append("inputMissionPackageID는 1 이상의 정수여야 합니다.")
    else:
        if existing_package_ids is not None and pkg_id in set(existing_package_ids):
            errors.append(f"inputMissionPackageID 중복: {pkg_id}")

    # 미션 ID 검증
    missions = cmpk.get("inputMissionList")
    if missions is None:
        missions = []
    if not isinstance(missions, list):
        errors.append("inputMissionList는 배열이어야 합니다.")
        return errors

    ids: List[int] = []
    for idx, m in enumerate(missions):
        if not isinstance(m, dict):
            errors.append(f"inputMissionList[{idx}]는 객체여야 합니다.")
            continue
        mid = m.get("inputMissionID")
        if not isinstance(mid, int) or mid < 1:
            errors.append(f"inputMissionList[{idx}].inputMissionID는 1 이상의 정수여야 합니다.")
        else:
            ids.append(mid)

    if ids:
        # 오름차순 & 1..N 연속성 검사
        n = len(ids)
        expected = list(range(1, n + 1))
        if ids != expected:
            errors.append(f"inputMissionID는 1..{n} 오름차순이어야 합니다. (현재: {ids})")
        # 중복 검사(위 검사에서 연속성 깨지면 중복 가능성 존재)
        if len(set(ids)) != len(ids):
            errors.append("inputMissionID에 중복이 있습니다.")

    return errors
