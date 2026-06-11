"""
latest_input_cache.py
임무계획 파이프라인에서 사용할 최신 0201/0203 식별자 캐시.

요구사항:
- SW 재기동 시마다 초기화되어야 하므로 모듈 전역 변수로만 보관한다.
- 외부에서 0201, 0203 메시지를 수신할 때 update_* 함수를 호출해 최신 ID를 갱신한다.
- 임무 계획 실행 시 get_* 계열 함수로 ID 및 부가 정보를 조회한다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import RLock
from typing import Any, Dict, Optional, Tuple


@dataclass(slots=True)
class _InputSnapshot:
    """각 메시지(0201/0203)에 대한 최근 상태."""

    msg_id: str
    package_id: Optional[int] = None
    timestamp: Optional[int] = None
    source: Optional[str] = None
    payload: Dict[str, Any] = field(default_factory=dict)

    def update_from_payload(self, payload: Optional[Dict[str, Any]]) -> None:
        """수신 페이로드에서 ID/타임스탬프 등을 추출해 상태 갱신."""
        if not isinstance(payload, dict):
            return

        # JSON 키는 소문자/대문자 혼재 가능성을 고려해 모두 탐색
        pid_key = "inputMissionPackageID" if self.msg_id == "0201" else "missionReferencePackageID"
        ts_key = "timestamp"
        src_key = "Source"

        def _first(keys: Tuple[str, ...]) -> Any:
            for key in keys:
                if key in payload:
                    return payload[key]
                lowered = key.lower()
                if lowered in payload:
                    return payload[lowered]
                uppered = key.upper()
                if uppered in payload:
                    return payload[uppered]
            return None

        pid_raw = _first((pid_key, "packageID", "packageId"))
        ts_raw = _first((ts_key,))
        src_raw = _first((src_key, "source"))

        if pid_raw is not None:
            try:
                self.package_id = int(pid_raw)
            except (TypeError, ValueError):
                pass
        if ts_raw is not None:
            try:
                self.timestamp = int(ts_raw)
            except (TypeError, ValueError):
                pass
        if src_raw is not None:
            try:
                self.source = str(src_raw)
            except Exception:
                pass

        self.payload = payload


_LOCK = RLock()
_STATE = {
    "0201": _InputSnapshot("0201"),
    "0203": _InputSnapshot("0203"),
}


def reset_latest_inputs() -> None:
    """모든 캐시 상태를 초기화한다."""
    with _LOCK:
        for snap in _STATE.values():
            snap.package_id = None
            snap.timestamp = None
            snap.source = None
            snap.payload = {}


def update_from_payload(msg_id: str, payload: Optional[Dict[str, Any]]) -> None:
    """0201/0203 메시지 페이로드를 전달해 최신 ID 상태를 갱신한다."""
    key = msg_id.strip().upper()
    if key not in _STATE:
        return
    with _LOCK:
        _STATE[key].update_from_payload(payload)


def get_latest_package_id(msg_id: str) -> Optional[int]:
    """등록된 최신 package ID를 반환한다."""
    key = msg_id.strip().upper()
    if key not in _STATE:
        return None
    with _LOCK:
        return _STATE[key].package_id


def get_latest_snapshot(msg_id: str) -> _InputSnapshot:
    """내부 스냅샷을 복사 없이 반환(읽기 전용 용도)."""
    key = msg_id.strip().upper()
    if key not in _STATE:
        raise KeyError(f"unsupported msg_id: {msg_id!r}")
    with _LOCK:
        snap = _STATE[key]
        # dataclass는 mutable이므로 얕은 복사를 만들어 외부 수정 차단
        copied = _InputSnapshot(
            msg_id=snap.msg_id,
            package_id=snap.package_id,
            timestamp=snap.timestamp,
            source=snap.source,
            payload=dict(snap.payload),
        )
        return copied


def describe_latest_ids() -> str:
    """디버깅/로그용 문자열을 만들어 준다."""
    with _LOCK:
        def _fmt(snap: _InputSnapshot) -> str:
            pid = snap.package_id if snap.package_id is not None else "-"
            ts = snap.timestamp if snap.timestamp is not None else "-"
            src = snap.source if snap.source is not None else "-"
            return f"{snap.msg_id}:ID={pid},ts={ts},src={src}"

        return ", ".join(_fmt(_STATE[msg]) for msg in ("0201", "0203"))


def resolve_path_from_cache(msg_id: str, directory) -> Optional[Any]:
    """
    캐시에 저장된 ID를 이용해 주어진 디렉터리에서 대응 JSON 경로를 찾는다.
    directory는 pathlib.Path 혹은 문자열을 받을 수 있다.
    """
    key = msg_id.strip().upper()
    if key not in _STATE:
        return None

    from pathlib import Path

    path_dir = Path(directory)
    with _LOCK:
        pid = _STATE[key].package_id
    if pid is None:
        return None

    candidate = path_dir / f"{pid}.json"
    return candidate if candidate.exists() else None
