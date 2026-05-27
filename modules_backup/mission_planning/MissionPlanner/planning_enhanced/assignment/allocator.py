from __future__ import annotations

from typing import Any, Dict, List, Tuple


def _extract_aircraft_id(entry: Any) -> int | None:
    if entry is None:
        return None
    if isinstance(entry, dict):
        entry = entry.get("aircraftID")
    if isinstance(entry, int):
        return entry
    if isinstance(entry, str):
        token = entry.strip().upper()
        if token.startswith(("UAV", "LAH")):
            token = "".join(ch for ch in token if ch.isdigit())
        if not token:
            return None
        try:
            return int(token)
        except ValueError:
            return None
    try:
        return int(entry)
    except (TypeError, ValueError):
        return None


def extract_uav_ids(cmpk: Dict[str, Any]) -> List[int]:
    raw = cmpk.get("availableAircraftList")
    if not isinstance(raw, list):
        return []
    ids: List[int] = []
    for item in raw:
        aid = _extract_aircraft_id(item)
        if aid is None:
            continue
        if 4 <= aid <= 6:
            ids.append(aid)
    return sorted(set(ids))


def resolve_uav_ids(cmpk: Dict[str, Any], override_count: int | None = None) -> List[int]:
    uav_ids = extract_uav_ids(cmpk)
    if override_count is None:
        if uav_ids:
            return uav_ids
        return [4, 5, 6]

    n = max(1, int(override_count))
    if uav_ids:
        return uav_ids[:n] if len(uav_ids) >= n else uav_ids
    base = [4, 5, 6]
    return base[:n] if n <= len(base) else base + list(range(7, 7 + (n - len(base))))


def assign_pieces_round_robin(total_pieces: int, uav_ids: List[int]) -> List[int]:
    if not uav_ids:
        return [0] * total_pieces
    assigned = []
    for idx in range(total_pieces):
        assigned.append(uav_ids[idx % len(uav_ids)])
    return assigned


def summarize_assignment(assigned_uavs: List[int]) -> List[Tuple[int, int]]:
    count: Dict[int, int] = {}
    for aid in assigned_uavs:
        count[aid] = count.get(aid, 0) + 1
    return sorted(count.items(), key=lambda x: x[0])
