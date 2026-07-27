from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from ..models import SplitPiece, SplitRunResult


def _to_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return default


def _to_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _coord_or_none(v: Any) -> Optional[Dict[str, float]]:
    if not isinstance(v, dict):
        return None
    if "latitude" not in v or "longitude" not in v:
        return None
    return {
        "latitude": _to_float(v.get("latitude"), 0.0),
        "longitude": _to_float(v.get("longitude"), 0.0),
        "altitude": _to_float(v.get("altitude"), 0.0),
    }


def _bearing_deg(c0: Optional[Dict[str, float]], c1: Optional[Dict[str, float]]) -> float:
    if not isinstance(c0, dict) or not isinstance(c1, dict):
        return 0.0
    lat0 = math.radians(_to_float(c0.get("latitude"), 0.0))
    lon0 = math.radians(_to_float(c0.get("longitude"), 0.0))
    lat1 = math.radians(_to_float(c1.get("latitude"), 0.0))
    lon1 = math.radians(_to_float(c1.get("longitude"), 0.0))
    dlon = lon1 - lon0
    x = math.sin(dlon) * math.cos(lat1)
    y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
    return float((math.degrees(math.atan2(x, y)) + 360.0) % 360.0)


def _segment_start(seg: Dict[str, Any]) -> float:
    return _to_float(seg.get("startSec"), 0.0)


def _segment_end(seg: Dict[str, Any]) -> float:
    return _to_float(seg.get("endSec"), _segment_start(seg))


def _shift_timeline_from(segments: List[Dict[str, Any]], from_sec: float, delta_sec: float) -> None:
    if delta_sec <= 0.0:
        return
    eps = 1e-6
    for seg in segments:
        if not isinstance(seg, dict):
            continue
        s = _segment_start(seg)
        e = _segment_end(seg)
        if s + eps < from_sec:
            continue
        seg["startSec"] = float(s + delta_sec)
        seg["endSec"] = float(e + delta_sec)
        seg["durationSec"] = float(max(0.0, (e + delta_sec) - (s + delta_sec)))


def _find_task_segment(row: Dict[str, Any], slot_idx: int, candidate_id: str) -> Optional[Dict[str, Any]]:
    segs = row.get("segments")
    if not isinstance(segs, list):
        return None
    for seg in segs:
        if not isinstance(seg, dict):
            continue
        if str(seg.get("kind", "")) != "task":
            continue
        if _to_int(seg.get("slotIndex"), 0) != int(slot_idx):
            continue
        if str(seg.get("candidateID", "")) == str(candidate_id):
            return seg
    for seg in segs:
        if not isinstance(seg, dict):
            continue
        if str(seg.get("kind", "")) != "task":
            continue
        if _to_int(seg.get("slotIndex"), 0) == int(slot_idx):
            return seg
    return None


def _find_slot_begin(row: Dict[str, Any], slot_idx: int, fallback_start: float) -> float:
    segs = row.get("segments")
    if not isinstance(segs, list):
        return float(fallback_start)
    vals: List[float] = []
    for seg in segs:
        if not isinstance(seg, dict):
            continue
        if _to_int(seg.get("slotIndex"), 0) != int(slot_idx):
            continue
        vals.append(_segment_start(seg))
    if not vals:
        return float(fallback_start)
    return float(min(vals))


def _find_prev_end_coord(row: Dict[str, Any], slot_begin: float) -> Optional[Dict[str, float]]:
    segs = row.get("segments")
    if isinstance(segs, list):
        best_end = -1e30
        best_coord: Optional[Dict[str, float]] = None
        for seg in segs:
            if not isinstance(seg, dict):
                continue
            if str(seg.get("kind", "")) != "task":
                continue
            e = _segment_end(seg)
            if e > slot_begin + 1e-6:
                continue
            c = _coord_or_none(seg.get("endCoord"))
            if c is None:
                continue
            if e >= best_end:
                best_end = e
                best_coord = c
        if best_coord is not None:
            return best_coord
    return None


def _segment_sort_key(seg: Dict[str, Any]) -> Tuple[float, float, int]:
    kind = str(seg.get("kind", "task"))
    if kind in {"prestart_hold", "sync_hold", "sync_wait"}:
        k = 0
    elif kind == "move":
        k = 1
    else:
        k = 2
    return (_segment_start(seg), _segment_end(seg), k)


def _recompute_timeline_row(row: Dict[str, Any]) -> None:
    segs = row.get("segments")
    if not isinstance(segs, list):
        row["segments"] = []
        row["taskSec"] = 0.0
        row["moveSec"] = 0.0
        row["totalSec"] = 0.0
        return

    segs = [s for s in segs if isinstance(s, dict)]
    segs.sort(key=_segment_sort_key)

    task_sec = 0.0
    move_sec = 0.0
    max_end = 0.0
    for seg in segs:
        s = _segment_start(seg)
        e = _segment_end(seg)
        if e < s:
            s, e = e, s
        dur = max(0.0, e - s)
        seg["startSec"] = float(s)
        seg["endSec"] = float(e)
        seg["durationSec"] = float(dur)
        if str(seg.get("kind", "")) == "move":
            move_sec += dur
        else:
            task_sec += dur
        if e > max_end:
            max_end = e

    row["segments"] = segs
    row["taskSec"] = float(task_sec)
    row["moveSec"] = float(move_sec)
    row["totalSec"] = float(max_end)


def inject_prestart_hold_missions(
    schedule: Dict[str, Any],
    min_delay_sec: float = 10.0,
    skip_first_slot: bool = True,
) -> Dict[str, Any]:
    if not isinstance(schedule, dict):
        return {
            "ok": False,
            "reason": "invalid_schedule",
            "insertedCount": 0,
        }
    timelines = schedule.get("timelines")
    slot_rows = schedule.get("slotAssignments")
    if not isinstance(timelines, list) or not isinstance(slot_rows, list):
        return {
            "ok": False,
            "reason": "missing_timeline",
            "insertedCount": 0,
        }

    tmap: Dict[int, Dict[str, Any]] = {}
    for row in timelines:
        if not isinstance(row, dict):
            continue
        u = _to_int(row.get("uavID"), 0)
        if u <= 0:
            continue
        if not isinstance(row.get("segments"), list):
            row["segments"] = []
        tmap[u] = row

    inserted: List[Dict[str, Any]] = []
    total_added = 0.0
    by_slot: Dict[int, int] = {}

    slot_rows_sorted = sorted(
        [r for r in slot_rows if isinstance(r, dict)],
        key=lambda r: _to_int(r.get("slotIndex"), 0),
    )

    for slot_row in slot_rows_sorted:
        slot_idx = _to_int(slot_row.get("slotIndex"), 0)
        slot_key = str(slot_row.get("slotKey", ""))
        assignments = slot_row.get("assignments")
        if slot_idx <= 0 or not isinstance(assignments, list):
            continue
        if bool(skip_first_slot) and slot_idx == 1:
            continue

        rows: List[Dict[str, Any]] = []
        for a in assignments:
            if not isinstance(a, dict):
                continue
            if bool(a.get("isDummy", False)):
                continue
            u = _to_int(a.get("uavID"), 0)
            if u <= 0:
                continue
            trow = tmap.get(u)
            if trow is None:
                continue
            cid = str(a.get("candidateID", ""))
            task_seg = _find_task_segment(trow, slot_idx, cid)
            if task_seg is None:
                continue
            task_start = _segment_start(task_seg)
            slot_begin = _find_slot_begin(trow, slot_idx, task_start)
            prev_coord = _find_prev_end_coord(trow, slot_begin)
            if prev_coord is None:
                continue
            rows.append(
                {
                    "uavID": u,
                    "row": trow,
                    "taskSeg": task_seg,
                    "taskStart": float(task_start),
                    "slotBegin": float(slot_begin),
                    "candidateID": cid,
                    "coord": prev_coord,
                }
            )

        if len(rows) <= 1:
            continue

        target_start = max(float(r["taskStart"]) for r in rows)
        for r in rows:
            delay = target_start - float(r["taskStart"])
            if delay <= 1e-6:
                continue
            hold_dur = max(float(delay), float(min_delay_sec))

            u = int(r["uavID"])
            trow = r["row"]
            slot_begin = float(r["slotBegin"])

            _shift_timeline_from(trow["segments"], slot_begin, hold_dur)

            hold_start = slot_begin
            hold_end = slot_begin + hold_dur
            coord = r.get("coord")
            task_seg = r["taskSeg"] if isinstance(r.get("taskSeg"), dict) else {}
            parent_order = _to_int(task_seg.get("parentOrder"), 0)
            next_start = _coord_or_none(task_seg.get("startCoord"))
            next_end = _coord_or_none(task_seg.get("endCoord"))
            bearing_deg = _bearing_deg(next_start, next_end)
            if bearing_deg <= 1e-6:
                bearing_deg = _bearing_deg(coord if isinstance(coord, dict) else None, next_start)
            hold_seg = {
                "kind": "sync_hold",
                "slotIndex": int(slot_idx),
                "slotKey": str(slot_key),
                "startSec": float(hold_start),
                "endSec": float(hold_end),
                "durationSec": float(hold_dur),
                "parentOrder": int(parent_order),
                "missionType": 5,
                "individualMissionType": 5,
                "patternType": 1,
                "bearingDeg": float(bearing_deg),
                "label": f"HOLD S{slot_idx}",
                "candidateID": f"PRE-S{slot_idx}-U{u}",
                "coordinateList": [coord] if isinstance(coord, dict) else [],
                "startCoord": coord if isinstance(coord, dict) else None,
                "endCoord": coord if isinstance(coord, dict) else None,
            }
            trow["segments"].append(hold_seg)

            ins = {
                "uavID": int(u),
                "slotIndex": int(slot_idx),
                "slotKey": str(slot_key),
                "beforeParentOrder": int(parent_order),
                "individualMissionType": 5,
                "patternType": 1,
                "bearingDeg": float(bearing_deg),
                "startSec": float(hold_start),
                "endSec": float(hold_end),
                "durationSec": float(hold_dur),
                "coordinateList": [coord] if isinstance(coord, dict) else [],
                "reason": "align_next_mission_start",
            }
            inserted.append(ins)
            total_added += float(hold_dur)
            by_slot[int(slot_idx)] = int(by_slot.get(int(slot_idx), 0) + 1)

    for row in tmap.values():
        _recompute_timeline_row(row)

    max_total = max((_to_float(r.get("totalSec"), 0.0) for r in tmap.values()), default=0.0)
    min_total = min((_to_float(r.get("totalSec"), 0.0) for r in tmap.values()), default=0.0) if tmap else 0.0
    schedule["timelines"] = list(tmap.values())
    schedule["timelineCount"] = int(len(tmap))
    schedule["totalMaxSec"] = float(max_total)
    schedule["totalMinSec"] = float(min_total)
    schedule["balanceGapSec"] = float(max_total - min_total)
    existing_inserted = schedule.get("insertedMissions")
    merged_inserted: List[Dict[str, Any]] = []
    if isinstance(existing_inserted, list):
        merged_inserted.extend([x for x in existing_inserted if isinstance(x, dict)])
    merged_inserted.extend(inserted)
    merged_inserted.sort(
        key=lambda r: (
            _to_float(r.get("startSec"), 0.0),
            _to_int(r.get("uavID"), 0),
            _to_int(r.get("slotIndex"), 0),
        )
    )
    schedule["insertedMissions"] = merged_inserted
    schedule["insertedMissionCount"] = int(len(merged_inserted))

    return {
        "ok": True,
        "reason": "done",
        "slotCount": int(len(slot_rows_sorted)),
        "insertedCount": int(len(inserted)),
        "insertedSec": float(total_added),
        "bySlot": by_slot,
        "balanceGapSec": float(max_total - min_total),
    }


def sync_piece_schedule_from_timeline(
    split_result: SplitRunResult,
    schedule: Dict[str, Any],
) -> Dict[str, Any]:
    if split_result is None or not isinstance(schedule, dict):
        return {"updatedPieceCount": 0}
    timelines = schedule.get("timelines")
    if not isinstance(timelines, list):
        return {"updatedPieceCount": 0}

    piece_map: Dict[Tuple[int, int], SplitPiece] = {
        (int(p.parent_order), int(p.piece_index)): p for p in split_result.pieces
    }
    updated = 0
    for row in timelines:
        if not isinstance(row, dict):
            continue
        u = _to_int(row.get("uavID"), 0)
        if u <= 0:
            continue
        segs = row.get("segments")
        if not isinstance(segs, list):
            continue
        for seg in segs:
            if not isinstance(seg, dict):
                continue
            if str(seg.get("kind", "")) != "task":
                continue
            refs = seg.get("pieceRefs")
            if not isinstance(refs, list):
                continue
            for pref in refs:
                if not isinstance(pref, (list, tuple)) or len(pref) < 2:
                    continue
                key = (_to_int(pref[0], 0), _to_int(pref[1], 0))
                piece = piece_map.get(key)
                if piece is None or not isinstance(piece.data, dict):
                    continue
                piece.data.setdefault("schedule", {})
                piece.data["schedule"].update(
                    {
                        "slotIndex": _to_int(seg.get("slotIndex"), 0),
                        "slotKey": str(seg.get("slotKey", "")),
                        "candidateID": str(seg.get("candidateID", "")),
                        "selectedVelKmh": _to_float(seg.get("speedKmh"), 0.0),
                        "startSec": _to_float(seg.get("startSec"), 0.0),
                        "endSec": _to_float(seg.get("endSec"), 0.0),
                        "durationSec": _to_float(seg.get("durationSec"), 0.0),
                        "distanceM": _to_float(seg.get("distanceM"), 0.0),
                        "uavID": int(u),
                        "pieceGroupRefs": [list(r) for r in refs if isinstance(r, (list, tuple)) and len(r) >= 2],
                    }
                )
                updated += 1
    return {"updatedPieceCount": int(updated)}
