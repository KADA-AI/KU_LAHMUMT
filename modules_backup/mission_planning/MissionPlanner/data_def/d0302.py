from __future__ import annotations
import os
from collections import OrderedDict
from typing import List, Dict, Set, Tuple

from .mission_helpers import now_ms_since_2000
try:
    from .id_allocator import (
        next_imp_id,
        reserve_imp_ids, reserve_individual_mission_ids, reserve_path_ids,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.data_def.id_allocator import (  # type: ignore
        next_imp_id,
        reserve_imp_ids, reserve_individual_mission_ids, reserve_path_ids,
    )




def _sw_code(default: str = "MMR") -> str:
    """Return SW code (MMR/MSM/MOB) based on KU_ROLE."""
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "mission": "MMR",
        "monitoring": "MSM",
        "decision": "MOB",
    }.get(role, default)


# ─────────────────────────────────────────────────────────────
# 0302 – 패키지/미션 검증 (리팩터)
# ─────────────────────────────────────────────────────────────
# 미션 유형별 필수 데이터 세트 --------------------------------
_MISSION_NEEDS_COORD  : set[int] = {5, 7, 9}     # 좌표점, 이동, 은엄폐
_MISSION_NEEDS_LINE   : set[int] = {6}           # 통로정찰
_MISSION_NEEDS_AREA   : set[int] = {3, 4}        # 영역수색·경계
_MISSION_NEEDS_TARGET : set[int] = {1, 2}        # 표적추적·공격

# ─────────────────────────────────────────────────────────────
def _clean_individual_mission(miss: dict,
                              cmpk_id: int) -> dict:
    """
    • GUI/렌더링용 여분 필드 제거
    • relatedMission 을 스펙(relatedMissionType=1) 에 맞게 재구성
    • individualMissionInfo 에 남아있는
      'relatedIndividualMissionIDList' 키 제거
    """
    im = dict(miss)            # shallow copy

    # ── 1) 최상위 불필요 필드 제거 ───────────────────────────
    for k in ("aircraftID", "color", "isFormationFlight",
              "formation"):
        im.pop(k, None)

    # ── 2) relatedMission 재설정 ───────────────────────────
    rel_src = miss.get("relatedMission") if isinstance(miss.get("relatedMission"), dict) else {}
    try:
        rel_type = int(rel_src.get("relatedMissionType", 1))
    except Exception:
        rel_type = 1
    try:
        prior_id = int(rel_src.get("priorMissionID", 0))
    except Exception:
        prior_id = 0

    input_mid = rel_src.get("inputMissionID") if isinstance(rel_src, dict) else None
    if input_mid is None:
        input_mid = miss.get("inputMissionID")
    try:
        input_mid = int(input_mid)
    except Exception:
        input_mid = 0

    if rel_type not in (0, 1, 2):
        rel_type = 1
    if prior_id < 0:
        prior_id = 0

    im["relatedMission"] = {
        "relatedMissionType": rel_type,
        "inputMissionID":    input_mid if input_mid > 0 else 0,
        "priorMissionID":    prior_id,
    }

    # ── 3) IM-Info 내부 불필요 필드 제거 ────────────────────
    info = dict(im["individualMissionInfo"])  # copy
    info.pop("relatedIndividualMissionIDList", None)   # ★ 여기!
    im["individualMissionInfo"] = info

    return im

# ─────────────────────────────────────────────────────────────
def _path_base(aid: int) -> int:
    return {
        1: 100_000_001, 2: 200_000_001, 3: 300_000_001,  # manned
        4: 400_000_001, 5: 500_000_001, 6: 600_000_001,  # UAV
    }.get(aid, 0)


def _has_nested_coordinate_rows(rows: object, *, min_points: int) -> bool:
    if not isinstance(rows, list):
        return False
    for row in rows:
        if not isinstance(row, dict):
            continue
        coords = row.get("coordinateList")
        if isinstance(coords, list) and len(coords) >= int(min_points):
            return True
    return False


def _sanitize_individual_mission_geometry(info: dict) -> None:
    if not isinstance(info, dict):
        return
    try:
        mission_type = int(info.get("individualMissionType", 0) or 0)
    except Exception:
        mission_type = 0
    area_present = _has_nested_coordinate_rows(info.get("areaList"), min_points=3)
    if mission_type in _MISSION_NEEDS_AREA and area_present:
        info["coordinateList"] = []
        return
    line_present = _has_nested_coordinate_rows(info.get("lineList"), min_points=2)
    if area_present and not line_present:
        info["coordinateList"] = []

def _validate_mission_packages(
    packages: list[dict],
    plan_pkg_map: dict[int, int] | None,
    cmpk_id: int,
) -> None:
    """0302 IMP → 사양 일치 여부 검증 (Mission/Path/ID 등)."""
    seen_pkg:   set[int] = set()
    seen_im_id: set[int] = set()
    seen_path:  set[int] = set()
    seen_aircraft: set[int] = set()

    for pkg in packages:  # ─ 패키지 루프
        for key in ("timestamp", "individualMissionPackageID",
                    "aircraftID", "individualMissionList"):
            if key not in pkg:
                raise ValueError(f"[0302] package missing '{key}'")

        aid      = pkg["aircraftID"]
        pkg_id   = pkg["individualMissionPackageID"]
        im_list  = pkg["individualMissionList"]

        # ── aircraftID 검증 (1..6) & 패키지 중복 금지
        if not (isinstance(aid, int) and 1 <= aid <= 6):
            raise ValueError(f"[0302] invalid aircraftID: {aid} (must be 1..6)")
        if aid in seen_aircraft:
            raise ValueError(f"[0302] duplicate package for aircraftID {aid}")
        seen_aircraft.add(aid)

        # ① 0301 패키지-매핑 확인 ---------------------------
        if plan_pkg_map and plan_pkg_map.get(aid) != pkg_id:
            raise ValueError(f"[0302] aircraft {aid}: packageID {pkg_id} "
                             f"≠ 0301 map {plan_pkg_map.get(aid)}")

        if pkg_id in seen_pkg:
            raise ValueError(f"[0302] duplicate IMP ID {pkg_id}")
        seen_pkg.add(pkg_id)

        if not isinstance(im_list, list) or not im_list:
            raise ValueError(f"[0302] aircraft {aid}: empty individualMissionList")

        base_path = _path_base(aid)
        upper_path = base_path + 100_000_000
        last_im_id = last_path_id = None

        # ② 개별 IM 검증 -----------------------------------
        for im in im_list:
            for key in ("individualMissionID", "isDone",
                        "relatedMission", "individualMissionInfo", "pathID"):
                if key not in im:
                    raise ValueError(f"[0302] IM missing '{key}'")

            im_id        = im["individualMissionID"]
            mission_info = im["individualMissionInfo"]
            mission_type = mission_info["individualMissionType"]
            path_id      = im["pathID"]

            # ── IM-ID 검증
            if not isinstance(im_id, int) or im_id < 900_000_001:
                raise ValueError(f"[0302] IM ID {im_id} < 900,000,001")
            if im_id in seen_im_id:
                raise ValueError(f"[0302] duplicate IM ID {im_id}")
            if last_im_id and im_id <= last_im_id:
                raise ValueError(f"[0302] IM IDs not ascending")
            seen_im_id.add(im_id)
            last_im_id = im_id

            # ── Path-ID 검증
            if not (base_path <= path_id < upper_path):
                raise ValueError(f"[0302] aircraft {aid}: PathID {path_id} out of range")
            if path_id in seen_path:
                raise ValueError(f"[0302] duplicate PathID {path_id}")
            if last_path_id and path_id <= last_path_id:
                raise ValueError(f"[0302] PathIDs not ascending")
            seen_path.add(path_id)
            last_path_id = path_id

            # ── RelatedMission 검증
            rel = im["relatedMission"]
            rel_type = rel.get("relatedMissionType")
            if rel_type not in (1, 2):
                raise ValueError(
                    f"[0302] IM {im_id}: relatedMissionType must be 1 or 2"
                )
            im_input_id = rel.get("inputMissionID")
            if not isinstance(im_input_id, int) or im_input_id <= 0:
                raise ValueError(
                    f"[0302] IM {im_id}: inputMissionID must be a positive integer"
                )
            prior_id = rel.get("priorMissionID", 0)
            if rel_type == 1 and prior_id not in (0,):
                raise ValueError(f"[0302] IM {im_id}: priorMissionID must be 0 for relatedMissionType=1")
            if rel_type == 2 and (not isinstance(prior_id, int) or prior_id <= 0):
                raise ValueError(f"[0302] IM {im_id}: priorMissionID must be positive for relatedMissionType=2")

            # ── Mission-Type별 필수 데이터 검증 --------------
            if mission_type in _MISSION_NEEDS_COORD and not mission_info.get("coordinateList"):
                raise ValueError(f"[0302] IM {im_id}: coordinateList required")
            if mission_type in _MISSION_NEEDS_LINE and not mission_info.get("lineList"):
                raise ValueError(f"[0302] IM {im_id}: lineList required")
            if mission_type in _MISSION_NEEDS_AREA and not mission_info.get("areaList"):
                raise ValueError(f"[0302] IM {im_id}: areaList required")
            if mission_type in _MISSION_NEEDS_AREA and mission_info.get("coordinateList"):
                raise ValueError(f"[0302] IM {im_id}: coordinateList must be empty for area missions")
            if mission_type in _MISSION_NEEDS_TARGET and not mission_info.get("targetID"):
                raise ValueError(f"[0302] IM {im_id}: targetID required")

def build_mission_packages(
    missions: list[dict],
    *,
    cmpk_id: int,
    plan_pkg_map: dict[int, int] | None = None,
) -> list[dict]:
    """
    missions: 0302 직전 단계(IM list, aircraftID 포함)
    ────────────────────────────────────────────────────────
    • aircraftID별 IMP 묶기 (1..6만 허용, 중복 패키지 불가)
    • PathID / individualMissionID 새로 발급
    • isDone → bool 캐스팅
    • altitude(int), lat/lon(float), width(float), isHole(bool) 강제 보정
    • patternType 기본값 보정
    """
    now_ms = now_ms_since_2000()

    # 1) aircraftID별 분류 ─────────────────────────────
    grouped: dict[int, list[dict]] = {}
    for m in missions:
        aid = m.get("aircraftID")
        if not (isinstance(aid, int) and 1 <= aid <= 6):
            raise ValueError(f"[0302] invalid aircraftID in missions: {aid} (must be 1..6)")
        grouped.setdefault(aid, []).append(m)

    out: list[dict] = []
    pending_pkg_ids: dict[int, int] = {}
    missing_pkg_aids = [aid for aid in grouped if not (plan_pkg_map or {}).get(aid)]
    if missing_pkg_aids:
        reserved_pkg_ids = reserve_imp_ids(len(missing_pkg_aids))
        pending_pkg_ids = {
            int(aid): int(pkg_id)
            for aid, pkg_id in zip(missing_pkg_aids, reserved_pkg_ids)
        }
    for aid, im_raw in grouped.items():

        # 2) 패키지 ID 결정
        pkg_id = plan_pkg_map.get(aid) if plan_pkg_map else None
        if not pkg_id:
            pkg_id = pending_pkg_ids.get(int(aid))
        if not pkg_id:
            pkg_id = next_imp_id()

        base_path   = _path_base(aid)
        upper_path  = base_path + 100_000_000
        seen_path: set[int] = set()
        invalid_path_rows: list[dict] = []
        for im in im_raw:
            pid = im.get("pathID", 0)
            if not (base_path <= pid < upper_path) or pid in seen_path:
                invalid_path_rows.append(im)
                continue
            seen_path.add(pid)
        reserved_path_ids_for_aid = (
            reserve_path_ids(aid, len(invalid_path_rows))
            if invalid_path_rows
            else []
        )
        reserved_path_iter = iter(reserved_path_ids_for_aid)
        seen_path.clear()

        # 3) PathID/필드 보정 ───────────────────────────
        fixed: list[dict] = []
        for im in im_raw:
            im["individualMissionPlanPackageID"] = pkg_id  # 내부 trace용(아래에서 출력 제외)

            # ▸ PathID 충돌/범위 확인
            pid = im.get("pathID", 0)
            if not (base_path <= pid < upper_path) or pid in seen_path:
                pid = int(next(reserved_path_iter))
            im["pathID"] = pid
            seen_path.add(pid)

            # ▸ autoZoomIn (UAV ≥ 4)
            info = im.setdefault("individualMissionInfo", {})
            info["autoZoomIn"] = (aid >= 4)

            # ▸ patternType 기본값 보정
            try:
                info["patternType"] = int(info.get("patternType", 0))
            except Exception:
                info["patternType"] = 0

            # ▸ isDone → bool 캐스팅
            im["isDone"] = bool(im.get("isDone", False))

            # ▸ coordinate/line/area 타입 강제
            # coordinateList (top-level in info)
            for p in info.get("coordinateList", []) or []:
                if "latitude" in p:
                    try: p["latitude"] = float(p["latitude"])
                    except Exception: p["latitude"] = 0.0
                if "longitude" in p:
                    try: p["longitude"] = float(p["longitude"])
                    except Exception: p["longitude"] = 0.0
                if "altitude" in p:
                    try: p["altitude"] = int(float(p["altitude"]))
                    except Exception: p["altitude"] = 0

            # lineList
            for blk in info.get("lineList", []) or []:
                if "width" in blk:
                    try: blk["width"] = float(blk["width"])
                    except Exception: blk["width"] = 0.0
                for p in blk.get("coordinateList", []) or []:
                    if "latitude" in p:
                        try: p["latitude"] = float(p["latitude"])
                        except Exception: p["latitude"] = 0.0
                    if "longitude" in p:
                        try: p["longitude"] = float(p["longitude"])
                        except Exception: p["longitude"] = 0.0
                    if "altitude" in p:
                        try: p["altitude"] = int(float(p["altitude"]))
                        except Exception: p["altitude"] = 0

            # areaList
            for blk in info.get("areaList", []) or []:
                if "isHole" in blk:
                    blk["isHole"] = bool(blk["isHole"])
                for p in blk.get("coordinateList", []) or []:
                    if "latitude" in p:
                        try: p["latitude"] = float(p["latitude"])
                        except Exception: p["latitude"] = 0.0
                    if "longitude" in p:
                        try: p["longitude"] = float(p["longitude"])
                        except Exception: p["longitude"] = 0.0
                    if "altitude" in p:
                        try: p["altitude"] = int(float(p["altitude"]))
                        except Exception: p["altitude"] = 0

            _sanitize_individual_mission_geometry(info)

            fixed.append(im)

        # 4) PathID 기준 정렬 → IM-ID 재발급
        fixed.sort(key=lambda m: m["pathID"])
        reserved_im_ids = reserve_individual_mission_ids(len(fixed)) if fixed else []
        for im, im_id in zip(fixed, reserved_im_ids):
            im["individualMissionID"] = int(im_id)

        # 5) 필드 순서/정리(불필요 키 제거)
        ordered_list: list[dict] = []
        for im in fixed:
            im = _clean_individual_mission(im, cmpk_id)  # relatedMission 재설정 + 잡키 제거
            ordered_list.append(OrderedDict([
                ("individualMissionID",   im["individualMissionID"]),
                ("isDone",                im["isDone"]),
                ("relatedMission",        im["relatedMission"]),
                ("individualMissionInfo", im["individualMissionInfo"]),
                ("pathID",                im["pathID"]),
            ]))

        # 6) IMP 패키지 작성
        out.append(OrderedDict([
            ("timestamp",                  now_ms),
            ("Source",          _sw_code()),
            ("individualMissionPackageID", pkg_id),
            ("aircraftID",                 aid),
            ("individualMissionList",      ordered_list),
        ]))

    # 7) 최종 스펙 검증
    _validate_mission_packages(out, plan_pkg_map, cmpk_id)
    return out
