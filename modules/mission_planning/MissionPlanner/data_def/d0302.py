from __future__ import annotations
from collections import OrderedDict
from typing import List, Dict, Set, Tuple

from .mission_helpers import now_ms_since_2000
from data_def.id_allocator import (
    next_imp_id, next_individual_mission_id, next_path_id,
)

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
    im["relatedMission"] = {
        "relatedMissionType": 1,      # 0=None, 1=CMPK, 2=PriorMission
        "inputMissionID":    cmpk_id, # 0301 inputMissionPackageID
        "priorMissionID":    0,
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

def _validate_mission_packages(
    packages: list[dict],
    plan_pkg_map: dict[int, int] | None,
    cmpk_id: int,
) -> None:
    """0302 IMP → 사양 일치 여부 검증 (Mission/Path/ID 등)."""
    seen_pkg:   set[int] = set()
    seen_im_id: set[int] = set()
    seen_path:  set[int] = set()

    for pkg in packages:                                   # ─ 패키지 루프
        for key in ("timestamp", "individualMissionPackageID",
                    "aircraftID", "individualMissionList"):
            if key not in pkg:
                raise ValueError(f"[0302] package missing '{key}'")

        aid      = pkg["aircraftID"]
        pkg_id   = pkg["individualMissionPackageID"]
        im_list  = pkg["individualMissionList"]

        # 1) 0301 패키지-매핑 확인 ---------------------------
        if plan_pkg_map and plan_pkg_map.get(aid) != pkg_id:
            raise ValueError(f"[0302] aircraft {aid}: packageID {pkg_id} "
                             f"≠ 0301 map {plan_pkg_map.get(aid)}")

        if pkg_id in seen_pkg:
            raise ValueError(f"[0302] duplicate IMP ID {pkg_id}")
        seen_pkg.add(pkg_id)

        base_path = _path_base(aid)
        upper_path = base_path + 100_000_000
        last_im_id = last_path_id = None

        # 2) 개별 IM 검증 ----------------------------------
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
            if im_id < 900_000_001:
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
            if rel["relatedMissionType"] != 1 or rel["inputMissionID"] != cmpk_id:
                raise ValueError(
                    f"[0302] IM {im_id}: relatedMission must type=1 & inputID={cmpk_id}"
                )
            if rel["priorMissionID"] != 0:
                raise ValueError(f"[0302] IM {im_id}: priorMissionID must be 0")

            # ── Mission-Type별 필수 데이터 검증 --------------
            if mission_type in _MISSION_NEEDS_COORD and not mission_info.get("coordinateList"):
                raise ValueError(f"[0302] IM {im_id}: coordinateList required")
            if mission_type in _MISSION_NEEDS_LINE and not mission_info.get("lineList"):
                raise ValueError(f"[0302] IM {im_id}: lineList required")
            if mission_type in _MISSION_NEEDS_AREA and not mission_info.get("areaList"):
                raise ValueError(f"[0302] IM {im_id}: areaList required")
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
    • aircraftID별 IMP 묶기
    • PathID / individualMissionID 새로 발급
    • isDone → bool 캐스팅
    • altitude 숫자, isHole bool 강제 보정
    """
    now_ms = now_ms_since_2000()

    # 1) aircraftID별 분류 ─────────────────────────────
    grouped: dict[int, list[dict]] = {}
    for m in missions:
        grouped.setdefault(m["aircraftID"], []).append(m)

    out: list[dict] = []
    for aid, im_raw in grouped.items():

        # 2) 패키지 ID 결정
        pkg_id = plan_pkg_map.get(aid) if plan_pkg_map else None
        if not pkg_id:
            pkg_id = next_imp_id()

        base_path   = _path_base(aid)
        upper_path  = base_path + 100_000_000
        seen_path: set[int] = set()

        # 3) PathID · autoZoomIn 부여 ──────────────────
        fixed: list[dict] = []
        for im in im_raw:
            im["individualMissionPlanPackageID"] = pkg_id

            # ▸ PathID 충돌/범위 확인
            pid = im.get("pathID", 0)
            if not (base_path <= pid < upper_path) or pid in seen_path:
                pid = next_path_id(aid)
            im["pathID"] = pid
            seen_path.add(pid)

            # ▸ autoZoomIn (UAV ≥ 4)
            im["individualMissionInfo"]["autoZoomIn"] = (aid >= 4)

            # ▸ isDone → bool 캐스팅
            im["isDone"] = bool(im.get("isDone", False))

            # ▸ areaList.isHole / altitude 타입 강제
            info = im["individualMissionInfo"]
            for blk in info.get("areaList", []) + info.get("lineList", []):
                if "isHole" in blk:
                    blk["isHole"] = bool(blk["isHole"])
                for p in blk.get("coordinateList", []):
                    if "altitude" in p:
                        try:
                            p["altitude"] = int(float(p["altitude"]))
                        except Exception:
                            p["altitude"] = 0
            fixed.append(im)

        # 4) PathID 기준 정렬 → IM-ID 재발급
        fixed.sort(key=lambda m: m["pathID"])
        for im in fixed:
            im["individualMissionID"] = next_individual_mission_id()

        # 5) 필드 순서/정리
        ordered_list: list[dict] = []
        for im in fixed:
            im = _clean_individual_mission(im, cmpk_id)
            ordered_list.append(OrderedDict([
                ("individualMissionID",   im["individualMissionID"]),
                ("isDone",                im["isDone"]),   # ← bool 보장
                ("relatedMission",        im["relatedMission"]),
                ("individualMissionInfo", im["individualMissionInfo"]),
                ("pathID",                im["pathID"]),
            ]))

        # 6) IMP 패키지 작성
        out.append(OrderedDict([
            ("timestamp",                  now_ms),
            ("individualMissionPackageID", pkg_id),
            ("aircraftID",                 aid),
            ("individualMissionList",      ordered_list),
        ]))

    # 7) 최종 스펙 검증
    _validate_mission_packages(out, plan_pkg_map, cmpk_id)
    return out

