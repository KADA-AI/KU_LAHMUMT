# KU/AnS/scheduling.py

import json
import os
from typing import List, Callable, Optional, Union
from pulp import (
    LpProblem, LpMinimize, LpVariable, lpSum,
    LpStatus, PULP_CBC_CMD, value
)

def run_pulp_scheduling(
    imp_path_in: str,
    imp_path_out: Optional[str] = None,
    uav_id_list: Optional[List[str]] = None,
    log: Callable[[str], None] = print,
) -> bool:
    """
    Single-IMP(0302) 파일을 읽어서 PuLP 스케줄링을 수행하고,
    AircraftID 필드를 균등하게 재배정하여 다시 저장합니다.

    Parameters
    ----------
    imp_path_in : str
        입력 IMP(0302) JSON 파일 경로
    imp_path_out : str | None
        출력 IMP(0302) JSON 파일 경로.
        None 이면 in-place 덮어쓰기
    uav_id_list : List[str] | None
        스케줄링에 사용할 UAV ID 목록.
        None 이면 IMP 파일 안의 단일 AircraftID 만 사용
    log : Callable[[str], None]
        진행 로그를 남길 콜백

    Returns
    -------
    bool
        스케줄링 성공 여부
    """
    # ── 1) 파일 로드 ─────────────────────────────────────────
    if not os.path.exists(imp_path_in):
        log(f"[PuLP] 입력 IMP가 없습니다: {imp_path_in}")
        return False

    with open(imp_path_in, "r", encoding="utf-8") as f:
        imp = json.load(f)

    missions = imp.get("IndividualMissionList", [])
    if not missions:
        log("[PuLP] IndividualMissionList가 비어 있습니다.")
        return False

    # ── 2) UAV 목록 확보 ─────────────────────────────────────
    if uav_id_list is None:
        uav_id_list = [imp.get("AircraftID")] if imp.get("AircraftID") else []
    num_uavs = len(uav_id_list)
    if num_uavs == 0:
        log("[PuLP] UAV가 0대입니다.")
        return False

    # ── 3) (mid, est_time) 리스트 구성 ─────────────────────────
    mission_info: List[tuple[int, float]] = []
    for m in missions:
        mid = m.get("IndividualMissionID")
        est = 0.0
        try:
            est = m["IndividualMissionInfo"]["MissionDetail"]["AreaList"]\
                   .get("EstimatedMissionTime", 0.0)
        except Exception:
            pass
        mission_info.append((mid, est))

    n_m = len(mission_info)
    if n_m == 0:
        log("[PuLP] 스케줄 대상 임무가 없습니다.")
        return False

    # ── 4) 그룹핑: UAV 개수만큼씩 묶음 ─────────────────────────
    gsize = num_uavs
    groups = [mission_info[i:i+gsize] for i in range(0, n_m, gsize)]
    log(f"[PuLP] 임무 {n_m}개 → {len(groups)}개 그룹 (그룹당 ≤{gsize}개)")

    # ── 5) MILP 모델 구축 ─────────────────────────────────────
    prob = LpProblem("UAV_Schedule", LpMinimize)

    # x[u,g,i]: 그룹 g 의 태스크 i 를 UAV u 가 수행
    x: dict[tuple[int,int,int], LpVariable] = {}
    for u in range(num_uavs):
        for g, grp in enumerate(groups):
            for i in range(len(grp)):
                x[u,g,i] = LpVariable(f"x_{u}_{g}_{i}", cat="Binary")

    # 각 UAV의 총 작업시간 T[u], 최대/최소 Tmax/Tmin
    T   = [LpVariable(f"T_{u}", lowBound=0) for u in range(num_uavs)]
    Tmax = LpVariable("Tmax", lowBound=0)
    Tmin = LpVariable("Tmin", lowBound=0)

    # (1) 각 태스크는 정확히 1대 UAV에
    for g, grp in enumerate(groups):
        for i in range(len(grp)):
            prob += lpSum(x[u,g,i] for u in range(num_uavs)) == 1

    # (2) 같은 그룹에서 UAV당 최대 1개 태스크
    for g, grp in enumerate(groups):
        for u in range(num_uavs):
            prob += lpSum(x[u,g,i] for i in range(len(grp))) <= 1

    # (3) T[u] = 할당된 태스크들의 시간 합
    for u in range(num_uavs):
        prob += T[u] == lpSum(
            x[u,g,i] * groups[g][i][1]
            for g in range(len(groups))
            for i in range(len(groups[g]))
        )

    # (4) Tmax/Tmin 제약
    for u in range(num_uavs):
        prob += Tmax >= T[u]
        prob += Tmin <= T[u]

    # Objective: minimize (Tmax - Tmin)
    prob += Tmax - Tmin

    # ── 6) solve ──────────────────────────────────────────────
    status = prob.solve(PULP_CBC_CMD(msg=0))
    log(f"[PuLP] Status = {LpStatus[status]}")

    if LpStatus[status] != "Optimal":
        log("[PuLP] 최적해를 찾지 못했습니다. 원본 IMP 저장.")
        outp = imp_path_out or imp_path_in
        with open(outp, "w", encoding="utf-8") as f:
            json.dump(imp, f, indent=4, ensure_ascii=False)
        return False

    diff = value(Tmax) - value(Tmin)
    log(f"[PuLP] 최적 편차 = {diff:.2f} s")

    # ── 7) 결과 반영: 각 미션에 할당된 UAV 갱신 ─────────────────
    assign: dict[int, str] = {}
    for g, grp in enumerate(groups):
        for i, (mid, _) in enumerate(grp):
            for u in range(num_uavs):
                if value(x[u,g,i]) > 0.5:
                    assign[mid] = uav_id_list[u]
    for m in missions:
        mid = m.get("IndividualMissionID")
        if mid in assign:
            m["AircraftID"] = assign[mid]

    # ── 8) 파일 쓰기 ─────────────────────────────────────────
    outp = imp_path_out or imp_path_in
    with open(outp, "w", encoding="utf-8") as f:
        json.dump(imp, f, indent=4, ensure_ascii=False)
    log(f"[PuLP] 저장 완료 → {outp}")

    return True
