# receive/message0203_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0203 import FlightReferenceInfo
from .database import received_db                 # DB 저장 모듈
from receive_center import notify                 # GUI 알림 함수

import json, traceback, sys, os

# 대/소문자 안전 접근
def _get(obj, *names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

# FlightReferenceInfo → dict (필수 필드만)
def _flight_reference_info_to_dict(info: FlightReferenceInfo) -> dict:
    return {
        "timestamp":                 _get(info, "timestamp", "Timestamp"),
        "missionReferencePackageID": _get(info, "missionReferencePackageID", "MissionReferencePackageID"),
    }

# ★ MissionReferenceInfo JSON 저장 경로 (파일명=missionReferencePackageID.json)
PLAN_DIR = r"C:\Users\LAHMUMT_2\Desktop\nFusion\missionPlanner\plannedMission\MissionReferenceInfo"

class FlightReferenceInfoReceiver_0203(
    IFusionReceive[FlightReferenceInfo], IsLocal, IsSingletone
):
    """0203 FlightReferenceInfo 메시지 수신 리시버 (timestamp + missionReferencePackageID 전용)"""
    __namespace__ = "FlightReferenceInfoReceiver_0203"

    def Receive(self, data: FlightReferenceInfo, src):
        try:
            # 1) DB 저장
            received_db.set_received_0203(data)

            # 2) DB 파일 로드
            body_min = _flight_reference_info_to_dict(data)
            pkg_id   = body_min["missionReferencePackageID"]
            json_path = os.path.join(PLAN_DIR, f"{pkg_id}.json")

            print(f"[0203] DB 참조! ({json_path})")

            if os.path.exists(json_path):
                with open(json_path, "r", encoding="utf-8") as f:
                    file_data = json.load(f)
                # 3) GUI 알림: 파일 내용 전체 전달
                notify("0203", json.dumps(file_data, ensure_ascii=False).encode("utf-8", "ignore"))
            else:
                # 파일 없음 → 최소 바디 + 오류 메시지 전달
                notify("0203", json.dumps({"error": "DB 파일 없음", **(body_min or {})}, ensure_ascii=False).encode("utf-8", "ignore"))

        except Exception:
            print("[ERROR][Receive-0203] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
