# generator/message0203_push.py
import os, glob, json
from datetime import datetime, timezone
from nFusion.Model.msg_0203 import FlightReferenceInfo

# ★ FlightReferenceInfo JSON 위치 (파일명=missionReferencePackageID.json)
PLAN_DIR = r"C:\Users\LAHMUMT_2\Desktop\nFusion\missionPlanner\plannedMission\MissionReferenceInfo"

# 2000-01-01 UTC 기준 ms
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
def _now_ms() -> int:
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def _dict_to_obj(body_dict: dict) -> FlightReferenceInfo:
    """
    dict → FlightReferenceInfo(C#)
    필수 필드만 세팅: timestamp, missionReferencePackageID
    (나머지는 DLL 기본값 사용)
    """
    obj = FlightReferenceInfo()
    obj.timestamp                 = int(body_dict["timestamp"])
    obj.missionReferencePackageID = int(body_dict["missionReferencePackageID"])
    return obj

def _list_package_ids() -> list:
    """PLAN_DIR의 *.json 파일명에서 숫자만 추출 → 오름차순"""
    ids = []
    try:
        for path in glob.glob(os.path.join(PLAN_DIR, "*.json")):
            stem = os.path.splitext(os.path.basename(path))[0]
            if stem.isdigit():
                ids.append(int(stem))
    except Exception:
        pass
    return sorted(ids)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """외부 dict를 C# 객체로 변환해 Push하고, 로그 bytes 반환"""
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0203] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0203] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    """
    • PLAN_DIR 의 JSON 파일명 → missionReferencePackageID 로 사용
    • {timestamp, missionReferencePackageID} 메시지를 순차 Push
    """
    logs = []
    ids = _list_package_ids()
    if not ids:
        # DB파일이 없으면 제너레이터에서 샘플 만들고 바로 Push
        from generator.message0203_generator import make_msg0203_body
        logs.append(make_and_push(make_msg0203_body(), node_messenger))
        return b"\n".join(logs)

    for pid in ids:
        body = {
            "timestamp":                 _now_ms(),
            "missionReferencePackageID": int(pid),
        }
        logs.append(make_and_push(body, node_messenger))
    return b"\n".join(logs)
