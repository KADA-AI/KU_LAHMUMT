# input_manager.py

import random
import string
import time
from datetime import datetime, timezone
from typing import Callable, Dict, Union


# ==============================================================================
# 1. 헬퍼 함수 (Helper Functions)
# - 원본 파일에 있던 데이터 생성을 위한 보조 함수들을 그대로 가져옵니다.
# ==============================================================================
UINT16_MAX = (1 << 16) - 1  # 65535
UINT32_MAX = (1 << 32) - 1
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
rand_str8 = lambda: "".join(random.choices(string.ascii_uppercase + string.digits, k=8))
rand_int = lambda lo=0, hi=2**16 - 1: random.randint(lo, hi)
rand_bool = lambda: bool(random.getrandbits(1))
rand_float = lambda lo, hi, nd=6: round(random.uniform(lo, hi), nd)
rand_uint32 = lambda: random.randint(0, UINT32_MAX)  # 0 ~ 4 294 967 295
rand_lat = lambda: rand_float(-90, 90, 6)
rand_lon = lambda: rand_float(-180, 180, 6)
rand_alt = lambda: rand_float(0, 50000, 1)
rand_float8 = lambda lo, hi: round(random.uniform(lo, hi), 8)
rand_int32 = lambda lo=0, hi=50000: random.randint(lo, hi)
rand_uint16 = lambda: random.randint(0, UINT16_MAX)
rand_uint: Callable[[], int] = lambda: random.randint(0, 2**32 - 1)
rand_str: Callable[[int], str] = lambda n: "".join(
    random.choices(string.ascii_uppercase + string.digits, k=n)
)

_now_ms = lambda: int(
    (
        datetime.now(timezone.utc).replace(tzinfo=timezone.utc) - _EPOCH_2000
    ).total_seconds()
    * 1000
)


def _rand_lat():
    return rand_float(-90, 90, 6)


def _rand_lon():
    return rand_float(-180, 180, 6)


def _rand_alt():
    return rand_float(0, 50000, 1)


def _coord():
    """위경고 좌표 객체를 생성합니다."""
    return {"latitude": _rand_lat(), "longitude": _rand_lon(), "altitude": _rand_alt()}


# ==============================================================================
# 2. 데이터 생성 함수 (Generator Functions)
# - 제공해주신 파일의 구조와 동일하게 실제 랜덤 데이터를 생성합니다.
# ==============================================================================


def make_msg0101_body():
    """SystemOperationMode(0101) 메시지 바디 생성 (소문자 카멜)"""
    return {"timestamp": int(time.time() * 1000), "systemMode": random.randint(0, 3)}


def make_msg0202_body():
    """
    PriorMissionInfo(0202) 메시지 바디 생성 (소문자 카멜)
    """
    now_ms = int(time.time() * 1000)
    body = {"timestamp": now_ms, "priorMissionList": []}
    for _ in range(rand_int(1, 3)):
        entry = {
            "priorMissionID": rand_uint32(),
            # 1: 좌표지향, 2: 표적추적
            "missionType": rand_int(1, 2),
            "coordinateOrientation": {
                "latitude": rand_lat(),
                "longitude": rand_lon(),
                "altitude": rand_alt(),
            },
            "targetOrientation": {"targetID": rand_int(1000, 9999)},
        }
        body["priorMissionList"].append(entry)
    return body


def _coord() -> dict:
    return {
        "latitude": rand_float8(-90, 90),
        "longitude": rand_float8(-180, 180),
        "altitude": rand_int32(0, 50000),
    }


def _make_agent_state(agent_id: int) -> dict:
    return {
        # ───────── 제약 반영 ─────────
        "aircraftID": agent_id,  # 0-6
        "isUnmanned": bool(random.getrandbits(1)),
        "coordinate": _coord(),
        "velocity": {
            "speed": round(random.uniform(0, 250), 1),
            "heading": round(random.uniform(0, 360), 1),
        },
        "fuel": round(random.uniform(0, 100), 1),
        "health": random.randint(0, 2),  # 0,1,2
        "mannedInfo": {
            "weapons": {
                "type1": random.randint(0, 10),
                "type2": random.randint(0, 10),
                "type3": random.randint(0, 10),
            },
            "datalinkStatus": {
                "isConnectedToUAV1": bool(random.getrandbits(1)),
                "isConnectedToUAV2": bool(random.getrandbits(1)),
                "isConnectedToUAV3": bool(random.getrandbits(1)),
            },
        },
        "unmannedInfo": {
            "currentWaypointID": {"waypointID": random.randint(1, 9999)},
            "flightMode": random.randint(0, 9),  # 0-9 (기존 그대로)
            "loiterCoordinate": _coord(),
            "targetFollowing": {"targetID": rand_uint32()},
            "leaderAircraftID": {"aircraftID": random.randint(0, 6)},
            "sensorInfo": {
                "operationalMode": random.randint(0, 3),
                "sensorType": random.randint(0, 3),  # 0-3
                "fov": round(random.uniform(10, 120), 1),
                "centerCoordinate": _coord(),
                "footprintCorner": [_coord() for _ in range(4)],
            },
            "payloadHealth": random.randint(0, 3),  # 0-3
            "fuelWarning": random.randint(0, 3),  # 0-3
        },
    }


# ────────── Main ──────────
def make_msg0401_body(num_agents: Union[int, None] = None) -> dict:
    if num_agents is None:
        num_agents = random.randint(1, 6)

    now_ms = int(time.time() * 1000)
    return {
        "timestamp": now_ms,
        "agentStateList": [
            _make_agent_state(random.randint(0, 6)) for _ in range(num_agents)
        ],
    }


def make_msg0402_body():
    """
    0402: 상황 인식 정보 (제공해주신 파일 구조와 동일)
    """
    body = {
        "timestamp": int(time.time() * 1000),
        "roiInfo": {
            "aircraftID": rand_int(),
            "coordinate": _coord(),
            "fov": rand_float(0, 180, 2),
        },
        "targetList": [],
    }
    for _ in range(random.randint(1, 4)):
        tgt = {
            "targetID": rand_int(0, 2**16 - 1),
            "targetType": rand_int(0, 255),
            "coordinate": _coord(),
            "watcher": {"aircraftID": rand_int()},
            "targetInFrame": rand_bool(),
            "isDestroyed": rand_bool(),
            "threat": rand_float(0, 100, 2),
        }
        body["targetList"].append(tgt)
    return body


def _make_uncompleted_input(num: int):
    return [{"inputMissionID": rand_uint32()} for _ in range(num)]


def _make_completed_input(num: int):
    return [{"inputMissionID": rand_uint32()} for _ in range(num)]


def _make_uncompleted_individual(num: int):
    return [{"individualMissionID": rand_uint32()} for _ in range(num)]


def _make_completed_individual(num: int):
    return [{"individualMissionID": rand_uint32()} for _ in range(num)]


def _make_individual_status():
    un_count = random.randint(0, 2)
    comp_count = random.randint(0, 2)

    return {
        "aircraftID": rand_int(0, 6),
        "uncompletedIndividualMissionList": _make_uncompleted_individual(un_count),
        "completedIndividualMissionList": _make_completed_individual(comp_count),
        "currentIndividualMission": {"individualMissionID": rand_uint32()},
        "currentPathID": {"pathID": rand_uint32()},
        "lastWaypointID": {"waypointID": rand_uint16()},
        "currentIndividualMissionProgress": round(random.uniform(0, 1), 2),
        "currentBasicAction": {
            "flightMode": random.randint(0, 3),
            "operationMode": random.randint(0, 3),
        },
        "mandatoryCommandType": rand_int(0, 5),
        "priorMissionID": rand_uint32(),
    }


def make_msg0501_body(
    num_individual_status: Union[int, None] = None,
    num_uncompleted_prior: Union[int, None] = None,
    num_completed_prior: Union[int, None] = None,
) -> dict:
    """
    MissionStateInfo(0501) 메시지 바디 생성
    - num_individual_status 미지정 시 1~3개 랜덤
    - num_uncompleted_prior 미지정 시 0~2개 랜덤
    - num_completed_prior 미지정 시 0~2개 랜덤
    """
    if num_individual_status is None:
        num_individual_status = random.randint(1, 3)
    if num_uncompleted_prior is None:
        num_uncompleted_prior = random.randint(0, 2)
    if num_completed_prior is None:
        num_completed_prior = random.randint(0, 2)

    now = int(time.time() * 1000)
    return {
        "timestamp": now,
        "currentMissionPlanID": {"missionPlanID": rand_uint32()},
        "inputMissionProgressStatus": {
            "inputMissionPackageID": rand_uint32(),
            "currentInputMissionID": rand_uint32(),
            "currentInputMissionProgress": random.randint(0, 100),
            "uncompletedInputMissionList": _make_uncompleted_input(
                random.randint(0, 2)
            ),
            "completedInputMissionList": _make_completed_input(random.randint(0, 2)),
        },
        "individualMissionProgressStatusList": [
            _make_individual_status() for _ in range(num_individual_status)
        ],
        "uncompletedPriorMissionIDList": [
            {"priorMissionID": rand_uint32()} for _ in range(num_uncompleted_prior)
        ],
        "completedPriorMissionIDList": [
            {"priorMissionID": rand_uint32()} for _ in range(num_completed_prior)
        ],
    }


def make_msg0802_body() -> dict:
    """
    0802 – MandatoryCommand 랜덤 바디 생성
    • Timestamp     : ulong (8 bytes, ms since 2000-01-01)
    • AircraftID    : uint  (4 bytes)  – 4=무인기1, 5=무인기2, 6=무인기3 중 선택
    • MandatoryType : uint  (4 bytes)  – 1=강제대기, 2=강제귀환, 3=강제임무복귀
    """
    return {
        "timestamp": _now_ms(),
        "aircraftID": random.choice([4, 5, 6]),
        "mandatoryType": random.choice([1, 2, 3]),
    }


def make_msg0803_body() -> dict:
    # 1: Next (다음 협업기저임무 수행)
    # 2: Redo (현재 협업기저임무 재수행)
    return {"timestamp": _now_ms(), "Execute": random.choice([1, 2])}


def make_msg0902_body() -> Dict:
    """0902‑ReplanRequest 메시지 바디(dict) 생성"""
    return {
        "timestamp": _now_ms(),
        "replanRequestTime": {"replanRequestTimestamp": _now_ms()},
        "replanLevel": random.randint(0, 4),
        "inputMissionIDList": [
            {"inputMissionID": rand_uint()} for _ in range(random.randint(1, 3))
        ],
        "individualMissionIDList": [
            {"individualMissionID": rand_uint()} for _ in range(random.randint(1, 3))
        ],
        "priorMissionList": [
            {"priorMissionID": rand_uint()} for _ in range(random.randint(1, 3))
        ],
        "replanReason": rand_str(random.randint(5, 20)),
        "optionList": [
            {
                "optionID": rand_uint(),
                "optionName": rand_str(random.randint(5, 15)),
                "missionPlanID": rand_uint(),
            }
            for _ in range(random.randint(1, 3))
        ],
    }


# ==============================================================================
# 3. 데이터 저장소 (Data Store)
# - 프로그램 시작 시, 정확한 구조의 데이터를 바로 생성하여 초기화합니다.
# ==============================================================================

print("🚀 데이터 저장소를 생성하고 실제 데이터로 즉시 채웁니다...")

csc_data_store = {
    "latest_0101_system_operation_mode": make_msg0101_body(),
    "latest_0202_prior_mission_info": make_msg0202_body(),
    "latest_0401_agent_state": make_msg0401_body(),
    "latest_0402_situation_awareness": make_msg0402_body(),
    "latest_0501_mission_state": make_msg0501_body(),
    "latest_0802_mandatory_command": make_msg0802_body(),
    "latest_0803_mission_pause_command": make_msg0803_body(),
    "latest_0902_replan_request": make_msg0902_body(),
}

print("✅ 데이터 저장소 생성 및 초기화 완료!")

# ==============================================================================
# 4. 데이터 접근 및 업데이트 함수
# ==============================================================================


def update_data_store():
    """데이터 저장소의 모든 데이터를 새로운 랜덤 값으로 새로고침합니다."""
    print("🔄 데이터 저장소를 최신 정보로 업데이트합니다...")
    for key, generator_func in {
        "latest_0101_system_operation_mode": make_msg0101_body,
        "latest_0202_prior_mission_info": make_msg0202_body,
        "latest_0401_agent_state": make_msg0401_body,
        "latest_0402_situation_awareness": make_msg0402_body,
        "latest_0501_mission_state": make_msg0501_body,
        "latest_0802_mandatory_command": make_msg0802_body,
        "latest_0803_mission_pause_command": make_msg0803_body,
        "latest_0902_replan_request": make_msg0902_body,
    }.items():
        csc_data_store[key] = generator_func()
    print("✅ 데이터 업데이트 완료!")


def get_data_store():
    """현재 csc_data_store 상태를 반환합니다."""
    return csc_data_store


# ==============================================================================
# 5. 메인 실행 블록 (Demonstration)
# ==============================================================================

if __name__ == "__main__":
    print("\n" + "=" * 80 + "\n")
    print("--- 1. 프로그램 시작 시 바로 생성된 데이터 (정확한 구조) ---")
    initial_data = get_data_store()

    print("\n" + "=" * 80 + "\n")
    print("--- 2. 'update_data_store()' 호출 후 (데이터 새로고침) ---")
    update_data_store()
    refreshed_data = get_data_store()
