# generator/message0401_generator.py
# ─────────────────────────────────────────────────────────────
import random, time, json

# ────────── Helper ──────────
rand_float8 = lambda lo, hi: round(random.uniform(lo, hi), 8)
rand_int32  = lambda lo=0, hi=50000: random.randint(lo, hi)
rand_uint32 = lambda lo=0, hi=(2**32 - 1): random.randint(lo, hi)

def _coord() -> dict:
    return {
        "latitude":  rand_float8(-90, 90),
        "longitude": rand_float8(-180, 180),
        "altitude":  rand_int32(0, 50000)
    }


def _make_agent_state(agent_id: int) -> dict:
    return {
        # ───────── 제약 반영 ─────────
        "aircraftID":  agent_id,                    # 0-6
        "isUnmanned":  bool(random.getrandbits(1)),
        "coordinate":  _coord(),
        "velocity":    {
            "speed":   round(random.uniform(0, 250), 1),
            "heading": round(random.uniform(0, 360), 1)
        },
        "fuel":        round(random.uniform(0, 100), 1),
        "health":      random.randint(0, 2),        # 0,1,2
        "mannedInfo":  {
            "weapons": {
                "type1": random.randint(0, 10),
                "type2": random.randint(0, 10),
                "type3": random.randint(0, 10)
            },
            "datalinkStatus": {
                "isConnectedToUAV1": bool(random.getrandbits(1)),
                "isConnectedToUAV2": bool(random.getrandbits(1)),
                "isConnectedToUAV3": bool(random.getrandbits(1))
            }
        },
        "unmannedInfo": {
            "currentWaypointID": { "waypointID": random.randint(1, 9999) },
            "flightMode":        random.randint(0, 9),   # 0-9 (기존 그대로)
            "loiterCoordinate": _coord(),
            "targetFollowing": {"targetID": rand_uint32() } ,
            "leaderAircraftID":  { "aircraftID": random.randint(0, 6) },
            "sensorInfo": {
                "operationalMode": random.randint(0, 3),
                "sensorType":      random.randint(0, 3),  # 0-3
                "fov":             round(random.uniform(10, 120), 1),
                "centerCoordinate": _coord(),
                "footprintCorner": [ _coord() for _ in range(4) ]
            },
            "payloadHealth": random.randint(0, 3),       # 0-3
            "fuelWarning":   random.randint(0, 3)        # 0-3
        }
    }

# ────────── Main ──────────
def make_msg0401_body(num_agents: int | None = None) -> dict:
    if num_agents is None:
        num_agents = random.randint(1, 6)

    now_ms = int(time.time() * 1000)
    return {
        "timestamp":      now_ms,
        "agentStateList": [_make_agent_state(random.randint(0, 6))
                           for _ in range(num_agents)]
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0401_body(), ensure_ascii=False, indent=2))
