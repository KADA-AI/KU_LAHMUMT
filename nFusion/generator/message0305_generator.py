# message0305_generator.py

import random
import json
import time

def make_msg0305_body():
    """ReplanStatus(0305) 메시지용 랜덤 바디 생성"""
    reasons = [
        "WeatherChange",
        "NoFlyZoneDetected",
        "HighPriorityMission",
        "FuelLow",
        "ObstacleDetected"
    ]
    return {
        "timestamp":              int(time.time() * 1000),
        "missionPlanningStatus":  random.randint(0, 2),
        "replanReason":           random.choice(reasons)
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0305_body(), ensure_ascii=False, indent=2))
