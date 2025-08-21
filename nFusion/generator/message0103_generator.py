# message0103_generator.py
import random
import json
import time

def make_msg0103_body():
    """SWStatus(0103) 메시지 바디 생성 (소문자 카멜)"""
    return {
        # 0 ~ 2 범위 내 (0: Unknown, 1: 정상, 2: 비정상)
        "timestamp": int(time.time() * 1000),
        "status":    random.randint(0, 2),
        # 0 ~ 3 범위 내 (0: Not used, 1: 초기모드, 2: 대기모드, 3: 운용모드; 4 이상은 Reserved)
        "mode":      random.randint(0, 3)
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0103_body(), ensure_ascii=False, indent=2))
