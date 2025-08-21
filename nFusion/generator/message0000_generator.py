# message0000_generator.py
import random
import json
import time

def make_msg0000_body(request_module_name: str):
    """Response(0000) 메시지 바디 생성 (소문자 카멜)"""
    return {
        # 0 ~ 2^64-1 범위 내(현재 ms 단위로 생성)
        "timestamp": int(time.time() * 1000),
        # 0 ~ 2^16-1 범위 내
        "requestModuleName": request_module_name[:4],
        # 0 ~ 2 범위 내 (0: None, 1: 정상 수신, 2: 비정상 수신)
        "messageID": random.randint(0, 2)
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0000_body(), ensure_ascii=False, indent=2))
