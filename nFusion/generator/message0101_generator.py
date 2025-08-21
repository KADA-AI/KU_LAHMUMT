# message0101_generator.py
import random
import json
import time

def make_msg0101_body():
    """SystemOperationMode(0101) 메시지 바디 생성 (소문자 카멜)"""
    return {
        "timestamp":   int(time.time() * 1000),
        "systemMode":  random.randint(0, 3)
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0101_body(), ensure_ascii=False, indent=2))
