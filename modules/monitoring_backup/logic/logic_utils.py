import datetime

def log_to_file(message):
    """디버그 메시지를 파일에 기록합니다."""
    with open("replan_debug.log", "a", encoding="utf-8") as f:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        f.write(f"[{timestamp}] {message}\n")

