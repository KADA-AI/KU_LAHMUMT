from pathlib import Path

lines = Path('modules/monitoring_ver2/logic/monitoring_logic_part.py').read_text(encoding='utf-8').splitlines()
for idx, line in enumerate(lines, 1):
    if '_notify_input_mission_completed' in line or '_send_0503_notification' in line:
        print(idx, line.strip())
