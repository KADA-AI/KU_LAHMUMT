from pathlib import Path

path = Path("modules/monitoring_ver2/logic/monitoring_logic_part.py")
with path.open("r", encoding="utf-8") as f:
    lines = list(f)
start = 220
end = 280
for idx in range(start, end):
    print(f"{idx+1}: {lines[idx].rstrip()}")
