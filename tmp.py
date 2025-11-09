from pathlib import Path
path = Path('modules/mission_planning/MissionPlanner/main_MP.py')
lines = path.read_text(encoding='utf-8').splitlines()
start = None
end = None
for i,line in enumerate(lines):
    if line.strip() == 'def _write_map_html(self):':
        start = i
    elif start is not None and line.strip().startswith('# '): # invalid
