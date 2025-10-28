from modules.common import db_paths
from pathlib import Path

base = db_paths.get_active_db_root()
print('base', base)
for pid in (800001523, 800001524, 800001525, 800001526, 800001527, 800001528):
    path = db_paths.get_db_subpath("IndividualMissionPlan", f"{pid}.json")
    print(pid, path, path.exists())
