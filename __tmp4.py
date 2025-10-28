from modules.monitoring_ver2.logic.monitoring_logic_part import MonitoringLogic
import inspect

source = inspect.getsource(MonitoringLogic._load_mission_plan_context)
for line in source.splitlines():
    indent = len(line) - len(line.lstrip(" "))
    print(f"{indent:02d}: {line}")
