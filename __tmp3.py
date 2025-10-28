from modules.monitoring_ver2.logic.monitoring_logic_part import MonitoringLogic
import inspect

class Dummy:
    def _log(self, *args, **kwargs):
        pass

source = inspect.getsource(MonitoringLogic._load_mission_plan_context)
print(source)
