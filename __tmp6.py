from modules.monitoring_ver2.logic.monitoring_logic_part import MonitoringLogic

class Dummy:
    def _log(self, *args, **kwargs):
        pass

logic = MonitoringLogic(Dummy())
context = logic._load_mission_plan_context(700000002)
logic._plan_context = context
logic._mission_file_map = {}
logic._mission_to_input = {}
logic._input_mission_tracker = {}
print("loaded")
