from modules.monitoring_ver2.logic.monitoring_logic_part import MonitoringLogic

class Dummy:
    def __init__(self):
        self.receive_store = type("RS", (), {"get_data": lambda *_: None})()
        self.logic_store = type("LS", (), {"get_data": lambda *_: None, "set_data": lambda *args, **kwargs: None})()
        self.push_store = type("PS", (), {"add_data": lambda *args, **kwargs: None})()
        self.node_messenger = None
        self.gui_update_callback = None
        def _log(*args, **kwargs):
            pass
        self._log = _log

logic = MonitoringLogic(Dummy())
context = logic._load_mission_plan_context(700000002)
logic._plan_context = context
logic._initialize_input_tracker(context)
marked = []
logic._mark_individual_mission_done = lambda key: marked.append(key)
mission = context["aircraft"][6]["missions"][0]
status = [{
    "aircraftID": 6,
    "individualMissionID": mission["individualMissionID"],
    "pathID": mission["pathID"],
    "inputMissionID": mission["inputMissionID"],
    "progress": 100,
}]
logic._update_input_mission_progress(status)
print(marked)
