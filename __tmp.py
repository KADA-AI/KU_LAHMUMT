from modules.monitoring_ver2.logic.monitoring_logic_part import MonitoringLogic

class Dummy:
    def _log(self, *args, **kwargs):
        print("LOG", args)

logic = MonitoringLogic(Dummy())
context = logic._load_mission_plan_context(700000002)
print("aircraft keys", sorted(context["aircraft"].keys()))
print("missions per aircraft", {aid: len(payload["missions"]) for aid, payload in context["aircraft"].items()})
print("first mission id", context["aircraft"][6]["missions"][0]["individualMissionID"])
print("last mission id", context["aircraft"][6]["missions"][-1]["individualMissionID"])
