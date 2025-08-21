# receive/message0401_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *             # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0401 import *               # LAHStatus, AgentState …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 공통 get 유틸 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _lah_status_to_dict(status: AgentStatus) -> dict:
    def coord2d(c: Coordinate):
        return {
            "latitude":  _get(c, "latitude",  "Latitude"),
            "longitude": _get(c, "longitude", "Longitude"),
            "altitude":  _get(c, "altitude",  "Altitude")
        }

    body = {
        "timestamp":      _get(status, "timestamp",           "Timestamp"),
        "agentStateList": []
    }

    for ag in (_get(status, "agentStateList", "AgentStateList") or []):
        vel   = _get(ag, "velocity",   "Velocity")
        coord = _get(ag, "coordinate", "Coordinate")

        ag_dict = {
            "aircraftID":  _get(ag, "aircraftID",  "AircraftID"),
            "isUnmanned":  _get(ag, "isUnmanned",  "IsUnmanned"),
            "coordinate":  coord2d(coord or Coordinate()),
            "velocity":    {
                "speed":   _get(vel, "speed",   "Speed"),
                "heading": _get(vel, "heading", "Heading")
            },
            "fuel":   _get(ag, "fuel",   "Fuel"),
            "health": _get(ag, "health", "Health")
        }

        # ---------- mannedInfo ----------
        mi = _get(ag, "mannedInfo", "MannedInfo")
        if mi:
            wep = _get(mi, "weapons",         "Weapons")
            dls = _get(mi, "datalinkStatus",  "DatalinkStatus")
            ag_dict["mannedInfo"] = {
                "weapons": {
                    "type1": _get(wep, "type1", "Type1"),
                    "type2": _get(wep, "type2", "Type2"),
                    "type3": _get(wep, "type3", "Type3")
                },
                "datalinkStatus": {
                    "isConnectedToUAV1": _get(dls, "isConnectedToUAV1", "IsConnectedToUAV1"),
                    "isConnectedToUAV2": _get(dls, "isConnectedToUAV2", "IsConnectedToUAV2"),
                    "isConnectedToUAV3": _get(dls, "isConnectedToUAV3", "IsConnectedToUAV3")
                }
            }

        # ---------- unmannedInfo ----------
        ui = _get(ag, "unmannedInfo", "UnmannedInfo")
        if ui:
            cw = _get(ui, "currentWaypointID", "CurrentWaypointID")
            la = _get(ui, "leaderAircraftID",    "LeaderAircraftID")
            si = _get(ui, "sensorInfo",          "SensorInfo")
            tf = _get(ui, "targetFollowing", "TargetFollowing")
            lc = _get(ui, "loiterCoordinate", "LoiterCoordinate")

            ui_dict = {
                "currentWaypointID": { "waypointID": _get(cw, "waypointID", "WaypointID") } if cw else {},
                "flightMode":        _get(ui, "flightMode",        "FlightMode"),
                "targetFollowing": {"targetID": _get(tf, "targetID", "TargetID")} if tf else {},
                "loiterCoordinate": coord2d(lc or Coordinate()),
                "leaderAircraftID":  { "aircraftID": _get(la, "aircraftID", "AircraftID") } if la else {},
                "payloadHealth":     _get(ui, "payloadHealth",     "PayloadHealth"),
                "fuelWarning":       _get(ui, "fuelWarning",       "FuelWarning")
            }

            # ---- sensorInfo ----
            if si:
                cen = _get(si, "centerCoordinate", "CenterCoordinate")
                fp  = _get(si, "footprintCorner",  "FootprintCorner")

                ui_dict["sensorInfo"] = {
                    "operationalMode": _get(si, "operationalMode", "OperationalMode"),
                    "sensorType":      _get(si, "sensorType",      "SensorType"),
                    "fov":             _get(si, "fov",             "Fov"),
                    "centerCoordinate": coord2d(cen or Coordinate()),
                    "footprintCorner":  [coord2d(p) for p in (fp or [])]
                }

            ag_dict["unmannedInfo"] = ui_dict

        body["agentStateList"].append(ag_dict)

    return body

# ────────── Receiver 클래스 ──────────
class LAHStatusReceiver_0401(
    IFusionReceive[AgentStatus], IsLocal, IsSingletone
):
    """0401 LAHStatus 메시지 수신 리시버"""
    __namespace__ = "LAHStatusReceiver_0401"

    def Receive(self, data: AgentStatus, src):
        try:
            # DB 저장
            received_db.set_received_0401(data)

            # GUI 알림
            notify(
                "0401",
                json.dumps(_lah_status_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0401] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
