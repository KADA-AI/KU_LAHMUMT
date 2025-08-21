from System.Collections.Generic import List
from nFusion.Model.msg_0401 import *                # LAHStatus, AgentState …
from generator.message0401_generator import make_msg0401_body
import json

# ───────── dict → CLR LAHStatus ─────────
def _dict_to_obj(body_dict: dict) -> AgentStatus:     # ← 반환 타입 명시
    status = AgentStatus()
    status.timestamp = body_dict["timestamp"]

    agents_clr = List[AgentState]()
    for ag in body_dict["agentStateList"]:
        ag_obj = AgentState()
        ag_obj.aircraftID = ag["aircraftID"]
        ag_obj.isUnmanned = ag["isUnmanned"]

        # Coordinate
        coord = Coordinate()
        coord.latitude  = ag["coordinate"]["latitude"]
        coord.longitude = ag["coordinate"]["longitude"]
        coord.altitude  = ag["coordinate"]["altitude"]
        ag_obj.coordinate = coord

        # Velocity
        vel = Velocity()
        vel.speed   = ag["velocity"]["speed"]
        vel.heading = ag["velocity"]["heading"]
        ag_obj.velocity = vel

        ag_obj.fuel   = ag["fuel"]
        ag_obj.health = ag["health"]

        # ── mannedInfo ──
        mi = MannedInfo()
        wp = Weapons()
        wp.type1 = ag["mannedInfo"]["weapons"]["type1"]
        wp.type2 = ag["mannedInfo"]["weapons"]["type2"]
        wp.type3 = ag["mannedInfo"]["weapons"]["type3"]
        mi.weapons = wp

        dl = DatalinkStatus()
        dl.isConnectedToUAV1 = ag["mannedInfo"]["datalinkStatus"]["isConnectedToUAV1"]
        dl.isConnectedToUAV2 = ag["mannedInfo"]["datalinkStatus"]["isConnectedToUAV2"]
        dl.isConnectedToUAV3 = ag["mannedInfo"]["datalinkStatus"]["isConnectedToUAV3"]
        mi.datalinkStatus = dl
        ag_obj.mannedInfo = mi

        # ── unmannedInfo ──
        ui = UnmannedInfo()
        cw = CurrentWaypointID(); cw.waypointID = ag["unmannedInfo"]["currentWaypointID"]["waypointID"]
        ui.currentWaypointID = cw
        ui.flightMode        = ag["unmannedInfo"]["flightMode"]

        # loiterCoordinate
        lc_src = ag["unmannedInfo"].get("loiterCoordinate")
        if lc_src:
            lc = LoiterCoordinate()
            lc.latitude  = lc_src["latitude"]
            lc.longitude = lc_src["longitude"]
            lc.altitude  = lc_src["altitude"]
            ui.loiterCoordinate = lc

        # targetFollowing
        tf_src = ag["unmannedInfo"].get("targetFollowing")
        if tf_src:
            tf = TargetFollowing()
            tf.targetID = tf_src["targetID"]
            ui.targetFollowing = tf

        la = LeaderAircraftID(); la.aircraftID = ag["unmannedInfo"]["leaderAircraftID"]["aircraftID"]
        ui.leaderAircraftID = la

        si_src = ag["unmannedInfo"]["sensorInfo"]
        si = SensorInfo()
        si.operationalMode = si_src["operationalMode"]
        si.sensorType      = si_src["sensorType"]
        si.fov             = si_src["fov"]

        cc = CenterCoordinate()
        cc.latitude  = si_src["centerCoordinate"]["latitude"]
        cc.longitude = si_src["centerCoordinate"]["longitude"]
        cc.altitude  = si_src["centerCoordinate"]["altitude"]
        si.centerCoordinate = cc

        fc_list = List[FootprintCorner]()
        for fc in si_src["footprintCorner"]:
            fc_obj = FootprintCorner()
            fc_obj.latitude  = fc["latitude"]
            fc_obj.longitude = fc["longitude"]
            fc_obj.altitude  = fc["altitude"]
            fc_list.Add(fc_obj)
        si.footprintCorner = fc_list
        ui.sensorInfo      = si

        ui.payloadHealth = ag["unmannedInfo"]["payloadHealth"]
        ui.fuelWarning   = ag["unmannedInfo"]["fuelWarning"]
        ag_obj.unmannedInfo = ui

        agents_clr.Add(ag_obj)

    status.agentStateList = agents_clr
    return status

# ───────── push helpers ─────────
def make_and_push(body_dict: dict, node_messenger):
    msg_obj = _dict_to_obj(body_dict)
    node_messenger.Push(msg_obj)
    log = (f"[0401] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
           f"[0401] PUSH 완료")
    return log.encode()

def make_random_and_push(node_messenger):
    return make_and_push(make_msg0401_body(), node_messenger)
