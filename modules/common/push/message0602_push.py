from System.Collections.Generic import List 
from nFusion.Model.msg_0602 import *  # msg_0701 타입 import
from generator.message0602_generator import make_msg0602_body  # 메시지 바디 생성기
import json 




def _dict_to_obj(body: dict):
    """
    dict(JSON, 소문자 키) → UAVControl(C# 객체)
    """
    uav_ctrl = UAVCommand()
    uav_ctrl.timestamp            = body["timestamp"]
    uav_ctrl.UavCommandModeType   = body["uavCommandModeType"]
    uav_ctrl.aircraftID           = body["aircraftID"]

    # ── FlightModeCommand ─────────────────────────────
    fmc_body = body["flightModeCommand"]
    fmc = FlightModeCommand()
    fmc.flightMode = fmc_body["flightMode"]

    # PathFollowing
    pf_body = fmc_body["pathFollowing"]
    pf = PathFollowing()
    pf.pathID         = pf_body["pathID"]
    pf.startWaypointID = pf_body["startWaypointID"]
    fmc.pathFollowing = pf

    # TargetTracking
    tt_body = fmc_body["targetTracking"]
    tt = TargetTracking()
    tt.targetID = tt_body["targetID"]
    fmc.targetTracking = tt

    # LoiterProperty
    lp_body = fmc_body["loiterProperty"]
    lp = LoiterProperty()
    coord_lp = lp_body["coordinate"]
    c_lp = Coordinate()
    c_lp.latitude  = coord_lp["latitude"]
    c_lp.longitude = coord_lp["longitude"]
    c_lp.altitude  = coord_lp["altitude"]
    lp.coordinate  = c_lp
    lp.loiterTime      = lp_body["loiterTime"]
    lp.loiterRadius    = lp_body["loiterRadius"]
    lp.loiterDirection = lp_body["loiterDirection"]
    lp.loiterSpeed     = lp_body["loiterSpeed"]
    fmc.loiterProperty = lp

    # FormationProperty
    fp_body = fmc_body["formationProperty"]
    fp = FormationProperty()
    fp.leaderAircraftID = fp_body["leaderAircraftID"]
    form_body = fp_body["formation"]
    form = Formation()
    form.dX = form_body["dX"]
    form.dY = form_body["dY"]
    form.dZ = form_body["dZ"]
    fp.formation = form
    fmc.formationProperty = fp

    uav_ctrl.flightModeCommand = fmc

    # ── FilmingModeCommand ─────────────────────────────
    fmc2_body = body["filmingModeCommand"]
    fmc2 = FilmingModeCommand()
    fmc2.operationMode       = fmc2_body["operationMode"]
    fmc2.sensorType          = fmc2_body["sensorType"]
    fmc2.fieldOfView         = fmc2_body["fieldOfView"]

    # CoordinateOrientation
    co_body = fmc2_body["coordinateOrientation"]["coordinate"]
    coord_co = Coordinate()
    coord_co.latitude  = co_body["latitude"]
    coord_co.longitude = co_body["longitude"]
    coord_co.altitude  = co_body["altitude"]
    co = CoordinateOrientation()
    co.coordinate = coord_co
    fmc2.coordinateOrientation = co

    # LineSearch
    ls_body = fmc2_body["lineSearch"]
    ls = LineSearch()
    # coordinateList
    coord_list = List[Coordinate]()
    for pt in ls_body["coordinateList"]:
        c = Coordinate()
        c.latitude  = pt["latitude"]
        c.longitude = pt["longitude"]
        c.altitude  = pt["altitude"]
        coord_list.Add(c)
    ls.coordinateList = coord_list
    ls.searchSpeed = ls_body["searchSpeed"]
    fmc2.lineSearch = ls

    # AutoTracking
    at_body = fmc2_body["autoTracking"]
    at = AutoTracking()
    at.targetID = at_body["targetID"]
    fmc2.autoTracking = at

    # AircraftFixed
    af_body = fmc2_body["aircraftFixed"]
    af = AircraftFixed()
    af.gimbalPitch = af_body["gimbalPitch"]
    af.gimbalYaw   = af_body["gimbalYaw"]
    fmc2.aircraftFixed = af

    # AutoScan
    asc_body = fmc2_body["autoScan"]
    asc = AutoScan()
    asc.gimbalPitch           = asc_body["gimbalPitch"]
    # GimbalYawLimits
    gl_body = asc_body["gimbalYawLimits"]
    gl = GimbalYawLimits()
    gl.leftLimit  = gl_body["leftLimit"]
    gl.rightLimit = gl_body["rightLimit"]
    asc.gimbalYawLimits = gl
    asc.gimbalYawAngularSpeed = asc_body["gimbalYawAngularSpeed"]
    fmc2.autoScan = asc

    uav_ctrl.filmingModeCommand = fmc2
    return uav_ctrl

def make_and_push(body_dict: dict, node_messenger) -> None:
    message = _dict_to_obj(body_dict)
    print(message)
    node_messenger.Push(message)  # 메시지 전송

    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0602] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0602] PUSH 완료"
    )
    #print(log_line)
    return log_line.encode()                 # ← push_center → _mark_sent 로 전달

def make_random_and_push(node_messenger) -> None:
    body_dict = make_msg0602_body()  # 랜덤 메시지 바디 생성
    return make_and_push(body_dict, node_messenger)  # 메시지 전송
