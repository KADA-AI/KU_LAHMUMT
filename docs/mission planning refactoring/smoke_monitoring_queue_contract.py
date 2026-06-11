from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("KU_ROLE", "mission")


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def cleanup_process_console_state() -> None:
    try:
        from modules.common import process_console

        tee_type = getattr(process_console, "_TeeStream", None)
        for stream_name in ("stdout", "stderr"):
            stream = getattr(sys, stream_name)
            if tee_type is not None and isinstance(stream, tee_type):
                setattr(sys, stream_name, getattr(stream, "_original", stream))
        sinks = list(getattr(process_console, "_ACTIVE_SINKS", {}).values())
        for sink in sinks:
            try:
                sink.close()
            except Exception:
                pass
        try:
            process_console._ACTIVE_SINKS.clear()
        except Exception:
            pass
        time.sleep(0.05)
    except Exception:
        pass


def with_temp_db(callback: Callable[[Path], None]) -> None:
    from modules.common import db_paths

    original_get_db_subpath = db_paths.get_db_subpath
    tmp_root = Path(tempfile.mkdtemp(prefix="mp_monitoring_queue_"))
    try:
        def fake_get_db_subpath(*parts: object) -> Path:
            return tmp_root.joinpath(*(str(part) for part in parts))

        db_paths.get_db_subpath = fake_get_db_subpath
        callback(tmp_root)
    finally:
        cleanup_process_console_state()
        db_paths.get_db_subpath = original_get_db_subpath
        shutil.rmtree(tmp_root, ignore_errors=True)


def queue_settings(
    *,
    delay_ms: int = 0,
    suppress: bool = True,
    release_on_option_info: bool = False,
) -> dict[str, Any]:
    return {
        "replan_queue": {
            "active_timeout_ms": 45000,
            "history_limit": 30,
            "target_dispatch_delay_ms": int(delay_ms),
            "release_on_option_info": bool(release_on_option_info),
            "suppress_active_target_options_on_new_detection": bool(suppress),
        },
        "target_detection": {
            "target_type_priority": [4, 2, 1],
        },
    }


def target_payload(
    *,
    plan_id: int,
    target_id: int,
    target_type: int,
    target_key: str,
    source_plan_id: int = 7000,
    trigger_type: str | None = None,
) -> dict[str, Any]:
    detail: dict[str, Any] = {
        "trigger": "0402",
        "sourceMissionPlanID": int(source_plan_id),
        "currentMissionPlanID": int(source_plan_id),
        "targetID": int(target_id),
        "targetType": int(target_type),
        "targetKey": str(target_key),
        "attackTargetIDs": [int(target_id)],
        "targetBundle": [{"targetID": int(target_id), "targetKey": str(target_key)}],
        "targetBundleCount": 1,
    }
    if trigger_type is not None:
        detail["triggerType"] = str(trigger_type)
    return {
        "source": "MSM",
        "replanLevel": 2,
        "replanRequest": f"target {target_id}",
        "inputMissionIDList": [{"inputMissionID": 11}],
        "pendingOptionList": [{"missionPlanID": int(plan_id), "optionName": f"option-{target_id}"}],
        "missionPlanIDList": [{"missionPlanID": int(plan_id) + 9000}],
        "replanDetail": detail,
    }


def check_plan_id_extraction_priority() -> None:
    from modules.monitoring.logic import replan_queue_manager as queue_module

    payload = {
        "pendingOptionList": [{"missionPlanID": "101"}],
        "optionList": [{"missionPlanID": "202"}],
        "missionPlanIDList": [{"missionPlanID": "303"}],
        "missionPlanID": "404",
    }
    if queue_module._extract_plan_ids(payload) != [101]:
        fail("queue plan id extraction no longer prefers pendingOptionList over optionList")

    payload = {
        "pendingOptionList": [],
        "optionList": [{"missionPlanID": "202"}],
        "missionPlanIDList": [{"missionPlanID": "303"}],
        "missionPlanID": "404",
    }
    if queue_module._extract_plan_ids(payload) != [202]:
        fail("queue plan id extraction no longer falls back from empty pendingOptionList to optionList")

    payload = {
        "optionList": [],
        "missionPlanIDList": [{"missionPlanID": "303"}, "304", {"missionPlanID": "303"}],
        "missionPlanID": "404",
    }
    if queue_module._extract_plan_ids(payload) != [303, 304]:
        fail("queue missionPlanIDList fallback/dedupe changed")

    if queue_module._extract_plan_ids({"missionPlanID": "404"}) != [404]:
        fail("queue root missionPlanID fallback changed")


def check_target_detection_priority_and_delay() -> None:
    from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=800))
    close_payload = target_payload(
        plan_id=600,
        target_id=60,
        target_type=2,
        target_key="T-60",
        trigger_type="attackClosedDestroyed",
    )
    fallback_payload = target_payload(plan_id=601, target_id=61, target_type=9, target_key="T-61")
    rank_two_payload = target_payload(plan_id=602, target_id=62, target_type=2, target_key="T-62")
    rank_one_payload = target_payload(plan_id=603, target_id=63, target_type=4, target_key="T-63")

    ordered = manager._order_payloads_for_source(
        [fallback_payload, close_payload, rank_two_payload, rank_one_payload],
        source="target_detection",
    )
    if ordered != [close_payload, rank_one_payload, rank_two_payload, fallback_payload]:
        fail("target detection source ordering no longer keeps close-first and target-type priority")

    ordered_manual = manager._order_payloads_for_source(
        [fallback_payload, close_payload],
        source="manual",
    )
    if ordered_manual != [fallback_payload, close_payload]:
        fail("non-target source ordering changed")

    result = manager.enqueue(
        payload=target_payload(plan_id=701, target_id=71, target_type=4, target_key="T-71"),
        source_tag="target_detection",
        now_ms=1000,
    )
    if result.get("dispatch") is not None:
        fail("target dispatch delay no longer queues the first target detection")
    queued = result["snapshot"]["queued"]
    if len(queued) != 1 or queued[0]["ready_at_ms"] != 1800:
        fail(f"target dispatch delay ready_at changed: {queued!r}")

    early = manager.poll(now_ms=1799)
    if early.get("dispatch") is not None:
        fail("target dispatch delay promoted before ready_at_ms")
    due = manager.poll(now_ms=1800)
    if not due.get("dispatch") or due["dispatch"]["item"]["queue_id"] != 1:
        fail(f"target dispatch delay did not promote at ready_at_ms: {due!r}")


def check_same_ready_non_target_priority() -> None:
    from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=0))
    active_result = manager.enqueue(
        payload={"missionPlanID": 501, "replanRequest": "active blocker"},
        source_tag="manual",
        now_ms=1100,
    )
    if not active_result.get("dispatch"):
        fail("manual active blocker did not dispatch")
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=1101)

    manager.enqueue(
        payload=target_payload(plan_id=502, target_id=52, target_type=4, target_key="T-52"),
        source_tag="target_detection",
        now_ms=1102,
    )
    manager.enqueue(
        payload={"missionPlanID": 503, "replanRequest": "manual queued"},
        source_tag="manual",
        now_ms=1102,
    )

    completed = manager.handle_signal(
        signal_name="0903",
        payload={"missionPlanID": 501},
        now_ms=1103,
    )
    dispatch = completed.get("dispatch")
    if not dispatch or dispatch["item"]["source_tag"] != "manual" or dispatch["item"]["queue_id"] != 3:
        fail(f"same-ready non-target priority changed: {completed!r}")


def check_queued_target_detection_merge() -> None:
    from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=0))
    manager.enqueue(
        payload={"missionPlanID": 601, "replanRequest": "active blocker"},
        source_tag="manual",
        now_ms=1200,
    )
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=1201)

    first = manager.enqueue(
        payload=target_payload(plan_id=602, target_id=62, target_type=4, target_key="T-62"),
        source_tag="target_detection",
        now_ms=1202,
    )
    if len(first["snapshot"]["queued"]) != 1:
        fail(f"first queued target detection shape changed: {first!r}")

    second = manager.enqueue(
        payload=target_payload(plan_id=603, target_id=63, target_type=2, target_key="T-63"),
        source_tag="target_detection",
        now_ms=1203,
    )
    events = second.get("events") or []
    if not events or events[0].get("type") != "merged":
        fail(f"queued target detection no longer merges into existing queued item: {events!r}")
    queued = second["snapshot"]["queued"]
    if len(queued) != 1:
        fail(f"queued target detection merge no longer keeps one queued item: {queued!r}")
    survivor = queued[0]
    if survivor["queue_id"] != 2 or survivor["target_id"] != 63 or survivor["plan_ids"] != [603]:
        fail(f"queued target detection survivor no longer carries latest payload: {survivor!r}")
    if survivor["suppress_options"]:
        fail(f"merged queued target detection should reset suppress_options: {survivor!r}")


def check_attack_close_waits_for_active_target_decision() -> None:
    from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=0))
    manager.enqueue(
        payload=target_payload(plan_id=701, target_id=71, target_type=4, target_key="T-71"),
        source_tag="target_detection",
        now_ms=1300,
    )
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=1301)
    manager.handle_signal(
        signal_name="0305",
        payload={"missionPlanningStatus": 2},
        now_ms=1302,
    )
    manager.handle_signal(
        signal_name="0701",
        payload={"missionPlanIDList": [{"missionPlanID": 701}]},
        now_ms=1303,
    )

    close_result = manager.enqueue(
        payload={
            "source": "MSM",
            "replanLevel": 2,
            "replanRequest": "target 71 destroyed",
            "inputMissionIDList": [{"inputMissionID": 11}],
            "missionPlanIDList": [{"missionPlanID": 702}],
            "replanDetail": {
                "trigger": "0402",
                "triggerType": "attackClosedDestroyed",
                "closureReason": "destroyed",
                "sourceMissionPlanID": 7000,
                "currentMissionPlanID": 7000,
                "missionPlanID": 702,
                "targetID": 71,
                "targetType": 4,
                "targetKey": "T-71",
            },
        },
        source_tag="target_detection",
        now_ms=1304,
    )
    if close_result.get("dispatch") is not None:
        fail(f"attack-close should wait behind active option decision: {close_result!r}")
    active = close_result["snapshot"]["active"]
    queued = close_result["snapshot"]["queued"]
    if not active or active["queue_id"] != 1 or active["stage"] != "options_sent":
        fail(f"active option item changed before 0702: {active!r}")
    if len(queued) != 1 or queued[0]["queue_id"] != 2:
        fail(f"attack-close was not queued behind active option item: {queued!r}")
    accepted, validation_logs = manager.validate_0702_decision(mission_plan_id=702, ignore_value=2)
    if accepted or not validation_logs:
        fail("out-of-order 0702 for queued attack-close plan was not rejected")

    rejected_0903 = manager.handle_signal(
        signal_name="0903",
        payload={"missionPlanID": 701},
        now_ms=1305,
    )
    if rejected_0903.get("dispatch") is not None or rejected_0903["snapshot"]["active"]["queue_id"] != 1:
        fail(f"option item should not complete on 0903 before 0702: {rejected_0903!r}")

    completed = manager.handle_signal(
        signal_name="0702",
        payload={"ignore": 2, "missionPlanID": 701},
        now_ms=1306,
    )
    history = completed["snapshot"]["history"]
    if not history or history[0]["status"] != "completed" or history[0]["completion_signal"] != "0702":
        fail(f"active option item did not complete on 0702: {history!r}")
    dispatch = completed.get("dispatch")
    dispatch_detail = (dispatch.get("payload") or {}).get("replanDetail") if dispatch else None
    if (
        not dispatch
        or not isinstance(dispatch_detail, dict)
        or dispatch_detail.get("triggerType") != "attackClosedDestroyed"
    ):
        fail(f"attack-close did not dispatch after active 0702: {dispatch!r}")


def check_suppress_flag_lifecycle(tmp_root: Path) -> None:
    from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=0, suppress=True))
    first = target_payload(plan_id=801, target_id=81, target_type=4, target_key="T-81")
    second = target_payload(plan_id=802, target_id=82, target_type=2, target_key="T-82")

    first_result = manager.enqueue(payload=first, source_tag="target_detection", now_ms=2000)
    if not first_result.get("dispatch") or first_result["dispatch"]["item"]["queue_id"] != 1:
        fail("first target detection no longer dispatches immediately when delay is zero")
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=2001)

    second_result = manager.enqueue(payload=second, source_tag="target_detection", now_ms=2002)
    event_types = [event.get("type") for event in second_result.get("events", [])]
    if "suppress_options_set" not in event_types:
        fail(f"new target detection no longer sets active suppress flag: {event_types!r}")
    active = second_result["snapshot"]["active"]
    if not active or active["queue_id"] != 1 or not active["suppress_options"]:
        fail(f"active target suppress_options state changed: {active!r}")

    flag_path = tmp_root / "DSS_Internal" / "suppress_option_request.json"
    if not flag_path.exists():
        fail("suppress option flag file was not written")
    flag = json.loads(flag_path.read_text(encoding="utf-8"))
    expected = {
        "active": True,
        "target_id": 81,
        "target_key": "T-81",
        "queue_id": 1,
        "plan_ids": [801],
        "created_ms": 2002,
    }
    for key, value in expected.items():
        if flag.get(key) != value:
            fail(f"suppress flag {key} changed: {flag!r}")

    completed = manager.handle_signal(
        signal_name="0903",
        payload={"missionPlanID": 801},
        now_ms=2003,
    )
    if flag_path.exists():
        fail("completed suppressed active item no longer clears suppress flag")
    if not completed.get("dispatch") or completed["dispatch"]["item"]["queue_id"] != 2:
        fail(f"completion no longer promotes queued target detection: {completed!r}")


def check_suppress_not_set_after_options_sent(tmp_root: Path) -> None:
    from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=0, suppress=True))
    manager.enqueue(
        payload=target_payload(plan_id=901, target_id=91, target_type=4, target_key="T-91"),
        source_tag="target_detection",
        now_ms=3000,
    )
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=3001)
    manager.handle_signal(
        signal_name="0701",
        payload={"missionPlanIDList": [{"missionPlanID": 901}]},
        now_ms=3002,
    )
    result = manager.enqueue(
        payload=target_payload(plan_id=902, target_id=92, target_type=2, target_key="T-92"),
        source_tag="target_detection",
        now_ms=3003,
    )
    event_types = [event.get("type") for event in result.get("events", [])]
    if "suppress_options_set" in event_types:
        fail("options-sent active target detection is now suppressible")
    if (tmp_root / "DSS_Internal" / "suppress_option_request.json").exists():
        fail("options-sent active target detection wrote a suppress flag")
    active = result["snapshot"]["active"]
    if not active or active["stage"] != "options_sent" or not active["options_delivered"]:
        fail(f"0701 active stage/options_delivered contract changed: {active!r}")
    if active["suppress_options"]:
        fail(f"0701 active item should not be marked suppress_options: {active!r}")


def check_0803_option_decision_blocker() -> None:
    from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager

    option_payload = {
        "source": "MSM",
        "replanLevel": 1,
        "replanRequest": "협업기저임무 재수행 요청",
        "inputMissionIDList": [{"inputMissionID": 11}],
        "pendingOptionList": [
            {"missionPlanID": 1101, "optionName": "1안"},
            {"missionPlanID": 1102, "optionName": "2안"},
            {"missionPlanID": 1103, "optionName": "3안"},
        ],
        "replanDetail": {
            "trigger": "0201",
            "triggerType": "collabReexecuteInputRefresh",
            "currentInputMissionID": 11,
        },
    }
    force_direct_payload = {
        "source": "MSM",
        "replanLevel": 3,
        "replanRequest": "_협업 기저 전환으로 인한 재계획",
        "inputMissionIDList": [{"inputMissionID": 7}],
        "pendingOptionList": [{"missionPlanID": 1201, "optionName": "직접갱신"}],
        "replanDetail": {
            "trigger": "0803",
            "triggerType": "nextCollaborativeMission",
            "forceDirectUpdate": True,
            "suppress0702Fallback": True,
        },
    }

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=0))
    active_result = manager.enqueue(
        payload={"missionPlanID": 1001, "replanRequest": "active non-option"},
        source_tag="manual",
        now_ms=4000,
    )
    if not active_result.get("dispatch"):
        fail("manual active blocker did not dispatch for 0803 option blocker test")
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=4001)
    manager.enqueue(payload=option_payload, source_tag="reexecute_0201", now_ms=4002)
    blocker = manager.find_0803_option_decision_blocker()
    if not blocker or blocker.get("queue_id") != 2:
        fail(f"0803 blocker did not find queued option decision item: {blocker!r}")

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=0))
    manager.enqueue(payload=option_payload, source_tag="reexecute_0201", now_ms=5000)
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=5001)
    blocker = manager.find_0803_option_decision_blocker()
    if not blocker or blocker.get("queue_id") != 1:
        fail(f"0803 blocker did not find active option decision item: {blocker!r}")
    result_0903 = manager.handle_signal(
        signal_name="0903",
        payload={"missionPlanID": 1101},
        now_ms=5002,
    )
    if result_0903["snapshot"].get("active") is None:
        fail("option decision item completed on 0903 before 0702 decision")
    result_0702 = manager.handle_signal(
        signal_name="0702",
        payload={"missionPlanID": 1101, "ignore": 2},
        now_ms=5003,
    )
    if result_0702["snapshot"].get("active") is not None:
        fail("option decision item did not complete on 0702 decision")

    manager = ReplanQueueManager(settings_getter=lambda: queue_settings(delay_ms=0))
    manager.enqueue(payload=force_direct_payload, source_tag="next_collab", now_ms=6000)
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=6001)
    if manager.find_0803_option_decision_blocker() is not None:
        fail("0803 blocker should ignore force-direct next-collab updates")


def make_window(gui: Any) -> Any:
    window = gui.MainWindow.__new__(gui.MainWindow)
    window._logs: list[str] = []
    window._append_log_line = lambda text: window._logs.append(str(text))
    return window


def check_gui_source_plan_rebound_and_suppress_matching() -> None:
    import importlib

    gui = importlib.import_module("modules.mission_planning.mission_planning_gui")
    window = make_window(gui)

    window._last_mission_plan_id = 1001
    window._active_plan_context = {"_option_meta": {1001: {"attack": True}}}
    if window._get_current_attack_source_plan_id() != 1001:
        fail("attack option source-plan rebound no longer returns last mission plan id")

    window._active_plan_context = {
        "_option_meta": {
            1001: {
                "attack": False,
                "postAttackRejoinContext": {
                    "sourcePlanID": "7001",
                    "sourceMissionPlanID": "7002",
                    "currentMissionPlanID": "7003",
                },
            }
        }
    }
    if window._get_current_attack_source_plan_id() != 7001:
        fail("post-attack source-plan rebound key priority changed")

    window._active_plan_context = {
        "_option_meta": {
            1001: {
                "attack": False,
                "postAttackRejoinContext": {"currentMissionPlanID": "7003"},
            }
        }
    }
    if window._get_current_attack_source_plan_id() != 7003:
        fail("post-attack currentMissionPlanID fallback changed")

    window._active_plan_context = {"_option_meta": {1001: {"attack": True}}}
    window._build_follow_up_attack_target_bundle = (
        lambda _detail, limit=3: [{"targetID": 77, "targetKey": "T-77"}]
    )
    detail = {"trigger": "0402"}
    if not window._prepare_follow_up_attack_detail(detail):
        fail("follow-up attack detail is no longer prepared from current source plan")
    if detail.get("sourceMissionPlanID") != 1001 or detail.get("currentMissionPlanID") != 1001:
        fail(f"follow-up attack detail source/current plan changed: {detail!r}")
    if not detail.get("followUpAttackMode") or detail.get("targetBundleMode") != "follow_up":
        fail(f"follow-up attack detail mode fields changed: {detail!r}")

    active_ctx = {
        "plan_ids": [801],
        "replan_detail": {"trigger": "0402", "targetID": 81, "targetKey": "T-81"},
    }
    matching_flag = {"plan_ids": [801], "target_id": 81, "target_key": "T-81"}
    if not window._suppress_flag_matches_active_context(matching_flag, active_ctx, phase="smoke"):
        fail("matching suppress flag no longer matches active context")

    if window._suppress_flag_matches_active_context(
        {"plan_ids": [999], "target_id": 81, "target_key": "T-81"},
        active_ctx,
        phase="smoke",
    ):
        fail("stale suppress flag with disjoint plan ids now matches")
    if window._suppress_flag_matches_active_context(
        {"plan_ids": [801], "target_id": 99, "target_key": "T-81"},
        active_ctx,
        phase="smoke",
    ):
        fail("stale suppress flag with mismatched target id now matches")
    if window._suppress_flag_matches_active_context(
        {"plan_ids": [801], "target_id": 81, "target_key": "T-XX"},
        active_ctx,
        phase="smoke",
    ):
        fail("stale suppress flag with mismatched target key now matches")

    stale_logs = "\n".join(window._logs)
    for fragment in ("flagPlans", "flagTarget", "flagKey"):
        if fragment not in stale_logs:
            fail(f"stale suppress log fragment missing: {fragment}")


def check_monitoring_gui_dispatch_source_rebound() -> None:
    import importlib

    monitoring_gui = importlib.import_module("modules.monitoring.monitoring_gui")
    window = monitoring_gui.MainWindow.__new__(monitoring_gui.MainWindow)
    window._current_mission_plan_id = 9101
    window._current_plan_id_from_viz = lambda: 9102
    window._append_log_line = lambda text: window._logs.append(str(text))
    window._logs = []
    window._replan_option_meta_by_plan_id = {}

    original_load_target_info = monitoring_gui.load_target_info
    original_build_target_bundle = monitoring_gui.build_target_bundle_from_target_info
    try:
        monitoring_gui.load_target_info = lambda: {}
        monitoring_gui.build_target_bundle_from_target_info = lambda *_args, **_kwargs: []

        payload_0402 = target_payload(
            plan_id=9111,
            target_id=111,
            target_type=4,
            target_key="T-111",
            source_plan_id=9000,
        )
        prepared_0402, bundle_ids = window._prepare_0902_payload_for_dispatch(payload_0402)
        detail_0402 = prepared_0402["replanDetail"]
        if bundle_ids:
            fail(f"non-follow-up 0402 dispatch unexpectedly returned bundle ids: {bundle_ids!r}")
        if (
            prepared_0402.get("sourceMissionPlanID") != 9101
            or prepared_0402.get("currentMissionPlanID") != 9101
            or detail_0402.get("sourceMissionPlanID") != 9101
            or detail_0402.get("currentMissionPlanID") != 9101
        ):
            fail(f"monitoring 0402 dispatch source/current rebound changed: {prepared_0402!r}")
        if "9000 -> 9101" not in "\n".join(window._logs):
            fail(f"monitoring 0402 source rebound log changed: {window._logs!r}")

        window._current_mission_plan_id = None
        payload_0401 = {
            "pendingOptionList": [{"missionPlanID": 9121}],
            "replanDetail": {"trigger": "0401", "sourceMissionPlanID": 9001},
        }
        prepared_0401, _ = window._prepare_0902_payload_for_dispatch(payload_0401)
        detail_0401 = prepared_0401["replanDetail"]
        if (
            prepared_0401.get("sourceMissionPlanID") != 9102
            or prepared_0401.get("currentMissionPlanID") != 9102
            or detail_0401.get("sourceMissionPlanID") != 9102
            or detail_0401.get("currentMissionPlanID") != 9102
        ):
            fail(f"monitoring 0401 dispatch viz fallback rebound changed: {prepared_0401!r}")
    finally:
        monitoring_gui.load_target_info = original_load_target_info
        monitoring_gui.build_target_bundle_from_target_info = original_build_target_bundle


def check_monitoring_source_plan_payload_contract() -> None:
    target_source = (
        PROJECT_ROOT / "modules" / "monitoring" / "logic" / "target_detection_replan.py"
    ).read_text(encoding="utf-8-sig", errors="ignore")
    rtb_source = (
        PROJECT_ROOT / "modules" / "monitoring" / "logic" / "rtb_replan.py"
    ).read_text(encoding="utf-8-sig", errors="ignore")

    required_target_fragments = [
        '"sourceMissionPlanID": int(current_mission_plan_id),',
        '"currentMissionPlanID": int(current_mission_plan_id),',
        '"sourceMissionPlanID": int(current_mission_plan_id) if current_mission_plan_id is not None else None,',
        '"currentMissionPlanID": int(current_mission_plan_id) if current_mission_plan_id is not None else None,',
    ]
    for fragment in required_target_fragments:
        if fragment not in target_source:
            fail(f"target detection source/current plan assignment changed: {fragment}")

    required_rtb_fragments = [
        '"sourceMissionPlanID": int(current_mission_plan_id) if current_mission_plan_id is not None else None,',
        '"currentMissionPlanID": int(current_mission_plan_id) if current_mission_plan_id is not None else None,',
    ]
    for fragment in required_rtb_fragments:
        if fragment not in rtb_source:
            fail(f"RTB source/current plan assignment changed: {fragment}")


def main() -> int:
    try:
        check_plan_id_extraction_priority()
        check_target_detection_priority_and_delay()
        check_same_ready_non_target_priority()
        check_queued_target_detection_merge()
        check_attack_close_waits_for_active_target_decision()
        with_temp_db(check_suppress_flag_lifecycle)
        with_temp_db(check_suppress_not_set_after_options_sent)
        check_0803_option_decision_blocker()
        check_gui_source_plan_rebound_and_suppress_matching()
        check_monitoring_gui_dispatch_source_rebound()
        check_monitoring_source_plan_payload_contract()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        cleanup_process_console_state()

    print("monitoring queue contract smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
