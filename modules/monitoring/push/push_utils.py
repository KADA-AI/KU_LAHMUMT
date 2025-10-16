# push/push_utils.py
import os
import importlib
from datetime import datetime, timezone
from pathlib import Path

# ── Constants ──────────────────────────────────────────────────────────────
EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

# ── Embedded TX/DB rules ───────────────────────────────────────────────────
TX_FIELD_WHITELIST = {
    "0201": ["timestamp", "inputMissionPackageID"],
    "0203": ["timestamp", "missionReferencePackageID"],
    "0301": ["timestamp", "missionPlanID"],
    "0302": ["timestamp", "individualMissionPackageID"],
    "0303": ["timestamp", "pathID"],
    "0304": ["timestamp", "pathID"],
}

DB_DIR_RULES = {
    "0201": "InputMissionPlan",
    "0203": "FlightReferenceInfo",
    "0301": "MissionPlan",
    "0302": "IndividualMissionPlan",
    "0303": "FlightPath",
    "0304": "FlightPath",
}

# ── Helper Functions ───────────────────────────────────────────────────────

now_ms = lambda: int(
    (
        datetime.now(timezone.utc).replace(tzinfo=timezone.utc) - EPOCH_2000
    ).total_seconds()
    * 1000
)


def try_set(obj, name: str, value) -> bool:
    """Set attribute on a C# object, trying both lowerCamel and PascalCase."""
    for k in (name, name[:1].upper() + name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False


def cs_new(name: str, msg_id: str):
    """Create a new instance of a C# type by searching in relevant namespaces."""
    t = None
    # Search order: global -> msg_ID module -> CommonType -> root
    if name in globals():
        t = globals()[name]

    if t is None:
        for modname in (
            f"nFusion.Model.msg_{msg_id}",
            "nFusion.Model.CommonType",
            "nFusion.Model",
        ):
            try:
                mod = importlib.import_module(modname)
                t = getattr(mod, name, None)
                if t is not None:
                    break
            except Exception:
                pass

    if t is None:
        raise NameError(f"C# type not found: {name}")
    return t()


def select_tx_fields(body: dict, fields: list) -> dict:
    """Filter a dictionary based on a whitelist of fields."""
    out = {}
    low = {k.lower(): k for k in body.keys()}

    def _get(key: str):
        kl = key.lower()
        if kl in low:
            return body[low[kl]]
        return None

    ts = _get("timestamp")
    if ts is not None:
        out["timestamp"] = int(ts)

    s = _get("source")
    sm = _get("sourceModuleName") or _get("sourcemodulename")
    rq = _get("requestModuleName") or _get("requestmodulename")
    src_val = s or sm or rq
    if src_val:
        out["sourceModuleName"] = str(src_val)

    for f in fields:
        if f in ("timestamp", "source", "sourceModuleName", "requestModuleName"):
            continue
        v = _get(f)
        if v is not None:
            try:
                out[f] = int(v)
            except Exception:
                out[f] = v
    return out


def project_root_for_push_file(__file_path: str):
    return Path(__file_path).resolve().parents[3]


def db_dir_for(msgid: str, __file_path: str) -> str:
    env_root = os.getenv("KU_MISSION_DB_ROOT")
    name = DB_DIR_RULES.get(msgid, f"msg_{msgid}")
    if env_root:
        return str(Path(env_root) / name)
    return str(project_root_for_push_file(__file_path) / "database" / name)


def make_and_push_based_on_rules(
    msg_id: str, file_path: str, body_generator, make_and_push_func, node_messenger
):
    """
    Generic function to create and push a message based on DB rules or a generator.
    """
    if msg_id in DB_DIR_RULES:
        dbdir = db_dir_for(msg_id, file_path)
        needs_prefix = "123" if msg_id == "0304" else None
        ids = list_numeric_ids(dbdir, needs_prefix)
        logs = []
        for vid in ids:
            wl = TX_FIELD_WHITELIST.get(msg_id, [])
            body = {
                "timestamp": now_ms(),
                "sourceModuleName": "DSC",
            }
            # Determine ID field
            if "inputMissionPackageID" in wl:
                body["inputMissionPackageID"] = vid
            if "missionReferencePackageID" in wl:
                body["missionReferencePackageID"] = vid
            if "missionPlanID" in wl:
                body["missionPlanID"] = vid
            if "individualMissionPackageID" in wl:
                body["individualMissionPackageID"] = vid
            if "pathID" in wl:
                body["pathID"] = vid
            logs.append(make_and_push_func(body, node_messenger))
        return b"\n".join(logs) if logs else b""
    else:
        body = body_generator()
        # Defensive coding for message 0102
        if msg_id == "0102":
            if not isinstance(body, dict) or not body:
                body = {
                    "timestamp": now_ms(),
                    "status": 1,  # Normal
                    "sourceModuleName": "DSC",
                }

        wl = TX_FIELD_WHITELIST.get(msg_id)
        if wl and isinstance(body, dict):
            body = select_tx_fields(body, wl)

        return make_and_push_func(body, node_messenger)

# 필요시 동적으로 늘릴 수 있게 리스트로 관리
SEARCH_PREFIXES = [
    "generator",  # ✅ 새로 추가: 최우선으로 generator/ 를 탐색
    "push_info",  # 공용 push_info
    "push",  # 공용 push
    "modules.common.push_info",  # 명시 common
    "modules.common.push",
    "modules.decision_support.push_info",  # DS fallback
    "modules.decision_support.push",
]


def push_message(msg_id: str, messenger, *, on_done=None, body_dict=None) -> bool:
    import importlib

    last_exc = None
    mod = None
    for pref in SEARCH_PREFIXES:
        try:
            mod = importlib.import_module(f"{pref}.message{msg_id}_push")
            # 디버깅에 도움: 실제 로딩된 모듈 경로를 한번 찍어두면 혼동이 없습니다.
            # print(f"[push_center] resolved: {mod.__name__} ({getattr(mod, '__file__', '?')})")
            break
        except Exception as e:
            print(f"DEBUG: Exception during module import: {e}")
            last_exc = e

    if mod is None:
        if last_exc:
            raise last_exc
        else:
            raise ImportError(f"message{msg_id}_push not found in {SEARCH_PREFIXES}")

    # 👇 변경 포인트: '비어있지 않은 dict'일 때만 make_and_push 사용
    use_manual = isinstance(body_dict, dict) and len(body_dict) > 0

    if use_manual and hasattr(mod, "make_and_push"):
        raw = mod.make_and_push(body_dict, messenger)
    elif hasattr(mod, "make_random_and_push"):
        raw = mod.make_random_and_push(messenger)
    elif hasattr(mod, "push"):
        raw = mod.push(messenger)
    else:
        raise AttributeError(
            f"{mod.__name__} has no push entrypoint (make_and_push/make_random_and_push/push)"
        )

    if on_done:
        on_done(msg_id, raw)
    return True