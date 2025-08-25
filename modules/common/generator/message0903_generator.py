# modules/common/generator/message0903_generator.py
# auto-generated at 2025-08-24T14:56:18.173150+00:00


import os, re, json, random
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
RULES_DIR = os.path.normpath(os.path.join(HERE, "..", "nFusion_MessageLIbrary", "rules"))
FIELD_PROFILES_PATH = os.path.join(RULES_DIR, "field_profiles.txt")
CONDITIONS_PATH     = os.path.join(RULES_DIR, "conditions.txt")
ID_POLICIES_PATH    = os.path.join(RULES_DIR, "id_policies.txt")
ID_STATE_PATH       = os.path.join(RULES_DIR, "id_state.json")

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

PRIMS = {"uint","ulong","int","float","double","bool","string",
         "uint32","uint64","int32","int64","float32","float64"}
TYPE_MAP = {"uint":"uint32","ulong":"uint64","int":"int32","float":"float32","double":"float64"}
SOURCE_ENUM_DEFAULT = ["DSC","IDM","MSM","MMR","UCC","MOB","CSP"]
LIST_LEN = 2

def _parse_kv_list(s: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    pat = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)\s*=')
    matches = list(pat.finditer(s))
    for i, m in enumerate(matches):
        key = m.group(1).upper()
        val_start = m.end()
        val_end = matches[i+1].start() if (i+1) < len(matches) else len(s)
        val = s[val_start:val_end].strip()
        if val.endswith(","): val = val[:-1].rstrip()
        out[key] = val
    return out

def _coerce_enum_list(enum_raw: str) -> List[Any]:
    items: List[Any] = []
    for token in [x.strip() for x in enum_raw.split(",") if x.strip()]:
        if ":" in token: token = token.split(":",1)[0].strip()
        if token.isdigit() or (token.startswith("-") and token[1:].isdigit()):
            items.append(int(token))
        else:
            items.append(token)
    return items

Profile = Dict[str, Any]
Profiles = Dict[str, Profile]
Conditions = Dict[str, list]

def load_field_profiles(path: str) -> Profiles:
    profs: Profiles = {}
    if not os.path.exists(path): return profs
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            t = line.strip()
            if not t or t.startswith("#"): continue
            m = re.match(r'^PROFILE\s+([A-Za-z0-9_]+)\s*=\s*(.+)$', t)
            if not m: continue
            name, rhs = m.group(1), m.group(2)
            kv = _parse_kv_list(rhs)
            if "ENUM" in kv: kv["ENUM"] = _coerce_enum_list(kv["ENUM"])
            profs[name] = kv
    return profs

def load_conditions(path: str) -> Conditions:
    cond: Conditions = {"REQUIRED_IF": [], "USE_PROFILE": [], "CONSTRAINT": []}
    if not os.path.exists(path): return cond
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"): continue
            h = line.find("#")
            if h != -1: line = line[:h].rstrip()
            if not line: continue

            m = re.match(
                r'^WHEN(?:\s+(?P<prefix>.*?))?\s*field=(?P<field>[A-Za-z0-9_.]+)\s+USE_PROFILE\s+(?P<prof>[A-Za-z0-9_]+)\s*$',
                line, re.I
            )
            if m:
                prefix = m.group('prefix') or ''
                field  = m.group('field'); prof = m.group('prof')
                mm = re.search(r'message=(\d{4})', prefix, re.I)
                msg = mm.group(1) if mm else None
                pp = re.search(r'IF\s+(.+)$', prefix, re.I)
                pred = pp.group(1).strip() if pp else None
                cond["USE_PROFILE"].append({"message": msg, "field": field, "profile": prof, "pred": pred})
                continue

            m = re.match(
                r'^WHEN(?:\s+(?P<prefix>.*?))?\s*field=(?P<field>[A-Za-z0-9_.]+)\s+REQUIRED_IF\s+(?P<pred>.+)$',
                line, re.I
            )
            if m:
                prefix = m.group('prefix') or ''
                field  = m.group('field'); pred = m.group('pred').strip()
                mm = re.search(r'message=(\d{4})', prefix, re.I)
                msg = mm.group(1) if mm else None
                cond["REQUIRED_IF"].append({"message": msg, "field": field, "pred": pred})
                continue

            m = re.match(r'^CONSTRAINT\s+([A-Za-z0-9_.]+)\s*:\s*(.+)$', line, re.I)
            if m:
                target, expr = m.group(1), m.group(2).strip()
                cond["CONSTRAINT"].append({"target": target, "expr": expr})
                continue
    return cond

class IDSeq:
    def __init__(self, start: int, step: int=1):
        self.start = int(start); self.step = int(step)

def load_id_policies(path: str) -> Dict[str, IDSeq]:
    seqs: Dict[str, IDSeq] = {}
    if not os.path.exists(path): return seqs
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"): continue
            m = re.match(r'^SEQUENCE\s+([A-Za-z0-9_.]+)\s*=\s*(.+)$', line)
            if not m: continue
            name, rhs = m.group(1), m.group(2)
            kv = _parse_kv_list(rhs)
            start = kv.get("START", "1"); step = kv.get("STEP", "1")
            seqs[name] = IDSeq(int(start), int(step))
    return seqs

def load_id_state(path: str) -> Dict[str, int]:
    if not os.path.exists(path): return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

def save_id_state(path: str, state: Dict[str, int]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)

FIELD_PROFILES = load_field_profiles(FIELD_PROFILES_PATH)
CONDITIONS     = load_conditions(CONDITIONS_PATH)
ID_POLICIES    = load_id_policies(ID_POLICIES_PATH)
ID_STATE       = load_id_state(ID_STATE_PATH)

def _profile_base(name: str) -> str:
    return name.split("_", 1)[0].lower()

PROFILE_BASE_INDEX: Dict[str, str] = {}
for pname in FIELD_PROFILES.keys():
    PROFILE_BASE_INDEX.setdefault(_profile_base(pname), pname)

DEFAULT_BINDINGS = {
    "timestamp": "timestamp_2000ms",
    "source":    "source_node3",
    "systemMode":"systemMode_v1",
    "status":    "status_v1",
    "mode":      "mode_sw_v1",
    "aircraftID":"aircraft_id_any_v1",
    "mainSensor":"mainSensor_v1",
    "missionType":"missionType_v1",
    "flightMode":"flightMode_v1",
}

class GenCtx:
    def __init__(self, msg_id: str):
        self.msg_id = msg_id

def _find_profile_for_field(msg_id: str, field_name: str) -> Optional[Dict[str, Any]]:
    base = re.sub(r'[^a-z0-9]', '', field_name.lower())
    if base in PROFILE_BASE_INDEX:
        pname = PROFILE_BASE_INDEX[base]
        prof = FIELD_PROFILES.get(pname)
        if prof: return prof
    if field_name in DEFAULT_BINDINGS:
        prof = FIELD_PROFILES.get(DEFAULT_BINDINGS[field_name])
        if prof: return prof
    if field_name == "source":
        return FIELD_PROFILES.get("source_node3", {"TYPE":"string","SIZE":"3","ENUM":SOURCE_ENUM_DEFAULT})
    if field_name == "systemMode":
        return {"TYPE":"uint32","MIN":"0","MAX":"3","ENUM":[0,1,2,3]}
    return None

def _gen_from_profile(field_name: str, base_type: str, prof: Dict[str, Any]):
    if "VALUE" in prof:
        val = prof["VALUE"]
        if isinstance(val, str) and (val.isdigit() or (val.startswith("-") and val[1:].isdigit())):
            try: return int(val)
            except Exception: return val
        return val
    enum_vals = prof.get("ENUM")
    if enum_vals:
        pick = random.choice(list(enum_vals))
        if isinstance(pick, str) and (pick.isdigit() or (pick.startswith("-") and pick[1:].isdigit())):
            try: return int(pick)
            except Exception: pass
        return pick
    min_v = prof.get("MIN"); max_v = prof.get("MAX"); size = prof.get("SIZE"); epoch = (prof.get("EPOCH") or "").lower()
    if field_name.lower() == "timestamp":
        if "1970" in epoch:
            return int(datetime.now(timezone.utc).timestamp() * 1000)
        return now_ms_2000()
    if base_type in ("uint32","int32","uint64","int64"):
        lo = int(min_v) if min_v is not None else (0 if base_type.startswith("u") else -100)
        hi = int(max_v) if max_v is not None else (100 if base_type.startswith("u") else 100)
        return random.randint(lo, hi)
    if base_type in ("float32","float64"):
        lo = float(min_v) if min_v is not None else 0.0
        hi = float(max_v) if max_v is not None else 100.0
        return round(random.uniform(lo, hi), 6)
    if base_type == "bool":
        return random.choice([True, False])
    if base_type == "string":
        if field_name == "source":
            if enum_vals and all(isinstance(x,str) for x in enum_vals):
                return random.choice(enum_vals)
            return random.choice(SOURCE_ENUM_DEFAULT)
        n = int(size) if size is not None else 8
        import string
        alpha = string.ascii_uppercase + string.digits
        return "".join(random.choice(alpha) for _ in range(n))
    return None

def _next_id(seq_name: str) -> int:
    seq = ID_POLICIES.get(seq_name)
    if seq is None:
        return random.randint(1, 1_000_000_000)
    last = ID_STATE.get(seq_name, seq.start - seq.step)
    nxt = last + seq.step
    if nxt < seq.start: nxt = seq.start
    ID_STATE[seq_name] = nxt
    return nxt

def _alloc_path_id(context: Dict[str, Any], msg_id: str) -> int:
    if msg_id == "0303":
        aid = context.get("aircraftID", 4)
        if aid == 4: return _next_id("PathID.UAV.UAV1")
        if aid == 5: return _next_id("PathID.UAV.UAV2")
        if aid == 6: return _next_id("PathID.UAV.UAV3")
        return _next_id("PathID.UAV.UAV1")
    if msg_id == "0304":
        aid = context.get("aircraftID", 1)
        if aid == 1: return _next_id("PathID.MANNED.COMMANDER")
        if aid == 2: return _next_id("PathID.MANNED.WING1")
        if aid == 3: return _next_id("PathID.MANNED.WING2")
        return _next_id("PathID.MANNED.COMMANDER")
    aid = context.get("aircraftID")
    if isinstance(aid, int):
        if 4 <= aid <= 6:
            if aid == 4: return _next_id("PathID.UAV.UAV1")
            if aid == 5: return _next_id("PathID.UAV.UAV2")
            if aid == 6: return _next_id("PathID.UAV.UAV3")
        elif 1 <= aid <= 3:
            if aid == 1: return _next_id("PathID.MANNED.COMMANDER")
            if aid == 2: return _next_id("PathID.MANNED.WING1")
            if aid == 3: return _next_id("PathID.MANNED.WING2")
    return _next_id("WaypointID")

def gen_value(field_name: str, type_token: str, ctx_msg_id: str, obj_context: Optional[Dict[str, Any]]=None, depth: int=0):
    if depth > 12: return None
    t = type_token.strip()
    if t.startswith("List<") and t.endswith(">"):
        inner = t[5:-1].strip()
        # 원시형/컴포지트 모두 처리: 컴포지트면 호출부에서 _gen_*로 생성
        base = inner
        if base in ("uint32","int32","uint64","int64","float32","float64"):
            return [gen_value(field_name, base, ctx_msg_id, obj_context, depth+1) for _ in range(LIST_LEN)]
        return [None for _ in range(LIST_LEN)]  # 컴포지트는 호출부에서 교체
    base = TYPE_MAP.get(t, t)
    lname = field_name.lower()
    # ID 정책 우선
    if lname == "missionplanid":                return _next_id("MissionPlanID")
    if lname == "individualmissionpackageid":   return _next_id("IndividualMissionPackageID")
    if lname == "individualmissionid":          return _next_id("IndividualMissionID")
    if lname == "pathid":                       return _alloc_path_id(obj_context or {}, ctx_msg_id)
    if lname in ("waypointid","priormissionid","inputmissionpackageid","inputmissionid",
                 "missionreferencepackageid","targetid","flightareaid","prohibitedareaid",
                 "behaviortreefileid"):
        return _next_id(field_name[0].upper() + field_name[1:])
    # 프로필
    prof = _find_profile_for_field(ctx_msg_id, field_name)
    if prof:
        val = _gen_from_profile(field_name, base, prof)
        if val is not None: return val
    # 기본
    if base in ("uint32","int32","uint64","int64"):
        if lname == "timestamp": return now_ms_2000()
        if lname in ("health","currentindividualmissionprogress"): return random.randint(0,100)
        return random.randint(0, 1000) if base.startswith("u") else random.randint(-1000, 1000)
    if base in ("float32","float64"):
        if lname in ("fuel","lowerlimit","upperlimit"): return round(random.uniform(0.0, 100.0), 6)
        if lname == "latitude":  return round(random.uniform(-90.0, 90.0), 6)
        if lname == "longitude": return round(random.uniform(-180.0, 180.0), 6)
        return round(random.uniform(0.0, 100.0), 6)
    if base == "bool":
        return random.choice([True, False])
    if base == "string":
        if field_name == "source": return random.choice(SOURCE_ENUM_DEFAULT)
        import string
        alpha = string.ascii_uppercase + string.digits
        return "".join(random.choice(alpha) for _ in range(8))
    # 컴포지트는 호출부에서 _gen_*로 생성
    return None

def _walk(obj: Any, fn):
    if isinstance(obj, dict):
        fn(obj)
        for v in obj.values(): _walk(v, fn)
    elif isinstance(obj, list):
        for it in obj: _walk(it, fn)

def _lookup_field_any(obj: Any, field: str) -> List[Tuple[dict, str]]:
    hits: List[Tuple[dict, str]] = []
    def visit(node: dict):
        for k in list(node.keys()):
            if k == field or k.lower() == field.lower():
                hits.append((node, k))
    _walk(obj, visit)
    return hits

def _eval_simple_predicate(obj: dict, pred: Optional[str]) -> bool:
    if not pred: return True
    m = re.match(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*==\s*(.+?)\s*$', pred)
    if not m: return False
    fname, raw = m.group(1), m.group(2)
    expect = int(raw) if (raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit())) else raw.strip().strip('"').strip("'")
    hits = _lookup_field_any(obj, fname)
    if not hits: return False
    for parent, key in hits:
        if parent.get(key) == expect: return True
    return False

def _get_ci(d: dict, k: str):
    kl = k.lower()
    for kk, vv in d.items():
        if kk.lower() == kl: return vv
    return None

def _eval_predicate_in_parent(parent: dict, pred: Optional[str]) -> bool:
    if not pred: return True
    h = pred.find("#")
    if h != -1: pred = pred[:h].rstrip()
    m = re.match(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*==\s*(.+?)\s*$', pred)
    if not m: return False
    fname, raw = m.group(1), m.group(2)
    expect = int(raw) if (raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit())) else raw.strip().strip('"').strip("'")
    cur = _get_ci(parent, fname)
    return (cur == expect)

def _create_required_field(parent: dict, field_name: str, ctx_msg_id: str) -> None:
    for k in parent.keys():
        if k.lower() == field_name.lower(): return
    key_to_set = field_name[:1].lower() + field_name[1:] if field_name else field_name
    # 타입을 알 수 없으므로 최소 스텁 + 프로필 기반 값
    val = gen_value(key_to_set, field_name, ctx_msg_id, parent, 0)
    if val is None:
        if key_to_set.endswith("list"): parent[key_to_set] = []
        elif key_to_set.lower() in ("loiterproperty","gimbalyawlimits"): parent[key_to_set] = {}
        else: parent[key_to_set] = 0
    else:
        parent[key_to_set] = val

def _apply_use_profile(obj: dict, msg_id: str):
    rules = CONDITIONS.get("USE_PROFILE", [])
    for r in rules:
        if r.get("message") and r["message"] != msg_id: continue
        field_path = r["field"]; prof_name = r["profile"]; pred = r.get("pred")
        if not _eval_simple_predicate(obj, pred): continue
        tokens = field_path.split("."); target = tokens[-1]
        hits = _lookup_field_any(obj, target)
        prof = FIELD_PROFILES.get(prof_name)
        if not prof: continue
        for parent, key in hits:
            cur = parent.get(key)
            if isinstance(cur, bool): base_type = "bool"
            elif isinstance(cur, int): base_type = "uint32"
            elif isinstance(cur, float): base_type = "float32"
            elif isinstance(cur, str): base_type = "string"
            else:
                base_type = prof.get("TYPE", "uint32")
                base_type = TYPE_MAP.get(base_type, base_type)
            parent[key] = _gen_from_profile(key, base_type, prof) or cur

def _apply_required_if(obj: dict, msg_id: str):
    rules = CONDITIONS.get("REQUIRED_IF", [])
    for r in rules:
        if r.get("message") and r["message"] != msg_id: continue
        field_path = r["field"]; pred = r.get("pred")
        target = field_path.split(".")[-1]
        hits = _lookup_field_any(obj, target)
        for parent, key in hits:
            if not _eval_predicate_in_parent(parent, pred):
                try: del parent[key]
                except Exception: pass
        def ensure_visit(node: dict):
            if _eval_predicate_in_parent(node, pred):
                for k in node.keys():
                    if k.lower() == target.lower(): break
                else:
                    _create_required_field(node, target, msg_id)
        _walk(obj, ensure_visit)

def _apply_constraints(obj: dict):
    def visit(node: dict):
        if "gimbalYawLimits" in node and isinstance(node["gimbalYawLimits"], dict):
            gl = node["gimbalYawLimits"]
            l = gl.get("leftLimit"); r = gl.get("rightLimit")
            if isinstance(l,(int,float)) and isinstance(r,(int,float)) and l > r:
                gl["leftLimit"], gl["rightLimit"] = r, l
        if "GimbalYawLimits" in node and isinstance(node["GimbalYawLimits"], dict):
            gl = node["GimbalYawLimits"]
            l = gl.get("leftLimit"); r = gl.get("rightLimit")
            if isinstance(l,(int,float)) and isinstance(r,(int,float)) and l > r:
                gl["leftLimit"], gl["rightLimit"] = r, l
    _walk(obj, visit)


MSG_ID = "0903"


def _gen_RequestRenewMission(source: str = "DS"):
    obj = {}
    obj["timestamp"] = gen_value("timestamp", "uint64", MSG_ID, obj, 0)
    obj["source"] = gen_value("source", "string", MSG_ID, obj, 0)
    obj["missionPlanID"] = gen_value("missionPlanID", "uint32", MSG_ID, obj, 0)
    return obj




def make_msg0903_body(source: str = "DS"):
    """자동 생성된 메시지 바디 — MessageID: 0903, RootType: RequestRenewMission"""
    ex = _gen_RequestRenewMission(source=source)
    _apply_use_profile(ex, MSG_ID)
    _apply_required_if(ex, MSG_ID)
    _apply_constraints(ex)
    try:
        save_id_state(ID_STATE_PATH, ID_STATE)
    except Exception:
        pass
    return ex

def make_random_and_push(node_messenger):
    from generator.message0903_push import make_and_push
    return make_and_push(make_msg0903_body(), node_messenger)

if __name__ == "__main__":
    print(json.dumps(make_msg0903_body(), ensure_ascii=False, indent=2))