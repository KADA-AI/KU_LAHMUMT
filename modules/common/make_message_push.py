# modules/common/make_message_push.py
# msg_**** 스키마를 스캔해 C# 모델로 매핑하는 push 모듈을 자동 생성합니다.
# 사용법:
#   python -m modules.common.make_message_push            # 모든 msg_**** 대상 일괄 생성
#   python -m modules.common.make_message_push 0301       # 단일 생성
#   python -m modules.common.make_message_push 0301,0304  # 다중 생성
# 생성 위치:
#   modules/common/push/messageXXXX_push.py

import os, re, sys, textwrap, datetime
from typing import Dict, List, Tuple, Optional

# ─────────────────────────────────────────────────────────
# 경로/확장자
# ─────────────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
_LIB_CAND = ["nFusion_MessageLIbrary", "nFusion_MessageLibrary"]
for _cand in _LIB_CAND:
    _p = os.path.join(HERE, _cand)
    if os.path.isdir(_p):
        LIB_ROOT = _p
        break
else:
    LIB_ROOT = os.path.join(HERE, "nFusion_MessageLIbrary")

COMMON_DIR = os.path.join(LIB_ROOT, "CommonType")
PUSH_DIR   = os.path.join(HERE, "push")
GEN_DIR    = os.path.join(HERE, "generator")

EXTS = (".nfpsh", ".nftype", ".txt")
IGNORE_NFPSH_FOR = {"0201","0203","0301","0302","0303","0304"}
def _exts_for_message(msgid: str) -> tuple:
    return (".nftype",) if msgid in IGNORE_NFPSH_FOR else EXTS

# ─────────────────────────────────────────────────────────
# 스키마 파서
# ─────────────────────────────────────────────────────────
DECL_RE  = re.compile(r'^(?P<type>List<[^>]+>|[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)?)\s+(?P<name>[A-Za-z_]\w*)\s*(?:[#/].*)?$')
MSGID_RE = re.compile(r'^\s*#\s*(?:MissionID|MessageID)\s+(\d{4})\b', re.I)

TYPE_MAP = {"uint":"uint32","ulong":"uint64","int":"int32","float":"float32","double":"float64"}
PRIMS = {
    "bool","string",
    "float","double","float32","float64",
    "int","uint","int32","uint32","int64","uint64",
    "short","ushort","byte","sbyte","long","ulong",
}

Field = Tuple[str, str]
TypeDef = Dict[str, List[Field]]

def _list_schema_files(folder: str, exts: tuple) -> List[str]:
    if not os.path.isdir(folder): return []
    return [os.path.join(folder, f) for f in os.listdir(folder) if f.lower().endswith(exts)]

def _read_lines(path: str) -> List[str]:
    with open(path, "r", encoding="utf-8") as f:
        return f.readlines()

def _parse_typename_from_filename(path: str) -> str:
    return os.path.splitext(os.path.basename(path))[0]

def _normalize_type_token(t: str) -> str:
    t = t.strip()
    if t.startswith("List<") and t.endswith(">"):
        inner = t[5:-1].strip()
        return f"List<{_normalize_type_token(inner)}>"
    if "." in t:
        ns, base = t.split(".", 1)
        return base if ns == "CommonType" else t
    return TYPE_MAP.get(t, t)

def _is_primitive(t: str) -> bool:
    return t in PRIMS

def _parse_fields(lines: List[str]) -> List[Field]:
    out: List[Field] = []
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue
        m = DECL_RE.match(line)
        if not m:
            continue
        tkn  = m.group("type").strip()
        name = m.group("name").strip()
        out.append((name, tkn))
    return out

def _scan_folder(folder: str, exts: tuple, msg_id: Optional[str]=None) -> Tuple[TypeDef, Optional[str]]:
    typedefs: TypeDef = {}
    root_candidate: Optional[str] = None
    files = _list_schema_files(folder, exts)
    for fp in files:
        lines = _read_lines(fp)
        for l in lines:
            m = MSGID_RE.match(l)
            if m and (msg_id is None or m.group(1) == msg_id):
                root_candidate = fp
                break
        tname = _parse_typename_from_filename(fp)
        flds  = _parse_fields(lines)
        if flds:
            typedefs[tname] = flds
    if not root_candidate:
        mission_named = [p for p in files if os.path.basename(p).lower().startswith("mission")]
        root_candidate = mission_named[0] if mission_named else (files[0] if files else None)
    return typedefs, root_candidate

# ─────────────────────────────────────────────────────────
# 루트 선택/토폴로지
# ─────────────────────────────────────────────────────────
def _field_cnt(tdef: Optional[List[Field]]) -> int:
    return 0 if not tdef else len(tdef)

def _all_prim(tdef: Optional[List[Field]]) -> bool:
    if not tdef: return True
    for _, t in tdef:
        tt = _normalize_type_token(t)
        if tt.startswith("List<"): return False
        if not _is_primitive(tt): return False
    return True

class Registry:
    def __init__(self, common_dir: str):
        self.common: TypeDef = {}
        self.messages: TypeDef = {}
        tds, _ = _scan_folder(common_dir, EXTS, msg_id=None)
        self.common.update(tds)
    def add_message_types(self, tds: TypeDef):
        self.messages.update(tds)
    def get_typedef(self, name: str) -> Optional[List[Field]]:
        if name in self.messages: return self.messages[name]
        if name in self.common:   return self.common[name]
        return None
    def all_message_types(self) -> List[str]:
        return list(self.messages.keys())

def _unwrap_single_wrapper(root_tname: str, reg: Registry) -> str:
    tdef = reg.get_typedef(root_tname)
    if not tdef or len(tdef) != 1:
        return root_tname
    _, inner = tdef[0]
    inner = _normalize_type_token(inner)
    if inner.startswith("List<") and inner.endswith(">"):
        base = _normalize_type_token(inner[5:-1])
        return base if reg.get_typedef(base) else root_tname
    base = _normalize_type_token(inner)
    return base if reg.get_typedef(base) else root_tname

def pick_better_root(root_tname: str, reg: Registry) -> str:
    cand = root_tname
    tdef = reg.get_typedef(cand)
    if _field_cnt(tdef) <= 2 and _all_prim(tdef):
        for suf in ("Data","Info","Root"):
            alt = root_tname + suf
            if reg.get_typedef(alt):
                cand = alt
                tdef = reg.get_typedef(cand)
                break
    cand = _unwrap_single_wrapper(cand, reg)
    tdef = reg.get_typedef(cand)
    if _field_cnt(tdef) <= 2:
        keys = reg.all_message_types()
        if keys:
            richest = max(keys, key=lambda k: _field_cnt(reg.get_typedef(k)))
            if _field_cnt(reg.get_typedef(richest)) > _field_cnt(tdef):
                cand = richest
    return cand

def _list_inner(t: str) -> Optional[str]:
    return t[5:-1].strip() if t.startswith("List<") and t.endswith(">") else None

def _topo_types(root: str, reg: Registry) -> List[str]:
    order, seen = [], set()
    def dfs(t: str):
        if t in seen: return
        seen.add(t)
        for _, ftype in (reg.get_typedef(t) or []):
            inner = _list_inner(_normalize_type_token(ftype))
            if inner:
                base = _normalize_type_token(inner)
                if not _is_primitive(base) and reg.get_typedef(base):
                    dfs(base)
                continue
            base = _normalize_type_token(ftype)
            if not _is_primitive(base) and reg.get_typedef(base):
                dfs(base)
        order.append(t)
    dfs(root)
    return order

# ─────────────────────────────────────────────────────────
# C# 기본형 매핑
# ─────────────────────────────────────────────────────────
CS_MAP = {
    "bool":"Boolean", "string":"String",
    "byte":"Byte", "sbyte":"SByte",
    "short":"Int16", "ushort":"UInt16",
    "int":"Int32", "uint":"UInt32",
    "long":"Int64", "ulong":"UInt64",
    "int32":"Int32", "uint32":"UInt32",
    "int64":"Int64", "uint64":"UInt64",
    "float":"Single", "float32":"Single",
    "double":"Double", "float64":"Double",
}
def _cstype_of(token: str) -> Optional[str]:
    return CS_MAP.get(token)

# ─────────────────────────────────────────────────────────
# 루트 C# 클래스명 오버라이드
# ─────────────────────────────────────────────────────────
CLASS_NAME_OVERRIDES = {
    "0000": "RequestData",
    "0101": "SystemOperationMode",
    "0102": "ModuleStatus",
    "0103": "SWStatus",
    "0201": "InputMissionPlan",
    "0202": "PriorMissionInfo",
    "0203": "FlightReferenceInfo",
    "0301": "MissionPlan",
    "0302": "IndividualMissionPlan",
    "0303": "UAVFlightPlan",
    "0304": "LAHFlightPlan",
    "0305": "ReplanStatus",
    "0401": "AgentStatus",
    "0402": "SituationAwarenessInfo",
    "0501": "MissionProgress",
    "0502": "EndMissionRequest",
    "0503": "MissionResult",
    "0601": "BasicAction",
    "0602": "UAVCommand",
    "0701": "MissionPlanOptionInfo",
    "0702": "PilotDecision",
    "0801": "InitialPlanCommand",
    "0802": "MandatoryCommand",
    "0803": "ExecutionCommand",
    "0805": "SystemEvent",
    "0806": "BootCommand",
    "0901": "RequestOptionInfo",
    "0902": "ReplanRequest",
    "0903": "RequestRenewMission",
    "0904": "RequestBehaviorTree",
}

# TX 화이트리스트(최소 필드만 전송)
TX_FIELD_WHITELIST = {
    "0201": ["timestamp", "inputMissionPackageID"],
    "0203": ["timestamp", "missionReferencePackageID"],
    "0301": ["timestamp", "missionPlanID"],
    "0302": ["timestamp", "individualMissionPackageID"],
    "0303": ["timestamp", "pathID"],
    "0304": ["timestamp", "pathID"],
}

# DB 디렉터리 규칙
DB_DIR_RULES = {
    "0201": "InputMissionPlan",
    "0203": "MissionReferenceInfo",
    "0301": "MissionPlan",
    "0302": "IndividualMissionPlan",
    "0303": "FlightPath",
    "0304": "FlightPath",  # 유인기
}

# ─────────────────────────────────────────────────────────
# 공통 유틸(생성된 파일에 임베드될 텍스트)
# ─────────────────────────────────────────────────────────
def _embedded_rules_text() -> str:
    # 모든 최상위 줄이 들여쓰기 0이 되도록 구성
    return textwrap.dedent("""\
# ── Embedded TX/DB rules (self-contained) ──────────────────────────────────
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
    "0203": "MissionReferenceInfo",
    "0301": "MissionPlan",
    "0302": "IndividualMissionPlan",
    "0303": "FlightPath",
    "0304": "FlightPath",
}

def _select_tx_fields(body: dict, fields: list) -> dict:
    \"\"\"화이트리스트로 선별: timestamp / source 계열 폴백 / 나머지 ID류만 남김\"\"\"
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

    s  = _get("source")
    sm = _get("sourceModuleName") or _get("sourcemodulename")
    rq = _get("requestModuleName") or _get("requestmodulename")
    src_val = s or sm or rq
    if src_val:
        out["sourceModuleName"] = str(src_val)

    for f in fields:
        if f in ("timestamp","source","sourceModuleName","requestModuleName"):
            continue
        v = _get(f)
        if v is not None:
            try:
                out[f] = int(v)
            except Exception:
                out[f] = v
    return out

def _project_root_for_push_file(__file_path: str):
    from pathlib import Path
    return Path(__file_path).resolve().parents[3]

def _db_dir_for(msgid: str, __file_path: str) -> str:
    import os
    from pathlib import Path
    env_root = os.getenv("KU_MISSION_DB_ROOT")
    name = DB_DIR_RULES.get(msgid, f"msg_{msgid}")
    if env_root:
        return str(Path(env_root) / name)
    return str(_project_root_for_push_file(__file_path) / "database" / name)

def _list_numeric_ids(dirname: str, prefix_first_char: str | None = None) -> list[int]:
    import os, glob
    ids = []
    for p in glob.glob(os.path.join(dirname, "*.json")):
        stem = os.path.splitext(os.path.basename(p))[0]
        if stem.isdigit():
            if prefix_first_char and stem[0] not in prefix_first_char:
                continue
            ids.append(int(stem))
    ids.sort()
    return ids
""").rstrip()

# ─────────────────────────────────────────────────────────
# push 코드 생성
# ─────────────────────────────────────────────────────────
def _emit_assign_line(fname: str, base: str) -> str:
    if base in ("uint32","int32","uint64","int64","uint","int","ulong","long"):
        return f'    if "{fname}" in data: _try_set(obj, "{fname}", int(data["{fname}"]))'
    if base in ("float32","float64","float","double"):
        return f'    if "{fname}" in data: _try_set(obj, "{fname}", float(data["{fname}"]))'
    if base == "bool":
        return f'    if "{fname}" in data: _try_set(obj, "{fname}", bool(data["{fname}"]))'
    if base == "string":
        if fname in ("source", "sourceModuleName"):
            alt = "source" if fname == "sourceModuleName" else "sourceModuleName"
            return "\n".join([
                f'    val_src = data.get("{fname}", data.get("source", data.get("sourceModuleName", data.get("requestModuleName", ""))))',
                '    if val_src != "":',
                f'        if not _try_set(obj, "{fname}", str(val_src)):',
                f'            _try_set(obj, "{alt}", str(val_src))',
            ])
        return f'    if "{fname}" in data: _try_set(obj, "{fname}", str(data["{fname}"]))'
    return f'    if "{fname}" in data: _try_set(obj, "{fname}", data["{fname}"])'

def _emit_push_code(msgid: str, reg: Registry, root_type: str) -> str:
    used_cs_prims: set = set()
    order = _topo_types(root_type, reg)

    # 사용되는 System 기본형 수집(List[T] 제네릭용)
    for t in order:
        for _, ftype in reg.get_typedef(t) or []:
            norm = _normalize_type_token(ftype)
            inner = _list_inner(norm)
            if inner:
                base = _normalize_type_token(inner)
                if _is_primitive(base):
                    ct = _cstype_of(base)
                    if ct: used_cs_prims.add(ct)
            else:
                base = _normalize_type_token(norm)
                if _is_primitive(base):
                    ct = _cstype_of(base)
                    if ct: used_cs_prims.add(ct)

    header = (
        f"# modules/common/push/message{msgid}_push.py\n"
        f"# auto-generated at {datetime.datetime.now(datetime.timezone.utc).isoformat()}\n"
    )

    sys_imports = ""
    if used_cs_prims:
        sys_types = ", ".join(sorted(used_cs_prims))
        sys_imports = f"from System import {sys_types}"

    top_lines = [
        "import json, importlib",
        "from datetime import datetime, timezone",
        "from System.Collections.Generic import List",
        f"from nFusion.Model.msg_{msgid} import *    # C# 모델(우선)",
        "from nFusion.Model.CommonType import *     # 공통 타입(항상)",
        sys_imports if sys_imports else "",
        f"from generator.message{msgid}_generator import make_msg{msgid}_body",
        "",
        "_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)",
        "_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)",
        f'MSG_ID = "{msgid}"',
        "",
        "def _try_set(obj, name: str, value) -> bool:",
        "    # lowerCamel 또는 PascalCase 둘 다 시도",
        "    for k in (name, name[:1].upper()+name[1:] if name else name):",
        "        try:",
        "            if hasattr(obj, k):",
        "                setattr(obj, k, value)",
        "                return True",
        "        except Exception:",
        "            pass",
        "    return False",
        "",
        "def _cs(name: str):",
        "    # 현재 전역 → msg_ID 모듈 → CommonType → 루트 순으로 검색",
        "    t = globals().get(name)",
        "    if t is not None: return t",
        "    for modname in (f'nFusion.Model.msg_{MSG_ID}', 'nFusion.Model.CommonType', 'nFusion.Model'):",
        "        try:",
        "            mod = importlib.import_module(modname)",
        "            t = getattr(mod, name, None)",
        "            if t is not None: return t",
        "        except Exception:",
        "            pass",
        "    return None",
        "",
        "def _new(name: str):",
        "    t = _cs(name)",
        "    if t is None:",
        "        raise NameError(f'type not found: {name}')",
        "    return t()",
    ]
    top = "\n".join([ln for ln in top_lines if ln != ""])

    # 타입별 dict→C#
    type_funcs: List[str] = []
    for t in order:
        fields = reg.get_typedef(t) or []
        lines: List[str] = []
        lines.append(f"def _dict_to_{t}(data: dict):")
        lines.append(f"    obj = _new('{t}')")
        for fname, ftype in fields:
            norm = _normalize_type_token(ftype)
            inner = _list_inner(norm)
            if inner:
                base = _normalize_type_token(inner)
                if _is_primitive(base):
                    cs = _cstype_of(base) or "Object"
                    lines.append(f'    if "{fname}" in data and isinstance(data["{fname}"], list):')
                    lines.append(f"        lst = List[{cs}]()")
                    conv = "int" if base in ("uint32","int32","uint64","int64","uint","int","ulong","long") else \
                           "float" if base in ("float32","float64","float","double") else \
                           "bool" if base == "bool" else \
                           "str" if base == "string" else None
                    if conv:
                        lines.append(f'        for x in data["{fname}"]: lst.Add({conv}(x))')
                    else:
                        lines.append(f'        for x in data["{fname}"]: lst.Add(x)')
                    lines.append(f'        _try_set(obj, "{fname}", lst)')
                else:
                    lines.append(f'    if "{fname}" in data and isinstance(data["{fname}"], list):')
                    lines.append(f"        T = _cs('{base}') or object")
                    lines.append(f"        lst = List[T]()")
                    lines.append(f'        for item in data["{fname}"]: lst.Add(_dict_to_{base}(item if isinstance(item, dict) else {{}}))')
                    lines.append(f'        _try_set(obj, "{fname}", lst)')
            else:
                base = _normalize_type_token(norm)
                if _is_primitive(base):
                    lines.append(_emit_assign_line(fname, base))
                else:
                    lines.append(f'    if "{fname}" in data and isinstance(data["{fname}"], dict):')
                    lines.append(f'        _try_set(obj, "{fname}", _dict_to_{base}(data["{fname}"]))')
        lines.append("    return obj")
        type_funcs.append("\n".join(lines))

    # 루트 결정 및 브리지
    cs_root = CLASS_NAME_OVERRIDES.get(msgid, root_type)
    if cs_root not in order:
        fields = reg.get_typedef(root_type) or []
        bl: List[str] = []
        bl.append(f"def _dict_to_{cs_root}(data: dict):")
        bl.append(f"    obj = _new('{cs_root}')")
        for fname, ftype in fields:
            norm = _normalize_type_token(ftype)
            inner = _list_inner(norm)
            if inner:
                base = _normalize_type_token(inner)
                if _is_primitive(base):
                    cs = _cstype_of(base) or "Object"
                    bl.append(f'    if "{fname}" in data and isinstance(data["{fname}"], list):')
                    bl.append(f"        lst = List[{cs}]()")
                    conv = "int" if base in ('uint32','int32','uint64','int64','uint','int','ulong','long') else \
                           "float" if base in ('float32','float64','float','double') else \
                           "bool" if base == "bool" else \
                           "str" if base == "string" else None
                    if conv:
                        bl.append(f'        for x in data["{fname}"]: lst.Add({conv}(x))')
                    else:
                        bl.append(f'        for x in data["{fname}"]: lst.Add(x)')
                    bl.append(f'        _try_set(obj, "{fname}", lst)')
                else:
                    bl.append(f'    if "{fname}" in data and isinstance(data["{fname}"], list):')
                    bl.append(f"        T = _cs('{base}') or object")
                    bl.append("        lst = List[T]()")
                    bl.append(f'        for item in data["{fname}"]: lst.Add(_dict_to_{base}(item if isinstance(item, dict) else {{}}))')
                    bl.append(f'        _try_set(obj, "{fname}", lst)')
            else:
                base = _normalize_type_token(norm)
                if _is_primitive(base):
                    bl.append(_emit_assign_line(fname, base))
                else:
                    bl.append(f'    if "{fname}" in data and isinstance(data["{fname}"], dict):')
                    bl.append(f'        _try_set(obj, "{fname}", _dict_to_{base}(data["{fname}"]))')
        bl.append("    return obj")
        type_funcs.append("\n".join(bl))

    # ── 루트/푸시 ──
    root_block = textwrap.dedent(f"""
    def _dict_to_obj(body_dict: dict):
        return _dict_to_{cs_root}(body_dict)

    def make_and_push(body_dict: dict, node_messenger) -> bytes:
        # TX 화이트리스트가 있으면 최종 전송 전 선별(제너레이터가 풍부하게 만들어도 최소필드만 보냄)
        wl = TX_FIELD_WHITELIST.get(MSG_ID)
        if wl and isinstance(body_dict, dict):
            body_dict = _select_tx_fields(body_dict, wl)
        msg = _dict_to_obj(body_dict)
        node_messenger.Push(msg)
        log_line = (
            f"[{msgid}] BODY  : {{json.dumps(body_dict, ensure_ascii=False)}}\\n"
            f"[{msgid}] PUSH 완료"
        )
        return log_line.encode("utf-8", "ignore")

    def make_random_and_push(node_messenger) -> bytes:
        # DB 기반 메시지는 DB의 파일명(숫자).json을 ID로 사용하여 최소 필드만 전송
        if MSG_ID in DB_DIR_RULES:
            dbdir = _db_dir_for(MSG_ID, __file__)
            # 0304(유인기 pathID)는 1/2/3 시작만 전송(기존 규칙 유지)
            needs_prefix = "123" if MSG_ID == "0304" else None
            ids = _list_numeric_ids(dbdir, needs_prefix)
            logs = []
            for vid in ids:
                wl = TX_FIELD_WHITELIST.get(MSG_ID, [])
                body = {{
                    "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                    "sourceModuleName": "DSC",
                }}
                # ID 필드 결정
                if "inputMissionPackageID" in wl:          body["inputMissionPackageID"] = vid
                if "missionReferencePackageID" in wl:      body["missionReferencePackageID"] = vid
                if "missionPlanID" in wl:                  body["missionPlanID"] = vid
                if "individualMissionPackageID" in wl:     body["individualMissionPackageID"] = vid
                if "pathID" in wl:                         body["pathID"] = vid
                logs.append(make_and_push(body, node_messenger))
            return b"\\n".join(logs) if logs else b""
        else:
            # 비 DB 메시지는 제너레이터 → 필요 시 화이트리스트로 선별
            body = make_msg{msgid}_body()
            # ★ 0102 방어: body가 비거나 dict가 아니면 최소 세트로 채움
            if MSG_ID == "0102":
                if not isinstance(body, dict) or not body:
                    body = {{
                        "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                        "status": 1,  # 정상
                        "sourceModuleName": "DSC",
                    }}
            wl = TX_FIELD_WHITELIST.get(MSG_ID)
            if wl and isinstance(body, dict):
                body = _select_tx_fields(body, wl)
            return make_and_push(body, node_messenger)
    """).rstrip()

    embedded = _embedded_rules_text()

    # 선택: 파일 시스템 기반 일괄 Push 헬퍼 (0201/0301/0304 그대로 유지)
    extra = ""
    if msgid in ("0201","0301","0304"):
        common_extra = textwrap.dedent("""
        import os, glob
        from pathlib import Path

        def _project_root() -> Path:
            return Path(__file__).resolve().parents[3]

        def _now_ms_2000() -> int:
            return _now_ms()
        """).rstrip()

        if msgid == "0201":
            extra = textwrap.dedent("""
            def _get_dir_0201() -> str:
                env_root = os.getenv("KU_MISSION_DB_ROOT")
                if env_root: return str(Path(env_root) / "InputMissionPlan")
                return str(_project_root() / "database" / "InputMissionPlan")

            def _list_ids_0201() -> list:
                ids = []
                for path in glob.glob(os.path.join(_get_dir_0201(), "*.json")):
                    stem = os.path.splitext(os.path.basename(path))[0]
                    if stem.isdigit(): ids.append(int(stem))
                return sorted(ids)

            def make_from_db_and_push(node_messenger) -> bytes | None:
                logs = []
                for pid in _list_ids_0201():
                    body = {"timestamp": _now_ms_2000(), "inputMissionPackageID": pid}
                    logs.append(make_and_push(body, node_messenger))
                return b"\\n".join(logs) if logs else None
            """).rstrip()
        elif msgid == "0301":
            extra = textwrap.dedent("""
            def _get_dir_0301() -> str:
                env_root = os.getenv("KU_MISSION_DB_ROOT")
                if env_root: return str(Path(env_root) / "MissionPlan")
                return str(_project_root() / "database" / "MissionPlan")

            def _list_ids_0301() -> list:
                ids = []
                for path in glob.glob(os.path.join(_get_dir_0301(), "*.json")):
                    stem = os.path.splitext(os.path.basename(path))[0]
                    if stem.isdigit(): ids.append(int(stem))
                return sorted(ids)

            def make_from_db_and_push(node_messenger) -> bytes | None:
                logs = []
                for mid in _list_ids_0301():
                    body = {"timestamp": _now_ms_2000(), "missionPlanID": mid}
                    logs.append(make_and_push(body, node_messenger))
                return b"\\n".join(logs) if logs else None
            """).rstrip()
        elif msgid == "0304":
            extra = textwrap.dedent("""
            def _get_dir_0304() -> str:
                env_root = os.getenv("KU_MISSION_DB_ROOT")
                if env_root: return str(Path(env_root) / "FlightPath")
                return str(_project_root() / "database" / "FlightPath")

            def _list_ids_0304() -> list:
                ids = []
                for path in glob.glob(os.path.join(_get_dir_0304(), "*.json")):
                    stem = os.path.splitext(os.path.basename(path))[0]
                    if stem.isdigit() and stem[0] in "123":
                        ids.append(int(stem))
                return sorted(ids)

            def make_from_db_and_push(node_messenger) -> bytes | None:
                logs = []
                for pid in _list_ids_0304():
                    body = {"timestamp": _now_ms_2000(), "pathID": pid}
                    logs.append(make_and_push(body, node_messenger))
                return b"\\n".join(logs) if logs else None
            """).rstrip()
        extra = "\n\n" + common_extra + "\n\n" + extra

    return "\n\n".join([header, top, embedded, *type_funcs, "", root_block, extra]).rstrip() + "\n"

# ─────────────────────────────────────────────────────────
# 드라이버
# ─────────────────────────────────────────────────────────
def _generate_for_one(msgid: str) -> str:
    msg_dir = os.path.join(LIB_ROOT, f"msg_{msgid}")
    if not os.path.isdir(msg_dir):
        raise SystemExit(f"Message folder not found: {msg_dir}")

    msg_exts = _exts_for_message(msgid)
    reg = Registry(COMMON_DIR)
    msg_types, root_file = _scan_folder(msg_dir, msg_exts, msg_id=msgid)
    if not msg_types:
        raise SystemExit(f"No schema files ({msg_exts}) found under: {msg_dir}")
    reg.add_message_types(msg_types)

    root_tname = _parse_typename_from_filename(root_file) if root_file else (next(iter(msg_types.keys())) if msg_types else None)
    if not root_tname:
        raise SystemExit(f"Root type not found in: {msg_dir}")

    root_tname = pick_better_root(root_tname, reg)
    code = _emit_push_code(msgid, reg, root_tname)

    os.makedirs(PUSH_DIR, exist_ok=True)
    out_path = os.path.join(PUSH_DIR, f"message{msgid}_push.py")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(code)
    return out_path

def _collect_all_msg_ids() -> List[str]:
    ids: List[str] = []
    for name in os.listdir(LIB_ROOT):
        if not name.startswith("msg_"): continue
        mid = name[4:]
        if re.fullmatch(r"\d{4}", mid): ids.append(mid)
    return sorted(ids)

def main(argv: List[str]) -> int:
    #   python -m modules.common.make_message_push
    #   python -m modules.common.make_message_push all
    #   python -m modules.common.make_message_push 0301,0304
    if len(argv) < 2 or argv[1] in ("all", "*"):
        targets = _collect_all_msg_ids()
        if not targets:
            print("No msg_**** folders found under library.", file=sys.stderr)
            return 2
    else:
        parts = [p.strip() for p in argv[1].split(",") if p.strip()]
        for p in parts:
            if not re.fullmatch(r"\d{4}", p):
                print(f"Invalid message id: {p}", file=sys.stderr)
                return 2
        targets = parts

    generated: List[str] = []
    for mid in targets:
        try:
            gen_file = os.path.join(GEN_DIR, f"message{mid}_generator.py")
            if not os.path.exists(gen_file):
                print(f"[warn {mid}] generator not found: {gen_file} (make_message_generator 먼저 실행 권장)", file=sys.stderr)
            outp = _generate_for_one(mid)
            print(f"generated: {outp}")
            generated.append(outp)
        except SystemExit as e:
            print(f"[skip {mid}] {e}", file=sys.stderr)
        except Exception as e:
            print(f"[error {mid}] {e}", file=sys.stderr)

    return 0 if generated else 1

if __name__ == "__main__":
    sys.exit(main(sys.argv))
