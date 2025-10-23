# modules/common/make_message_receiver.py
# msg_**** 스키마를 스캔해 C# 모델(IFusionReceive[T])용 receiver 모듈을 자동 생성합니다.
# 사용법:
#   python -m modules.common.make_message_receiver            # 모든 msg_**** 대상 일괄 생성
#   python -m modules.common.make_message_receiver 0301       # 단일 생성
#   python -m modules.common.make_message_receiver 0201,0301  # 다중 생성
# 생성 위치:
#   modules/common/receive/messageXXXX_receiver.py
#
# 설계 포인트
#  - 스키마(.nftype/.nfpsh)에서 필드(원시/리스트/컴포지트)를 읽어, CLR 객체→dict 변환 함수를 타입별로 생성
#  - 대/소문자 안전 접근(_get)과 source 별칭(requestModuleName/Source) 처리
#  - DB 기반(0201/0203)은 수신 시, ID에 해당하는 DB JSON을 읽어 그대로 notify (없으면 최소 바디)
#  - received_db.set_received_XXXX(data)가 있으면 호출(없으면 무시)
#  - 0000은 예외적으로 timestamp/source/messageID만 뽑아 알림(샘플과 동일 동작)

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
RECV_DIR   = os.path.join(HERE, "receive")

EXTS = (".nfpsh", ".nftype", ".txt")
IGNORE_NFPSH_FOR = {"0201","0203","0301","0302","0303","0304"}  # 생성기/푸시와 동일
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
    return order  # 하위 먼저, 루트 마지막

# ─────────────────────────────────────────────────────────
# 수신 루트 타입명/클래스명 오버라이드
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
    "0503": "MissionResult",   # ✔ MissionResults 아님
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

# DB 규칙 / 받으면서 DB JSON 표시할 메시지
DB_DIR_RULES = {
    "0201": "InputMissionPlan",
    "0203": "FlightReferenceInfo",
    "0301": "MissionPlan",
    "0302": "IndividualMissionPlan",
    "0303": "UAVFlightPlan",
    "0304": "FlightPath",
}
DB_FETCH_ON_RECEIVE = {"0201", "0203"}

TX_FIELD_WHITELIST = {
    "0201": ["timestamp", "inputMissionPackageID"],
    "0203": ["timestamp", "missionReferencePackageID"],
    "0301": ["timestamp", "missionPlanID"],
    "0302": ["timestamp", "individualMissionPackageID"],
    "0303": ["timestamp", "pathID"],
    "0304": ["timestamp", "pathID"],
}
ID_FIELD_FOR = {
    mid: next((f for f in fl if f not in ("timestamp","source","Source","requestModuleName")), None)
    for mid, fl in TX_FIELD_WHITELIST.items()
}

# ─────────────────────────────────────────────────────────
# 코드 생성기(타입별 CLR→dict)
# ─────────────────────────────────────────────────────────
def _emit_to_dict_lines(t: str, reg: Registry) -> List[str]:
    """타입 t 에 대한 CLR→dict 함수 본문 줄 목록 생성"""
    fields = reg.get_typedef(t) or []
    lines: List[str] = []
    lines.append(f"def _to_dict_{t}(obj):")
    lines.append("    d = {}")
    if not fields:
        lines.append("    return d")
        return lines

    for fname, ftype in fields:
        norm  = _normalize_type_token(ftype)
        inner = _list_inner(norm)

        # source / Source / requestModuleName 별칭 처리 (문자열만)
        if _normalize_type_token(norm) == "string" and fname in ("source","Source","requestModuleName"):
            pas = fname[:1].upper() + fname[1:]
            lines.append(
                "    _sval = _get(obj, "
                f"'{fname}', '{pas}', "
                "'source','Source','Source','Source','requestModuleName','RequestModuleName')"
            )
            lines.append(f"    if _sval is not None and _sval != '': d['{fname}'] = str(_sval)")
            continue

        if inner:  # List<...>
            base = _normalize_type_token(inner)
            pas  = fname[:1].upper()+fname[1:]
            lines.append(f"    _coll = _get(obj, '{fname}', '{pas}') or []")
            if _is_primitive(base):
                conv = "int" if base in ('uint32','int32','uint64','int64','uint','int','ulong','long','short','ushort','byte','sbyte') else \
                       "float" if base in ('float32','float64','float','double') else \
                       "bool" if base == "bool" else \
                       "str"  if base == "string" else None
                lines.append("    if _coll:")
                if conv:
                    lines.append(f"        d['{fname}'] = [{conv}(x) for x in _coll]")
                else:
                    lines.append(f"        d['{fname}'] = [x for x in _coll]")
            else:
                lines.append("    if _coll:")
                lines.append(f"        d['{fname}'] = [_to_dict_{base}(it) for it in _coll]")
        else:
            base = _normalize_type_token(norm)
            pas  = fname[:1].upper()+fname[1:]
            if _is_primitive(base):
                getter = f"_get(obj, '{fname}', '{pas}')"
                if base in ('uint32','int32','uint64','int64','uint','int','ulong','long','short','ushort','byte','sbyte'):
                    lines.append(f"    _v = {getter}")
                    lines.append(f"    if _v is not None: d['{fname}'] = int(_v)")
                elif base in ('float32','float64','float','double'):
                    lines.append(f"    _v = {getter}")
                    lines.append(f"    if _v is not None: d['{fname}'] = float(_v)")
                elif base == 'bool':
                    lines.append(f"    _v = {getter}")
                    lines.append(f"    if _v is not None: d['{fname}'] = bool(_v)")
                elif base == 'string':
                    lines.append(f"    _v = {getter}")
                    lines.append(f"    if _v is not None and _v != '': d['{fname}'] = str(_v)")
                else:
                    lines.append(f"    _v = {getter}")
                    lines.append(f"    if _v is not None: d['{fname}'] = _v")
            else:
                # Composite
                lines.append(f"    _sub = _get(obj, '{fname}', '{pas}')")
                lines.append(f"    if _sub is not None: d['{fname}'] = _to_dict_{base}(_sub)")
    lines.append("    return d")
    return lines

def _emit_receiver_code(msgid: str, reg: Registry, root_type: str) -> str:
    order = _topo_types(root_type, reg)

    # 헤더/유틸/상수(최상위)
    header = (
        f"# modules/common/receive/message{msgid}_receiver.py\n"
        f"# auto-generated at {datetime.datetime.now(datetime.timezone.utc).isoformat()}\n\n"
        "from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone\n"
        f"from nFusion.Model.msg_{msgid} import *            # C# 모델\n"
        "from nFusion.Model.CommonType import *             # 공통 타입\n"
        "from .database import received_db\n"
        "from receive_center import notify\n"
        "import json, traceback, sys, os, importlib\n\n"
        "# 대/소문자 안전 접근\n"
        "_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)\n\n"
    )

    embedded_consts = textwrap.dedent(f"""
    # ── Embedded rules (TX/DB 공용) ──────────────────────────────────────────
    TX_FIELD_WHITELIST = {repr(TX_FIELD_WHITELIST)}
    DB_DIR_RULES        = {repr(DB_DIR_RULES)}
    DB_FETCH_ON_RECEIVE = {repr(DB_FETCH_ON_RECEIVE)}
    ID_FIELD_FOR        = {repr(ID_FIELD_FOR)}

    def _project_root_for_recv_file(__file_path: str):
        from pathlib import Path
        return Path(__file_path).resolve().parents[3]

    def _db_dir_for(msgid: str, __file_path: str) -> str:
        from pathlib import Path
        env_root = os.getenv("KU_MISSION_DB_ROOT")
        name = DB_DIR_RULES.get(msgid)
        if not name:
            return str(_project_root_for_recv_file(__file_path))
        if env_root:
            return str(Path(env_root) / name)
        return str(_project_root_for_recv_file(__file_path) / "database" / name)

    def _try_save_received(msgid: str, data_obj):
        try:
            fn = getattr(received_db, f"set_received_{{msgid}}")
            fn(data_obj)
        except Exception:
            pass

    def _try_read_db_body(msgid: str, data_obj):
        \"\"\"DB_FETCH_ON_RECEIVE에 포함된 메시지는 ID 필드로 DB JSON을 찾아 반환(없으면 None).\"\"\"
        try:
            if msgid not in DB_FETCH_ON_RECEIVE:
                return None
            id_field = ID_FIELD_FOR.get(msgid)
            if not id_field:
                return None
            # 객체에서 ID 값을 추출(대/소문자 안전)
            _val = _get(data_obj, id_field, id_field[:1].upper()+id_field[1:])
            if _val is None:
                return None
            vid = int(_val)
            dbdir = _db_dir_for(msgid, __file__)
            fpath = os.path.join(dbdir, f"{{vid}}.json")
            print(f"[{{msgid}}] DB 참조! ({{fpath}})")
            if os.path.exists(fpath):
                with open(fpath, "r", encoding="utf-8") as f:
                    return json.load(f)
            return None
        except Exception:
            return None
    """).strip() + "\n\n"

    # 타입별 to_dict 함수
    type_funcs: List[str] = []
    for t in order:
        type_funcs.append("\n".join(_emit_to_dict_lines(t, reg)))

    # 루트/브리지
    cs_root = CLASS_NAME_OVERRIDES.get(msgid, root_type)
    bridge_block = ""
    if cs_root not in order:
        fields = reg.get_typedef(root_type) or []
        bl: List[str] = []
        bl.append(f"def _to_dict_{cs_root}(obj):")
        bl.append("    d = {}")
        for fname, ftype in fields:
            norm  = _normalize_type_token(ftype)
            inner = _list_inner(norm)
            pas   = fname[:1].upper()+fname[1:]
            if inner:
                base = _normalize_type_token(inner)
                bl.append(f"    _coll = _get(obj, '{fname}', '{pas}') or []")
                if _is_primitive(base):
                    conv = "int" if base in ('uint32','int32','uint64','int64','uint','int','ulong','long','short','ushort','byte','sbyte') else \
                           "float" if base in ('float32','float64','float','double') else \
                           "bool" if base == 'bool' else \
                           "str"  if base == 'string' else None
                    bl.append("    if _coll:")
                    if conv:
                        bl.append(f"        d['{fname}'] = [{conv}(x) for x in _coll]")
                    else:
                        bl.append(f"        d['{fname}'] = [x for x in _coll]")
                else:
                    bl.append("    if _coll:")
                    bl.append(f"        d['{fname}'] = [_to_dict_{base}(it) for it in _coll]")
            else:
                base = _normalize_type_token(norm)
                if _is_primitive(base):
                    if base in ('uint32','int32','uint64','int64','uint','int','ulong','long','short','ushort','byte','sbyte'):
                        bl.append(f"    _v = _get(obj, '{fname}', '{pas}')")
                        bl.append(f"    if _v is not None: d['{fname}'] = int(_v)")
                    elif base in ('float32','float64','float','double'):
                        bl.append(f"    _v = _get(obj, '{fname}', '{pas}')")
                        bl.append(f"    if _v is not None: d['{fname}'] = float(_v)")
                    elif base == 'bool':
                        bl.append(f"    _v = _get(obj, '{fname}', '{pas}')")
                        bl.append(f"    if _v is not None: d['{fname}'] = bool(_v)")
                    elif base == 'string':
                        if fname in ('source','Source','requestModuleName'):
                            bl.append(
                                "    _sval = _get(obj, "
                                f"'{fname}', '{pas}', "
                                "'source','Source','Source','Source','requestModuleName','RequestModuleName')"
                            )
                            bl.append(f"    if _sval is not None and _sval != '': d['{fname}'] = str(_sval)")
                        else:
                            bl.append(f"    _v = _get(obj, '{fname}', '{pas}')")
                            bl.append(f"    if _v is not None and _v != '': d['{fname}'] = str(_v)")
                    else:
                        bl.append(f"    _v = _get(obj, '{fname}', '{pas}')")
                        bl.append(f"    if _v is not None: d['{fname}'] = _v")
                else:
                    bl.append(f"    _sub = _get(obj, '{fname}', '{pas}')")
                    bl.append(f"    if _sub is not None: d['{fname}'] = _to_dict_{base}(_sub)")
        bl.append("    return d")
        bridge_block = "\n".join(bl) + "\n\n"

    # 0000 최소 바디
    special_0000 = ""
    if msgid == "0000":
        special_0000 = textwrap.dedent("""
        def _to_dict_RequestData_min(obj):
            return {
                "timestamp": _get(obj, "timestamp","Timestamp"),
                "source":    _get(obj, "source","Source","requestModuleName","RequestModuleName","Source","Source"),
                "messageID": _get(obj, "messageID","MessageID"),
            }
        """).strip() + "\n\n"

    # Receiver 클래스
    class_name = "ResponseReceiver_0000" if msgid == "0000" else f"{cs_root}Receiver_{msgid}"
    ns_name    = class_name

    receive_block = []
    receive_block.append(f"class {class_name}(IFusionReceive[{cs_root}], IsLocal, IsSingletone):")
    receive_block.append(f"    \"\"\"{msgid} {cs_root} 메시지 수신 리시버\"\"\"")
    receive_block.append(f"    __namespace__ = \"{ns_name}\"")
    receive_block.append("")
    receive_block.append(f"    def Receive(self, data: {cs_root}, src):")
    receive_block.append("        try:")
    receive_block.append(f"            _try_save_received('{msgid}', data)")
    receive_block.append("")
    receive_block.append(f"            body = _try_read_db_body('{msgid}', data)")
    receive_block.append("            if body is None:")
    if msgid == "0000":
        receive_block.append("                body = _to_dict_RequestData_min(data)")
    else:
        receive_block.append(f"                body = _to_dict_{cs_root}(data)")
    receive_block.append("")
    receive_block.append(f"            notify(\"{msgid}\", json.dumps(body, ensure_ascii=False).encode(\"utf-8\",\"ignore\"))")
    receive_block.append("")
    receive_block.append("        except Exception:")
    receive_block.append(f"            print(\"[ERROR][Receive-{msgid}] traceback ↓↓↓\")")
    receive_block.append("            traceback.print_exc(file=sys.stderr)")
    receive_block.append("")

    parts = [
        header,
        embedded_consts,
        "\n\n".join(type_funcs) + "\n\n",
        bridge_block,
        special_0000,
        "\n".join(receive_block),
    ]
    return "".join(parts)

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
    code = _emit_receiver_code(msgid, reg, root_tname)

    os.makedirs(RECV_DIR, exist_ok=True)
    out_path = os.path.join(RECV_DIR, f"message{msgid}_receiver.py")
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
    #   python -m modules.common.make_message_receiver
    #   python -m modules.common.make_message_receiver all
    #   python -m modules.common.make_message_receiver 0301,0304
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
