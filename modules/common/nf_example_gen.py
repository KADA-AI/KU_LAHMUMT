# modules/common/nf_example_gen.py

# 사용법:
#   python modules/common/nf_example_gen.py 0000
#   (선택) 환경변수 KU_RULES_DEBUG=1 로 디버그 로그

from __future__ import annotations
import os, re, sys, json, random
from typing import Dict, List, Tuple, Optional, Any
from datetime import datetime, timezone

# ─────────────────────────────────────────────────────────
# 경로/확장자
# ─────────────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
LIB_ROOT = os.path.join(HERE, "nFusion_MessageLIbrary")          # 철자 주의: LIbrary
COMMON_DIR = os.path.join(LIB_ROOT, "CommonType")
RULES_DIR = os.path.join(LIB_ROOT, "rules")
FIELD_PROFILES_PATH     = os.path.join(RULES_DIR, "field_profiles.txt")
CONDITIONS_PATH         = os.path.join(RULES_DIR, "conditions.txt")
ID_POLICIES_PATH        = os.path.join(RULES_DIR, "id_policies.txt")
ID_STATE_PATH           = os.path.join(RULES_DIR, "id_state.json")

EXTS = (".nfpsh", ".nftype", ".txt")

IGNORE_NFPSH_FOR = {"0201","0203","0301","0302","0303","0304"}

DEBUG = os.getenv("KU_RULES_DEBUG", "0") == "1"
def dlog(*a): 
    if DEBUG: print("[DEBUG]", *a, file=sys.stderr)

# ─────────────────────────────────────────────────────────
# 시간 기준(2000 epoch ms) — push/receive와 일치
# ─────────────────────────────────────────────────────────
_EPOCH2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH2000).total_seconds() * 1000)

# ─────────────────────────────────────────────────────────
# 기본 타입 / 매핑
# ─────────────────────────────────────────────────────────
PRIMS = {"uint","ulong","int","float","double","bool","string",
         "uint32","uint64","int32","int64","float32","float64"}

TYPE_MAP = {
    "uint": "uint32",
    "ulong": "uint64",
    "int": "int32",
    "float": "float32",
    "double": "float64",
}

SOURCE_ENUM_DEFAULT = ["DSC","IDM","MSM","MMR","UCC","MOB","CSP"]

Field = Tuple[str, str]            # (name, type_token)
TypeDef = Dict[str, List[Field]]   # { type_name: [(field, token), ...] }

# 한 줄 선언 파서: "Type name" 또는 "List<...> name" / 주석 허용
DECL_RE = re.compile(r'^(?P<type>List<[^>]+>|[A-Za-z_][\w]*(?:\.[A-Za-z_][\w]*)?)\s+(?P<name>[A-Za-z_][\w]*)\s*(?:[#/].*)?$')
MSGID_RE     = re.compile(r'^\s*#\s*(?:MissionID|MessageID)\s+(\d{4})\b', re.I)

# ─────────────────────────────────────────────────────────
# 규칙 파일 파서
#  - field_profiles.txt
#  - message_bindings.txt
#  - conditions.txt (간단 적용: REQUIRED_IF / USE_PROFILE / CONSTRAINT)
#  - id_policies.txt (+ id_state.json)
# ─────────────────────────────────────────────────────────
Profile = Dict[str, Any]                 # {TYPE, UNIT, MIN, MAX, SIZE, ENUM(list), EPOCH, DEFAULT, VALUE, ...}
Profiles = Dict[str, Profile]            # {profile_name: Profile}
Conditions = Dict[str, list]             # {"REQUIRED_IF":[...], "USE_PROFILE":[...], "CONSTRAINT":[...]}

def _exts_for_message(msg_id: str) -> tuple:
    """메시지별로 스키마 파일 확장자 허용범위를 결정한다."""
    return (".nftype",) if msg_id in IGNORE_NFPSH_FOR else EXTS

def _parse_kv_list(s: str) -> Dict[str, Any]:
    """
    'KEY=VALUE, KEY2=VALUE2, ENUM=a,b,c, ...' 같은 라인을
    다음 KEY=가 시작되는 지점까지를 VALUE로 취급하여 안전하게 파싱.
    값 내부에 콤마(,)가 있어도 안전.
    """
    out: Dict[str, Any] = {}
    # KEY= 패턴들을 모두 찾고, 각 KEY의 값 범위를 다음 KEY 시작 직전까지로 잡는다.
    pat = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)\s*=')
    matches = list(pat.finditer(s))
    for i, m in enumerate(matches):
        key = m.group(1).upper()
        val_start = m.end()
        val_end = matches[i+1].start() if (i+1) < len(matches) else len(s)
        val = s[val_start:val_end].strip()
        # 끝에 달라붙은 쉼표/공백 제거
        if val.endswith(","):
            val = val[:-1].rstrip()
        out[key] = val
    return out

def _coerce_enum_list(enum_raw: str) -> List[Any]:
    """
    ENUM=DSC,IDM,...   혹은 ENUM=0,1,2 또는 0:초기화,1:대기
    → label이 있으면 콜론 앞 값만 사용
    → '0','1' 등은 int로 변환
    """
    items: List[Any] = []
    for token in [x.strip() for x in enum_raw.split(",") if x.strip()]:
        if ":" in token:
            token = token.split(":",1)[0].strip()
        if token.isdigit() or (token.startswith("-") and token[1:].isdigit()):
            items.append(int(token))
        else:
            items.append(token)
    return items

def load_field_profiles(path: str) -> Profiles:
    profiles: Profiles = {}
    if not os.path.exists(path): 
        dlog("field_profiles.txt not found, skip.")
        return profiles
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            t = line.strip()
            if not t or t.startswith("#"): 
                continue
            m = re.match(r'^PROFILE\s+([A-Za-z0-9_]+)\s*=\s*(.+)$', t)
            if not m: 
                continue
            name, rhs = m.group(1), m.group(2)
            kv = _parse_kv_list(rhs)
            if "ENUM" in kv:
                kv["ENUM"] = _coerce_enum_list(kv["ENUM"])
            profiles[name] = kv
    dlog(f"Loaded profiles: {list(profiles.keys())}")
    return profiles

def load_conditions(path: str) -> Conditions:
    cond: Conditions = {"REQUIRED_IF": [], "USE_PROFILE": [], "CONSTRAINT": []}
    if not os.path.exists(path):
        dlog("conditions.txt not found, skip.")
        return cond

    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            # ★★★ 인라인 주석 제거: '... REQUIRED_IF X==2   # Loiter' → '... REQUIRED_IF X==2'
            hash_pos = line.find('#')
            if hash_pos != -1:
                line = line[:hash_pos].rstrip()
            if not line:
                continue

            m = re.match(
                r'^WHEN(?:\s+(?P<prefix>.*?))?\s*field=(?P<field>[A-Za-z0-9_.]+)\s+USE_PROFILE\s+(?P<prof>[A-Za-z0-9_]+)\s*$',
                line, re.I
            )
            if m:
                prefix    = m.group('prefix') or ''
                fieldpath = m.group('field')
                prof      = m.group('prof')
                msg = None
                mm = re.search(r'message=(\d{4})', prefix, re.I)
                if mm: msg = mm.group(1)
                pred = None
                pp = re.search(r'IF\s+(.+)$', prefix, re.I)
                if pp: pred = pp.group(1).strip()
                cond["USE_PROFILE"].append({"message": msg, "field": fieldpath, "profile": prof, "pred": pred})
                continue

            m = re.match(
                r'^WHEN(?:\s+(?P<prefix>.*?))?\s*field=(?P<field>[A-Za-z0-9_.]+)\s+REQUIRED_IF\s+(?P<pred>.+)$',
                line, re.I
            )
            if m:
                prefix    = m.group('prefix') or ''
                fieldpath = m.group('field')
                pred      = m.group('pred')
                msg = None
                mm = re.search(r'message=(\d{4})', prefix, re.I)
                if mm: msg = mm.group(1)
                cond["REQUIRED_IF"].append({"message": msg, "field": fieldpath, "pred": pred.strip()})
                continue

            m = re.match(r'^CONSTRAINT\s+([A-Za-z0-9_.]+)\s*:\s*(.+)$', line, re.I)
            if m:
                target, expr = m.group(1), m.group(2).strip()
                cond["CONSTRAINT"].append({"target": target, "expr": expr})
                continue

    dlog(f"Loaded conditions: {{'REQUIRED_IF': {len(cond['REQUIRED_IF'])}, 'USE_PROFILE': {len(cond['USE_PROFILE'])}, 'CONSTRAINT': {len(cond['CONSTRAINT'])}}}")
    return cond

# ID Policies
class IDSeq:
    def __init__(self, start: int, step: int=1):
        self.start = int(start)
        self.step = int(step)

def load_id_policies(path: str) -> Dict[str, IDSeq]:
    seqs: Dict[str, IDSeq] = {}
    if not os.path.exists(path): 
        dlog("id_policies.txt not found, skip.")
        return seqs
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"): 
                continue
            m = re.match(r'^SEQUENCE\s+([A-Za-z0-9_.]+)\s*=\s*(.+)$', line)
            if not m: 
                continue
            name, rhs = m.group(1), m.group(2)
            kv = _parse_kv_list(rhs)
            start = kv.get("START", "1")
            step  = kv.get("STEP", "1")
            seqs[name] = IDSeq(int(start), int(step))
    dlog(f"Loaded sequences: {list(seqs.keys())}")
    return seqs

def load_id_state(path: str) -> Dict[str, int]:
    if not os.path.exists(path): 
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

def save_id_state(path: str, state: Dict[str, int]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)

# 전역 규칙 로드
FIELD_PROFILES: Profiles     = load_field_profiles(FIELD_PROFILES_PATH)
CONDITIONS: Conditions       = load_conditions(CONDITIONS_PATH)
ID_POLICIES: Dict[str, IDSeq]= load_id_policies(ID_POLICIES_PATH)
ID_STATE: Dict[str, int]     = load_id_state(ID_STATE_PATH)

# 프로필 이름 → base 키 (언더스코어 이전) 매핑(자동 매칭용)
def _profile_base(name: str) -> str:
    return name.split("_", 1)[0].lower()

PROFILE_BASE_INDEX: Dict[str, str] = {}
for pname in FIELD_PROFILES.keys():
    PROFILE_BASE_INDEX.setdefault(_profile_base(pname), pname)

# 메시지 기본 바인딩(없을 때 자동)
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

# ─────────────────────────────────────────────────────────
# 스키마 스캔/파서
# ─────────────────────────────────────────────────────────
def list_schema_files(folder: str, exts: tuple = EXTS) -> List[str]:
    if not os.path.isdir(folder): 
        return []
    return [
        os.path.join(folder, f)
        for f in os.listdir(folder)
        if f.lower().endswith(exts)
    ]

def parse_typename_from_filename(path: str) -> str:
    return os.path.splitext(os.path.basename(path))[0]

def read_file(path: str) -> List[str]:
    with open(path, "r", encoding="utf-8") as f:
        return f.readlines()

def parse_fields(lines: List[str]) -> List[Field]:
    fields: List[Field] = []
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue
        m = DECL_RE.match(line)
        if not m: 
            continue
        tkn = m.group("type").strip()
        name = m.group("name").strip()
        fields.append((name, tkn))
    return fields

def scan_types_in_folder(folder: str, exts: tuple = EXTS, msg_id: Optional[str] = None) -> Tuple[TypeDef, Optional[str]]:
    """폴더 내 모든 스키마 파일을 읽어 type registry 구성. 루트 후보 파일(주석 MissionID) 반환."""
    typedefs: TypeDef = {}
    root_candidate: Optional[str] = None
    for fp in list_schema_files(folder, exts):
        lines = read_file(fp)
        # 파일 내부에 '# MissionID 0201' 또는 '# MessageID 0201' 코멘트가 있으면 루트 후보로
        for l in lines:
            m = MSGID_RE.match(l)
            if m and (msg_id is None or m.group(1) == msg_id):
                root_candidate = fp
                break
        tname = parse_typename_from_filename(fp)
        flds = parse_fields(lines)
        if flds:
            typedefs[tname] = flds
    if not root_candidate:
        mission_named = [
            p for p in list_schema_files(folder, exts)
            if os.path.basename(p).lower().startswith("mission")
        ]
        root_candidate = (
            mission_named[0] if mission_named
            else (list_schema_files(folder, exts)[0] if list_schema_files(folder, exts) else None)
        )
    return typedefs, root_candidate

def normalize_type_token(t: str) -> str:
    t = t.strip()
    if t.startswith("List<") and t.endswith(">"):
        inner = t[5:-1].strip()
        return f"List<{normalize_type_token(inner)}>"
    if "." in t:
        ns, base = t.split(".", 1)
        return base if ns == "CommonType" else t
    return TYPE_MAP.get(t, t)

def is_primitive(t: str) -> bool:
    base = TYPE_MAP.get(t, t)
    return base in ("uint32","int32","uint64","int64","float32","float64","bool","string")

def _field_count(tdef: Optional[List[Field]]) -> int:
    return 0 if not tdef else len(tdef)

def _all_primitive(tdef: Optional[List[Field]]) -> bool:
    if not tdef: return True
    for _, t in tdef:
        tt = normalize_type_token(t)
        if tt.startswith("List<"):
            return False
        if not is_primitive(tt):
            return False
    return True

# ─────────────────────────────────────────────────────────
# 레지스트리
# ─────────────────────────────────────────────────────────
class Registry:
    def __init__(self, common_dir: str):
        self.common: TypeDef = {}
        self.messages: TypeDef = {}
        self._load_common(common_dir)
    def _load_common(self, folder: str):
        tds, _ = scan_types_in_folder(folder, exts=EXTS, msg_id=None)
        self.common.update(tds)
    def add_message_types(self, tds: TypeDef):
        self.messages.update(tds)
    def get_typedef(self, name: str) -> Optional[List[Field]]:
        if name in self.messages: return self.messages[name]
        if name in self.common:   return self.common[name]
        return None
    def all_message_types(self) -> List[str]:
        return list(self.messages.keys())

def _get_ci(d: dict, key: str):
    """dict d에서 key(대소문자 무시) 값 조회"""
    kl = key.lower()
    for k, v in d.items():
        if k.lower() == kl:
            return v
    return None

def _create_required_field(parent: dict, field_name: str, ctx: GenCtx) -> None:
    # 이미 있으면 스킵 (대소문자 무시)
    for k in parent.keys():
        if k.lower() == field_name.lower():
            return

    # ★★★ 실제 키 이름은 lowerCamelCase로 맞춘다 (LoiterProperty → loiterProperty)
    def _to_lc_camel(name: str) -> str:
        return name[:1].lower() + name[1:] if name else name

    key_to_set = _to_lc_camel(field_name)

    # 타입 추정 생성
    guess_type = field_name
    val = gen_value(key_to_set, guess_type, ctx, parent, depth=0)

    # 최소 스텁
    if isinstance(val, dict) and len(val) == 0:
        lname = field_name.lower()
        if lname == "loiterproperty":
            dir_prof = FIELD_PROFILES.get("direction_v1")
            direction = _gen_from_profile("direction", "uint32", dir_prof) if dir_prof else random.randint(0, 2)
            val = {
                "radius": random.randint(100, 1000),
                "direction": direction,            # 0/1/2
                "time": random.randint(30, 900),   # sec
                "speed": round(random.uniform(5.0, 100.0), 6),
            }
        elif lname.endswith("orientation"):
            val = gen_value(key_to_set, guess_type, ctx, parent, depth=0)
        elif lname.endswith("list"):
            val = []

    parent[key_to_set] = val
    

def _eval_predicate_in_parent(parent: dict, pred: Optional[str]) -> bool:
    if not pred:
        return True
    # ★★★ 혹시 남아있을 인라인 주석 제거 (이중 안전장치)
    if '#' in pred:
        pred = pred.split('#', 1)[0].strip()
    m = re.match(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*==\s*(.+?)\s*$', pred)
    if not m:
        return False
    fname, raw = m.group(1), m.group(2)
    if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
        expect = int(raw)
    else:
        expect = raw.strip().strip('"\'')
    cur = _get_ci(parent, fname)  # 대소문자 무시 조회
    return (cur == expect)

# ─────────────────────────────────────────────────────────
# 루트 보정: 얇은 루트 → Data/Info/Root 선호, 래퍼 언랩, 최대필드 타입
# ─────────────────────────────────────────────────────────
def _unwrap_single_wrapper(root_tname: str, reg: Registry) -> str:
    tdef = reg.get_typedef(root_tname)
    if not tdef or len(tdef) != 1:
        return root_tname
    _, inner = tdef[0]
    inner = normalize_type_token(inner)
    if inner.startswith("List<") and inner.endswith(">"):
        base = normalize_type_token(inner[5:-1])
        return base if reg.get_typedef(base) else root_tname
    base = normalize_type_token(inner)
    return base if reg.get_typedef(base) else root_tname

def pick_better_root(root_tname: str, reg: Registry) -> str:
    cand = root_tname
    tdef = reg.get_typedef(cand)

    if _field_count(tdef) <= 2 and _all_primitive(tdef):
        for suf in ("Data","Info","Root"):
            alt = root_tname + suf
            if reg.get_typedef(alt):
                cand = alt
                tdef = reg.get_typedef(cand)
                break

    cand = _unwrap_single_wrapper(cand, reg)
    tdef = reg.get_typedef(cand)

    if _field_count(tdef) <= 2:
        keys = reg.all_message_types()
        if keys:
            richest = max(keys, key=lambda k: _field_count(reg.get_typedef(k)))
            if _field_count(reg.get_typedef(richest)) > _field_count(tdef):
                cand = richest
    return cand

# ─────────────────────────────────────────────────────────
# 규칙 적용 유틸
# ─────────────────────────────────────────────────────────
def _normalize_name(s: str) -> str:
    return re.sub(r'[^a-z0-9]', '', s.lower())

def _find_profile_for_field(msg_id: str, field_name: str) -> Optional[Profile]:
    """
    message_bindings 없이 동작:
      1) field_profiles.txt 에서 자동 매칭(프로필 base명)
      2) DEFAULT_BINDINGS로 지정된 기본 프로필
      3) 필드명 특수 처리(소스/시스템모드 등)
    """
    # 1) 프로필 base명 자동 매칭 (예: latitude ← latitude_deg)
    base = _normalize_name(field_name)
    if base in PROFILE_BASE_INDEX:
        pname = PROFILE_BASE_INDEX[base]
        prof = FIELD_PROFILES.get(pname)
        if prof:
            return prof

    # 2) DEFAULT_BINDINGS
    if field_name in DEFAULT_BINDINGS:
        prof = FIELD_PROFILES.get(DEFAULT_BINDINGS[field_name])
        if prof:
            return prof

    # 3) 특수 필드 안전망
    if field_name == "source":
        return FIELD_PROFILES.get("source_node3", {"TYPE":"string","SIZE":"3","ENUM":["DSC","IDM","MSM","MMR","UCC","MOB","CSP"]})

    if field_name == "systemMode":
        # 프로필이 없어도 0~3 중 랜덤 나오도록 보장
        return {"TYPE":"uint32","MIN":"0","MAX":"3","ENUM":[0,1,2,3]}

    return None

def _coerce_num(v: Any, is_float: bool=False) -> Any:
    if v is None: return None
    try:
        return float(v) if is_float else int(v)
    except Exception:
        return v

# ID 시퀀스 발급
def _next_id(seq_name: str) -> int:
    seq = ID_POLICIES.get(seq_name)
    if seq is None:
        # 정책 없으면 랜덤으로라도 생성
        n = random.randint(1, 1_000_000_000)
        dlog(f"[ID] No policy for {seq_name}, random={n}")
        return n
    last = ID_STATE.get(seq_name, seq.start - seq.step)
    nxt = last + seq.step
    if nxt < seq.start: nxt = seq.start
    ID_STATE[seq_name] = nxt
    return nxt

# PathID 선택 (메시지/aircraftID 힌트)
def _alloc_path_id(context: Dict[str, Any], msg_id: str) -> int:
    # 메시지 기반 우선
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
    # aircraftID 힌트
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
    # fallback
    return _next_id("WaypointID")

# ─────────────────────────────────────────────────────────
# 값 생성기(규칙 적용)
# ─────────────────────────────────────────────────────────
class GenCtx:
    def __init__(self, msg_id: str, reg: Registry):
        self.msg_id = msg_id
        self.reg = reg

def _gen_from_profile(field_name: str, base_type: str, prof: Profile) -> Any:
    # VALUE 우선
    if "VALUE" in prof:
        val = prof["VALUE"]
        if isinstance(val, str) and val.isdigit(): val = int(val)
        return val

    # ENUM
    enum_vals = prof.get("ENUM")
    if enum_vals:
        pick = random.choice(list(enum_vals))
        # 숫자 문자열이면 int로
        if isinstance(pick, str) and pick.isdigit():
            return int(pick)
        return pick

    # 숫자 범위
    min_v = prof.get("MIN")
    max_v = prof.get("MAX")
    size  = prof.get("SIZE")
    epoch = (prof.get("EPOCH") or "").lower()

    # timestamp 특례
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
        # 프로필에 ENUM=0,1 등 있을 수 있으나 여기선 단순
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

def gen_value(field_name: str, type_token: str, ctx: GenCtx, obj_context: Optional[Dict[str, Any]]=None, depth: int=0) -> Any:
    if depth > 12:
        return None
    t = normalize_type_token(type_token)

    # 리스트
    if t.startswith("List<") and t.endswith(">"):
        inner = t[5:-1]
        # 예시 2개 생성
        return [gen_value(field_name, inner, ctx, obj_context, depth+1) for _ in range(2)]

    base = TYPE_MAP.get(t, t)
    prof = _find_profile_for_field(ctx.msg_id, field_name)

    lname = field_name.lower()

    if field_name == "systemMode" and not prof and base in ("uint32","int32","uint64","int64"):
        return random.randint(0, 3)
    # ID 정책 필드 우선 처리
    lname = field_name.lower()
    if lname == "missionplanid":
        return _next_id("MissionPlanID")
    if lname == "individualmissionpackageid":
        return _next_id("IndividualMissionPackageID")
    if lname == "individualmissionid":
        return _next_id("IndividualMissionID")
    if lname == "pathid":
        # context를 보고 manned/uav 시퀀스 선택
        base_val = _alloc_path_id(obj_context or {}, ctx.msg_id)
        return base_val
    if lname in ("waypointid","priormissionid","inputmissionpackageid","inputmissionid",
                 "missionreferencepackageid","targetid","flightareaid","prohibitedareaid",
                 "behaviortreefileid"):
        return _next_id(field_name[0].upper() + field_name[1:])  # 시퀀스 이름 대소일치 가정

    # 프로필이 있으면 그 규칙으로 생성
    if prof:
        val = _gen_from_profile(field_name, base, prof)
        if val is not None:
            return val

    # 기본 생성
    if base in ("uint32","int32","uint64","int64"):
        if lname == "timestamp":
            return now_ms_2000()
        if lname in ("health","currentindividualmissionprogress"):
            return random.randint(0,100)
        return random.randint(0, 1000) if base.startswith("u") else random.randint(-1000, 1000)

    if base in ("float32","float64"):
        if lname in ("fuel","lowerlimit","upperlimit"):
            return round(random.uniform(0.0, 100.0), 6)
        # 좌표 범위 자동 추정
        if lname == "latitude":
            return round(random.uniform(-90.0, 90.0), 6)
        if lname == "longitude":
            return round(random.uniform(-180.0, 180.0), 6)
        return round(random.uniform(0.0, 100.0), 6)

    if base == "bool":
        return random.choice([True, False])

    if base == "string":
        if field_name == "source":
            return random.choice(SOURCE_ENUM_DEFAULT)
        import string
        alpha = string.ascii_uppercase + string.digits
        return "".join(random.choice(alpha) for _ in range(8))

    # 컴포지트: 재귀
    typedef = ctx.reg.get_typedef(base)
    if typedef is None:
        return {}
    obj: Dict[str, Any] = {}
    for fname, ftype in typedef:
        obj[fname] = gen_value(fname, ftype, ctx, obj, depth+1)
    return obj

# ─────────────────────────────────────────────────────────
# 조건/제약 적용(간단)
# ─────────────────────────────────────────────────────────
def _walk(obj: Any, fn):
    if isinstance(obj, dict):
        fn(obj)
        for v in obj.values():
            _walk(v, fn)
    elif isinstance(obj, list):
        for it in obj:
            _walk(it, fn)

def _apply_constraints(obj: dict):
    # 예: GimbalYawLimits: leftLimit<=rightLimit
    def visit(node: dict):
        # 간단히 키가 gimbalYawLimits / GimbalYawLimits 둘 다 탐색
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

# (REQUIRED_IF / USE_PROFILE)는 생성 후 보완이 필요한데,
# 여기서는 이미 스키마에 해당 필드가 존재하면 값 보정만 시도 (존재하지 않으면 생성 스킵)
def _lookup_field_any(obj: Any, field: str) -> List[Tuple[dict, str]]:
    """
    객체 트리에서 주어진 필드명과 일치하는 (부모, 키) 목록 반환
    """
    hits: List[Tuple[dict, str]] = []
    def visit(node: dict):
        for k in list(node.keys()):
            if k == field or k.lower() == field.lower():
                hits.append((node, k))
    _walk(obj, visit)
    return hits

def _eval_simple_predicate(obj: dict, pred: Optional[str]) -> bool:
    # 매우 단순한 "Field==number" / "Field==string" 만 지원
    if not pred:
        return True
    m = re.match(r'^\s*([A-Za-z_][A-Za-z0-9_]*)\s*==\s*(.+?)\s*$', pred)
    if not m:
        return False
    fname, raw = m.group(1), m.group(2)
    # 값 해석
    if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
        expect = int(raw)
    else:
        expect = raw.strip().strip('"\'')
    # 객체에서 첫 매칭 필드값 찾기
    hits = _lookup_field_any(obj, fname)
    if not hits: 
        return False
    for parent, key in hits:
        if parent.get(key) == expect:
            return True
    return False

def _apply_use_profile(obj: dict, msg_id: str, ctx: GenCtx):
    rules = CONDITIONS.get("USE_PROFILE", [])
    for r in rules:
        if r.get("message") and r["message"] != msg_id:
            continue
        field_path = r["field"]
        prof_name  = r["profile"]
        pred       = r.get("pred")
        if not _eval_simple_predicate(obj, pred):
            continue
        # 대상 필드가 존재하면 프로필로 값 재생성
        # (중첩 경로 "A.B"는 마지막 토큰만 값 보정)
        tokens = field_path.split(".")
        target = tokens[-1]
        hits = _lookup_field_any(obj, target)
        prof = FIELD_PROFILES.get(prof_name)
        if not prof: 
            continue
        for parent, key in hits:
            # 타입 추정이 어려우므로 현재 값/프로필 타입으로 생성
            cur = parent.get(key)
            base_type = None
            if isinstance(cur, bool): base_type = "bool"
            elif isinstance(cur, int): base_type = "uint32"
            elif isinstance(cur, float): base_type = "float32"
            elif isinstance(cur, str): base_type = "string"
            else:
                # 알 수 없으면 숫자 취급
                base_type = prof.get("TYPE", "uint32")
                base_type = TYPE_MAP.get(base_type, base_type)
            parent[key] = _gen_from_profile(key, base_type, prof) or cur

def _apply_required_if(obj: dict, msg_id: str, ctx: GenCtx):
    """
    REQUIRED_IF:
      - 조건이 '참'이면: 필드가 없으면 생성, 있으면 유지
      - 조건이 '거짓'이면: 필드가 있으면 삭제
    ※ 조건 평가는 '필드의 부모 dict'에서만 수행(컨텍스트 민감).
    """
    rules = CONDITIONS.get("REQUIRED_IF", [])
    for r in rules:
        if r.get("message") and r["message"] != msg_id:
            continue
        field_path = r["field"]
        pred       = r.get("pred")
        target     = field_path.split(".")[-1]  # relatedMission.priorMissionID → priorMissionID

        # 1) 먼저 전체 트리를 돌며 'target'이 존재하는 경우: 조건에 따라 삭제/유지
        hits = _lookup_field_any(obj, target)
        for parent, key in hits:
            cond_ok = _eval_predicate_in_parent(parent, pred)
            if not cond_ok:
                try:
                    del parent[key]
                except Exception:
                    pass

        # 2) 다시 전체 dict 노드들을 순회: 조건이 '참'인데 'target'이 없으면 생성
        def ensure_visit(node: dict):
            cond_ok = _eval_predicate_in_parent(node, pred)
            if cond_ok:
                # node에 target(대소문자 무시)이 없으면 생성
                for k in node.keys():
                    if k.lower() == target.lower():
                        break
                else:
                    _create_required_field(node, target, ctx)
        _walk(obj, ensure_visit)

# ─────────────────────────────────────────────────────────
# 생성 메인
# ─────────────────────────────────────────────────────────
def choose_root_typename(root_file: Optional[str]) -> Optional[str]:
    return parse_typename_from_filename(root_file) if root_file else None

def generate_example(msg_id: str) -> Dict[str, object]:
    msg_dir = os.path.join(LIB_ROOT, f"msg_{msg_id}")
    if not os.path.isdir(msg_dir):
        raise FileNotFoundError(f"Message folder not found: {msg_dir}")

    reg = Registry(COMMON_DIR)

    # ★ 추가: 메시지별 확장자 규칙 적용
    msg_exts = _exts_for_message(msg_id)
    dlog(f"Schema extensions for msg_{msg_id}: {msg_exts}")

    # ★ 변경: 지정된 확장자로만 스캔
    msg_types, root_file = scan_types_in_folder(msg_dir, exts=msg_exts, msg_id=msg_id)
    if not msg_types:
        raise FileNotFoundError(f"No schema files ({msg_exts}) found under: {msg_dir}")

    reg.add_message_types(msg_types)

    root_tname = choose_root_typename(root_file)
    if not root_tname:
        root_tname = next(iter(msg_types.keys()))
    root_tname = pick_better_root(root_tname, reg)
    # ★ 0201 전용 힌트 적용
    root_tname = pick_root_by_signature(msg_id, reg, root_tname)
    dlog(f"Root chosen: {root_tname}")

    root_def = reg.get_typedef(root_tname)
    if not root_def:
        raise RuntimeError(f"Root type not found: {root_tname}")

    ctx = GenCtx(msg_id, reg)

    example: Dict[str, object] = {}
    for fname, ftype in root_def:
        example[fname] = gen_value(fname, ftype, ctx, example, depth=0)

    _apply_use_profile(example, msg_id, ctx)
    _apply_required_if(example, msg_id, ctx)
    _apply_constraints(example)

    if ID_POLICIES:
        save_id_state(ID_STATE_PATH, ID_STATE)

    return example

# 0201 루트가 반드시 가져야 하는 키 세트
ROOT_SIG = {
    "0201": {"timestamp","inputMissionPackageID","inputMissionPackageType",
             "mainSensor","availableAircraftList","inputMissionList"}
}

def pick_root_by_signature(msg_id: str, reg: Registry, fallback: str) -> str:
    need = ROOT_SIG.get(msg_id)
    if not need:
        return fallback
    best = fallback
    best_hits = -1
    for tname in reg.all_message_types():
        tdef = reg.get_typedef(tname)
        if not tdef: 
            continue
        fields = {fname for fname, _ in tdef}
        hits = len(fields & need)
        # 시그니처를 더 많이 만족하는 타입을 선택
        if hits > best_hits:
            best_hits = hits
            best = tname
    return best


def main():
    if len(sys.argv) < 2:
        print("Usage: python modules/common/nf_example_gen.py <message_id>\n  e.g., python modules/common/nf_example_gen.py 0000", file=sys.stderr)
        sys.exit(2)
    msg_id = sys.argv[1]
    if not re.match(r"^\d{4}$", msg_id):
        print("message_id must be 4 digits like 0000, 0203", file=sys.stderr)
        sys.exit(2)

    try:
        ex = generate_example(msg_id)
        print(json.dumps(ex, ensure_ascii=False, indent=2))
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
