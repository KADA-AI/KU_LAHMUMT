import os
from typing import Any

ROLE_TO_SOURCE = {
    "mission": "MMR",
    "mission_planning": "MMR",
    "monitoring": "MSM",
    "decision": "MOB",
    "decision_support": "MOB",
    "info": "INF",
    "info_manage": "INF",
    "integration": "INT",
}


def get_default_source_code(default: str = "DSC") -> str:
    role = (os.environ.get("KU_ROLE") or "").strip().lower()
    return ROLE_TO_SOURCE.get(role, default)


def override_source_fields(payload: Any, source: str) -> Any:
    if not source:
        return payload
    if isinstance(payload, dict):
        for key in list(payload.keys()):
            lower = key.lower()
            if lower in ("source", "requestmodulename", "sourcemodulename"):
                payload[key] = source
            else:
                override_source_fields(payload[key], source)
    elif isinstance(payload, list):
        for item in payload:
            override_source_fields(item, source)
    return payload
