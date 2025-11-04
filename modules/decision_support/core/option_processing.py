"""Decoding and building mission option payloads."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence, Optional

from modules.common.option_codes import (
    DEFAULT_OPTION_CODE_SEQUENCE,
    normalize_option_code,
    option_code_to_label,
)
from .time_utils import now_ms_since_2000


def _safe_int(value: Any) -> int:
    try:
        return int(value)
    except Exception:
        return 0


@dataclass
class OptionRequestDecoder:
    """Parses raw 0901 payloads into normalized option entries."""

    def load(self, raw: bytes | None) -> Any:
        if not raw:
            return None
        try:
            text = raw.decode("utf-8", "ignore")
        except Exception:
            return None
        match = re.search(r"{.*}", text, flags=re.S)
        target = match.group(0) if match else text.strip()
        if not target:
            return None
        try:
            return json.loads(target)
        except Exception:
            return None

    def decode(self, raw: bytes | None) -> list[dict[str, Any]]:
        return self.decode_payload(self.load(raw))

    def decode_payload(self, payload: Any) -> list[dict[str, Any]]:
        if not isinstance(payload, dict):
            return []
        result: list[dict[str, Any]] = []
        for item in payload.get("pendingOptionList") or []:
            if not isinstance(item, dict):
                continue
            try:
                option_id = int(item.get("optionID"))
                mission_plan_id = int(item.get("missionPlanID"))
            except Exception:
                continue
            name_value = item.get("optionName")
            code = normalize_option_code(name_value)
            if code is None:
                continue
            result.append(
                {
                    "optionID": option_id,
                    "missionPlanID": mission_plan_id,
                    "optionName": code,
                    "optionLabel": option_code_to_label(code),
                }
            )
        return result


@dataclass
class OptionPayloadBuilder:
    """Composes 0701 bodies and persists them to disk."""

    db_paths: Any
    templates: Sequence[dict[str, int]] = field(
        default_factory=lambda: [
            {"survivalRate": 0, "timeContraction": 0, "recogEffectiveness": 0},
            {"survivalRate": -1, "timeContraction": 0, "recogEffectiveness": 1},
            {"survivalRate": 0, "timeContraction": 1, "recogEffectiveness": -1},
        ]
    )
    last_error: Optional[Exception] = field(default=None, init=False)

    def build_option_list(self, entries: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
        option_list: list[dict[str, Any]] = []
        templates = list(self.templates) or [
            {"survivalRate": 0, "timeContraction": 0, "recogEffectiveness": 0}
        ]
        fallback = templates[-1]
        default_codes = list(DEFAULT_OPTION_CODE_SEQUENCE) or [1]
        for idx, entry in enumerate(entries):
            metrics = templates[idx] if idx < len(templates) else fallback
            try:
                option_id = int(entry["optionID"])
                mission_plan_id = int(entry["missionPlanID"])
            except Exception:
                continue
            code = normalize_option_code(entry.get("optionName"))
            if code is None:
                code = default_codes[idx] if idx < len(default_codes) else default_codes[-1]
            option_list.append(
                {
                    "optionID": option_id,
                    "optionName": code,
                    "missionPlanID": mission_plan_id,
                    "survivalRate": int(metrics.get("survivalRate", 0)),
                    "timeContraction": int(metrics.get("timeContraction", 0)),
                    "recogEffectiveness": int(metrics.get("recogEffectiveness", 0)),
                    "distance": 50000,
                    "target": 0,
                }
            )
        return option_list

    def build_body(self, entries: Iterable[dict[str, Any]], source: str = "MOB") -> dict[str, Any]:
        option_list = self.build_option_list(entries)
        return {
            "timestamp": now_ms_since_2000(),
            "source": source,
            "autoExecution": False,
            "optionList": option_list,
        }

    def persist_body(self, body: dict[str, Any]) -> Path | None:
        self.last_error = None
        try:
            info = self.db_paths.get_info()
        except Exception:
            info = {}
        scenario_dir = info.get("scenario_dir")
        agency_code = info.get("agency") or os.environ.get("KU_AGENCY_CODE") or "SBC3"
        try:
            if scenario_dir:
                save_dir = Path(scenario_dir) / agency_code / "MissionPlanOptionInfo"
                save_dir.mkdir(parents=True, exist_ok=True)
            else:
                save_dir = self.db_paths.ensure_db_payload("MissionPlanOptionInfo")
        except Exception:
            self.last_error = RuntimeError("failed to determine save directory")
            return None

        next_id = 1
        try:
            existing = [
                int(p.stem)
                for p in save_dir.glob("*.json")
                if p.is_file() and p.stem.isdigit()
            ]
            if existing:
                next_id = max(existing) + 1
        except Exception:
            pass

        output_path = save_dir / f"{next_id}.json"
        try:
            text = json.dumps(body, ensure_ascii=False, indent=2)
            output_path.write_text(text, encoding="utf-8")
            return output_path
        except Exception as exc:
            self.last_error = exc
            return None


__all__ = ["OptionRequestDecoder", "OptionPayloadBuilder"]
