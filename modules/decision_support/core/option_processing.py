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


def _target_id_value(value: Any) -> int | None:
    if isinstance(value, dict):
        value = value.get("targetID", value.get("TargetID"))
    try:
        target_id = int(value)
    except Exception:
        return None
    return target_id if target_id > 0 else None


def _target_id_list(
    meta: dict[str, Any],
    *,
    allow_count_fallback: bool = True,
) -> list[dict[str, int]]:
    raw_list = (
        meta.get("targetIDList")
        or meta.get("TargetIDList")
        or meta.get("targetIDs")
        or meta.get("targetIds")
        or meta.get("targetList")
        or meta.get("attackTargets")
    )
    ids: list[int] = []
    if isinstance(raw_list, list):
        for raw in raw_list:
            value = _target_id_value(raw)
            if value is not None and value not in ids:
                ids.append(value)

    if not ids:
        for key in (
            "targetID",
            "TargetID",
            "targetId",
            "target",
            "primaryTarget",
        ):
            value = _target_id_value(meta.get(key))
            if value is not None and value not in ids:
                ids.append(value)

    if not ids and allow_count_fallback:
        target_count = meta.get("targetCount", meta.get("attackTargetCount"))
        try:
            count = int(target_count)
        except Exception:
            count = 0
        if meta.get("attack") and count <= 0:
            count = 1
        if count > 0:
            ids.extend(range(1, count + 1))

    return [{"targetID": target_id} for target_id in ids]


def _target_meta(entry: dict[str, Any]) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    raw_meta = entry.get("optionMeta")
    if isinstance(raw_meta, dict):
        meta.update(raw_meta)
    for key in (
        "targetIDList",
        "TargetIDList",
        "targetIDs",
        "targetIds",
        "targetList",
        "targetID",
        "TargetID",
        "targetId",
        "target",
        "targetCount",
        "attackTargetCount",
        "attackTargets",
        "primaryTarget",
        "attack",
    ):
        if key in entry and key not in meta:
            meta[key] = entry[key]
    return meta


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
            entry = {
                "optionID": option_id,
                "missionPlanID": mission_plan_id,
                "optionName": code,
                "optionLabel": option_code_to_label(code),
            }
            meta = item.get("optionMeta")
            if isinstance(meta, dict):
                entry["optionMeta"] = dict(meta)
            else:
                inline_meta = {
                    key: item[key]
                    for key in (
                        "targetIDList",
                        "TargetIDList",
                        "targetIDs",
                        "targetIds",
                        "targetID",
                        "TargetID",
                        "targetId",
                        "target",
                        "targetCount",
                        "attackTargetCount",
                        "attackTargets",
                        "primaryTarget",
                        "attack",
                    )
                    if key in item
                }
                if inline_meta:
                    entry["optionMeta"] = inline_meta
            result.append(entry)
        return result


@dataclass
class OptionPayloadBuilder:
    """Composes 0701 bodies and persists them to disk."""

    db_paths: Any
    templates: Sequence[dict[str, int]] = field(
        default_factory=lambda: [
            {"survivalRate": 0, "timeContraction": 0, "recogEffectiveness": 0},
            {"survivalRate": -1, "timeContraction": -1, "recogEffectiveness": 1},
            {"survivalRate": 0, "timeContraction": 1, "recogEffectiveness": -1},
        ]
    )
    last_error: Optional[Exception] = field(default=None, init=False)

    def _target_ids_from_mission_plan(self, mission_plan_id: int) -> list[dict[str, int]]:
        """Recover real attack target IDs from generated 0301/0302 artifacts.

        PendingOption (0901) does not define option metadata in the ICD, so
        target metadata attached internally can be removed by serialization.
        The generated attack missions remain authoritative and preserve the
        actual target IDs.
        """

        try:
            plan_path = self.db_paths.get_db_subpath(
                "MissionPlan", f"{int(mission_plan_id)}.json"
            )
            plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
        except Exception:
            return []

        ids: list[int] = []
        for aircraft in plan.get("aircraftList") or []:
            if not isinstance(aircraft, dict):
                continue
            package_id = _safe_int(aircraft.get("individualMissionPackageID"))
            if package_id <= 0:
                continue
            try:
                imp_path = self.db_paths.get_db_subpath(
                    "IndividualMissionPlan", f"{package_id}.json"
                )
                imp = json.loads(Path(imp_path).read_text(encoding="utf-8"))
            except Exception:
                continue
            for mission in imp.get("individualMissionList") or []:
                if not isinstance(mission, dict):
                    continue
                info = mission.get("individualMissionInfo")
                if not isinstance(info, dict):
                    continue
                # 0302 individualMissionType=2 is the ICD attack mission.
                if _safe_int(info.get("individualMissionType")) != 2:
                    continue
                target_id = _target_id_value(info.get("targetID"))
                if target_id is not None and target_id not in ids:
                    ids.append(target_id)
        return [{"targetID": target_id} for target_id in ids]

    def build_option_list(self, entries: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
        option_list: list[dict[str, Any]] = []
        templates = list(self.templates) or [
            {"survivalRate": 0, "timeContraction": 0, "recogEffectiveness": 0}
        ]
        fallback = templates[-1]
        default_codes = list(DEFAULT_OPTION_CODE_SEQUENCE) or [1]
        for idx, entry in enumerate(entries):
            meta = _target_meta(entry)
            if meta.get("attack"):
                metrics = {"survivalRate": -1, "timeContraction": -1, "recogEffectiveness": 0}
                code = 2
            else:
                metrics = templates[idx] if idx < len(templates) else fallback
                code = normalize_option_code(entry.get("optionName"))
                if code is None:
                    code = default_codes[idx] if idx < len(default_codes) else default_codes[-1]
            try:
                option_id = int(entry["optionID"])
                mission_plan_id = int(entry["missionPlanID"])
            except Exception:
                continue
            target_ids = _target_id_list(meta, allow_count_fallback=False)
            if code == 2:
                generated_target_ids = self._target_ids_from_mission_plan(mission_plan_id)
                if generated_target_ids:
                    target_ids = generated_target_ids
            if not target_ids:
                # Retain the legacy count fallback only when neither metadata
                # nor generated attack missions expose a concrete target ID.
                target_ids = _target_id_list(meta)
            recommend = len(option_list) == 0
            option_list.append(
                {
                    "optionID": option_id,
                    "recommend": recommend,
                    "optionName": code,
                    "missionPlanID": mission_plan_id,
                    "survivalRate": int(metrics.get("survivalRate", 0)),
                    "timeContraction": int(metrics.get("timeContraction", 0)),
                    "recogEffectiveness": int(metrics.get("recogEffectiveness", 0)),
                    "distance": 50000,
                    "targetIDListN": len(target_ids),
                    "targetIDList": target_ids,
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
