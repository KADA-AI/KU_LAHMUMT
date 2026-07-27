"""Helpers for latest 0201/0203 input package handling."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping


@dataclass(frozen=True, slots=True)
class InputMessageSpec:
    msg_id: str
    directory_name: str
    package_key: str
    core_keys: tuple[str, ...]


INPUT_MESSAGE_SPECS: dict[str, InputMessageSpec] = {
    "0201": InputMessageSpec(
        msg_id="0201",
        directory_name="InputMissionPlan",
        package_key="inputMissionPackageID",
        core_keys=("inputMissionList", "availableAircraftList"),
    ),
    "0203": InputMessageSpec(
        msg_id="0203",
        directory_name="MissionReferenceInfo",
        package_key="missionReferencePackageID",
        core_keys=("takeOverInfoList", "flightAreaList", "handOverInfoList"),
    ),
}

INPUT_MESSAGE_ORDER = ("0201", "0203")


def normalize_input_msg_id(msg_id: str) -> str:
    return str(msg_id or "").strip().upper()


def get_input_message_spec(msg_id: str) -> InputMessageSpec | None:
    return INPUT_MESSAGE_SPECS.get(normalize_input_msg_id(msg_id))


def payload_has_core_data(msg_id: str, payload: Mapping[str, Any] | None) -> bool:
    spec = get_input_message_spec(msg_id)
    if spec is None or not isinstance(payload, Mapping):
        return False
    return any(bool(payload.get(key)) for key in spec.core_keys)


def prepare_cached_payload_for_file(
    msg_id: str,
    package_id: Any,
    payload: Mapping[str, Any] | None,
) -> tuple[str, int, dict[str, Any]] | None:
    spec = get_input_message_spec(msg_id)
    if spec is None or not payload_has_core_data(msg_id, payload):
        return None
    try:
        package_id_int = int(package_id)
    except Exception:
        return None
    payload_copy = dict(payload or {})
    payload_copy[spec.package_key] = package_id_int
    return spec.directory_name, package_id_int, payload_copy


def extract_payload_source(payload: Mapping[str, Any] | None) -> Any | None:
    if not isinstance(payload, Mapping):
        return None
    source = payload.get("Source") or payload.get("source")
    return source if source else None


def build_input_banner_info(
    db_root: Path,
    *,
    get_latest_package_id: Callable[[str], Any],
    resolve_path_from_cache: Callable[[str, Path], Any],
) -> tuple[str, str]:
    entries: list[str] = []
    tips: list[str] = []
    root = Path(db_root)

    for msg_id in INPUT_MESSAGE_ORDER:
        spec = INPUT_MESSAGE_SPECS[msg_id]
        directory = root / spec.directory_name
        pid = get_latest_package_id(msg_id)
        pid_text = str(pid) if pid is not None else "미수신"

        resolved_path: Path | None = None
        if pid is not None:
            try:
                candidate = directory / f"{pid}.json"
                if candidate.exists():
                    resolved_path = candidate
                else:
                    cached = resolve_path_from_cache(msg_id, directory)
                    if cached and Path(cached).exists():
                        resolved_path = Path(cached)
            except Exception:
                resolved_path = None

        file_label = "파일 없음"
        file_tip = f"{directory} (파일 없음)"
        if resolved_path:
            file_label = f"{resolved_path.parent.name}/{resolved_path.name}"
            file_tip = str(resolved_path)
        elif pid is not None:
            file_label = "미존재"
            file_tip = str(directory / f"{pid}.json")

        entries.append(f"{msg_id}: {pid_text} ({file_label})")
        tips.append(f"{msg_id}: ID={pid_text}, file={file_tip}")

    return " | ".join(entries), "\n".join(tips)
