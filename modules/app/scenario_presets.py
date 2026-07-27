# -*- coding: utf-8 -*-
"""Quick-action scenario staging for the KU dashboard."""

from __future__ import annotations

import json
import shutil
import tempfile
from pathlib import Path
from typing import Any


SCENARIO_PRESETS: tuple[dict[str, Any], ...] = (
    {
        "key": "anti_armor_strike",
        "button_label": "대기갑 타격",
        "display_name": "대기갑 항공타격임무",
        "folder": "대기갑항공타격작전",
        "input_file": "0201_대기갑항공타격작전.json",
        "reference_file": "0203_대기갑항공타격작전.json",
        "package_type": 1,
    },
    {
        "key": "ground_mobility",
        "button_label": "지상기동 보장",
        "display_name": "지상작전부대 기동여건 보장 작전",
        "folder": "지상작전부대_기동여건_보장",
        "input_file": "0201_지상작전부대_기동여건_보장.json",
        "reference_file": "0203_지상작전부대_기동여건_보장.json",
        "package_type": 2,
    },
    {
        "key": "air_assault_cover",
        "button_label": "공중강습 엄호",
        "display_name": "공중강습작전부대 엄호 작전",
        # The generated asset uses "공습강습" in its actual directory name.
        "folder": "공습강습작전부대_엄호",
        "input_file": "0201_공습강습작전부대_엄호.json",
        "reference_file": "0203_공습강습작전부대_엄호.json",
        "package_type": 3,
    },
    {
        "key": "critical_facility_defense",
        "button_label": "중요시설 방호",
        "display_name": "항공지원작전-중요시설 방호",
        "folder": "항공지원작전-중요시설방호",
        "input_file": "0201_항공지원작전-중요시설방호.json",
        "reference_file": "0203_항공지원작전-중요시설방호.json",
        "package_type": 4,
    },
    {
        "key": "urban_operations",
        "button_label": "도시지역 작전",
        "display_name": "도시지역 작전",
        "folder": "도시지역_작전",
        "input_file": "0201_도시지역_작전.json",
        "reference_file": "0203_도시지역_작전.json",
        "package_type": 5,
    },
)

_PRESET_BY_KEY = {str(preset["key"]): preset for preset in SCENARIO_PRESETS}


def _load_json_object(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"시나리오 파일을 찾을 수 없습니다: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8-sig"))
    except Exception as exc:
        raise ValueError(f"JSON을 읽을 수 없습니다: {path.name} ({exc})") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"JSON 최상위 값이 객체가 아닙니다: {path.name}")
    return payload


def _positive_integer_field(payload: dict[str, Any], field: str, path: Path) -> int:
    value = payload.get(field)
    if isinstance(value, bool):
        raise ValueError(f"{path.name}의 {field}가 올바른 정수가 아닙니다.")
    try:
        integer = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path.name}에 {field}가 없습니다.") from exc
    if integer <= 0 or isinstance(value, float) and not value.is_integer():
        raise ValueError(f"{path.name}의 {field}가 올바른 양의 정수가 아닙니다.")
    return integer


def _clear_directory_contents(directory: Path, *, logs_root: Path) -> None:
    """Remove only direct children of an approved Logs subdirectory."""

    if directory.is_symlink():
        raise RuntimeError(f"심볼릭 링크 대상은 교체할 수 없습니다: {directory}")
    resolved_logs_root = logs_root.resolve()
    resolved_directory = directory.resolve()
    if resolved_directory.parent != resolved_logs_root:
        raise RuntimeError(f"허용되지 않은 대상 경로입니다: {resolved_directory}")
    if directory.exists() and not directory.is_dir():
        raise RuntimeError(f"대상 경로가 폴더가 아닙니다: {directory}")
    directory.mkdir(parents=True, exist_ok=True)
    for child in directory.iterdir():
        if child.is_symlink() or child.is_file():
            child.unlink()
        elif child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def stage_scenario_preset(
    preset_key: str,
    *,
    logs_root: Path | str,
) -> dict[str, Any]:
    """Validate and copy one generated 0201/0203 pair into Logs staging.

    Source assets are never moved. Both destination folders are backed up and
    restored together if replacement fails, preventing a half-applied pair.
    """

    preset = _PRESET_BY_KEY.get(str(preset_key))
    if preset is None:
        raise KeyError(f"알 수 없는 임무 유형입니다: {preset_key}")

    logs_dir = Path(logs_root).resolve()
    generated_root = logs_dir / "GeneratedScenario"
    source_dir = generated_root / str(preset["folder"])
    input_source = source_dir / str(preset["input_file"])
    reference_source = source_dir / str(preset["reference_file"])

    input_payload = _load_json_object(input_source)
    reference_payload = _load_json_object(reference_source)
    input_id = _positive_integer_field(
        input_payload,
        "inputMissionPackageID",
        input_source,
    )
    reference_id = _positive_integer_field(
        reference_payload,
        "missionReferencePackageID",
        reference_source,
    )
    if input_id != reference_id:
        raise ValueError(
            "0201/0203 패키지 ID가 일치하지 않습니다: "
            f"{input_id} != {reference_id}"
        )

    expected_type = int(preset["package_type"])
    actual_type = _positive_integer_field(
        input_payload,
        "inputMissionPackageType",
        input_source,
    )
    if actual_type != expected_type:
        raise ValueError(
            f"{input_source.name}의 임무 유형이 예상값과 다릅니다: "
            f"{actual_type} != {expected_type}"
        )

    destinations = (
        ("InputMissionPlan", input_source),
        ("MissionReferenceInfo", reference_source),
    )
    logs_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".scenario_preset_", dir=logs_dir) as temp_text:
        temp_root = Path(temp_text)
        staged_root = temp_root / "staged"
        backup_root = temp_root / "backup"
        staged_files: dict[str, Path] = {}
        destination_existed: dict[str, bool] = {}

        # Complete and verify both temporary copies before changing live data.
        for directory_name, source_path in destinations:
            staged_dir = staged_root / directory_name
            staged_dir.mkdir(parents=True, exist_ok=True)
            staged_path = staged_dir / f"{input_id}.json"
            shutil.copy2(source_path, staged_path)
            if staged_path.read_bytes() != source_path.read_bytes():
                raise OSError(f"임시 복사 검증에 실패했습니다: {source_path.name}")
            staged_files[directory_name] = staged_path

            destination_dir = logs_dir / directory_name
            destination_existed[directory_name] = destination_dir.exists()
            if destination_dir.exists():
                if not destination_dir.is_dir():
                    raise RuntimeError(f"대상 경로가 폴더가 아닙니다: {destination_dir}")
                shutil.copytree(
                    destination_dir,
                    backup_root / directory_name,
                    dirs_exist_ok=True,
                    symlinks=True,
                )

        try:
            for directory_name, _source_path in destinations:
                destination_dir = logs_dir / directory_name
                _clear_directory_contents(destination_dir, logs_root=logs_dir)
                shutil.copy2(
                    staged_files[directory_name],
                    destination_dir / f"{input_id}.json",
                )
        except Exception as replace_exc:
            rollback_errors: list[str] = []
            for directory_name, _source_path in destinations:
                destination_dir = logs_dir / directory_name
                try:
                    _clear_directory_contents(destination_dir, logs_root=logs_dir)
                    backup_dir = backup_root / directory_name
                    if destination_existed.get(directory_name) and backup_dir.exists():
                        shutil.copytree(
                            backup_dir,
                            destination_dir,
                            dirs_exist_ok=True,
                            symlinks=True,
                        )
                except Exception as rollback_exc:
                    rollback_errors.append(f"{directory_name}: {rollback_exc}")
            detail = f"임무 파일 교체에 실패했습니다: {replace_exc}"
            if rollback_errors:
                detail += " / 복구 실패: " + "; ".join(rollback_errors)
            raise RuntimeError(detail) from replace_exc

    return {
        "preset_key": str(preset["key"]),
        "display_name": str(preset["display_name"]),
        "package_id": input_id,
        "input_path": logs_dir / "InputMissionPlan" / f"{input_id}.json",
        "reference_path": logs_dir / "MissionReferenceInfo" / f"{input_id}.json",
    }


__all__ = ["SCENARIO_PRESETS", "stage_scenario_preset"]
