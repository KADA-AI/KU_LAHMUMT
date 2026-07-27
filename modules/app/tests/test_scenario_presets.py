from __future__ import annotations

import json
from pathlib import Path

import pytest

from modules.app import scenario_presets
from modules.app.scenario_presets import SCENARIO_PRESETS, stage_scenario_preset


def _write_preset_sources(logs_root: Path, preset: dict, *, reference_id: int = 3) -> tuple[Path, Path]:
    source_dir = logs_root / "GeneratedScenario" / str(preset["folder"])
    source_dir.mkdir(parents=True, exist_ok=True)
    input_path = source_dir / str(preset["input_file"])
    reference_path = source_dir / str(preset["reference_file"])
    input_path.write_text(
        json.dumps(
            {
                "timestamp": 1234,
                "inputMissionPackageID": 3,
                "inputMissionPackageType": int(preset["package_type"]),
                "inputMissionList": [],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    reference_path.write_text(
        json.dumps(
            {
                "timestamp": 1234,
                "missionReferencePackageID": reference_id,
                "inputTimestamp": 1234,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return input_path, reference_path


@pytest.mark.parametrize("preset", SCENARIO_PRESETS, ids=lambda item: str(item["key"]))
def test_stage_each_preset_replaces_both_directories(tmp_path: Path, preset: dict) -> None:
    logs_root = tmp_path / "Logs"
    input_source, reference_source = _write_preset_sources(logs_root, preset)
    input_destination = logs_root / "InputMissionPlan"
    reference_destination = logs_root / "MissionReferenceInfo"
    input_destination.mkdir(parents=True)
    reference_destination.mkdir(parents=True)
    (input_destination / "1.json").write_text("old input", encoding="utf-8")
    (input_destination / "old").mkdir()
    (input_destination / "old" / "nested.txt").write_text("old", encoding="utf-8")
    (reference_destination / "1.json").write_text("old reference", encoding="utf-8")

    result = stage_scenario_preset(str(preset["key"]), logs_root=logs_root)

    assert result["package_id"] == 3
    assert [path.name for path in input_destination.iterdir()] == ["3.json"]
    assert [path.name for path in reference_destination.iterdir()] == ["3.json"]
    assert (input_destination / "3.json").read_bytes() == input_source.read_bytes()
    assert (reference_destination / "3.json").read_bytes() == reference_source.read_bytes()


def test_invalid_pair_keeps_existing_destinations_unchanged(tmp_path: Path) -> None:
    logs_root = tmp_path / "Logs"
    preset = SCENARIO_PRESETS[0]
    _write_preset_sources(logs_root, preset, reference_id=4)
    input_destination = logs_root / "InputMissionPlan"
    reference_destination = logs_root / "MissionReferenceInfo"
    input_destination.mkdir(parents=True)
    reference_destination.mkdir(parents=True)
    input_old = input_destination / "1.json"
    reference_old = reference_destination / "1.json"
    input_old.write_bytes(b"old input")
    reference_old.write_bytes(b"old reference")

    with pytest.raises(ValueError, match="패키지 ID"):
        stage_scenario_preset(str(preset["key"]), logs_root=logs_root)

    assert input_old.read_bytes() == b"old input"
    assert reference_old.read_bytes() == b"old reference"
    assert [path.name for path in input_destination.iterdir()] == ["1.json"]
    assert [path.name for path in reference_destination.iterdir()] == ["1.json"]


def test_mid_replacement_failure_restores_both_directories(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    logs_root = tmp_path / "Logs"
    preset = SCENARIO_PRESETS[0]
    _write_preset_sources(logs_root, preset)
    input_destination = logs_root / "InputMissionPlan"
    reference_destination = logs_root / "MissionReferenceInfo"
    input_destination.mkdir(parents=True)
    reference_destination.mkdir(parents=True)
    (input_destination / "1.json").write_bytes(b"old input")
    (reference_destination / "1.json").write_bytes(b"old reference")

    real_clear = scenario_presets._clear_directory_contents
    call_count = 0

    def _fail_second_clear(directory: Path, *, logs_root: Path) -> None:
        nonlocal call_count
        call_count += 1
        if call_count == 2:
            raise OSError("simulated replacement failure")
        real_clear(directory, logs_root=logs_root)

    monkeypatch.setattr(scenario_presets, "_clear_directory_contents", _fail_second_clear)

    with pytest.raises(RuntimeError, match="교체에 실패"):
        stage_scenario_preset(str(preset["key"]), logs_root=logs_root)

    assert (input_destination / "1.json").read_bytes() == b"old input"
    assert (reference_destination / "1.json").read_bytes() == b"old reference"
    assert [path.name for path in input_destination.iterdir()] == ["1.json"]
    assert [path.name for path in reference_destination.iterdir()] == ["1.json"]
