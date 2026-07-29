from __future__ import annotations

import json
import os
import re
from pathlib import Path

from modules.common import db_paths
from modules.common.version_info import (
    CHANGE_LOG_PATH,
    ReleaseInfo,
    SCENARIO_VERSION_LOG_FILENAME,
    load_release_info,
    write_scenario_version_log,
)


def _first_change_log_entry() -> tuple[str, str, str]:
    """Independently parse the newest ``(date, version)`` from change_log.md.

    The release identity moves with every entry, so tests derive the expected
    value from the file instead of pinning a literal that dies each release.
    """

    for line in CHANGE_LOG_PATH.read_text(encoding="utf-8").splitlines():
        match = re.match(
            r"^(\d{4}-\d{2}-\d{2})(?:\s+(\d{2}:\d{2}(?::\d{2})?))?"
            r"\s+v(\d+(?:\.\d+)+)\s+-",
            line.strip(),
        )
        if match:
            return match.group(1), match.group(3), str(match.group(2) or "")
    raise AssertionError("change_log.md has no valid release line")


def test_current_release_comes_from_change_log() -> None:
    expected_date, expected_version, expected_time = _first_change_log_entry()
    release = load_release_info()

    assert release.version == expected_version
    assert release.display_version == f"v{expected_version}"
    assert release.release_date == expected_date
    assert release.release_time == expected_time
    expected_modified_at = " ".join(
        part for part in (expected_date, expected_time) if part
    )
    assert release.code_label == f"{expected_modified_at} 최종 수정본"


def test_missing_release_metadata_fails_closed_to_current_release(tmp_path: Path) -> None:
    release = load_release_info(tmp_path / "missing_change_log.md")

    assert release.version == "1.4.0"
    assert release.release_date == "2026-07-24"
    assert release.release_time == ""


def test_release_time_is_optional_and_parsed_when_present(tmp_path: Path) -> None:
    change_log = tmp_path / "change_log.md"
    change_log.write_text(
        "2026-07-29 15:45:30 v1.5.28 - header metadata\n",
        encoding="utf-8",
    )

    release = load_release_info(change_log)

    assert release.display_version == "v1.5.28"
    assert release.release_date == "2026-07-29"
    assert release.release_time == "15:45:30"
    assert release.modified_at == "2026-07-29 15:45:30"


def test_scenario_version_log_is_utf8_json_with_run_identity(tmp_path: Path) -> None:
    scenario_dir = tmp_path / "Scenario_2026-07-24T164255"
    release = ReleaseInfo(
        version="1.4.0",
        release_date="2026-07-24",
        change_summary="test release",
    )

    path = write_scenario_version_log(
        scenario_dir,
        timestamp_ms=123456,
        scenario_iso="2026-07-24T164255",
        agency="SBC3",
        release=release,
        recorded_at="2026-07-24T16:42:55.000+09:00",
    )

    assert path == scenario_dir / SCENARIO_VERSION_LOG_FILENAME
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["version"] == "1.4.0"
    assert payload["currentCode"] == "2026-07-24 최종 수정본"
    assert payload["summary"] == (
        "현재 돌린 코드: 2026-07-24 최종 수정본 / 버전: 1.4.0"
    )
    assert payload["recordedAt"] == "2026-07-24T16:42:55.000+09:00"
    assert payload["scenario"] == {
        "timestampMs": 123456,
        "iso": "2026-07-24T164255",
        "agency": "SBC3",
    }
    assert not list(scenario_dir.glob(".*.tmp"))


def test_reactivation_keeps_one_file_and_updates_the_current_run_version(
    tmp_path: Path,
) -> None:
    scenario_dir = tmp_path / "Scenario_repeat"
    first = ReleaseInfo("1.3.81", "2026-07-22", "first")
    current = ReleaseInfo("1.4.0", "2026-07-24", "current")

    write_scenario_version_log(
        scenario_dir,
        timestamp_ms=1,
        scenario_iso="repeat",
        agency="SBC3",
        release=first,
        recorded_at="2026-07-21T00:00:00+09:00",
    )
    path = write_scenario_version_log(
        scenario_dir,
        timestamp_ms=1,
        scenario_iso="repeat",
        agency="SBC3",
        release=current,
        recorded_at="2026-07-22T00:00:00+09:00",
    )

    assert [item.name for item in scenario_dir.iterdir()] == [
        SCENARIO_VERSION_LOG_FILENAME
    ]
    assert json.loads(path.read_text(encoding="utf-8"))["version"] == "1.4.0"


def test_activate_scenario_writes_version_before_publishing_info(
    tmp_path: Path,
    monkeypatch,
) -> None:
    base_root = tmp_path / "Logs"
    info_path = tmp_path / "settings" / "current_scenario.json"
    old_cache = dict(db_paths._cache)
    env_names = (
        db_paths.ENV_DB_ROOT,
        db_paths.ENV_SCENARIO_ROOT,
        db_paths.ENV_SCENARIO_BASE_ROOT,
    )
    old_env = {name: os.environ.get(name) for name in env_names}
    monkeypatch.setattr(db_paths, "DEFAULT_SCENARIO_BASE", base_root)
    monkeypatch.setattr(db_paths, "INFO_PATH", info_path)
    db_paths._cache.update(
        {
            "mtime": None,
            "db_root": None,
            "scenario_dir": None,
            "timestamp_ms": None,
            "iso": None,
            "agency": None,
            "source": None,
            "base_root": str(base_root),
        }
    )

    try:
        info = db_paths.activate_scenario(838195438233, copy_legacy=False)
        version_path = Path(info["version_log"])
        persisted_info = json.loads(info_path.read_text(encoding="utf-8"))

        assert version_path.parent == Path(info["scenario_dir"])
        assert version_path.name == SCENARIO_VERSION_LOG_FILENAME
        assert version_path.is_file()
        assert persisted_info["version_log"] == str(version_path)
        _, expected_version, _ = _first_change_log_entry()
        assert (
            json.loads(version_path.read_text(encoding="utf-8"))["version"]
            == expected_version
        )
    finally:
        db_paths._cache.clear()
        db_paths._cache.update(old_cache)
        for name, value in old_env.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def test_version_log_failure_does_not_block_scenario_activation(monkeypatch) -> None:
    def fail_writer(*_args, **_kwargs):
        raise OSError("read-only test root")

    monkeypatch.setattr(db_paths, "write_scenario_version_log", fail_writer)
    path, error = db_paths._record_scenario_version_safe(
        Path("Scenario_test"),
        timestamp_ms=1,
        scenario_iso="test",
        agency="SBC3",
    )

    assert path is None
    assert error == "OSError: read-only test root"
