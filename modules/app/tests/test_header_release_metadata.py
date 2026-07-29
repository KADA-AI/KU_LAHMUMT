from __future__ import annotations

import inspect
from types import SimpleNamespace

from modules.app.ui.main_window import MainWindow


def test_change_log_rows_accept_timestamped_and_legacy_entries(tmp_path) -> None:
    change_log = tmp_path / "change_log.md"
    change_log.write_text(
        "2026-07-29 15:49:00 v1.5.28 - timestamped\n\n"
        "2026-07-28 v1.5.27 - legacy\n",
        encoding="utf-8",
    )
    harness = SimpleNamespace(_version_notes_path=change_log)

    rows = MainWindow._load_change_log_rows(harness)

    assert rows == [
        ("2026-07-29 15:49:00", "v1.5.28", "timestamped"),
        ("2026-07-28", "v1.5.27", "legacy"),
    ]


def test_header_keeps_only_developer_action_and_unifies_release_metadata() -> None:
    source = inspect.getsource(MainWindow._build_header_bar)

    assert "HeaderReleasePanel" in source
    assert "HeaderVersionValue" in source
    assert "HeaderModifiedDate" in source
    assert "HeaderModifiedTime" in source
    assert 'QPushButton("개발관리"' in source
    assert 'QPushButton("참고 문서"' not in source
    assert 'QPushButton("FOV DB 선택"' not in source
    assert "HeaderFovDbLabel" not in source
    assert "top.addWidget(title" in source
    assert "top.addWidget(release_panel" in source
    assert "title_col" not in source
