from pathlib import Path

from modules.common.settings_paths import (
    ensure_fusion_settings_file,
    fusion_settings_candidates,
    fusion_settings_runtime_targets,
)


def test_nfusion_settings_stay_in_settings_and_modules(tmp_path: Path) -> None:
    common_dir = tmp_path / "modules" / "common"
    common_dir.mkdir(parents=True)
    source = common_dir / "nFusionSettings.json"
    source.write_text('{"Middleware":{}}', encoding="utf-8")

    legacy_app_ui = tmp_path / "app" / "ui"
    canonical = ensure_fusion_settings_file(
        project_root=tmp_path,
        common_dir=common_dir,
        ds_dir=legacy_app_ui,
    )

    assert canonical == tmp_path / "settings" / "nFusionSettings.json"
    assert canonical.is_file()
    assert (tmp_path / "modules" / "app" / "ui" / "nFusionSettings.json").is_file()
    assert not (tmp_path / "nFusionSettings.json").exists()
    assert not legacy_app_ui.exists()

    candidates = fusion_settings_candidates(
        project_root=tmp_path,
        common_dir=common_dir,
        ds_dir=legacy_app_ui,
    )
    targets = fusion_settings_runtime_targets(
        project_root=tmp_path,
        common_dir=tmp_path / "common",
        ds_dir=legacy_app_ui,
    )
    assert legacy_app_ui / "nFusionSettings.json" not in candidates
    assert legacy_app_ui / "nFusionSettings.json" not in targets
