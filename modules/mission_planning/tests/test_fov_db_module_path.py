from pathlib import Path

from modules.mission_planning.MissionPlanner import runtime_settings
from modules.mission_planning.runtime.cache import initial_plan_templates


def test_fov_db_resolves_inside_modules_without_modules_run_collision() -> None:
    project_root = Path(__file__).resolve().parents[3]

    assert runtime_settings._project_root() == project_root
    assert runtime_settings.fov_db_path() == (
        project_root / "modules" / "resource" / "db" / "fov_db_탱크_72_38_6_3.2.csv"
    )
    assert runtime_settings.fov_db_path().is_file()


def test_selected_fov_db_is_loadable() -> None:
    rows = runtime_settings.load_fov_db_rows()

    assert rows
    assert min(row["fov"] for row in rows) >= 3.2


def test_initial_plan_cache_key_changes_with_fov_db_signature(monkeypatch, tmp_path: Path) -> None:
    cmpk = tmp_path / "cmpk.json"
    mrpk = tmp_path / "mrpk.json"
    cmpk.write_text("{}", encoding="utf-8")
    mrpk.write_text("{}", encoding="utf-8")

    monkeypatch.setattr(
        initial_plan_templates,
        "_fov_db_signature",
        lambda: {"pathName": "fov.csv", "size": 1, "sha256": "first"},
    )
    first = initial_plan_templates.make_initial_plan_template_key(
        cmpk_path=cmpk,
        mrpk_path=mrpk,
        runtime_payload={},
        option_code=0,
        trust_input_aircraft=False,
    )
    monkeypatch.setattr(
        initial_plan_templates,
        "_fov_db_signature",
        lambda: {"pathName": "fov.csv", "size": 1, "sha256": "second"},
    )
    second = initial_plan_templates.make_initial_plan_template_key(
        cmpk_path=cmpk,
        mrpk_path=mrpk,
        runtime_payload={},
        option_code=0,
        trust_input_aircraft=False,
    )

    assert first != second
