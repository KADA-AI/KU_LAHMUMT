from __future__ import annotations

import json
import os
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def fail(message: str) -> None:
    raise AssertionError(message)


def expect_equal(label: str, actual: object, expected: object) -> None:
    if actual != expected:
        fail(f"{label} changed: expected {expected!r}, got {actual!r}")


def expect_path(label: str, actual: object, expected: Path) -> None:
    actual_path = Path(actual)
    if actual_path != expected:
        fail(f"{label} changed: expected {expected}, got {actual_path}")


def assert_source_contains(rel_path: str, *snippets: str) -> None:
    path = PROJECT_ROOT / rel_path
    text = path.read_text(encoding="utf-8", errors="ignore")
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing manifest source markers: {missing!r}")


@contextmanager
def patched_db_root(db_root: Path) -> Iterator[None]:
    from modules.common import db_paths

    original_active = db_paths.get_active_db_root
    original_subpath = db_paths.get_db_subpath
    db_paths.get_active_db_root = lambda: db_root  # type: ignore[assignment]
    db_paths.get_db_subpath = lambda *parts: db_root.joinpath(*parts)  # type: ignore[assignment]
    try:
        yield
    finally:
        db_paths.get_active_db_root = original_active  # type: ignore[assignment]
        db_paths.get_db_subpath = original_subpath  # type: ignore[assignment]


@contextmanager
def patched_env(name: str, value: str) -> Iterator[None]:
    old_value = os.environ.get(name)
    os.environ[name] = value
    try:
        yield
    finally:
        if old_value is None:
            os.environ.pop(name, None)
        else:
            os.environ[name] = old_value


def check_db_root_contract() -> None:
    from modules.common import db_paths

    expect_path("db_paths PROJECT_ROOT", db_paths.PROJECT_ROOT, PROJECT_ROOT)
    expect_path("legacy DB root", db_paths.LEGACY_DB_ROOT, PROJECT_ROOT / "temp" / "database")
    expect_path("default scenario base", db_paths.DEFAULT_SCENARIO_BASE, PROJECT_ROOT / "Logs")
    expect_path("scenario info path", db_paths.INFO_PATH, PROJECT_ROOT / "settings" / "current_scenario.json")
    expect_equal("DB root env name", db_paths.ENV_DB_ROOT, "KU_MISSION_DB_ROOT")
    expect_equal("scenario root env name", db_paths.ENV_SCENARIO_ROOT, "KU_SCENARIO_ROOT")
    expect_equal("scenario base env name", db_paths.ENV_SCENARIO_BASE_ROOT, "KU_SCENARIO_BASE_ROOT")
    expect_equal(
        "active DB scaffold dirs",
        db_paths._DB_SUBDIRS,
        (
            "DSS_Internal",
            "FlightPath",
            "IndividualMissionPlan",
            "InputMissionPlan",
            "VehicleStatus",
            "MissionPlan",
            "MissionPlanOptionInfo",
            "MissionReferenceInfo",
            "mission_output",
        ),
    )


def check_agent_status_paths(db_root: Path) -> None:
    from modules.common import agent_status_snapshot as snapshot

    expect_equal("0401 snapshot filename", snapshot.SNAPSHOT_FILENAME, "latest_0401_agent_status.json")
    expect_equal("0401 JSONL log filename", snapshot.LOG_FILENAME, "log_0401_agent_status_sim.jsonl")
    expect_equal("0401 JSON-array log dir", snapshot.SIM_0401_LOG_DIRNAME, "simlog_0401")
    expect_equal("0401 JSON-array log basename", snapshot.SIM_0401_LOG_BASENAME, "0401")
    expect_equal("0401 JSON-array max bytes", snapshot.SIM_0401_LOG_MAX_BYTES, 5 * 1024 * 1024)

    with patched_db_root(db_root):
        expect_path(
            "0401 latest snapshot path",
            snapshot._snapshot_path(),
            db_root / "DSS_Internal" / "latest_0401_agent_status.json",
        )
        expect_path(
            "0401 JSONL log path",
            snapshot._log_path(),
            db_root / "DSS_Internal" / "log_0401_agent_status_sim.jsonl",
        )
        expect_path("0401 JSON-array default dir", snapshot._sim_0401_log_dir(), db_root / "simlog_0401")
        expect_path(
            "0401 JSON-array base file",
            snapshot._sim_0401_log_file(db_root / "simlog_0401", 0),
            db_root / "simlog_0401" / "0401.json",
        )
        expect_path(
            "0401 JSON-array rotated file",
            snapshot._sim_0401_log_file(db_root / "simlog_0401", 3),
            db_root / "simlog_0401" / "0401_3.json",
        )

    override_dir = db_root / "override_0401"
    with patched_env("KU_SIM_0401_LOG_DIR", str(override_dir)):
        expect_path("0401 JSON-array env override", snapshot._sim_0401_log_dir(), override_dir)


def check_replan_sidecar_paths(db_root: Path) -> None:
    from modules.common import imaging_schedule_replan_store
    from modules.common import mission_area_replan_store
    from modules.common import path_deviation_replan_store
    from modules.common import prior_replan_store
    from modules.common import prior_target_rediscovery_store
    from modules.common import replan_request_transport_store
    from modules.mission_planning.runtime import debug_artifacts
    from modules.mission_planning.runtime import next_collab_replan_store
    from modules.mission_planning.runtime import replan_store

    expect_equal("transport primary mode env", replan_request_transport_store._MODE_ENV, "REPLAN_0902_SIDECAR_MODE")
    expect_equal("transport fallback mode env", replan_request_transport_store._FALLBACK_MODE_ENV, "REPLAN_SIDECAR_MODE")
    expect_equal("runtime artifact primary mode env", debug_artifacts._MODE_ENV, "REPLAN_RUNTIME_ARTIFACT_MODE")
    expect_equal("runtime artifact fallback mode env", debug_artifacts._FALLBACK_MODE_ENV, "REPLAN_DEBUG_ARTIFACT_MODE")
    expect_equal("next-collab store name", next_collab_replan_store._STORE_NAME, "next_collab_replan")
    expect_equal("next-collab detail prefix", next_collab_replan_store._DETAIL_PREFIX, "next_collab_detail")
    expect_equal("mission-area store dir", mission_area_replan_store._STORE_DIR, "mission_area_replan")
    expect_equal("mission-area detail prefix", mission_area_replan_store._DETAIL_PREFIX, "mission_area_snapshot")
    expect_equal("mission-area audit basename", mission_area_replan_store._AUDIT_BASENAME, "mission_area_snapshot_audit.jsonl")

    with patched_db_root(db_root):
        expect_path(
            "0902 replan request transport dir",
            replan_request_transport_store._base_dir(),
            db_root / "DSS_Internal" / "replan_request_transport",
        )
        expect_path(
            "0902 replan request payload path",
            replan_request_transport_store.payload_path_for_timestamp(833886405198),
            db_root / "DSS_Internal" / "replan_request_transport" / "replan_request_833886405198.json",
        )
        expect_path(
            "generic next-collab detail path",
            replan_store._detail_path("next_collab_replan", "next_collab_detail", 700001),
            db_root / "DSS_Internal" / "next_collab_replan" / "next_collab_detail_700001.json",
        )
        expect_path(
            "prior detail path",
            prior_replan_store._detail_path(700002),
            db_root / "DSS_Internal" / "prior_replan" / "prior_detail_700002.json",
        )
        expect_path(
            "imaging schedule detail path",
            imaging_schedule_replan_store._detail_path(700003),
            db_root / "DSS_Internal" / "imaging_schedule_replan" / "imaging_schedule_detail_700003.json",
        )
        expect_path(
            "path deviation detail path",
            path_deviation_replan_store._detail_path(700004),
            db_root / "DSS_Internal" / "path_deviation_replan" / "path_deviation_detail_700004.json",
        )
        expect_path(
            "mission area snapshot path",
            mission_area_replan_store._detail_path(700005),
            db_root / "DSS_Internal" / "mission_area_replan" / "mission_area_snapshot_700005.json",
        )
        expect_path(
            "mission area audit path",
            mission_area_replan_store._audit_path(),
            db_root / "DSS_Internal" / "mission_area_replan" / "mission_area_snapshot_audit.jsonl",
        )
        expect_path(
            "prior target rediscovery state path",
            prior_target_rediscovery_store._state_path(),
            db_root / "DSS_Internal" / "prior_target_rediscovery" / "state.json",
        )


def check_runtime_resource_paths() -> None:
    from modules.common import fusion_network
    from modules.mission_planning.MissionPlanner import runtime_settings

    expect_equal("runtime FOV DB relative path", runtime_settings.DEFAULT_FOV_DB_RELATIVE_PATH, "resource/db/fov_db.csv")
    expect_path("runtime default FOV DB path", runtime_settings.default_fov_db_path(), PROJECT_ROOT / "resource" / "db" / "fov_db.csv")
    expect_path(
        "runtime payload FOV DB path",
        runtime_settings.get_runtime_fov_db_path({"values": {"fov_db_path": "resource/db/fov_db.csv"}}),
        PROJECT_ROOT / "resource" / "db" / "fov_db.csv",
    )

    uav_params_path = PROJECT_ROOT / "settings" / "uav_params.json"
    uav_params = json.loads(uav_params_path.read_text(encoding="utf-8"))
    current_fov_db = ((uav_params.get("values") or {}).get("fov_db_path") or "").strip()
    if not current_fov_db:
        fail("uav_params.json missing values.fov_db_path")
    current_fov_db_path = PROJECT_ROOT / current_fov_db
    expect_path("runtime current FOV DB override", runtime_settings.fov_db_path(), current_fov_db_path)
    if not current_fov_db_path.exists():
        fail(f"runtime current FOV DB override missing on disk: {current_fov_db_path}")

    expect_equal(
        "fusion config candidates",
        fusion_network._candidate_config_paths(),
        (
            PROJECT_ROOT / "settings" / "nFusionSettings.json",
            PROJECT_ROOT / "nFusionSettings.json",
            PROJECT_ROOT / "modules" / "common" / "nFusionSettings.json",
            PROJECT_ROOT / "settings" / "FusionSettings.json",
            PROJECT_ROOT / "FusionSettings.json",
            PROJECT_ROOT / "modules" / "common" / "FusionSettings.json",
            PROJECT_ROOT / "nFusion" / "FusionSettings.json",
        ),
    )

    assert_source_contains(
        "run.py",
        "ensure_fusion_settings_file(",
        "ensure_fusion_license_file(",
        'msg_dir = COMMON_DIR / "msg_files"',
        'stem = msg_dir / "MessageLibrary"',
    )
    assert_source_contains(
        "modules/common/dll_files/nFusionImports.py",
        '_MSG_DIR = _MODULES_DIR / "msg_files"',
        '_MSG_ASM = _MSG_DIR / "MessageLibrary.dll"',
    )
    assert_source_contains(
        "modules/sim/config.py",
        'RESOURCE_DIR = _env_path("SIM_RESOURCE_DIR", ROOT_DIR / "resource")',
        'MBTILES_PATH = _env_path("SIM_MBTILES_PATH", RESOURCE_DIR / "korea.mbtiles")',
        'DEM_DIR = _env_path("SIM_DEM_DIR", RESOURCE_DIR)',
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py",
        'DEM_PATH       = os.path.join(os.path.dirname(__file__), "DEM.jpg")',
        '_ID_COUNTER_FILE = os.path.join(os.path.dirname(__file__), "_id_counters.json")',
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/data_def/mission_helpers.py",
        '_DEM_DIR = _PROJECT_ROOT / "resource"',
        'return log_dir / "dem_usage.jsonl"',
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/data_def/lah_attack_assistance.py",
        'candidates = _list_tif_files("resource")',
        'candidates.extend(_list_tif_files("resources"))',
        "No GeoTIFF (*.tif) files found.",
    )
    assert_source_contains(
        "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py",
        '_FOV_DB_PATH = _PROJECT_ROOT / "resource" / "db" / "fov_db.csv"',
    )
    assert_source_contains(
        "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py",
        'bundle_root = _MISSION_PLANNER_DIR / "portable_mission_bundle"',
        'model_path = bundle_root / "models" / "latest_model.zip"',
        'resource_dir = _PROJECT_ROOT / "resource"',
        'f"n{tile_lat}_e{tile_lon}_1arc_v3.tif"',
        '"Jipo_48km.tif"',
    )
    assert_source_contains(
        "modules/mission_planning/manual/lah_rl_planner_gui.py",
        '_RESOURCE_DIR = _PROJECT_ROOT / "resource"',
        '_BUNDLE_ROOT = _MISSION_ROOT / "MissionPlanner" / "portable_mission_bundle"',
        '_MODEL_PATH = _BUNDLE_ROOT / "models" / "latest_model.zip"',
        '"Jipo_48km.tif"',
        '"Hongik_48km.tif"',
        '"Inje_48km.tif"',
    )
    assert_source_contains(
        "modules/mission_planning/MissionPlanner/portable_mission_bundle/portable_mission/service.py",
        'self.models_dir = bundle_root / "models"',
        'self.inputs_dir = bundle_root / "data" / "inputs"',
        'self.work_dir = bundle_root / "data" / "work"',
        'self.model_path = self.models_dir / "latest_model.zip"',
        'self.config_path = self.models_dir / "model_config.json"',
    )


def check_log_artifact_markers() -> None:
    assert_source_contains(
        "modules/common/process_console.py",
        'return db_paths.get_db_subpath("DSS_Internal", "module_logs", f"{self.module_name}.log")',
        'return root / "DSS_Internal" / "module_logs" / f"{self.module_name}.log"',
    )
    assert_source_contains(
        "modules/mission_planning/runtime/logging/plan_file_logger.py",
        'self._base_dir / f"missionPlan_{plan_id}.json"',
        'self._base_dir / f"missionPlan_{plan_id}_{token}.json"',
        'self._base_dir / f"missionPlan_pending_{token}.json"',
        'base_dir = db_paths.ensure_db_payload("DSS_Internal")',
    )
    assert_source_contains(
        "modules/mission_planning/mission_planning_gui.py",
        'log_dir / f"mission_planning_gui_{token}.log"',
        "out_root_base = db_root / 'mission_output'",
        'dss_dir = db_root / "DSS_Internal" / "replan_inputs"',
        'path = dss_dir / f"0201_override_{source_tag}_{suffix}.json"',
    )
    assert_source_contains(
        "modules/mission_planning/replanning/triggers/next_collab/pipeline.py",
        'log_dir = db_paths.get_db_subpath("DSS_Internal")',
        'log_path = log_dir / f"NextCollab_{int(target_input_id)}_{int(now_ms)}.json"',
    )


def main() -> int:
    check_db_root_contract()
    with tempfile.TemporaryDirectory(prefix="mp_runtime_artifact_paths_") as tmp:
        db_root = Path(tmp) / "scenario" / "SBC3"
        check_agent_status_paths(db_root)
        check_replan_sidecar_paths(db_root)
    check_runtime_resource_paths()
    check_log_artifact_markers()
    print("runtime artifact/resource path manifest smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
