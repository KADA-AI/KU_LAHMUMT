"""The concealment sidecar is display metadata and must never break a plan."""

from __future__ import annotations

import json
from pathlib import Path

from modules.mission_planning.pipelines import lah_tactical_point_log as sidecar


def test_records_and_reads_back_conceal_and_hold_points(tmp_path: Path) -> None:
    assert sidecar.record_tactical_points(
        100000025,
        conceal_waypoint_ids=[27104],
        hold_waypoint_ids=[27104],
        role="relay",
        plan={"hideAchievedS": 9.4, "status": "green_valid", "uavLinkCount": 3},
        db_root=tmp_path,
    )

    stored = sidecar.load_tactical_points(tmp_path)
    entry = stored["100000025"]
    assert entry["concealWaypointIDs"] == [27104]
    assert entry["holdWaypointIDs"] == [27104]
    assert entry["role"] == "relay"
    assert entry["hideAchievedS"] == 9.4
    assert entry["uavLinkCount"] == 3
    assert sidecar.sidecar_path(tmp_path).parent.name == "DSS_Internal"


def test_records_are_per_path_and_do_not_overwrite_each_other(tmp_path: Path) -> None:
    sidecar.record_tactical_points(1, conceal_waypoint_ids=[11], db_root=tmp_path)
    sidecar.record_tactical_points(2, conceal_waypoint_ids=[22], db_root=tmp_path)

    stored = sidecar.load_tactical_points(tmp_path)
    assert stored["1"]["concealWaypointIDs"] == [11]
    assert stored["2"]["concealWaypointIDs"] == [22]


def test_nothing_is_written_when_there_is_no_tactical_point(tmp_path: Path) -> None:
    assert sidecar.record_tactical_points(3, db_root=tmp_path) is False
    assert sidecar.load_tactical_points(tmp_path) == {}
    assert not sidecar.sidecar_path(tmp_path).exists()


def test_invalid_path_id_and_ids_are_rejected_without_raising(tmp_path: Path) -> None:
    assert sidecar.record_tactical_points(None, conceal_waypoint_ids=[1], db_root=tmp_path) is False
    assert sidecar.record_tactical_points("x", conceal_waypoint_ids=[1], db_root=tmp_path) is False
    assert sidecar.record_tactical_points(
        4, conceal_waypoint_ids=[0, -1, "bad", None], db_root=tmp_path
    ) is False


def test_a_corrupt_sidecar_reads_as_no_roles_known(tmp_path: Path) -> None:
    path = sidecar.sidecar_path(tmp_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("{not json", encoding="utf-8")

    assert sidecar.load_tactical_points(tmp_path) == {}
    # A later record still succeeds and replaces the unreadable content.
    assert sidecar.record_tactical_points(5, conceal_waypoint_ids=[55], db_root=tmp_path)
    assert sidecar.load_tactical_points(tmp_path)["5"]["concealWaypointIDs"] == [55]


def test_sidecar_is_valid_json_on_disk(tmp_path: Path) -> None:
    sidecar.record_tactical_points(6, conceal_waypoint_ids=[66], db_root=tmp_path)
    raw = json.loads(sidecar.sidecar_path(tmp_path).read_text(encoding="utf-8"))
    assert raw["6"]["concealWaypointIDs"] == [66]
