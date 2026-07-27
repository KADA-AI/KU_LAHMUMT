"""A serviced target's firing command must not survive into the resume plan.

Observed: after 표적14 격파 후 복귀, LAH3's resume package still carried
``attack.targetID = 14`` on a waypoint, even though the branch it lived in
declared ``targetID = 15``. Removal is keyed on the branch's declared target, so
a waypoint carried over from the earlier engagement slipped through and the
manned aircraft resumed holding a live attack on a target that was already gone.
"""

from __future__ import annotations

from typing import Any

from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
    _clear_waypoint_attacks_for_targets,
)


def _path(*targets: int) -> dict[str, Any]:
    return {
        "lahWaypointList": [
            {
                "waypointID": 31770 + index,
                "coordinate": {"latitude": 37.95, "longitude": 127.31, "altitude": 210},
                "speed": 73.61,
                "attack": {"targetID": target, "weaponType": 2 if target else 0},
            }
            for index, target in enumerate(targets)
        ]
    }


def test_the_serviced_target_is_cleared() -> None:
    payload = _path(14, 0)

    assert _clear_waypoint_attacks_for_targets([payload], {14}) == 1
    attack = payload["lahWaypointList"][0]["attack"]
    assert attack == {"targetID": 0, "weaponType": 0}


def test_a_different_live_target_is_untouched() -> None:
    """Only the finished engagement is stripped."""

    payload = _path(15)

    assert _clear_waypoint_attacks_for_targets([payload], {14}) == 0
    assert payload["lahWaypointList"][0]["attack"]["targetID"] == 15


def test_the_waypoint_geometry_survives_the_scrub() -> None:
    """The leg is still flown; it just no longer commands a shot."""

    payload = _path(14)
    coordinate = dict(payload["lahWaypointList"][0]["coordinate"])
    speed = payload["lahWaypointList"][0]["speed"]

    _clear_waypoint_attacks_for_targets([payload], {14})

    assert payload["lahWaypointList"][0]["coordinate"] == coordinate
    assert payload["lahWaypointList"][0]["speed"] == speed


def test_an_empty_target_set_changes_nothing() -> None:
    payload = _path(14, 15)

    assert _clear_waypoint_attacks_for_targets([payload], set()) == 0
    assert [
        row["attack"]["targetID"] for row in payload["lahWaypointList"]
    ] == [14, 15]


def test_uav_lists_are_scrubbed_too() -> None:
    payload = {
        "waypointList": [{"attack": {"targetID": 14, "weaponType": 2}}],
        "uavWaypointList": [{"attack": {"targetID": 14, "weaponType": 1}}],
    }

    assert _clear_waypoint_attacks_for_targets([payload], {14}) == 2
