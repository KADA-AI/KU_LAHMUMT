"""A short ID reservation must not kill the whole replan.

Block sizes are estimated before the builders run. When an estimate came up one
short the builder raised `reserved ID block exhausted: pathID[2]`, the attack
pipeline died, and the operator was offered only the exclusion option - observed
live in Scenario_2026-07-26T163420 and again in 172124.
"""

from __future__ import annotations

import pytest

from modules.mission_planning.runtime.ids.replan_reservation import (
    ReplanIdReservation,
    ReservedIdBlock,
)


def test_a_short_block_extends_instead_of_failing() -> None:
    pulled: list[int] = []

    def refill(count: int) -> list[int]:
        pulled.append(count)
        start = 900 + sum(pulled[:-1])
        return list(range(start, start + count))

    block = ReservedIdBlock("pathID[2]", [1, 2], refill=refill)

    assert [block.next() for _ in range(5)] == [1, 2, 900, 901, 902]
    assert block.overflow_count > 0


def test_extended_ids_stay_unique() -> None:
    counter = {"n": 500}

    def refill(count: int) -> list[int]:
        out = list(range(counter["n"], counter["n"] + count))
        counter["n"] += count
        return out

    block = ReservedIdBlock("pathID[3]", [7], refill=refill)
    ids = [block.next() for _ in range(20)]

    assert len(set(ids)) == len(ids)


def test_a_block_without_a_refill_keeps_the_old_contract() -> None:
    block = ReservedIdBlock("waypoint", [1])

    assert block.next() == 1
    with pytest.raises(RuntimeError, match="reserved ID block exhausted"):
        block.next()


def test_a_refill_that_yields_nothing_still_fails_loudly() -> None:
    """Never hand back a fabricated or duplicate ID."""

    block = ReservedIdBlock("individualMission", [], refill=lambda _n: [])

    with pytest.raises(RuntimeError, match="reserved ID block exhausted"):
        block.next()


def test_overflow_is_counted_so_a_bad_estimate_is_visible() -> None:
    block = ReservedIdBlock("pathID[2]", [], refill=lambda n: list(range(n)))

    assert block.overflow_count == 0
    block.next()
    assert block.overflow_count > 0


def test_a_reservation_survives_more_paths_than_it_reserved() -> None:
    """The exact live failure: one more path than the estimate allowed."""

    reservation = ReplanIdReservation.reserve(path_count_by_aircraft={2: 2})

    first = [reservation.next_path(2) for _ in range(2)]
    # Third path is beyond the reservation and must still be served.
    third = reservation.next_path(2)

    assert third not in first
    assert isinstance(third, int)


def test_an_aircraft_with_no_block_at_all_still_reports_clearly() -> None:
    reservation = ReplanIdReservation.reserve(path_count_by_aircraft={2: 1})

    with pytest.raises(RuntimeError, match="no reserved pathID block"):
        reservation.next_path(9)


def test_waypoint_and_mission_blocks_extend_too() -> None:
    reservation = ReplanIdReservation.reserve(
        imp_count=1, individual_count=1, waypoint_count=1
    )

    imps = [reservation.next_imp() for _ in range(3)]
    individuals = [reservation.next_individual() for _ in range(3)]
    waypoints = [reservation.next_waypoint() for _ in range(3)]

    for values in (imps, individuals, waypoints):
        assert len(set(values)) == len(values)
