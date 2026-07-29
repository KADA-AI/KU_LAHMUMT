from __future__ import annotations

from typing import Iterable


def resolve_post_attack_option_indices(
    *,
    option_count: int,
    attack_option_indices: Iterable[int],
    attack_exclusion_option_indices: Iterable[int],
    attack_plan_materialized: bool,
) -> tuple[list[int], set[int]]:
    """Return ordinary variants that may still be published after attack planning.

    Attack-specialized variants are materialized by the dedicated attack
    pipeline and are therefore always removed from the ordinary variant loop.
    An attack-exclusion alternative is paired with that attack result.  When
    the requested attack result was not materialized, publishing the exclusion
    alternative by itself would invert the operator's request and hide the
    attack failure.  In that case every paired exclusion index is suppressed.

    An explicit exclusion-only request remains valid because it has no attack
    option index to pair with.
    """

    count = max(0, int(option_count or 0))
    attack_indices = {
        int(index)
        for index in (attack_option_indices or ())
        if 0 <= int(index) < count
    }
    exclusion_indices = {
        int(index)
        for index in (attack_exclusion_option_indices or ())
        if 0 <= int(index) < count
    }

    suppressed_exclusion_indices: set[int] = set()
    if attack_indices and not bool(attack_plan_materialized):
        suppressed_exclusion_indices = set(exclusion_indices)

    removed_indices = attack_indices | suppressed_exclusion_indices
    keep_indices = [index for index in range(count) if index not in removed_indices]
    return keep_indices, suppressed_exclusion_indices
