from __future__ import annotations

from typing import Any, Iterator, Sequence


def iter_valid_target_permutations_in_order(
    uav_subset: Sequence[int],
    target_keys: Sequence[str],
    candidate_by_uav_target: dict[tuple[int, str], Any],
) -> Iterator[tuple[str, ...]]:
    """Yield valid target permutations in the same relative order as itertools.permutations."""
    assign_count = len(uav_subset)
    if assign_count <= 0:
        yield ()
        return
    if assign_count > len(target_keys):
        return

    normalized_targets = [str(target_key) for target_key in target_keys]
    used = [False] * len(normalized_targets)
    current: list[str] = []

    def _walk(depth: int) -> Iterator[tuple[str, ...]]:
        aid = int(uav_subset[depth])
        for idx, target_key in enumerate(normalized_targets):
            if used[idx]:
                continue
            if (aid, target_key) not in candidate_by_uav_target:
                continue
            used[idx] = True
            current.append(target_key)
            if depth + 1 == assign_count:
                yield tuple(current)
            else:
                yield from _walk(depth + 1)
            current.pop()
            used[idx] = False

    yield from _walk(0)
