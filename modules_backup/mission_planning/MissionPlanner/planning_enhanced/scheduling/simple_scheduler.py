from __future__ import annotations

from typing import List

from ..models import SplitPiece


def schedule_by_parent_order(pieces: List[SplitPiece]) -> List[SplitPiece]:
    """
    Deterministic schedule for testing:
    parent mission order first, then generated piece index.
    """
    return sorted(pieces, key=lambda p: (p.parent_order, p.piece_index))
