from __future__ import annotations

REFERENCE_TURN_RADIUS_TABLE_MPS: tuple[tuple[float, float], ...] = (
    (30.0, 340.0),
    (40.0, 450.0),
    (50.0, 560.0),
)


def interpolate_reference_turn_radius(speed_mps: float) -> float:
    table = REFERENCE_TURN_RADIUS_TABLE_MPS
    speed = float(speed_mps)
    if speed <= table[0][0]:
        return float(table[0][1])
    if speed >= table[-1][0]:
        return float(table[-1][1])
    for left, right in zip(table[:-1], table[1:]):
        s0, r0 = left
        s1, r1 = right
        if s0 <= speed <= s1:
            alpha = (speed - s0) / max(1e-6, (s1 - s0))
            return float(r0 + (r1 - r0) * alpha)
    return float(table[-1][1])
