"""Compatibility imports for reference-radius callers."""

from .turn_dynamics import (
    REFERENCE_TURN_RADIUS_TABLE_MPS,
    interpolate_reference_turn_radius,
    interpolate_turn_radius,
)

__all__ = [
    "REFERENCE_TURN_RADIUS_TABLE_MPS",
    "interpolate_reference_turn_radius",
    "interpolate_turn_radius",
]
