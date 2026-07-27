from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .shinil_mission_profile import ShinilCameraOverlay


Target3 = tuple[float, float, float]


def _operation_mode(filming: Any) -> int:
    if not isinstance(filming, dict):
        return 0
    value = filming.get("operationMode")
    if value is None:
        value = filming.get("operationalMode")
    try:
        return int(value or 0)
    except Exception:
        return 0


def _fov(filming: Any) -> float | None:
    if not isinstance(filming, dict):
        return None
    value = filming.get("fieldOfView")
    if value is None:
        value = filming.get("fov")
    try:
        parsed = float(value)
    except Exception:
        return None
    return parsed if math.isfinite(parsed) else None


@dataclass(frozen=True)
class ShinilCameraCommand:
    """One immutable reference to a planned payload command."""

    filming: dict | None
    wp_id: int | None
    max_sep_m: float | None
    phase: str
    signature: tuple[object, ...]

    @property
    def operation_mode(self) -> int:
        return _operation_mode(self.filming)

    @classmethod
    def build(
        cls,
        *,
        filming: dict | None,
        wp_id: int | None,
        max_sep_m: float | None,
        phase: str,
    ) -> "ShinilCameraCommand":
        # Mission filming dictionaries are stable objects. Object identity keeps
        # this check O(1) even when a lineSearch contains thousands of points;
        # mode/FOV remain explicit so safe in-place metadata updates are noticed.
        signature = (
            int(wp_id) if wp_id is not None else None,
            id(filming) if isinstance(filming, dict) else None,
            _operation_mode(filming),
            _fov(filming),
        )
        return cls(
            filming=filming if isinstance(filming, dict) else None,
            wp_id=int(wp_id) if wp_id is not None else None,
            max_sep_m=float(max_sep_m) if max_sep_m is not None else None,
            phase=str(phase or "default"),
            signature=signature,
        )


@dataclass
class ShinilCameraCommandResponse:
    """Observed 0401 payload-command delay without mutating mission data."""

    initialized: bool = False
    active: ShinilCameraCommand | None = None
    pending: ShinilCameraCommand | None = None
    pending_elapsed_s: float = 0.0
    pending_delay_s: float = 0.0

    def clear(self) -> None:
        self.initialized = False
        self.active = None
        self.pending = None
        self.pending_elapsed_s = 0.0
        self.pending_delay_s = 0.0

    @property
    def settling(self) -> bool:
        return self.pending is not None

    @property
    def remaining_s(self) -> float:
        if self.pending is None:
            return 0.0
        return max(0.0, float(self.pending_delay_s) - float(self.pending_elapsed_s))

    def resolve(
        self,
        desired: ShinilCameraCommand,
        *,
        dt: float,
        overlay: ShinilCameraOverlay,
        enabled: bool,
    ) -> tuple[ShinilCameraCommand, bool]:
        """Return the active command and whether it became active this tick."""

        if not enabled:
            self.initialized = True
            self.active = desired
            self.pending = None
            self.pending_elapsed_s = 0.0
            self.pending_delay_s = 0.0
            return desired, False

        if not self.initialized or self.active is None:
            self.initialized = True
            self.active = desired
            self.pending = None
            return desired, False

        if desired.signature == self.active.signature:
            # Refresh the reference/phase while preserving the settled command.
            self.active = desired
            self.pending = None
            self.pending_elapsed_s = 0.0
            self.pending_delay_s = 0.0
            return desired, False

        # Removing a filming command must never leave a stale sensor active.
        if desired.filming is None:
            self.active = desired
            self.pending = None
            self.pending_elapsed_s = 0.0
            self.pending_delay_s = 0.0
            return desired, True

        if self.pending is None or desired.signature != self.pending.signature:
            self.pending = desired
            self.pending_elapsed_s = 0.0
            previous_mode = self.active.operation_mode
            if desired.operation_mode == 2:
                self.pending_delay_s = (
                    float(overlay.fixed_to_line_search_delay_s)
                    if previous_mode == 4
                    else float(overlay.line_search_delay_s)
                )
            else:
                self.pending_delay_s = float(overlay.command_delay_s)

        self.pending_elapsed_s += max(0.0, float(dt or 0.0))
        if self.pending_elapsed_s + 1e-9 < self.pending_delay_s:
            return self.active, False

        activated = self.pending
        self.active = activated
        self.pending = None
        self.pending_elapsed_s = 0.0
        self.pending_delay_s = 0.0
        return activated, True


def _direction(origin: Target3, target: Target3) -> tuple[Target3, float] | None:
    dx = float(target[0]) - float(origin[0])
    dy = float(target[1]) - float(origin[1])
    dz = float(target[2]) - float(origin[2])
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    if not math.isfinite(distance) or distance <= 1e-6:
        return None
    return (dx / distance, dy / distance, dz / distance), distance


def _slerp_direction(start: Target3, end: Target3, max_angle_rad: float) -> Target3:
    dot = max(-1.0, min(1.0, sum(float(a) * float(b) for a, b in zip(start, end))))
    angle = math.acos(dot)
    if angle <= max(0.0, float(max_angle_rad)) + 1e-12:
        return end
    if angle <= 1e-9:
        return end

    fraction = max(0.0, min(1.0, float(max_angle_rad) / angle))
    sin_angle = math.sin(angle)
    if dot <= -1.0 + 1e-8:
        # Linear interpolation collapses to a zero vector halfway between
        # antipodal commands. Pick a deterministic perpendicular great circle.
        basis = (1.0, 0.0, 0.0) if abs(start[0]) < 0.9 else (0.0, 1.0, 0.0)
        orthogonal = (
            start[1] * basis[2] - start[2] * basis[1],
            start[2] * basis[0] - start[0] * basis[2],
            start[0] * basis[1] - start[1] * basis[0],
        )
        orth_norm = math.sqrt(sum(component * component for component in orthogonal))
        orthogonal = tuple(component / orth_norm for component in orthogonal)
        travelled = angle * fraction
        blended = tuple(
            start[i] * math.cos(travelled) + orthogonal[i] * math.sin(travelled)
            for i in range(3)
        )
    elif abs(sin_angle) <= 1e-8:
        blended = tuple((1.0 - fraction) * a + fraction * b for a, b in zip(start, end))
    else:
        left = math.sin((1.0 - fraction) * angle) / sin_angle
        right = math.sin(fraction * angle) / sin_angle
        blended = tuple(left * a + right * b for a, b in zip(start, end))
    norm = math.sqrt(sum(component * component for component in blended))
    if norm <= 1e-9 or not math.isfinite(norm):
        return end
    return tuple(float(component / norm) for component in blended)  # type: ignore[return-value]


@dataclass
class ShinilCameraLosResponse:
    """World-frame camera line-of-sight state, separate from scan progress."""

    direction: Target3 | None = None
    angular_error_deg: float = 0.0
    operation_mode: int = 0

    def clear(self) -> None:
        self.direction = None
        self.angular_error_deg = 0.0
        self.operation_mode = 0

    def resolve(
        self,
        *,
        origin: Target3,
        desired_target: Target3,
        dt: float,
        max_rate_dps: float,
        operation_mode: int,
        enabled: bool,
        seed_target: Target3 | None = None,
        bypass_limit: bool = False,
    ) -> Target3:
        desired = _direction(origin, desired_target)
        if desired is None:
            self.clear()
            return desired_target
        desired_direction, desired_range = desired

        if self.direction is None and seed_target is not None:
            seeded = _direction(origin, seed_target)
            if seeded is not None:
                self.direction = seeded[0]

        if self.direction is None or not enabled or bypass_limit:
            self.direction = desired_direction
        else:
            max_angle = math.radians(max(0.0, float(max_rate_dps)) * max(0.0, float(dt or 0.0)))
            self.direction = _slerp_direction(self.direction, desired_direction, max_angle)

        dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(self.direction, desired_direction))))
        self.angular_error_deg = math.degrees(math.acos(dot))
        self.operation_mode = int(operation_mode)
        distance = max(1.0, float(desired_range))
        return (
            float(origin[0]) + self.direction[0] * distance,
            float(origin[1]) + self.direction[1] * distance,
            float(origin[2]) + self.direction[2] * distance,
        )
