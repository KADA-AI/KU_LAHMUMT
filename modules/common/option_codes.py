"""Shared helpers for mission option codes."""

from __future__ import annotations

from typing import Iterable, Optional

OPTION_CODE_TO_LABEL = {
    1: "시스템추천",
    2: "공격특화",
    3: "공격배제",
    4: "정찰특화",
    5: "최소시간",
}

# Default option codes used by current mission-planning pipeline (no 공격 variants).
DEFAULT_OPTION_CODE_SEQUENCE: tuple[int, ...] = (1, 4, 5)


def normalize_option_code(value: object, fallback: Optional[int] = None) -> Optional[int]:
    """Return a valid option code for the given raw value, or fallback if unknown."""
    candidate: Optional[int] = None
    if isinstance(value, int):
        candidate = value
    else:
        # Attempt numeric conversion first.
        if isinstance(value, str):
            stripped = value.strip()
            if stripped.isdigit():
                try:
                    candidate = int(stripped)
                except Exception:
                    candidate = None
            else:
                # Try to map from label text.
                for code, label in OPTION_CODE_TO_LABEL.items():
                    if stripped == label:
                        candidate = code
                        break
        else:
            try:
                candidate = int(value)  # type: ignore[arg-type]
            except Exception:
                candidate = None
    if candidate in OPTION_CODE_TO_LABEL:
        return candidate
    return fallback


def ensure_option_code_sequence(values: Iterable[object], count: int) -> list[int]:
    """Normalize raw option values into a sequence of codes with sensible defaults."""
    result: list[int] = []
    defaults = list(DEFAULT_OPTION_CODE_SEQUENCE) or [1]
    defaults_len = len(defaults)
    for idx in range(count):
        raw = None
        if isinstance(values, (list, tuple)):
            if idx < len(values):
                raw = values[idx]
        else:
            try:
                raw = next(values)  # type: ignore[misc]
            except Exception:
                raw = None
        code = normalize_option_code(raw)
        if code is None:
            code = defaults[idx] if idx < defaults_len else defaults[-1]
        result.append(code)
    return result


def option_code_to_label(code: object) -> str:
    """Return the human-readable label for the given option code."""
    try:
        code_int = int(code)  # type: ignore[arg-type]
    except Exception:
        return str(code)
    return OPTION_CODE_TO_LABEL.get(code_int, str(code_int))
