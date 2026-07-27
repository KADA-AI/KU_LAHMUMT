"""Shared helpers for mission option codes."""

from __future__ import annotations

from typing import Callable, Iterable, Optional

# Canonical option codes and labels (ICD-aligned).
OPTION_CODE_TO_LABEL: dict[int, str] = {
    1: "시스템 추천",
    2: "공격특화",
    3: "공격배제",
    4: "정찰특화",
    5: "최소시간",
    6: "정찰/시간 균형",
}


def _normalize_label(text: str) -> str:
    """Normalize labels for tolerant string matching."""
    return "".join(str(text).split()).lower()


# Normalized label lookup -> option code.
_LABEL_TO_CODE_NORM: dict[str, int] = {
    _normalize_label(label): code for code, label in OPTION_CODE_TO_LABEL.items()
}
_LABEL_TO_CODE_NORM.update(
    {
        _normalize_label("공격 추천"): 2,
        _normalize_label("공격추천"): 2,
        _normalize_label("공격 특화"): 2,
        _normalize_label("공격특화"): 2,
        _normalize_label("공격 배제"): 3,
        _normalize_label("공격배제"): 3,
        _normalize_label("정찰 특화"): 4,
        _normalize_label("정찰특화"): 4,
        _normalize_label("최소 시간"): 5,
        _normalize_label("최소시간"): 5,
        _normalize_label("정찰 / 시간 균형"): 6,
        _normalize_label("정찰/시간 균형"): 6,
        _normalize_label("정찰시간균형"): 6,
    }
)

# Default option codes used by current monitoring replan flows.
DEFAULT_OPTION_CODE_SEQUENCE: tuple[int, ...] = (6, 4, 5)


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
                # Try to map from (possibly normalized) label text.
                norm = _normalize_label(stripped)
                candidate = _LABEL_TO_CODE_NORM.get(norm)
        else:
            try:
                candidate = int(value)  # type: ignore[arg-type]
            except Exception:
                candidate = None
    if candidate in OPTION_CODE_TO_LABEL:
        return candidate
    return fallback


def is_known_option_value(value: object) -> bool:
    return normalize_option_code(value, fallback=None) is not None


def is_option_code_value(value: object, expected_code: int) -> bool:
    return normalize_option_code(value, fallback=None) == int(expected_code)


def ensure_option_code_sequence(
    values: Iterable[object],
    count: int,
    *,
    on_unknown: Optional[Callable[[int, object, int], None]] = None,
) -> list[int]:
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
            if raw not in (None, "") and on_unknown is not None:
                on_unknown(idx, raw, code)
        result.append(code)
    return result


def option_code_to_label(code: object) -> str:
    """Return the human-readable label for the given option code."""
    try:
        code_int = int(code)  # type: ignore[arg-type]
    except Exception:
        return str(code)
    return OPTION_CODE_TO_LABEL.get(code_int, str(code_int))
