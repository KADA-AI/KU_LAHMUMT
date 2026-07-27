from pathlib import Path
import sys

_PROJECT_ROOT = next(
    (
        parent
        for parent in Path(__file__).resolve().parents
        if (parent / "modules" / "common").exists()
    ),
    Path(__file__).resolve().parents[4],
)
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

from .pipeline import run_enhanced_divide_and_pattern

__all__ = ["run_enhanced_divide_and_pattern"]
