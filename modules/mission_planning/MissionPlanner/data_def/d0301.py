"""Compatibility wrapper for moved 0301 mission artifact builder."""

from importlib import import_module
from pathlib import Path
import sys

_PROJECT_ROOT = Path(__file__).resolve().parents[4]
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

sys.modules[__name__] = import_module(
    "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0301"
)
