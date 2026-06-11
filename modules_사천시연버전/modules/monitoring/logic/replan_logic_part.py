"""Delegates to the existing ver2 replan logic interface."""

from pathlib import Path
import sys
import importlib

_VER2_ROOT = Path(__file__).resolve().parents[3] / "modules" / "monitoring_ver2"
if _VER2_ROOT.exists():
    ver2_path = str(_VER2_ROOT)
    if ver2_path not in sys.path:
        sys.path.insert(0, ver2_path)

ver2_push_pkg = importlib.import_module("modules.monitoring_ver2.push")
sys.modules["push"] = ver2_push_pkg
sys.modules["push.push_center"] = importlib.import_module("modules.monitoring_ver2.push.push_center")
sys.modules["push.push_utils"] = importlib.import_module("modules.monitoring_ver2.push.push_utils")

from modules.monitoring_ver2.logic.replan_logic_part import *  # noqa: F401,F403
