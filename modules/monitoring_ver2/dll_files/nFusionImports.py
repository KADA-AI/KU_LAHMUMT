# modules/monitoring_ver2/dll_files/nFusionImports.py
# Load nFusion + message assemblies for the monitoring module (pythonnet).

from __future__ import annotations

import sys
from pathlib import Path

from pythonnet import load

# Ensure pythonnet uses CoreCLR before any clr.AddReference calls.
load("coreclr")

import clr  # type: ignore

# --- Resolve important directories -------------------------------------------------

_CURRENT_DIR = Path(__file__).resolve().parent  # modules/monitoring_ver2/dll_files
_MONITORING_DIR = _CURRENT_DIR.parent           # modules/monitoring_ver2
_MODULES_ROOT = _MONITORING_DIR.parent          # modules/
_COMMON_DIR = _MODULES_ROOT / "common"
_MSG_DIRS = (
    _MONITORING_DIR / "msg_files",
    _COMMON_DIR / "msg_files",
)

# Keep sys.path aware of the directories that expose managed assemblies and helpers.
for candidate in (_MODULES_ROOT, _MONITORING_DIR, _CURRENT_DIR, *_MSG_DIRS, _COMMON_DIR):
    if candidate.exists():
        candidate_str = str(candidate)
        if candidate_str not in sys.path:
            sys.path.append(candidate_str)

# --- Helper -----------------------------------------------------------------------

def _add_reference(name):
    try:
        clr.AddReference(str(name))
    except Exception:
        try:
            clr.AddReference(Path(name).stem)
        except Exception:
            pass

# --- Framework & nFusion assemblies ------------------------------------------------

_DLL_REFERENCES = (
    "Microsoft.Bcl.AsyncInterfaces",
    "Microsoft.Extensions.Configuration.Abstractions",
    "Microsoft.Extensions.Configuration",
    "Microsoft.Extensions.Configuration.FileExtensions",
    "Microsoft.Extensions.Configuration.Json",
    "Microsoft.Extensions.DependencyInjection.Abstractions",
    "Microsoft.Extensions.DependencyInjection",
    "Microsoft.Extensions.FileProviders.Abstractions",
    "Microsoft.Extensions.FileProviders.Physical",
    "Microsoft.Extensions.FileSystemGlobbing",
    "Microsoft.Extensions.Primitives",
    "nFusion.Core",
    "nFusion.Interface.Contracts",
    "nFusion.Nodes.Core",
    "nFusion.SimpleMiddleware",
)

for dll in _DLL_REFERENCES:
    dll_path = _CURRENT_DIR / f"{dll}.dll"
    if dll_path.exists():
        _add_reference(dll_path)
    else:
        _add_reference(dll)

# Message library (contains nFusion.Model.msg_xxxx types)
for msg_dir in _MSG_DIRS:
    dll_path = msg_dir / "MessageLibrary.dll"
    if dll_path.exists():
        _add_reference(dll_path)

_add_reference("MessageLibrary")  # fallback if directory scan above failed

# Attempt to pre-import a few frequently used namespaces so later imports succeed.
try:
    import importlib

    for module_name in (
        "nFusion.Model.msg_0101",
        "nFusion.Model.msg_0102",
        "nFusion.Model.msg_0401",
        "nFusion.Model.msg_0504",
    ):
        if module_name not in sys.modules:
            importlib.import_module(module_name)
except Exception:
    pass

# Re-export commonly used types so legacy imports ("from dll_files.nFusionImports import *") keep working.
from nFusion.Nodes.Core import *  # type: ignore  # noqa: F401,F403
from nFusion.Interface.Contracts import *  # type: ignore  # noqa: F401,F403
from nFusion.Nodes.Core.Ioc import *  # type: ignore  # noqa: F401,F403
from nFusion.Nodes.Core.Consumer import *  # type: ignore  # noqa: F401,F403
from nFusion.Nodes.Core.Provider import *  # type: ignore  # noqa: F401,F403
