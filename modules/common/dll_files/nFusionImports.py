#파이선에서 C#을 이용 pip로 pythonnet 설치 필요
from pythonnet import load
load("coreclr")
import clr
import os
import sys
from pathlib import Path
from System.Threading.Tasks import Task
# System.Exception을 'Exception' 이름으로 내보내면 star-import한 모듈에서 builtins.Exception이
# 가려져 `except Exception:`이 파이썬 예외를 잡지 못한다(.NET 예외만 매칭). builtins.Exception은
# pythonnet 3에서 .NET 예외도 포괄하므로 .NET 타입은 NetException 별칭으로만 노출한다.
from System import Exception as NetException
from System import Action, Attribute, Type, Environment
from System.Reflection import Assembly
from System import Array, String
from System.Threading.Tasks import Task
from System import Boolean

# Ensure references are loaded relative to this file so callers can import from any cwd.
_DLL_DIR = Path(__file__).resolve().parent
_MODULES_PARENT = _DLL_DIR.parents[1]  # modules
_MODULES_DIR = _DLL_DIR.parent  # modules/common
_MSG_DIR = _MODULES_DIR / "msg_files"

for _path in (_MODULES_PARENT, _MODULES_DIR, _DLL_DIR, _MSG_DIR):
    if _path.exists() and str(_path) not in sys.path:
        sys.path.append(str(_path))


def _runtime_version_key(_path):
    try:
        return tuple(int(_part) for _part in _path.name.split("."))
    except Exception:
        return (0,)


def _load_shared_framework_reference(_dll_name):
    _dotnet_root = Path(os.environ.get("DOTNET_ROOT", r"C:\Program Files\dotnet"))
    _shared_root = _dotnet_root / "shared"
    _runtime_version = str(Environment.Version)
    _frameworks = ("Microsoft.WindowsDesktop.App", "Microsoft.AspNetCore.App")

    _candidates = []
    for _framework in _frameworks:
        _framework_root = _shared_root / _framework
        _candidates.append(_framework_root / _runtime_version)
        if _framework_root.exists():
            try:
                _candidates.extend(
                    sorted(
                        (_p for _p in _framework_root.iterdir() if _p.is_dir()),
                        key=_runtime_version_key,
                        reverse=True,
                    )
                )
            except Exception:
                pass

    _seen = set()
    for _candidate in _candidates:
        _dll_path = _candidate / _dll_name
        _dll_key = str(_dll_path).lower()
        if _dll_key in _seen:
            continue
        _seen.add(_dll_key)
        if _dll_path.exists():
            try:
                clr.AddReference(str(_dll_path))
                return
            except Exception:
                pass


for _shared_dll in (
    "System.Diagnostics.EventLog.dll",
    "System.Diagnostics.PerformanceCounter.dll",
    "System.Security.Permissions.dll",
    "System.Threading.AccessControl.dll",
    "System.Windows.Extensions.dll",
    "System.Drawing.Common.dll",
    "Microsoft.Win32.SystemEvents.dll",
    "System.Formats.Nrbf.dll",
):
    _load_shared_framework_reference(_shared_dll)

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

for _dll in _DLL_REFERENCES:
    try:
        _dll_path = _DLL_DIR / f"{_dll}.dll"
        clr.AddReference(str(_dll_path if _dll_path.exists() else _dll))
    except Exception:
        pass

_MSG_ASM = _MSG_DIR / "MessageLibrary.dll"
if _MSG_ASM.exists():
    try:
        clr.AddReference(str(_MSG_ASM.resolve()))
    except Exception:
        try:
            clr.AddReference("MessageLibrary")
        except Exception:
            pass
else:
    try:
        clr.AddReference("MessageLibrary")
    except Exception:
        pass

# Optional: attempt to preload frequently used message namespaces so that
# subsequent `from nFusion.Model.msg_xxxx import *` succeeds even if the
# assemblies are not yet referenced by name.
try:
    import importlib

    for _mod in (
        "nFusion.Model.msg_0101",
        "nFusion.Model.msg_0102",
        "nFusion.Model.msg_0103",
    ):
        if _mod not in sys.modules:
            importlib.import_module(_mod)
except Exception:
    pass

from nFusion.Nodes.Core import *
from nFusion.Interface.Contracts import *
from nFusion.Nodes.Core.Ioc import *

#소비를 위한 인터페이스
from nFusion.Nodes.Core.Consumer import *

#공급을 위한 인터페이스
from nFusion.Nodes.Core.Provider import *
