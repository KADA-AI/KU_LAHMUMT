#파이선에서 C#을 이용 pip로 pythonnet 설치 필요
from pythonnet import load
load("coreclr")
import clr
import sys
from pathlib import Path
from System.Threading.Tasks import Task
from System import Exception, Action, Attribute, Type
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
