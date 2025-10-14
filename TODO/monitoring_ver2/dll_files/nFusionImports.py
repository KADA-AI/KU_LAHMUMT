#파이선에서 C#을 이용 pip로 pythonnet 설치 필요
import os # Added for os.path.join
from pythonnet import load
load("coreclr")
import clr
from System.Threading.Tasks import Task
from System import Exception, Action, Attribute, Type
from System.Reflection import Assembly
from System import Array, String
from System.Threading.Tasks import Task
from System import Boolean

# Get the directory of the current script (nFusionImports.py)
current_dir = os.path.dirname(os.path.abspath(__file__))
dll_path = os.path.abspath(os.path.join(current_dir))

# nFusion 프레임워크 및 종속성 로드
# Using os.path.join for robustness
clr.AddReference(os.path.join(dll_path, 'Microsoft.Bcl.AsyncInterfaces'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.Configuration.Abstractions'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.Configuration'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.Configuration.FileExtensions'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.Configuration.Json'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.DependencyInjection.Abstractions'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.DependencyInjection'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.FileProviders.Abstractions'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.FileProviders.Physical'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.FileSystemGlobbing'))
clr.AddReference(os.path.join(dll_path, 'Microsoft.Extensions.Primitives'))
clr.AddReference(os.path.join(dll_path, 'nFusion.Core'))
clr.AddReference(os.path.join(dll_path, 'nFusion.Interface.Contracts'))
clr.AddReference(os.path.join(dll_path, 'nFusion.Nodes.Core'))
clr.AddReference(os.path.join(dll_path, 'nFusion.SimpleMiddleware'))
for _msg_candidate in (
    os.path.join(os.path.dirname(current_dir), 'msg_files', 'MessageLibrary'),
    os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'common', 'msg_files', 'MessageLibrary'),
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(current_dir))), 'modules', 'common', 'msg_files', 'MessageLibrary'),
):
    try:
        clr.AddReference(_msg_candidate)
        break
    except Exception:
        continue
else:
    clr.AddReference('MessageLibrary')

from nFusion.Nodes.Core import *
from nFusion.Interface.Contracts import *
from nFusion.Nodes.Core.Ioc import *

#소비를 위한 인터페이스
from nFusion.Nodes.Core.Consumer import *

#공급을 위한 인터페이스
from nFusion.Nodes.Core.Provider import *