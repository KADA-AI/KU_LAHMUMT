#파이선에서 C#을 이용 pip로 pythonnet 설치 필요
from pythonnet import load
load("coreclr")
import clr
from System.Threading.Tasks import Task
from System import Exception, Action, Attribute, Type
from System.Reflection import Assembly
from System import Array, String
from System.Threading.Tasks import Task
from System import Boolean

#nFusion 프레임워크 로드
clr.AddReference('./dll_files/nFusion.Interface.Contracts')
clr.AddReference('./dll_files/nFusion.Nodes.Core')

from nFusion.Nodes.Core import *
from nFusion.Interface.Contracts import *
from nFusion.Nodes.Core.Ioc import *

#소비를 위한 인터페이스
from nFusion.Nodes.Core.Consumer import *

#공급을 위한 인터페이스
from nFusion.Nodes.Core.Provider import *