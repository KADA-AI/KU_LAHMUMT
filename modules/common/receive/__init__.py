# -*- coding: utf-8 -*-
# modules/common/receive/__init__.py

from dll_files.nFusionImports import FusionNodeIoc

# ① 각 리시버 모듈 import (존재하는 파일 전부 나열)
from .message0000_receiver import ResponseReceiver_0000

from .message0101_receiver import SystemOperationModeReceiver_0101
from .message0102_receiver import ModuleStatusReceiver_0102  # 클래스명은 네 구현에 맞게
from .message0103_receiver import SWStatusReceiver_0103      # 클래스명은 네 구현에 맞게

from .message0201_receiver import InputMissionPlanReceiver_0201
from .message0202_receiver import PriorMissionInfoReceiver_0202
from .message0203_receiver import FlightReferenceInfoReceiver_0203

from .message0301_receiver import MissionPlanReceiver_0301
from .message0302_receiver import IndividualMissionPlanReceiver_0302
from .message0303_receiver import UAVFlightPlanReceiver_0303
from .message0304_receiver import LAHFlightPlanReceiver_0304
from .message0305_receiver import ReplanStatusReceiver_0305           # 클래스명은 네 구현에 맞게

from .message0401_receiver import LAHStatusReceiver_0401
from .message0402_receiver import SituationAwarenessInfoReceiver_0402

from .message0501_receiver import MissionStateInfoReceiver_0501
from .message0502_receiver import EndMissionRequestReceiver_0502     # 클래스명은 네 구현에 맞게
from .message0503_receiver import MissionResultReceiver_0503     # 클래스명은 네 구현에 맞게

from .message0601_receiver import BasicActionReceiver_0601           # 클래스명은 네 구현에 맞게

from .message0701_receiver import MissionPlanOptionReceiver_0701
from .message0702_receiver import MissionProgressReceiver_0702

from .message0801_receiver import ReplanCommandReceiver_0801
from .message0802_receiver import MandatoryCommandReceiver_0802
from .message0803_receiver import StartNextMissionCommandReceiver_0803
# from .message0804_receiver import MissionRestartCommandReceiver_0804  # 필요 시 주석 해제
from .message0805_receiver import EndMissionCommandReceiver_0805
from .message0806_receiver import EndSWCommandReceiver_0806

from .message0901_receiver import RequestOptionInfoReceiver_0901
from .message0902_receiver import ReplanRequestReceiver_0902
from .message0903_receiver import RequestRenewMissionReceiver_0903

# ② 대표 타입 하나만 등록 (중복 등록 방지 가드 포함)
try:
    _RECEIVE_CONSUMERS_REGISTERED  # noqa: F401
except NameError:
    _RECEIVE_CONSUMERS_REGISTERED = False

if not _RECEIVE_CONSUMERS_REGISTERED:
    FusionNodeIoc.AddConsumerFromAssemblyContainsType(ResponseReceiver_0000)
    _RECEIVE_CONSUMERS_REGISTERED = True
