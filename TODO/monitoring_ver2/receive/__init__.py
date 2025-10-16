# -*- coding: utf-8 -*-
# modules/common/receive/__init__.py
import sys, os

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from dll_files.nFusionImports import FusionNodeIoc

# ① 각 리시버 모듈 import (존재하는 파일 전부 나열)

from .message0101_receiver import SystemOperationModeReceiver_0101
from .message0201_receiver import InputMissionPlanReceiver_0201
from .message0202_receiver import PriorMissionInfoReceiver_0202
from .message0203_receiver import FlightReferenceInfoReceiver_0203

from .message0301_receiver import MissionPlanReceiver_0301
from .message0302_receiver import IndividualMissionPlanReceiver_0302
from .message0303_receiver import UAVFlightPlanReceiver_0303
from .message0304_receiver import LAHFlightPlanReceiver_0304

from .message0401_receiver import AgentStatusReceiver_0401
from .message0402_receiver import SituationAwarenessInfoReceiver_0402

from .message0601_receiver import BasicActionReceiver_0601  # 클래스명은 네 구현에 맞게

from .message0702_receiver import PilotDecisionReceiver_0702

from .message0801_receiver import InitialPlanCommandReceiver_0801
from .message0802_receiver import MandatoryCommandReceiver_0802
from .message0803_receiver import ExecutionCommandReceiver_0803

# from .message0804_receiver import MissionRestartCommandReceiver_0804  # 필요 시 주석 해제
from .message0805_receiver import SystemEventReceiver_0805
from .message0806_receiver import BootCommandReceiver_0806

from .message0903_receiver import RequestRenewMissionReceiver_0903


FusionNodeIoc.AddConsumerFromAssemblyContainsType(SystemOperationModeReceiver_0101)
