using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0501;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0501 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            MissionProgress missionProgress = new MissionProgress
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                currentMissionPlanID = 700000000,
                currentInputMissionID = 1,

            };

            missionProgress.individualMissionProgressStatusList = new List<IndividualMissionProgressStatus>();

            IndividualMissionProgressStatus individualMissionProgressStatus = new IndividualMissionProgressStatus
            {
                aircraftID = 4,
                currentIndividualMission = new CurrentIndividualMission
                {
                    individualMissionID = 1
                },
                currentIndividualMissionProgress = 0
            };

            missionProgress.individualMissionProgressStatusList.Add(individualMissionProgressStatus);

            return missionProgress;
        }
    }
}