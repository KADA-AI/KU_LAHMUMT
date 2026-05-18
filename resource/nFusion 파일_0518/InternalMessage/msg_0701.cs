using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0701;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0701 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            MissionPlanOptionInfo missionPlanOptionInfo = new MissionPlanOptionInfo
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                autoExecution = false,
            };

            missionPlanOptionInfo.optionList = new List<Option>
            {
                new Option
                {
                    optionID = 1,
                    optionName = 1,
                    missionPlanID = 700000002,
                    survivalRate = 0,
                    timeContraction = 0,
                    recogEffectiveness = 0,
                    fuelWarning = null,
                    distance = 50000,
                    target = 0
                },
                new Option
                {
                    optionID = 2,
                    optionName = 4,
                    missionPlanID = 700000003,
                    survivalRate = -1,
                    timeContraction = 0,
                    recogEffectiveness = 1,
                    fuelWarning = null,
                    distance = 50000,
                    target = 0
                },
                new Option{

                    optionID = 3,
                    optionName = 5,
                    missionPlanID = 700000004,
                    survivalRate = 0,
                    timeContraction = 1,
                    recogEffectiveness = -1,
                    fuelWarning = null,
                    distance = 50000,
                    target = 0
                }
            };

            return missionPlanOptionInfo;
        }
    }
}