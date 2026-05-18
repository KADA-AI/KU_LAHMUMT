using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0305 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            return new nFusion.Model.msg_0305.ReplanStatus { timestamp = Utility.GenerateTimestampUlong(), source = "Dummy", missionPlanningStatus = 1, replanReason = "초기임무계획"};
        }
    }
}