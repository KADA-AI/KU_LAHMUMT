using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0903;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0903 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            RequestRenewMission requestRenewMission = new RequestRenewMission
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                missionPlanID = 70000001
            };

            return requestRenewMission;
        }
    }
}