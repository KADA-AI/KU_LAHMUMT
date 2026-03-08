using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0702;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0702 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            PilotDecision pilotDecision = new PilotDecision
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                ignore = 0,
                missionPlanID = 10
            };

            return pilotDecision;
        }
    }
}