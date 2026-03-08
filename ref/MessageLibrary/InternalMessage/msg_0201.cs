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
    public class Dummy0201 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            return new nFusion.Model.msg_0201.InputMissionPlan { timestamp = Utility.GenerateTimestampUlong(), source = "Dummy", inputMissionPackageID = 100};
        }
    }
}