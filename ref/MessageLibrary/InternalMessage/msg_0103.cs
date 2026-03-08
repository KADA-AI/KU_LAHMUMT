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
    public class Dummy0103 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            return new nFusion.Model.msg_0103.SWStatus { timestamp = Utility.GenerateTimestampUlong(), source = "Dummy", status = 1, mode = 2};
        }
    }
}