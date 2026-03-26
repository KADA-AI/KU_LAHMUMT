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
    public class Dummy0102 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            return new nFusion.Model.msg_0102.ModuleStatus { timestamp = Utility.GenerateTimestampUlong(), source = "Dummy", status = 1 };
        }
    }
}
