using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0502;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0502 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            EndMissionRequest endMissionRequest = new EndMissionRequest
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy"
            };

            return endMissionRequest;
        }
    }
}