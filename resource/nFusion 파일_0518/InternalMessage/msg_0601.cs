using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0601;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0601 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            BasicAction basicAction = new BasicAction
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                aircraftID = 4,
                flightMode = 1,
                filmingMode = 1
            };

            return basicAction;
        }
    }
}