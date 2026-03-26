using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0802;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0802 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            MandatoryCommand mandatoryCommand = new MandatoryCommand
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                aircraftID = 4,
                mandatoryType = 1
            };

            return mandatoryCommand;
        }
    }
}