using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0803;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0803 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            ExecutionCommand executionCommand = new ExecutionCommand
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                execute = 1
            };

            return executionCommand;
        }
    }
}