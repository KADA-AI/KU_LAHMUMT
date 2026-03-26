using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0806;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0806 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            BootCommand bootCommand = new BootCommand
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                command = 1
            };

            return bootCommand;
        }
    }
}