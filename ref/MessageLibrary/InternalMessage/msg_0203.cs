using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0203;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0203 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            FlightReferenceInfo flightReferenceInfo= new FlightReferenceInfo
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                missionReferencePackageID = 0
            };

            return flightReferenceInfo;
        }
    }
}