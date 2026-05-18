using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0504;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0504 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            FuelWarning fuelWarning = new FuelWarning
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                aircraftID = 4, // 무인기 번호
                fuelLevel = 1
            };

            return fuelWarning;
        }
    }
}