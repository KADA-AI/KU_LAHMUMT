using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0202;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0202 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            nFusion.Model.msg_0202.PriorMissionInfo priorMissionInfo = new nFusion.Model.msg_0202.PriorMissionInfo
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
            };

            priorMissionInfo.priorMissionList = new List<nFusion.Model.CommonType.PriorMission>();
            nFusion.Model.CommonType.PriorMission priorMission = new nFusion.Model.CommonType.PriorMission
            {
                priorMissionID = 1,
                missionType = 1,
                coordinateOrientation = new CoordinateOrientation {
                    coordinate = new Coordinate
                    {
                        latitude = 37.5665f,
                        longitude = 126.9780f,
                        altitude = 100
                    }
                },
                targetOrientation = new TargetOrientation { targetID = 7 },
            };

            priorMissionInfo.priorMissionList.Add(priorMission);

            return priorMissionInfo;
        }
    }
}