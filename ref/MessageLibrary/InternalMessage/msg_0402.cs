using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0402;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0402 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            SituationAwarenessInfo situationAwarenessInfo = new SituationAwarenessInfo
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                roiInfo = new ROIInfo
                {
                    aircraftID = 1,
                    coordinate = new Coordinate
                    {
                        latitude = 37.5665f,
                        longitude = 126.9780f,
                        altitude = 100
                    },
                    fov = 60.0f
                },
                targetList = new List<Target>
                {
                    new Target
                    {
                        targetID = 7,
                        targetType = 1,
                        coordinate = new Coordinate
                        {
                            latitude = 37.5665f,
                            longitude = 126.9780f,
                            altitude = 100
                        },
                        watcher = new Watcher
                        {
                            aircraftID = 1
                        },
                        targetInFrame = true,
                        isDestroyed = false,
                        threat = 0.5f
                    }
                }
            };

            return situationAwarenessInfo;
        }
    }
}