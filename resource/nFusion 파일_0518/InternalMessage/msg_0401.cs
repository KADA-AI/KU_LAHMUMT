using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0401;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0401 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            AgentStatus agentStatus = new AgentStatus
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                agentStateList = new List<AgentState>
                {
                    new AgentState
                    {
                        aircraftID = 1,
                        isUnmanned = true,
                        coordinate = new Coordinate
                        {
                            latitude = 37.5665f,
                            longitude = 126.9780f,
                            altitude = 100
                        },
                        velocity = new Velocity
                        {
                            speed = 10.0f,
                            heading = 30.0f
                        },
                        fuel = 100.0f,
                        health = 1,
                        mannedInfo = new MannedInfo
                        {
                            weapons = new Weapons { type1 = 1, type2 = 1, type3 = 1 },
                            datalinkStatus = new DatalinkStatus
                            {
                                isConnectedToUAV1 = true,
                                isConnectedToUAV2 = true,
                                isConnectedToUAV3 = true
                            }
                        },
                        unmannedInfo = new UnmannedInfo
                        {
                            currentWaypointID = new CurrentWaypointID
                            {
                                waypointID = 1
                            },
                            flightMode = 6,
                            leaderAircraftID = new LeaderAircraftID
                            {
                                aircraftID = 1
                            },
                            sensorInfo = new SensorInfo
                            {
                                operationalMode = 1,
                                sensorType = 1,
                                fov = 20.0f,
                                centerCoordinate = new CenterCoordinate
                                {
                                    latitude = 37.5665f,
                                    longitude = 126.9780f,
                                    altitude = 100
                                },
                                footprintCornerList = new FootprintCorner[4] // <-- 여기서 미리 할당
                            },
                            payloadHealth = 1,
                            fuelWarning = 0
                        }
                    }
                }
            };

            agentStatus.agentStateList![0].unmannedInfo!.sensorInfo!.footprintCornerList![0] = new FootprintCorner
            {
                latitude = 37.5665f,
                longitude = 126.9780f,
                altitude = 100
            };

            agentStatus.agentStateList[0].unmannedInfo!.sensorInfo!.footprintCornerList![1] = new FootprintCorner
            {
                latitude = 37.5666f,
                longitude = 126.9781f,
                altitude = 100
            };

            agentStatus.agentStateList[0].unmannedInfo!.sensorInfo!.footprintCornerList![2] = new FootprintCorner
            {
                latitude = 37.5667f,
                longitude = 126.9782f,
                altitude = 100
            };

            agentStatus.agentStateList[0].unmannedInfo!.sensorInfo!.footprintCornerList![3] = new FootprintCorner
            {
                latitude = 37.5668f,
                longitude = 126.9783f,
                altitude = 100
            };

            return agentStatus;
        }
    }
}