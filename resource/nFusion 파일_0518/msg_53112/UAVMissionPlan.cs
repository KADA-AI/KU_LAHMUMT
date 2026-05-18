using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_53112;
using static System.Net.Mime.MediaTypeNames;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_53112
{
    public class UAVMissionPlan : nFusion.Model.msg_53112.UAVMissionPlan
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(missionPlanID ?? 0);
            bw.Write(aircraftID ?? 0);
            bw.Write(missionSegmentN ?? 0);

            foreach (var missionSegment in missionSegmentList ?? Array.Empty<MissionSegment>())
            {
                bw.Write(missionSegment?.missionSegmentID ?? 0);
                bw.Write(missionSegment?.isDone ?? false);
                bw.Write(missionSegment?.missionSegmentType ?? 0);
                bw.Write(missionSegment?.individualMissionListN ?? 0);

                foreach (var individualMission in missionSegment?.individualMissionList ?? Array.Empty<IndividualMission>())
                {
                    var allocatedArea = individualMission?.allocatedArea;

                    bw.Write(individualMission?.individualMissionID ?? 0);
                    bw.Write(individualMission?.isDone ?? false);
                    bw.Write(allocatedArea?.coordinateListN ?? 0);
                    if ((allocatedArea?.coordinateListN ?? 0) > 0)
                    {
                        foreach (var coordinate in allocatedArea?.coordinateList ?? Array.Empty<Coordinate>())
                        {
                            bw.Write(coordinate?.latitude ?? 0);
                            bw.Write(coordinate?.longitude ?? 0);
                            bw.Write(coordinate?.altitude ?? 0);
                        }
                    }

                    bw.Write(individualMission?.flightType ?? 0);
                    switch (individualMission?.flightType)
                    {
                        case 1:
                            bw.Write(individualMission?.waypointListN ?? 0);
                            foreach (var waypoint in individualMission?.waypointList ?? Array.Empty<Waypoint>())
                            {
                                var filmingProperty = waypoint?.filmingProperty;

                                bw.Write(waypoint?.waypointID ?? 0);
                                bw.Write(waypoint?.coordinate?.latitude ?? 0);
                                bw.Write(waypoint?.coordinate?.longitude ?? 0);
                                bw.Write(waypoint?.coordinate?.altitude ?? 0);
                                bw.Write(waypoint?.speed ?? 0);
                                bw.Write(waypoint?.eta ?? 0);
                                bw.Write(waypoint?.ecf ?? 0);
                                bw.Write(waypoint?.nextWaypointID ?? 0);
                                bw.Write(waypoint?.waypointPassType ?? 0);

                                if (waypoint?.waypointPassType == 2)
                                {
                                    bw.Write(waypoint?.loiterProperty?.radius ?? 0);
                                    bw.Write(waypoint?.loiterProperty?.direction ?? 0);
                                    bw.Write(waypoint?.loiterProperty?.time ?? 0);
                                    bw.Write(waypoint?.loiterProperty?.speed ?? 0);
                                }

                                bw.Write(waypoint?.filmingPlanned ?? false);

                                if (waypoint?.filmingPlanned == true)
                                {
                                    bw.Write(filmingProperty?.fieldOfView ?? 0);
                                    bw.Write(filmingProperty?.sensorType ?? 0);
                                    bw.Write(filmingProperty?.operationMode ?? 0);

                                    if (filmingProperty?.operationMode == 1 || filmingProperty?.operationMode == 2)
                                    {
                                        bw.Write(filmingProperty?.coordinateListN ?? 0);
                                        if ((filmingProperty?.coordinateListN ?? 0) > 0)
                                        {
                                            foreach (var coordinate in filmingProperty?.coordinateList ?? Array.Empty<Coordinate>())
                                            {
                                                bw.Write(coordinate?.latitude ?? 0);
                                                bw.Write(coordinate?.longitude ?? 0);
                                                bw.Write(coordinate?.altitude ?? 0);
                                            }
                                        }
                                    }

                                    if (filmingProperty?.operationMode == 2)
                                    {
                                        bw.Write(filmingProperty?.searchSpeed ?? 0);
                                    }

                                    if (filmingProperty?.operationMode == 3)
                                    {
                                        bw.Write(filmingProperty?.targetID ?? 0);
                                    }

                                    if (filmingProperty?.operationMode == 4)
                                    {
                                        bw.Write(filmingProperty?.sensorYaw ?? 0);
                                    }

                                    if (filmingProperty?.operationMode == 4 || filmingProperty?.operationMode == 5)
                                    {
                                        bw.Write(filmingProperty?.sensorPitch ?? 0);
                                    }

                                    if (filmingProperty?.operationMode == 5)
                                    {
                                        bw.Write(filmingProperty?.sensorYawAngularSpeed?.leftLimit ?? 0);
                                        bw.Write(filmingProperty?.sensorYawAngularSpeed?.rightLimit ?? 0);
                                        bw.Write(filmingProperty?.sensorYawAngularSpeed?.angularRate ?? 0);
                                    }
                                }
                            }
                            break;

                        case 2:
                            {
                                bw.Write(individualMission?.formationInfo?.leaderAircraftID ?? 0);
                                bw.Write(individualMission?.formationInfo?.formation?.dX ?? 0);
                                bw.Write(individualMission?.formationInfo?.formation?.dY ?? 0);
                                bw.Write(individualMission?.formationInfo?.formation?.dZ ?? 0);
                            }
                            break;

                        default:
                            throw new InvalidOperationException($"지원하지 않는 flightType: {individualMission?.flightType}");
                    }
                }
            }
            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static UAVMissionPlan Deserialize(byte[] data)
        {
            var obj = new UAVMissionPlan();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);

            obj.missionPlanID = br.ReadUInt32();
            obj.aircraftID = br.ReadUInt32();
            obj.missionSegmentN = br.ReadUInt32();
            obj.missionSegmentList = new MissionSegment[obj.missionSegmentN ?? 0];

            for (int i = 0; i < obj.missionSegmentN; i++)
            {

                var missionSegment = new MissionSegment
                {
                    missionSegmentID = br.ReadUInt32(),
                    isDone = br.ReadBoolean(),
                    missionSegmentType = br.ReadUInt32(),
                    individualMissionListN = br.ReadUInt32()
                };
                missionSegment.individualMissionList = new IndividualMission[missionSegment.individualMissionListN ?? 0];
                for (int j = 0; j < missionSegment.individualMissionListN; j++)
                {
                    var individualMission = new IndividualMission
                    {
                        individualMissionID = br.ReadUInt32(),
                        isDone = br.ReadBoolean(),
                        allocatedArea = new AllocatedArea
                        {
                            coordinateListN = br.ReadUInt32()
                        }
                    };

                    individualMission.allocatedArea.coordinateList = new Coordinate[individualMission.allocatedArea.coordinateListN ?? 0];
                    for (int k = 0; k < individualMission.allocatedArea.coordinateListN; k++)
                    {
                        Coordinate coord = new Coordinate
                        {
                            latitude = br.ReadSingle(),
                            longitude = br.ReadSingle(),
                            altitude = br.ReadInt32()
                        };
                        individualMission.allocatedArea.coordinateList[k] = coord;
                    }

                    individualMission.flightType = br.ReadUInt32();
                    switch (individualMission.flightType)
                    {
                        case 1:
                            individualMission.waypointListN = br.ReadUInt32();
                            individualMission.waypointList = new Waypoint[individualMission.waypointListN ?? 0];

                            for (int k = 0; k < individualMission.waypointListN; k++)
                            {
                                var waypoint = new Waypoint
                                {
                                    waypointID = br.ReadUInt32(),
                                };

                                waypoint.coordinate = new Coordinate
                                {
                                    latitude = br.ReadSingle(),
                                    longitude = br.ReadSingle(),
                                    altitude = br.ReadInt32()
                                };

                                waypoint.speed = br.ReadSingle();
                                waypoint.eta = br.ReadUInt32();
                                waypoint.ecf = br.ReadSingle();
                                waypoint.nextWaypointID = br.ReadUInt32();
                                waypoint.waypointPassType = br.ReadUInt32();

                                if (waypoint.waypointPassType == 2)
                                {
                                    waypoint.loiterProperty = new LoiterProperty
                                    {
                                        radius = br.ReadUInt32(),
                                        direction = br.ReadUInt32(),
                                        time = br.ReadUInt32(),
                                        speed = br.ReadSingle()
                                    };
                                }

                                waypoint.filmingPlanned = br.ReadBoolean();
                                if (waypoint.filmingPlanned == true)
                                {
                                    waypoint.filmingProperty = new FilmingProperty
                                    {
                                        fieldOfView = br.ReadSingle(),
                                        sensorType = br.ReadUInt32(),
                                        operationMode = br.ReadUInt32(),
                                    };

                                    if (waypoint.filmingProperty.operationMode == 1 || waypoint.filmingProperty.operationMode == 2)
                                    {
                                        waypoint.filmingProperty.coordinateListN = br.ReadUInt32();
                                        waypoint.filmingProperty.coordinateList = new Coordinate[waypoint.filmingProperty.coordinateListN ?? 0];
                                        for (int l = 0; l < waypoint.filmingProperty.coordinateListN; l++)
                                        {
                                            var coordinate = new Coordinate
                                            {
                                                latitude = br.ReadSingle(),
                                                longitude = br.ReadSingle(),
                                                altitude = br.ReadInt32()
                                            };
                                            waypoint.filmingProperty.coordinateList[l] = coordinate;
                                        }
                                    }

                                    if (waypoint.filmingProperty.operationMode == 2)
                                    {
                                        waypoint.filmingProperty.searchSpeed = br.ReadSingle();
                                    }

                                    if (waypoint.filmingProperty.operationMode == 3)
                                    {
                                        waypoint.filmingProperty.targetID = br.ReadUInt32();
                                    }

                                    if (waypoint.filmingProperty.operationMode == 4)
                                    {
                                        waypoint.filmingProperty.sensorYaw = br.ReadSingle();
                                    }

                                    if (waypoint.filmingProperty.operationMode == 4 || waypoint.filmingProperty.operationMode == 5)
                                    {
                                        waypoint.filmingProperty.sensorPitch = br.ReadSingle();
                                    }

                                    if (waypoint.filmingProperty.operationMode == 5)
                                    {
                                        waypoint.filmingProperty.sensorYawAngularSpeed = new SensorYawAngularSpeed
                                        {
                                            leftLimit = br.ReadSingle(),
                                            rightLimit = br.ReadSingle(),
                                            angularRate = br.ReadSingle()
                                        };
                                    }
                                }

                                individualMission.waypointList[k] = waypoint;
                            }

                            break;

                        case 2:

                            individualMission.formationInfo = new FormationInfo
                            {
                                leaderAircraftID = br.ReadUInt32(),
                            };
                            individualMission.formationInfo.formation = new Formation
                            {
                                dX = br.ReadInt32(),
                                dY = br.ReadInt32(),
                                dZ = br.ReadInt32()
                            };
                            break;

                        default:
                            throw new InvalidOperationException($"지원하지 않는 flightType: {individualMission.flightType}");
                    }
                    missionSegment.individualMissionList[j] = individualMission;
                }
                obj.missionSegmentList[i] = missionSegment;
            }
            return obj;
        }
    }
}
