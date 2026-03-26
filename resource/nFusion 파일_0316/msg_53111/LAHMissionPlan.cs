using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_53111;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_53111
{
    public class LAHMissionPlan : nFusion.Model.msg_53111.LAHMissionPlan
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

            foreach (var missionSegment in missionSegmentList)
            {
                bw.Write(missionSegment.missionSegmentID ?? 0);
                bw.Write(missionSegment.isDone ?? false);
                bw.Write(missionSegment.missionSegmentType ?? 0);
                bw.Write(missionSegment.individualMissionListN ?? 0);

                foreach (var individualMission in missionSegment.individualMissionList)
                {
                    bw.Write(individualMission.individualMissionID ?? 0);
                    bw.Write(individualMission.isDone ?? false);
                    bw.Write(individualMission.waypointListN ?? 0);

                    if (individualMission.waypointListN > 0)
                    {
                        foreach (var waypoint in individualMission.waypointList)
                        {
                            bw.Write(waypoint.waypointID ?? 0);
                            bw.Write(waypoint.coordinate.latitude ?? 0);
                            bw.Write(waypoint.coordinate.longitude ?? 0);
                            bw.Write(waypoint.coordinate.altitude ?? 0);
                            bw.Write(waypoint.speed ?? 0);
                            bw.Write(waypoint.eta ?? 0);
                            bw.Write(waypoint.nextWaypointID ?? 0);
                            bw.Write(waypoint.hovering ?? 0);
                            bw.Write(waypoint.attack.targetID ?? 0);
                            bw.Write(waypoint.attack.weaponType ?? 0);
                        }
                    }
                }
            }
            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static LAHMissionPlan Deserialize(byte[] data)
        {
            var obj = new LAHMissionPlan();

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
                        waypointListN = br.ReadUInt32()
                    };
                    individualMission.waypointList = new Waypoint[individualMission.waypointListN ?? 0];
                    for (int k = 0; k < individualMission.waypointListN; k++)
                    {
                        var waypoint = new Waypoint
                        {
                            waypointID = br.ReadUInt32(),
                            coordinate = new Coordinate
                            {
                                latitude = br.ReadSingle(),
                                longitude = br.ReadSingle(),
                                altitude = br.ReadInt32()
                            },
                            speed = br.ReadSingle(),
                            eta = br.ReadUInt32(),
                            nextWaypointID = br.ReadUInt32(),
                            hovering = br.ReadUInt32(),
                            attack = new Attack
                            {
                                targetID = br.ReadUInt32(),
                                weaponType = br.ReadUInt32()
                            }
                        };
                        individualMission.waypointList[k] = waypoint;
                    }

                    missionSegment.individualMissionList[j] = individualMission;
                }
                obj.missionSegmentList[i] = missionSegment;
            }
            return obj;
        }
    }
}
