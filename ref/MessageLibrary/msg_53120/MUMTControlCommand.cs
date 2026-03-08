using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_53120;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_53120
{
    public class MUMTControlCommand : nFusion.Model.msg_53120.MUMTControlCommand
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(aircraftID ?? 0);
            bw.Write(controlType ?? 0);

            // FlightModeCommand
            if (controlType == 1 || controlType == 3)
            {
                bw.Write(flightModeCommand.flightMode ?? 0);

                // FormationProperty
                if (flightModeCommand.flightMode == 6)
                {
                    bw.Write(flightModeCommand.formationProperty.leaderAircraftID ?? 0);
                    bw.Write(flightModeCommand.formationProperty.formation.dX ?? 0);
                    bw.Write(flightModeCommand.formationProperty.formation.dY ?? 0);
                    bw.Write(flightModeCommand.formationProperty.formation.dZ ?? 0);
                }
                // PathFollowing
                if (flightModeCommand.flightMode == 7)
                {
                    bw.Write(flightModeCommand.pathFollowing.waypointID ?? 0);
                }
                // LoiterProperty
                if (flightModeCommand.flightMode == 8)
                {
                    bw.Write(flightModeCommand.loiterProperty.radius ?? 0);
                    bw.Write(flightModeCommand.loiterProperty.direction ?? 0);
                    bw.Write(flightModeCommand.loiterProperty.coordinate.latitude ?? 0);
                    bw.Write(flightModeCommand.loiterProperty.coordinate.longitude ?? 0);
                    bw.Write(flightModeCommand.loiterProperty.coordinate.altitude ?? 0);
                    bw.Write(flightModeCommand.loiterProperty.time ?? 0);
                    bw.Write(flightModeCommand.loiterProperty.speed ?? 0);
                }
                // TargetTracking
                if (flightModeCommand.flightMode == 9)
                {
                    bw.Write(flightModeCommand.targetTracking.targetID ?? 0);
                }
            }

            if (controlType == 2 || controlType == 3)
            {
                // CameraModeCommand
                bw.Write(cameraModeCommand.fieldOfView ?? 0);
                bw.Write(cameraModeCommand.sensorType ?? 0);
                bw.Write(cameraModeCommand.operationMode ?? 0);

                if (cameraModeCommand.operationMode == 1 || cameraModeCommand.operationMode == 2)
                {
                    bw.Write(cameraModeCommand.coordinateListN ?? 0);
                    foreach (var coordinate in cameraModeCommand.coordinateList)
                    {
                        bw.Write(coordinate.latitude ?? 0);
                        bw.Write(coordinate.longitude ?? 0);
                        bw.Write(coordinate.altitude ?? 0);
                    }
                }


                if (cameraModeCommand.operationMode == 2)
                {
                    bw.Write(cameraModeCommand.searchSpeed ?? 0);
                }

                if (cameraModeCommand.operationMode == 3)
                {
                    bw.Write(cameraModeCommand.targetID ?? 0);
                }
                if (cameraModeCommand.operationMode == 4)
                {
                    bw.Write(cameraModeCommand.sensorYaw ?? 0);
                    bw.Write(cameraModeCommand.sensorPitch ?? 0);
                }

                if (cameraModeCommand.operationMode == 5)
                {
                    bw.Write(cameraModeCommand.sensorPitch ?? 0);
                    bw.Write(cameraModeCommand.sensorYawAngularSpeed.leftLimit ?? 0);
                    bw.Write(cameraModeCommand.sensorYawAngularSpeed.rightLimit ?? 0);
                    bw.Write(cameraModeCommand.sensorYawAngularSpeed.angularRate ?? 0);
                }

            }
                

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static MUMTControlCommand Deserialize(byte[] data)
        {
            var obj = new MUMTControlCommand();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);

            obj.aircraftID = br.ReadUInt32();
            obj.controlType = br.ReadUInt32();

            // FlightModeCommand
            if (obj.controlType == 1 || obj.controlType == 3)
            {
                obj.flightModeCommand = new FlightModeCommand();
                obj.flightModeCommand.flightMode = br.ReadUInt32();

                // FormationProperty
                if (obj.flightModeCommand.flightMode == 6)
                {
                    obj.flightModeCommand.formationProperty = new FormationProperty
                    {
                        leaderAircraftID = br.ReadUInt32(),
                        formation = new Formation
                        {
                            dX = br.ReadInt32(),
                            dY = br.ReadInt32(),
                            dZ = br.ReadInt32(),
                        }
                    };
                }

                // PathFollowing
                if (obj.flightModeCommand.flightMode == 7)
                {
                    obj.flightModeCommand.pathFollowing = new PathFollowing
                    {
                        waypointID = br.ReadUInt32()
                    };
                }

                // LoiterProperty
                if (obj.flightModeCommand.flightMode == 8)
                {
                    obj.flightModeCommand.loiterProperty = new LoiterProperty
                    {
                        radius = br.ReadUInt32(),
                        direction = br.ReadUInt32()
                    };
                    
                    obj.flightModeCommand.loiterProperty.coordinate = new Coordinate
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    };

                    obj.flightModeCommand.loiterProperty.time = br.ReadUInt32();
                    obj.flightModeCommand.loiterProperty.speed = br.ReadSingle();
                }

                // TargetTracking
                if (obj.flightModeCommand.flightMode == 9)
                {
                    obj.flightModeCommand.targetTracking = new TargetTracking
                    {
                        targetID = br.ReadUInt32(),
                    };
                }

            }

            // CameraModeCommand
            if (obj.controlType == 2 || obj.controlType == 3)
            {
                obj.cameraModeCommand = new CameraModeCommand();
                obj.cameraModeCommand.fieldOfView = br.ReadSingle();
                obj.cameraModeCommand.sensorType = br.ReadUInt32();
                obj.cameraModeCommand.operationMode = br.ReadUInt32();

                if (obj.cameraModeCommand.operationMode == 1 || obj.cameraModeCommand.operationMode == 2)
                {
                    obj.cameraModeCommand.coordinateListN = br.ReadUInt32();
                    obj.cameraModeCommand.coordinateList = new Coordinate[obj.cameraModeCommand.coordinateListN ?? 0];

                    for (int i = 0; i < obj.cameraModeCommand.coordinateListN; i++)
                    {
                        obj.cameraModeCommand.coordinateList[i] = new Coordinate
                        {
                            latitude = br.ReadSingle(),
                            longitude = br.ReadSingle(),
                            altitude = br.ReadInt32()
                        };
                    }
                }

                if (obj.cameraModeCommand.operationMode == 2)
                {
                    obj.cameraModeCommand.searchSpeed = br.ReadSingle();
                }
                if (obj.cameraModeCommand.operationMode == 3)
                {
                    obj.cameraModeCommand.targetID = br.ReadUInt32();
                }
                if (obj.cameraModeCommand.operationMode == 4)
                {
                    obj.cameraModeCommand.sensorYaw = br.ReadSingle();
                }
                if (obj.cameraModeCommand.operationMode == 4 || obj.cameraModeCommand.operationMode == 5)
                {
                    obj.cameraModeCommand.sensorPitch = br.ReadSingle();
                }

                if (obj.cameraModeCommand.operationMode == 5)
                {
                    obj.cameraModeCommand.sensorYawAngularSpeed = new SensorYawAngularSpeed
                    {
                        leftLimit = br.ReadSingle(),
                        rightLimit = br.ReadSingle(),
                        angularRate = br.ReadSingle()
                    };
                }
            }

            return obj;
        }
    }
}
