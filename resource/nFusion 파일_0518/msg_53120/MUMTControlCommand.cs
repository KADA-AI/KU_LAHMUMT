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

            var flightModeCommandInfo = flightModeCommand;
            var cameraModeCommandInfo = cameraModeCommand;

            // FlightModeCommand
            if (controlType == 1 || controlType == 3)
            {
                bw.Write(flightModeCommandInfo?.flightMode ?? 0);

                // FormationProperty
                if (flightModeCommandInfo?.flightMode == 6)
                {
                    bw.Write(flightModeCommandInfo?.formationProperty?.leaderAircraftID ?? 0);
                    bw.Write(flightModeCommandInfo?.formationProperty?.formation?.dX ?? 0);
                    bw.Write(flightModeCommandInfo?.formationProperty?.formation?.dY ?? 0);
                    bw.Write(flightModeCommandInfo?.formationProperty?.formation?.dZ ?? 0);
                }
                // PathFollowing
                if (flightModeCommandInfo?.flightMode == 7)
                {
                    bw.Write(flightModeCommandInfo?.pathFollowing?.waypointID ?? 0);
                }
                // LoiterProperty
                if (flightModeCommandInfo?.flightMode == 8)
                {
                    bw.Write(flightModeCommandInfo?.loiterProperty?.radius ?? 0);
                    bw.Write(flightModeCommandInfo?.loiterProperty?.direction ?? 0);
                    bw.Write(flightModeCommandInfo?.loiterProperty?.coordinate?.latitude ?? 0);
                    bw.Write(flightModeCommandInfo?.loiterProperty?.coordinate?.longitude ?? 0);
                    bw.Write(flightModeCommandInfo?.loiterProperty?.coordinate?.altitude ?? 0);
                    bw.Write(flightModeCommandInfo?.loiterProperty?.time ?? 0);
                    bw.Write(flightModeCommandInfo?.loiterProperty?.speed ?? 0);
                }
                // TargetTracking
                if (flightModeCommandInfo?.flightMode == 9)
                {
                    bw.Write(flightModeCommandInfo?.targetTracking?.targetID ?? 0);
                }
            }

            if (controlType == 2 || controlType == 3)
            {
                // CameraModeCommand
                bw.Write(cameraModeCommandInfo?.fieldOfView ?? 0);
                bw.Write(cameraModeCommandInfo?.sensorType ?? 0);
                bw.Write(cameraModeCommandInfo?.operationMode ?? 0);

                if (cameraModeCommandInfo?.operationMode == 1 || cameraModeCommandInfo?.operationMode == 2)
                {
                    bw.Write(cameraModeCommandInfo?.coordinateListN ?? 0);
                    foreach (var coordinate in cameraModeCommandInfo?.coordinateList ?? Array.Empty<Coordinate>())
                    {
                        bw.Write(coordinate?.latitude ?? 0);
                        bw.Write(coordinate?.longitude ?? 0);
                        bw.Write(coordinate?.altitude ?? 0);
                    }
                }


                if (cameraModeCommandInfo?.operationMode == 2)
                {
                    bw.Write(cameraModeCommandInfo?.searchSpeed ?? 0);
                }

                if (cameraModeCommandInfo?.operationMode == 3)
                {
                    bw.Write(cameraModeCommandInfo?.targetID ?? 0);
                }
                if (cameraModeCommandInfo?.operationMode == 4)
                {
                    bw.Write(cameraModeCommandInfo?.sensorYaw ?? 0);
                    bw.Write(cameraModeCommandInfo?.sensorPitch ?? 0);
                }

                if (cameraModeCommandInfo?.operationMode == 5)
                {
                    bw.Write(cameraModeCommandInfo?.sensorPitch ?? 0);
                    bw.Write(cameraModeCommandInfo?.sensorYawAngularSpeed?.leftLimit ?? 0);
                    bw.Write(cameraModeCommandInfo?.sensorYawAngularSpeed?.rightLimit ?? 0);
                    bw.Write(cameraModeCommandInfo?.sensorYawAngularSpeed?.angularRate ?? 0);
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
