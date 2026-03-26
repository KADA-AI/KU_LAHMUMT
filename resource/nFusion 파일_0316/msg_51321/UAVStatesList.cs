using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51321;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51321
{
    public class UAVStatesList : nFusion.Model.msg_51321.UAVStatesList
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(uavStatesN ?? 0);

            foreach (var uavStates in uavStates)
            {
                bw.Write(uavStates.aircraftID ?? 0);
                bw.Write(uavStates.coordinate.latitude ?? 0);
                bw.Write(uavStates.coordinate.longitude ?? 0);
                bw.Write(uavStates.coordinate.altitude ?? 0);
                bw.Write(uavStates.velocity.speed ?? 0);
                bw.Write(uavStates.velocity.heading ?? 0);
                bw.Write(uavStates.attitude.roll ?? 0);
                bw.Write(uavStates.attitude.pitch ?? 0);
                bw.Write(uavStates.attitude.yaw ?? 0);
                bw.Write(uavStates.fuel ?? 0);

                bw.Write(uavStates.flightMode.flightMode ?? 0);
                bw.Write(uavStates.flightMode.onMission ?? 0);

                if (uavStates.flightMode.flightMode == 1 || uavStates.flightMode.flightMode == 2
                    || uavStates.flightMode.flightMode == 3 || uavStates.flightMode.flightMode == 4 || uavStates.flightMode.flightMode == 7)
                {
                    bw.Write(uavStates.flightMode.currentWaypointID ?? 0);
                }

                if (uavStates.flightMode.flightMode == 6)
                {
                    bw.Write(uavStates.flightMode.leaderID ?? 0);
                }

                if (uavStates.flightMode.flightMode == 8)
                {
                    bw.Write(uavStates.flightMode.coordinate.latitude ?? 0);
                    bw.Write(uavStates.flightMode.coordinate.longitude ?? 0);
                    bw.Write(uavStates.flightMode.coordinate.altitude ?? 0);
                }

                if (uavStates.flightMode.flightMode == 9)
                {
                    bw.Write(uavStates.flightMode.targetID ?? 0);
                }

                bw.Write(uavStates.cameraMode.sensorType ?? 0);
                bw.Write(uavStates.cameraMode.operationMode ?? 0);
                if (uavStates.cameraMode.operationMode != 0)
                {
                    bw.Write(uavStates.cameraMode.fieldOfView ?? 0);
                    bw.Write(uavStates.cameraMode.centerCoordinate.latitude ?? 0);
                    bw.Write(uavStates.cameraMode.centerCoordinate.longitude ?? 0);
                    bw.Write(uavStates.cameraMode.centerCoordinate.altitude ?? 0);
                    bw.Write(uavStates.cameraMode.cornerUpperLeft.latitude ?? 0);
                    bw.Write(uavStates.cameraMode.cornerUpperLeft.longitude ?? 0);
                    bw.Write(uavStates.cameraMode.cornerUpperRight.latitude ?? 0);
                    bw.Write(uavStates.cameraMode.cornerUpperRight.longitude ?? 0);
                    bw.Write(uavStates.cameraMode.cornerLowerRight.latitude ?? 0);
                    bw.Write(uavStates.cameraMode.cornerLowerRight.longitude ?? 0);
                    bw.Write(uavStates.cameraMode.cornerLowerLeft.latitude ?? 0);
                    bw.Write(uavStates.cameraMode.cornerLowerLeft.longitude ?? 0);
                }

                bw.Write(timestamp);
            }
            
            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static UAVStatesList Deserialize(byte[] data)
        {
            var obj = new UAVStatesList();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.uavStatesN = br.ReadUInt32();
            obj.uavStates = new UAVStates[obj.uavStatesN ?? 0];

            for (int i = 0; i < obj.uavStatesN; i++)
            {
                var uavStates = new UAVStates
                {
                    aircraftID = br.ReadUInt32(),
                    coordinate = new Coordinate
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    },

                    velocity = new Velocity
                    {
                        speed = br.ReadSingle(),
                        heading = br.ReadSingle()
                    },

                    attitude = new Attitude
                    {
                        roll = br.ReadSingle(),
                        pitch = br.ReadSingle(),
                        yaw = br.ReadSingle()
                    },

                    fuel = br.ReadSingle(),

                    flightMode = new FlightMode
                    {
                        flightMode = br.ReadUInt32(),
                        onMission = br.ReadUInt32()
                    }
                };

                if (uavStates.flightMode.flightMode == 1 || uavStates.flightMode.flightMode == 2
                    || uavStates.flightMode.flightMode == 3 || uavStates.flightMode.flightMode == 4 || uavStates.flightMode.flightMode == 7)
                {
                    uavStates.flightMode.currentWaypointID = br.ReadUInt32();
                }

                if (uavStates.flightMode.flightMode == 6)
                {
                    uavStates.flightMode.leaderID = br.ReadUInt32();
                }

                if (uavStates.flightMode.flightMode == 8)
                {
                    uavStates.flightMode.coordinate = new nFusion.Model.msg_51321.Coordinate();
                    uavStates.flightMode.coordinate.latitude = br.ReadSingle();
                    uavStates.flightMode.coordinate.longitude = br.ReadSingle();
                    uavStates.flightMode.coordinate.altitude = br.ReadInt32();
                }

                if (uavStates.flightMode.flightMode == 9)
                {
                    uavStates.flightMode.targetID = br.ReadUInt32();
                }

                // CameraMode
                uavStates.cameraMode = new CameraMode
                {
                    sensorType = br.ReadUInt32(),
                    operationMode = br.ReadInt32()
                };

                if (uavStates.cameraMode.operationMode != 0)
                {
                    uavStates.cameraMode.fieldOfView = br.ReadSingle();
                    
                    uavStates.cameraMode.centerCoordinate = new CenterCoordinate
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    };

                    uavStates.cameraMode.cornerUpperLeft = new LatLon
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle()
                    };

                    uavStates.cameraMode.cornerUpperRight = new LatLon
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle()
                    };

                    uavStates.cameraMode.cornerLowerRight = new LatLon
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle()
                    };

                    uavStates.cameraMode.cornerLowerLeft = new LatLon
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle()
                    };
                }

                uavStates.lastSignalTime = br.ReadBytes(5);

                obj.uavStates[i] = uavStates;
            }

            return obj;
        }
    }
}
