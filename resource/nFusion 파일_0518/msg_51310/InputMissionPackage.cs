using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51310;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51310
{
    public class InputMissionPackage : nFusion.Model.msg_51310.InputMissionPackage
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            if (timestamp != null)
                bw.Write(timestamp);
            else
                bw.Write(new byte[5]);
            bw.Write(inputMissionPackageID ?? 0);
            bw.Write(missionType ?? 0);
            bw.Write(dateAndNight ?? 0);
            bw.Write(aircraftIDsN ?? 0);
            if ((aircraftIDsN ?? 0) > 0)
            {
                if (aircraftIDs != null)
                {
                    foreach (var aircraftID in aircraftIDs)
                    {
                        bw.Write(aircraftID?.aircraftID ?? 0);
                    }
                }
                else
                {
                    Console.WriteLine("aircraftIDs 배열이 null입니다.");
                }
            }
            else
            {
                Console.WriteLine("AircraftIDsN이 0입니다. 유무인기가 없으면 안됩니다.");
            }

            bw.Write(inputMissionListN ?? 0);

            foreach (var inputMission in inputMissionList ?? Array.Empty<InputMission>())
            {
                bw.Write(inputMission?.inputMissionID ?? 0);
                bw.Write(inputMission?.sequenceNumber ?? 0);
                bw.Write(inputMission?.inputMissionType ?? 0);
                bw.Write(inputMission?.regionType ?? 0);
                bw.Write(inputMission?.isDone ?? false);
                bw.Write(inputMission?.shapeType ?? 0);

                switch (inputMission?.shapeType)
                {
                    case 1: // Coordinate
                        var missionCoordinate = inputMission?.coordinate;
                        if (missionCoordinate != null)
                        {
                            bw.Write(missionCoordinate.latitude ?? 0);
                            bw.Write(missionCoordinate.longitude ?? 0);
                            bw.Write(missionCoordinate.altitude ?? 0);
                        }
                        else
                        {
                            Console.WriteLine("inputMission.coordinate가 null입니다.");
                        }
                        break;

                    case 2: // Polyline
                        if (inputMission?.polyline != null)
                        {
                            bw.Write(inputMission.polyline.width ?? 0);
                            bw.Write(inputMission.polyline.coordinateListN ?? 0);
                            foreach (var polylineCoordinate in inputMission.polyline.coordinateList ?? Array.Empty<Coordinate>())
                            {
                                bw.Write(polylineCoordinate?.latitude ?? 0);
                                bw.Write(polylineCoordinate?.longitude ?? 0);
                                bw.Write(polylineCoordinate?.altitude ?? 0);
                            }
                        }
                        else
                        {
                            Console.WriteLine("inputMission.polyline이 null입니다.");
                        }
                        break;

                    case 3: // Polygons
                        if (inputMission?.polygons != null)
                        {
                            bw.Write(inputMission.polygons.areaListN ?? 0);
                            foreach (var area in inputMission.polygons.areaList ?? Array.Empty<Area>())
                            {
                                bw.Write(area?.isHole ?? false);
                                bw.Write(area?.coordinateListN ?? 0);
                                foreach (var areaCoordinate in area?.coordinateList ?? Array.Empty<Coordinate>())
                                {
                                    bw.Write(areaCoordinate?.latitude ?? 0);
                                    bw.Write(areaCoordinate?.longitude ?? 0);
                                    bw.Write(areaCoordinate?.altitude ?? 0);
                                }
                            }
                        }
                        else
                        {
                            Console.WriteLine("inputMission.polygons가 null입니다.");
                        }
                        break;

                    default:
                        if (inputMission != null)
                            throw new InvalidOperationException($"지원하지 않는 shapeType: {inputMission.shapeType}");
                        break;
                }
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static InputMissionPackage Deserialize(byte[] data)
        {
            var obj = new InputMissionPackage();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);

            obj.inputMissionPackageID = br.ReadUInt32();
            obj.missionType = br.ReadUInt32();
            obj.dateAndNight = br.ReadUInt32();

            obj.aircraftIDsN = br.ReadUInt32();
            if (obj.aircraftIDsN > 0)
            {
                obj.aircraftIDs = new AircraftID[obj.aircraftIDsN.Value];
                for (int i = 0; i < obj.aircraftIDsN.Value; i++)
                {
                    var aircraftID = new AircraftID
                    {
                        aircraftID = br.ReadUInt32()
                    };

                    obj.aircraftIDs[i] = aircraftID;
                }
            }
            else
            {
                Console.WriteLine("AircraftIDsN이 0입니다. 유무인기가 없으면 안됩니다.");
            }

            obj.inputMissionListN = br.ReadUInt32();
            int inputMissionListCount = (int)(obj.inputMissionListN ?? 0);
            obj.inputMissionList = new InputMission[inputMissionListCount];

            for (int i = 0; i < inputMissionListCount; i++)
            {
                var inputMission = new InputMission
                {
                    inputMissionID = br.ReadUInt32(),
                    sequenceNumber = br.ReadUInt32(),
                    inputMissionType = br.ReadUInt32(),
                    regionType = br.ReadUInt32(),
                    isDone = br.ReadBoolean(),
                    shapeType = br.ReadUInt32()
                };

                switch (inputMission.shapeType)
                {
                    case 1: // Coordinate
                        inputMission.coordinate = new Coordinate
                        {
                            latitude = br.ReadSingle(),
                            longitude = br.ReadSingle(),
                            altitude = br.ReadInt32()
                        };
                        break;

                    case 2: // Polyline
                        inputMission.polyline = new Polyline
                        {
                            width = br.ReadUInt32(),
                            coordinateListN = br.ReadUInt32()
                        };
                        int polylineCoordCount = (int)(inputMission.polyline.coordinateListN ?? 0);
                        inputMission.polyline.coordinateList = new Coordinate[polylineCoordCount];

                        for (int j = 0; j < polylineCoordCount; j++)
                        {
                            inputMission.polyline.coordinateList[j] = new Coordinate
                            {
                                latitude = br.ReadSingle(),
                                longitude = br.ReadSingle(),
                                altitude = br.ReadInt32()
                            };
                        }
                        break;

                    case 3: // Polygons
                        inputMission.polygons = new Polygons();
                        inputMission.polygons.areaListN = br.ReadUInt32();
                        int areaListCount = (int)(inputMission.polygons.areaListN ?? 0);
                        inputMission.polygons.areaList = new Area[areaListCount];

                        for (int j = 0; j < areaListCount; j++)
                        {
                            var area = new Area();
                            area.isHole = br.ReadBoolean();
                            area.coordinateListN = br.ReadUInt32();
                            int areaCoordCount = (int)(area.coordinateListN ?? 0);
                            area.coordinateList = new Coordinate[areaCoordCount];

                            for (int k = 0; k < areaCoordCount; k++)
                            {
                                area.coordinateList[k] = new Coordinate
                                {
                                    latitude = br.ReadSingle(),
                                    longitude = br.ReadSingle(),
                                    altitude = br.ReadInt32()
                                };
                            }

                            inputMission.polygons.areaList![j] = area;
                        }
                        break;

                    default:
                        throw new InvalidOperationException($"지원하지 않는 shapeType: {inputMission.shapeType}");
                }

                obj.inputMissionList![i] = inputMission;
            }

            return obj;
        }
    }
}
