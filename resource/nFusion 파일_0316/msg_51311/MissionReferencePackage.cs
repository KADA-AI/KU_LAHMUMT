using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51311;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51311
{
    public class MissionReferencePackage : nFusion.Model.msg_51311.MissionReferencePackage
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);

            bw.Write(missionReferencePackageID ?? 0);
            bw.Write(takeOverInfoListN ?? 0);
            foreach (var takeOverInfo in takeOverInfoList)
            {
                bw.Write(takeOverInfo.aircraftID ?? 0);
                bw.Write(takeOverInfo.coordinate.latitude ?? 0);
                bw.Write(takeOverInfo.coordinate.longitude ?? 0);
                bw.Write(takeOverInfo.coordinate.altitude ?? 0);
            }

            bw.Write(handOverInfoListN ?? 0);
            foreach (var handOverInfo in handOverInfoList)
            {
                bw.Write(handOverInfo.aircraftID ?? 0);
                bw.Write(handOverInfo.coordinate.latitude ?? 0);
                bw.Write(handOverInfo.coordinate.longitude ?? 0);
                bw.Write(handOverInfo.coordinate.altitude ?? 0);
            }

            bw.Write(rtbCoordinateListN ?? 0);
            foreach (var rtbCoordinate in rtbCoordinateList)
            {
                bw.Write(rtbCoordinate.latitude ?? 0);
                bw.Write(rtbCoordinate.longitude ?? 0);
                bw.Write(rtbCoordinate.altitude ?? 0);
            }

            bw.Write(flightAreaListN ?? 0);
            foreach (var flightArea in flightAreaList)
            {
                bw.Write(flightArea.areaLatLonListN ?? 0);
                foreach (var areaLatLon in flightArea.areaLatLonList)
                {
                    bw.Write(areaLatLon.latitude ?? 0);
                    bw.Write(areaLatLon.longitude ?? 0);
                }
                bw.Write(flightArea.altitudeLimits.lowerLimit ?? 0);
                bw.Write(flightArea.altitudeLimits.upperLimit ?? 0);
            }

            bw.Write(prohibitedAreaListN ?? 0);
            foreach (var prohibitedArea in prohibitedAreaList)
            {
                bw.Write(prohibitedArea.areaLatLonListN ?? 0);
                foreach (var areaLatLon in prohibitedArea.areaLatLonList)
                {
                    bw.Write(areaLatLon.latitude ?? 0);
                    bw.Write(areaLatLon.longitude ?? 0);
                }
                bw.Write(prohibitedArea.altitudelimits.lowerLimit ?? 0);
                bw.Write(prohibitedArea.altitudelimits.upperLimit ?? 0);
            }

            bw.Write(regionInfoListN ?? 0);
            foreach (var regionInfo in regionInfoList)
            {
                bw.Write(regionInfo.regionType ?? 0);
                bw.Write(regionInfo.coordinate.latitude ?? 0);
                bw.Write(regionInfo.coordinate.longitude ?? 0);
                bw.Write(regionInfo.coordinate.altitude ?? 0);
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static MissionReferencePackage Deserialize(byte[] data)
        {
            var obj = new MissionReferencePackage();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);

            obj.missionReferencePackageID = br.ReadUInt32();
            obj.takeOverInfoListN = br.ReadUInt32();
            obj.takeOverInfoList = new TakeOverInfo[obj.takeOverInfoListN ?? 0];

            for (int i = 0; i < obj.takeOverInfoListN; i++)
            {
                var takeOverInfo = new TakeOverInfo
                {
                    aircraftID = br.ReadUInt32(),
                    coordinate = new Coordinate {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    }
                };

                obj.takeOverInfoList[i] = takeOverInfo;
            }

            obj.handOverInfoListN = br.ReadUInt32();
            obj.handOverInfoList = new HandOverInfo[obj.handOverInfoListN ?? 0];

            for (int i = 0; i < obj.handOverInfoListN; i++)
            {
                var handOverInfo = new HandOverInfo
                {
                    aircraftID = br.ReadUInt32(),
                    coordinate = new Coordinate
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    }
                };

                obj.handOverInfoList[i] = handOverInfo;
            }

            obj.rtbCoordinateListN = br.ReadUInt32();
            obj.rtbCoordinateList = new RTBCoordinate[obj.rtbCoordinateListN ?? 0];

            for (int i = 0; i < obj.rtbCoordinateListN; i++)
            {
                var rtbCoordinate = new RTBCoordinate
                {
                    latitude = br.ReadSingle(),
                    longitude = br.ReadSingle(),
                    altitude = br.ReadInt32()
                };

                obj.rtbCoordinateList[i] = rtbCoordinate;
            }

            obj.flightAreaListN = br.ReadUInt32();
            obj.flightAreaList = new FlightArea[obj.flightAreaListN ?? 0];

            for (int i = 0; i < obj.flightAreaListN; i++)
            {
                var flightArea = new FlightArea();
                flightArea.areaLatLonListN = br.ReadUInt32();

                flightArea.areaLatLonList = new AreaLatLon[flightArea.areaLatLonListN ?? 0];
                for (int j = 0; j < flightArea.areaLatLonListN; j++)
                {
                    var areaLatLon = new AreaLatLon
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                    };
                    flightArea.areaLatLonList[j] = areaLatLon;
                }

                flightArea.altitudeLimits = new AltitudeLimits
                {
                    lowerLimit = br.ReadInt32(),
                    upperLimit = br.ReadInt32(),
                };

                obj.flightAreaList[i] = flightArea;
            }

            obj.prohibitedAreaListN = br.ReadUInt32();
            obj.prohibitedAreaList = new ProhibitedArea[obj.prohibitedAreaListN ?? 0];

            for (int i = 0; i < obj.prohibitedAreaListN; i++)
            {
                var prohibitedArea = new ProhibitedArea();
                prohibitedArea.areaLatLonListN = br.ReadUInt32();

                prohibitedArea.areaLatLonList = new AreaLatLon[prohibitedArea.areaLatLonListN ?? 0];
                for (int j = 0; j < prohibitedArea.areaLatLonListN; j++)
                {
                    var areaLatLon = new AreaLatLon
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                    };
                    prohibitedArea.areaLatLonList[j] = areaLatLon;
                }

                prohibitedArea.altitudelimits = new AltitudeLimits
                {
                    lowerLimit = br.ReadInt32(),
                    upperLimit = br.ReadInt32(),
                };

                obj.prohibitedAreaList[i] = prohibitedArea;
            }

            obj.regionInfoListN = br.ReadUInt32();
            obj.regionInfoList = new RegionInfo[obj.regionInfoListN ?? 0];

            for (int i = 0; i < obj.regionInfoListN; i++)
            {
                var regionInfo = new RegionInfo
                {
                    regionType = br.ReadUInt32(),
                    coordinate = new Coordinate
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    }
                };

                obj.regionInfoList[i] = regionInfo;
            }

            return obj;
        }
    }
}
