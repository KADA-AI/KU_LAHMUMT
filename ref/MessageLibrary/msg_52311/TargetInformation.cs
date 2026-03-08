using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_52311;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_52311
{
    public class TargetInformation : nFusion.Model.msg_52311.TargetInformation
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            //using var bw = new BinaryWriter(ms);
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(targetID ?? 0);
            bw.Write(status ?? 0);

            if(status == 1 || status == 2)
            {
                bw.Write(information.targetType ?? 0);
                bw.Write(information.threat ?? 0);
                bw.Write(information.coordinate.latitude ?? 0);
                bw.Write(information.coordinate.longitude ?? 0);
                bw.Write(information.coordinate.altitude ?? 0);
                bw.Write(information.aircraftID ?? 0);
                bw.Write(information.centerPixel.px ?? 0);
                bw.Write(information.centerPixel.py ?? 0);
                bw.Write(information.boundingBox.width ?? 0);
                bw.Write(information.boundingBox.height ?? 0);

                //byte[] snapShotDir = Utility.MakeFixedSize(information.snapShotDir, 36);
                //bw.Write(snapShotDir);
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static TargetInformation Deserialize(byte[] data)
        {
            var obj = new TargetInformation();

            using var ms = new MemoryStream(data);
            //using var br = new BinaryReader(ms);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.targetID = br.ReadUInt32();
            obj.status = br.ReadUInt32();

            if (obj.status == 1 || obj.status == 2)
            {
                obj.information = new Information
                {
                    targetType = br.ReadUInt32(),
                    threat = br.ReadSingle(),
                    coordinate = new Coordinate
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    },
                    aircraftID = br.ReadUInt32(),
                    centerPixel = new CenterPixel
                    {
                        px = br.ReadInt32(),
                        py = br.ReadInt32()
                    },
                    boundingBox = new BoundingBox
                    {
                        width = br.ReadUInt32(),
                        height = br.ReadUInt32(),
                    },
                    //snapShotDir = Encoding.UTF8.GetString(br.ReadBytes(36)).TrimEnd('\0')
                };
            }

            return obj;
        }
    }
}
