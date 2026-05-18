using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_52310;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_52310
{
    public class RegionOfInterest : nFusion.Model.msg_52310.RegionOfInterest
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
            bw.Write(aircraftID ?? 0);
            bw.Write(centerCoordinate?.latitude ?? 0);
            bw.Write(centerCoordinate?.longitude ?? 0);
            bw.Write(centerCoordinate?.altitude ?? 0);
            bw.Write(fov ?? 0);

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static RegionOfInterest Deserialize(byte[] data)
        {
            var obj = new RegionOfInterest();

            using var ms = new MemoryStream(data);
            //using var br = new BinaryReader(ms);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.aircraftID = br.ReadUInt32();

            obj.centerCoordinate = new CenterCoordinate
            {
                latitude = br.ReadSingle(),
                longitude = br.ReadSingle(),
                altitude = br.ReadInt32()
            };

            obj.fov = br.ReadSingle();

            return obj;
        }
    }
}
