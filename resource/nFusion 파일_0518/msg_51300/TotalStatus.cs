using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51300;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51300
{
    public class TotalStatus : nFusion.Model.msg_51300.TotalStatus
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
            bw.Write(mode ?? 0);
            bw.Write(sbc1Status ?? 0);
            bw.Write(sbc2Status ?? 0);
            bw.Write(sbc3Status ?? 0);

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static TotalStatus Deserialize(byte[] data)
        {
            var obj = new TotalStatus();

            using var ms = new MemoryStream(data);
            //using var br = new BinaryReader(ms);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.aircraftID = br.ReadUInt32();
            obj.mode = br.ReadUInt32();
            obj.sbc1Status = br.ReadUInt32();
            obj.sbc2Status = br.ReadUInt32();
            obj.sbc3Status = br.ReadUInt32();

            return obj;
        }
    }
}
