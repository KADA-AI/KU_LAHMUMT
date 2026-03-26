using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_53100;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_53100
{
    public class SBCStatus : nFusion.Model.msg_53100.SBCStatus
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
            bw.Write(status ?? 0);
            bw.Write(mode ?? 0);

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static SBCStatus Deserialize(byte[] data)
        {
            var obj = new SBCStatus();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.status = br.ReadUInt32();
            obj.mode = br.ReadUInt32();

            return obj;
        }
    }
}
