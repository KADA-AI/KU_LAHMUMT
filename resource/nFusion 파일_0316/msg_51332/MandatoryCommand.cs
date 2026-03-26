using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51332;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51332
{
    public class MandatoryCommand : nFusion.Model.msg_51332.MandatoryCommand
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
            bw.Write(mandatoryType ?? 0);

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static MandatoryCommand Deserialize(byte[] data)
        {
            var obj = new MandatoryCommand();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.aircraftID = br.ReadUInt32();
            obj.mandatoryType = br.ReadUInt32();

            return obj;
        }
    }
}
