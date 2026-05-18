using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51301;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51301
{
    public class BootCommand : nFusion.Model.msg_51301.BootCommand
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(command ?? 0);

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static BootCommand Deserialize(byte[] data)
        {
            var obj = new BootCommand();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.command = br.ReadUInt32();

            return obj;
        }
    }
}
