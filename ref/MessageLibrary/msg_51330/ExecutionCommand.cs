using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51330;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51330
{
    public class ExecutionCommand : nFusion.Model.msg_51330.ExecutionCommand
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(excute ?? 0);

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static ExecutionCommand Deserialize(byte[] data)
        {
            var obj = new ExecutionCommand();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.excute = br.ReadUInt32();

            return obj;
        }
    }
}
