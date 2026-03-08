using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_53140;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_53140
{
    public class RequestShowMessage : nFusion.Model.msg_53140.RequestShowMessage
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

            bw.Write(type ?? 0);

            byte[] messageByte = Utility.MakeFixedSize(message, 60);
            bw.Write(messageByte);
            

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static RequestShowMessage Deserialize(byte[] data)
        {
            var obj = new RequestShowMessage();

            using var ms = new MemoryStream(data);
            //using var br = new BinaryReader(ms);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.type = br.ReadUInt32();
            obj.message = Encoding.UTF8.GetString(br.ReadBytes(60)).TrimEnd('\0');


            return obj;
        }
    }
}
