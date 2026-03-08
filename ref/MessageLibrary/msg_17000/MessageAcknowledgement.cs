using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_17000;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_17000
{
    public class MessageAcknowledgement : nFusion.Model.msg_17000.MessageAcknowledgement
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
            bw.Write(originalMessageTimeStamp);
            bw.Write(originalMessageType);
            bw.Write(acknowledgementType);

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static MessageAcknowledgement Deserialize(byte[] data)
        {
            var obj = new MessageAcknowledgement();

            using var ms = new MemoryStream(data);
            //using var br = new BinaryReader(ms);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.originalMessageTimeStamp = br.ReadBytes(5);
            obj.originalMessageType = br.ReadBytes(2);
            obj.acknowledgementType = br.ReadBytes(1);

            return obj;
        }
    }
}
