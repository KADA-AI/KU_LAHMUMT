using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51331;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51331
{
    public class PilotDecision : nFusion.Model.msg_51331.PilotDecision
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(ignore ?? true);
            if(ignore == false)
            {
                bw.Write(optionID ?? 0);
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static PilotDecision Deserialize(byte[] data)
        {
            var obj = new PilotDecision();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.ignore = br.ReadBoolean();
            if (obj.ignore == false)
            {
                obj.optionID = br.ReadUInt32();
            }

            return obj;
        }
    }
}
