using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51333;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51333
{
    public class PriorMissionCommand : nFusion.Model.msg_51333.PriorMissionCommand
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(priorMissionType ?? 0);

            switch (priorMissionType)
            {
                case 1: // CoordinateOrientation
                    bw.Write(orient.latitude ?? 0);
                    bw.Write(orient.longitude ?? 0);
                    bw.Write(orient.altitude ?? 0);
                    break;

                case 2: // TargetOrientation
                    bw.Write(targetID ?? 0);
                    break;

                default:
                    throw new InvalidOperationException($"지원하지 않는 priorMissionType: {priorMissionType}");
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static PriorMissionCommand Deserialize(byte[] data)
        {
            var obj = new PriorMissionCommand();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.priorMissionType = br.ReadUInt32();

            switch (obj.priorMissionType)
            {
                case 1: // CoordinateOrientation
                    obj.orient = new Orient
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    };
                    break;

                case 2: // TargetOrientation
                    obj.targetID = br.ReadUInt32();
                    break;


                default:
                    throw new InvalidOperationException($"지원하지 않는 priorMissionType: {obj.priorMissionType}");
            }

            return obj;
        }
    }
}
