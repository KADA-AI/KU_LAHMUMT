using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_53130;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_53130
{
    public class MissionProgress : nFusion.Model.msg_53130.MissionProgress
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(currentMissionSegmentID ?? 0);
            bw.Write(individualMissionProgressN ?? 0);

            // IndividualMissionProgress
            if (individualMissionProgressN > 0)
            {
                foreach (var individualMissionProgress in individualMissionProgressList)
                {
                    bw.Write(individualMissionProgress.aircraftID ?? 0);
                    bw.Write(individualMissionProgress.currentIndividualMissionID ?? 0);
                    bw.Write(individualMissionProgress.progress ?? 0);
                }
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static MissionProgress Deserialize(byte[] data)
        {
            var obj = new MissionProgress();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);

            obj.currentMissionSegmentID = br.ReadUInt32();
            obj.individualMissionProgressN = br.ReadUInt32();

            // IndividualMissionProgress
            if (obj.individualMissionProgressN > 0)
            {
                obj.individualMissionProgressList = new IndividualMissionProgress[obj.individualMissionProgressN ?? 0];

                for (int i = 0; i < obj.individualMissionProgressN; i++)
                {
                    IndividualMissionProgress individualMissionProgress = new IndividualMissionProgress()
                    {
                        aircraftID = br.ReadUInt32(),
                        currentIndividualMissionID = br.ReadUInt32(),
                        progress = br.ReadUInt32()
                    };

                    obj.individualMissionProgressList[i] = individualMissionProgress;
                }
            }

            return obj;
        }
    }
}
