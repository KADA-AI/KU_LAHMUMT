using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_53113;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_53113
{
    public class MissionPlanOptionInfo : nFusion.Model.msg_53113.MissionPlanOptionInfo
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(autoExecution ?? false);
            bw.Write(optionListN ?? 0);

            foreach (var option in optionList ?? Array.Empty<Option>())
            {
                bw.Write(option?.optionID ?? 0);
                bw.Write(option?.recommend ?? false);
                bw.Write(option?.optionName ?? 0);
                bw.Write(option?.survivalRate ?? 0);
                bw.Write(option?.timeContraction ?? 0);
                bw.Write(option?.recogEffectiveness ?? 0);
                bw.Write(option?.fuelWarning ?? 0);
                bw.Write(option?.distance ?? 0);
                bw.Write(option?.target ?? 0);

                bw.Write(option?.uavMissionPlanIDListN ?? 0);
                foreach (var id in option?.uavMissionPlanIDList ?? Array.Empty<UAVMissionPlanID>())
                {
                    bw.Write(id?.uavMissionPlanID ?? 0);
                }

                bw.Write(option?.lahMissionPlanIDListN ?? 0);
                if ((option?.lahMissionPlanIDListN ?? 0) > 0)
                {
                    foreach (var id in option?.lahMissionPlanIDList ?? Array.Empty<LAHMissionPlanID>())
                    {
                        bw.Write(id?.lahMissionPlanID ?? 0);
                    }
                }
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static MissionPlanOptionInfo Deserialize(byte[] data)
        {
            var obj = new MissionPlanOptionInfo();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.autoExecution = br.ReadBoolean();
            obj.optionListN = br.ReadUInt32();
            obj.optionList = new Option[obj.optionListN ?? 0];

            for (int i = 0; i < obj.optionListN; i++)
            {
                var option = new Option
                {
                    optionID = br.ReadUInt32(),
                    recommend = br.ReadBoolean(),
                    optionName = br.ReadUInt32(),
                    survivalRate = br.ReadInt32(),
                    timeContraction = br.ReadInt32(),
                    recogEffectiveness = br.ReadInt32(),
                    fuelWarning = br.ReadInt32(),
                    distance = br.ReadUInt32(),
                    target = br.ReadUInt32()
                };

                option.uavMissionPlanIDListN = br.ReadUInt32();
                option.uavMissionPlanIDList = new UAVMissionPlanID[option.uavMissionPlanIDListN ?? 0];
                for (int j = 0; j < option.uavMissionPlanIDListN; j++)
                {
                    option.uavMissionPlanIDList[j] = new UAVMissionPlanID();
                    option.uavMissionPlanIDList[j].uavMissionPlanID = br.ReadUInt32();
                }

                option.lahMissionPlanIDListN = br.ReadUInt32();
                option.lahMissionPlanIDList = new LAHMissionPlanID[option.lahMissionPlanIDListN ?? 0];
                for (int j = 0; j < option.lahMissionPlanIDListN; j++)
                {
                    option.lahMissionPlanIDList[j] = new LAHMissionPlanID();
                    option.lahMissionPlanIDList[j].lahMissionPlanID = br.ReadUInt32();
                }

                obj.optionList[i] = option;
            }
            return obj;
        }
    }
}
