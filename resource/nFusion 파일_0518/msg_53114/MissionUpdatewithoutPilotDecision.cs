using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_53114;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_53114
{
    public class MissionUpdatewithoutPilotDecision : nFusion.Model.msg_53114.MissionUpdatewithoutPilotDecision
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);

            bw.Write(uavMissionPlanIDListN ?? 0);
            foreach (var id in uavMissionPlanIDList ?? Array.Empty<UAVMissionPlanID>())
            {
                bw.Write(id?.uavMissionPlanID ?? 0);
            }

            bw.Write(lahMissionPlanIDListN ?? 0);
            if ((lahMissionPlanIDListN ?? 0) > 0)
            {
                foreach (var id in lahMissionPlanIDList ?? Array.Empty<LAHMissionPlanID>())
                {
                    bw.Write(id?.lahMissionPlanID ?? 0);
                }
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static MissionUpdatewithoutPilotDecision Deserialize(byte[] data)
        {
            var obj = new MissionUpdatewithoutPilotDecision();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);

            obj.uavMissionPlanIDListN = br.ReadUInt32();
            obj.uavMissionPlanIDList = new UAVMissionPlanID[obj.uavMissionPlanIDListN ?? 0];
            for (int j = 0; j < obj.uavMissionPlanIDListN; j++)
            {
                obj.uavMissionPlanIDList[j] = new UAVMissionPlanID();
                obj.uavMissionPlanIDList[j].uavMissionPlanID = br.ReadUInt32();
            }

            obj.lahMissionPlanIDListN = br.ReadUInt32();
            obj.lahMissionPlanIDList = new LAHMissionPlanID[obj.lahMissionPlanIDListN ?? 0];
            for (int j = 0; j < obj.lahMissionPlanIDListN; j++)
            {
                obj.lahMissionPlanIDList[j] = new LAHMissionPlanID();
                obj.lahMissionPlanIDList[j].lahMissionPlanID = br.ReadUInt32();
            }

            return obj;
        }
    }
}
