using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51323;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51323
{
    public class MalfunctionList : nFusion.Model.msg_51323.MalfunctionList
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(lahN ?? 0);

            foreach (var lah in lahs ?? Array.Empty<LAH>())
            {
                bw.Write(lah?.aircraftID ?? 0);
                bw.Write(lah?.health ?? 0);
                bw.Write(lah?.datalinkStatus?.isConnectedToUAV1 ?? false);
                bw.Write(lah?.datalinkStatus?.isConnectedToUAV2 ?? false);
                bw.Write(lah?.datalinkStatus?.isConnectedToUAV3 ?? false);
            }

            bw.Write(uavN ?? 0);

            foreach (var uav in uavs ?? Array.Empty<UAV>())
            {
                bw.Write(uav?.aircraftID ?? 0);
                bw.Write(uav?.health ?? 0);
                bw.Write(uav?.payloadHealth ?? 0);
                bw.Write(uav?.fuelWarning ?? 0);
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static MalfunctionList Deserialize(byte[] data)
        {
            var obj = new MalfunctionList();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.lahN = br.ReadUInt32();
            obj.lahs = new LAH[obj.lahN ?? 0];

            for (int i = 0; i < obj.lahN; i++)
            {
                var lah = new LAH
                {
                    aircraftID = br.ReadUInt32(),
                    health = br.ReadUInt32(),
                    datalinkStatus = new DatalinkStatus
                    {
                        isConnectedToUAV1 = br.ReadBoolean(),
                        isConnectedToUAV2 = br.ReadBoolean(),
                        isConnectedToUAV3 = br.ReadBoolean()
                    }
                };

                obj.lahs[i] = lah;
            }

            obj.uavN = br.ReadUInt32();
            obj.uavs = new UAV[obj.uavN ?? 0];

            for (int i = 0; i < obj.uavN; i++)
            {
                var uav = new UAV
                {
                    aircraftID = br.ReadUInt32(),
                    health = br.ReadUInt32(),
                    payloadHealth = br.ReadUInt32(),
                    fuelWarning = br.ReadUInt32()
                };

                obj.uavs[i] = uav;
            }

            return obj;
        }
    }
}
