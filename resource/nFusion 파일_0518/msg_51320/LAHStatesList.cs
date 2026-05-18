using System;
using System.Collections.Generic;
using System.Text;
using nFusion.Model.msg_51320;
using MiscUtil.IO;
using MiscUtil.Conversion;

namespace MessageLibrary.msg_51320
{
    public class LAHStatesList : nFusion.Model.msg_51320.LAHStatesList
    {
        // 바이트 배열로 직렬화
        public byte[] Serialize()
        {
            using var ms = new MemoryStream();
            using var bw = new EndianBinaryWriter(new BigEndianBitConverter(), ms);

            bw.Write(presenceVector ?? 0);
            timestamp = Utility.GenerateTimestamp();
            bw.Write(timestamp);
            bw.Write(lahStatesN ?? 0);

            foreach (var lahstates in lahStates ?? Array.Empty<LAHStates>())
            {
                bw.Write(lahstates?.aircraftID ?? 0);
                bw.Write(lahstates?.coordinate?.latitude ?? 0);
                bw.Write(lahstates?.coordinate?.longitude ?? 0);
                bw.Write(lahstates?.coordinate?.altitude ?? 0);
                bw.Write(lahstates?.velocity?.speed ?? 0);
                bw.Write(lahstates?.velocity?.heading ?? 0);
                bw.Write(lahstates?.fuel ?? 0);
                bw.Write(lahstates?.weapons?.type1 ?? 0);
                bw.Write(lahstates?.weapons?.type2 ?? 0);
                bw.Write(lahstates?.weapons?.type3 ?? 0);
                bw.Write(lahstates?.lastSignalTime ?? timestamp ?? new byte[5]);
            }

            return ms.ToArray();
        }

        // 바이트 배열에서 역직렬화
        public static LAHStatesList Deserialize(byte[] data)
        {
            var obj = new LAHStatesList();

            using var ms = new MemoryStream(data);
            using var br = new EndianBinaryReader(new BigEndianBitConverter(), ms);

            obj.presenceVector = br.ReadByte();
            obj.timestamp = br.ReadBytes(5);
            obj.lahStatesN = br.ReadUInt32();
            obj.lahStates = new LAHStates[obj.lahStatesN ?? 0];

            for (int i = 0; i < obj.lahStatesN; i++)
            {
                var lahStates = new LAHStates
                {
                    aircraftID = br.ReadUInt32(),

                    coordinate = new Coordinate
                    {
                        latitude = br.ReadSingle(),
                        longitude = br.ReadSingle(),
                        altitude = br.ReadInt32()
                    },

                    velocity = new Velocity
                    {
                        speed = br.ReadSingle(),
                        heading = br.ReadSingle()
                    },

                    fuel = br.ReadSingle(),

                    weapons = new Weapons
                    {
                        type1 = br.ReadUInt32(),
                        type2 = br.ReadUInt32(),
                        type3 = br.ReadUInt32()
                    },

                    lastSignalTime = br.ReadBytes(5)
                };

                obj.lahStates[i] = lahStates;
            }

            return obj;
        }
    }
}
