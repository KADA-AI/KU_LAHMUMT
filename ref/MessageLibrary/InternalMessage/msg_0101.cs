using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0101 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            nFusion.Model.msg_0101.SystemOperationMode systemOperationMode = new nFusion.Model.msg_0101.SystemOperationMode { timestamp = Utility.GenerateTimestampUlong(), source = "Dummy", systemMode = 0 };

            Console.Write("보낼 모드 입력");
            Console.Write("0: 초기화, 1: 대기, 2: 초기임무계획, 3:임무수행");
            string input = Console.ReadLine();

            Console.WriteLine("-------------------------------------------------------");
            if (input.ToLower() == "exit")
                return 0; // 종료

            if (!uint.TryParse(input, out uint mode) || mode < 0 || mode > 4)
            {
                Console.WriteLine("잘못된 입력입니다. 0으로 기본 설정되었습니다.");
            }
            else
            {
                systemOperationMode.systemMode = mode;
            }

            return systemOperationMode;
        }
    }
}
