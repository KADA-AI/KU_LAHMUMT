using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0805;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0805 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            SystemEvent systemEvent = new SystemEvent
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                eventType = 3
            };

            Console.Write("보낼 명령 입력. 기본값은 3");
            Console.Write("1: 모의 시작, 2: 모의종료, 3: 임무시작(통제권 인계후), 4: 임무종료(통제권 인계후)");
            string input = Console.ReadLine() ?? string.Empty;

            Console.WriteLine("-------------------------------------------------------");
            if (string.Equals(input, "exit", StringComparison.OrdinalIgnoreCase))
                return 0; // 종료

            if (!uint.TryParse(input, out uint command) || command < 0 || command > 3)
            {
                Console.WriteLine("잘못된 입력입니다. 3으로 기본 설정되었습니다.");
            }
            else
            {
                systemEvent.eventType = command;
            }

            return systemEvent;
        }
    }
}
