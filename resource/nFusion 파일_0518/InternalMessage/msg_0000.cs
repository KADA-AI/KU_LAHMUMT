using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using nFusion.Model.msg_0000;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0000 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            Console.WriteLine("조회를 원하는 메시지를 입력하세요");
            string input = Console.ReadLine();
            uint statusInput = 0;

            try
            {
                if (input.Length != 4 || !input.All(char.IsDigit))
                {
                    Console.WriteLine("메시지 ID는 4자리 숫자여야 합니다.");
                    return false;
                }
                // 입력 값 형식 검증 (0 ~ 2 사이의 숫자)
                else
                {
                    statusInput = uint.Parse(input);
                    var message = new nFusion.Model.msg_0000.RequestData { timestamp = Utility.GenerateTimestampUlong(), source = "Dummy", messageID = statusInput };
                    return message;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"오류: {ex.Message}");
                return false;
            }
        }
    }
}
