using nFusion.Nodes.Core;
using MessageLibrary.InternalMessage;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MessageLibrary;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using nFusion.Model.msg_0904;
using nFusion.Model.CommonType;

namespace MessageLibrary.InternalMessage
{
    public class Dummy0904 : IDummyMessageStrategy
    {
        public object CreateDummyMessage()
        {
            RequestBehaviorTree requestBehaviorTree = new RequestBehaviorTree
            {
                timestamp = Utility.GenerateTimestampUlong(),
                source = "Dummy",
                mode = 1
            };


            Console.Write("보낼 명령 입력");
            Console.Write("0: 중지, 1: 시작");
            string input = Console.ReadLine();

            Console.WriteLine("-------------------------------------------------------");
            if (input.ToLower() == "exit")
                return 0; // 종료

            if (!uint.TryParse(input, out uint command) || command < 0 || command > 1)
            {
                Console.WriteLine("잘못된 입력입니다. 1으로 기본 설정되었습니다.");
            }
            else
            {
                requestBehaviorTree.mode = command;
            }

            return requestBehaviorTree;
        }
    }
}