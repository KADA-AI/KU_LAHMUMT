using System;
using System.Collections.Generic;
using System.Reflection;
using System.Linq;

namespace MessageLibrary.InternalMessage
{
    //public class Dummy0801 : IDummyMessageStrategy
    //{
    //    public object CreateDummyMessage()
    //    {
    //        return new nFusion.Model.msg_0801.ReplanCommand
    //        {
    //            OperatorReplanRequestTime = 1.0f,
    //            IsOnGround = true,
    //            InputMissionPackageID = "InputMissionPackage1",
    //            MissionReferencePackageID = "MissionReferencePackage1"
    //        };
    //    }
    //}

    //public class Dummy0804 : IDummyMessageStrategy
    //{
    //    public object CreateDummyMessage()
    //    {
    //        return new nFusion.Model.msg_0804.MissionRestartCommand
    //        {
    //            Timestamp = 1.0f,
    //            InputMissionID = "InputMissionID"
    //        };
    //    }
    //}

    //public class Dummy0805 : IDummyMessageStrategy
    //{
    //    public object CreateDummyMessage()
    //    {
    //        return new nFusion.Model.msg_0805.EndMissionCommand
    //        {
    //            Timestamp = 1.0f
    //        };
    //    }
    //}

    //public class Dummy0806 : IDummyMessageStrategy
    //{
    //    public object CreateDummyMessage()
    //    {
    //        return new nFusion.Model.msg_0806.EndSWCommand
    //        {
    //            Timestamp = 1.0f
    //        };
    //    }
    //}

    //public class Dummy0904 : IDummyMessageStrategy
    //{
    //    public object CreateDummyMessage()
    //    {
    //        return new nFusion.Model.msg_0904.RequestBehaviorTree
    //        {
    //            BehaviorTreeFileID = "BehaviorTreeFileID"
    //        };
    //    }
    //}

    public class DummyMessageStrategyFactory
    {
        private static readonly Dictionary<string, IDummyMessageStrategy> _strategies;

        static DummyMessageStrategyFactory()
        {
            _strategies = new Dictionary<string, IDummyMessageStrategy>
            {
                { "0000", new Dummy0000() },
                { "0101", new Dummy0101() },
                { "0102", new Dummy0102() },
                { "0103", new Dummy0103() },
                { "0201", new Dummy0201() },
                { "0202", new Dummy0202() },
                { "0203", new Dummy0203() },
                { "0301", new Dummy0301() },
                { "0305", new Dummy0305() },
                { "0401", new Dummy0401() },
                { "0402", new Dummy0402() },
                { "0501", new Dummy0501() },
                { "0502", new Dummy0502() },
                { "0503", new Dummy0503() },
                { "0504", new Dummy0504() },
                { "0601", new Dummy0601() },
                { "0702", new Dummy0702() },
                //{ "0801", new Dummy0801() },
                { "0802", new Dummy0802() },
                { "0803", new Dummy0803() },
                //{ "0804", new Dummy0804() },
                { "0805", new Dummy0805() },
                { "0806", new Dummy0806() },
                { "0903", new Dummy0903() },
                { "0904", new Dummy0904() }
            };
        }

        public static IDummyMessageStrategy GetStrategy(string messageId)
        {
            if (_strategies.TryGetValue(messageId, out var strategy))
                return strategy;
            throw new InvalidOperationException($"지원하지 않는 메시지 ID입니다: {messageId}");
        }
    }

    public static class MessageFactoryPilotDecision
    {
        public static object CreateDummyMessage(string messageIdStr)
        {
            var strategy = DummyMessageStrategyFactory.GetStrategy(messageIdStr);
            return strategy.CreateDummyMessage();
        }
    }
}
