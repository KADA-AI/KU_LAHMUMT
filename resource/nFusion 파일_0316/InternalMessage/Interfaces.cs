using System;
using System.Collections.Generic;
using System.Text;

namespace MessageLibrary.InternalMessage
{
    public interface IDummyMessageStrategy
    {
        object CreateDummyMessage();
    }
}
