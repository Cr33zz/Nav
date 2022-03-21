using System;
using System.Diagnostics;

namespace Nav
{
    public class ScopedTimer : IDisposable
    {
        public ScopedTimer(Stopwatch timer, bool restart = true)
        {
            Timer = timer;
            if (restart)
                Timer.Restart();
            else
                Timer.Start();
        }

        public void Dispose()
        {
            Timer.Stop();
        }

        private Stopwatch Timer = new Stopwatch();
    }
}
