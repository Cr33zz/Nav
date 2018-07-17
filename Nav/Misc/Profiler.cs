using System;
using System.Diagnostics;

namespace Nav
{
    public class Profiler : IDisposable
    {
        public Profiler(string msg, int log_threshold = -1)
        {
            this.log_threshold = log_threshold;
            message = msg;
            timer.Start();
        }

        public void Dispose()
        {
            timer.Stop();

            if (timer.ElapsedMilliseconds > log_threshold)
                Trace.WriteLine(message.Replace("{t}", timer.ElapsedMilliseconds.ToString() + " ms"));
        }

        private Stopwatch timer = new Stopwatch();
        private string message;
        private int log_threshold;
    }
}
