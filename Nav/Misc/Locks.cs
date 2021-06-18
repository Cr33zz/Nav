using System;
using System.Diagnostics;
using System.Threading;

namespace Nav
{
    public class ReadLock : IDisposable
    {
        public ReadLock(ReaderWriterLockSlim rwl, bool upgradeable = false, string context = null)
        {
            this.rwl = rwl;
            this.upgradeable = upgradeable;
            this.context = context;

            if (context != null)
                Trace.WriteLine($"Wants ReadLock '{context}'");

                Stopwatch timer = null;
            if (context != null)
                timer = Stopwatch.StartNew();

            if (upgradeable)
                rwl.EnterUpgradeableReadLock();
            else
                rwl.EnterReadLock();

            if (context != null)
            {
                Trace.WriteLine($"Entered ReadLock '{context}' enter wait duration {(timer.ElapsedMilliseconds)}ms");
                lockTimer = Stopwatch.StartNew();
            }
        }

        public void Dispose()
        {
            if (upgradeable)
                rwl.ExitUpgradeableReadLock();
            else
                rwl.ExitReadLock();

            if (context != null)
                Trace.WriteLine($"Exited ReadLock '{context}' lock duration {lockTimer.ElapsedMilliseconds}ms");
        }

        private ReaderWriterLockSlim rwl;
        private bool upgradeable;
        private string context;
        private Stopwatch lockTimer;
    }

    public class WriteLock : IDisposable
    {
        public WriteLock(ReaderWriterLockSlim rwl, string context = null)
        {
            this.rwl = rwl;
            this.context = context;

            if (context != null)
                Trace.WriteLine($"Wants WriteLock '{context}'");

            Stopwatch timer = null;
            if (context != null)
                timer = Stopwatch.StartNew();

            rwl.EnterWriteLock();

            if (context != null)
            {
                Trace.WriteLine($"Entered WriteLock '{context}' enter wait duration {(timer.ElapsedMilliseconds)}ms");
                lockTimer = Stopwatch.StartNew();
            }
        }

        public void Dispose()
        {
            rwl.ExitWriteLock();

            if (context != null)
                Trace.WriteLine($"Exited WriteLock '{context}' lock duration {lockTimer.ElapsedMilliseconds}ms");
        }

        private ReaderWriterLockSlim rwl;
        private string context;
        private Stopwatch lockTimer;
    }
}
