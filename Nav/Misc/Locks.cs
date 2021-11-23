using System;
using System.Diagnostics;
using System.Threading;

namespace Nav
{
    public class EmptyLock : IDisposable
    {
        ~EmptyLock()
        {
            // Finalizer calls Dispose(false)
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
        }
    }

    public class ReadLock : EmptyLock
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

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                if (upgradeable)
                    rwl.ExitUpgradeableReadLock();
                else
                    rwl.ExitReadLock();

                if (context != null)
                    Trace.WriteLine($"Exited ReadLock '{context}' lock duration {lockTimer.ElapsedMilliseconds}ms");
            }

            base.Dispose(disposing);
        }

        private ReaderWriterLockSlim rwl;
        private bool upgradeable;
        private string context;
        private Stopwatch lockTimer;
    }

    public class WriteLock : EmptyLock
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

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                rwl.ExitWriteLock();

                if (context != null)
                    Trace.WriteLine($"Exited WriteLock '{context}' lock duration {lockTimer.ElapsedMilliseconds}ms");
            }
        }

        private ReaderWriterLockSlim rwl;
        private string context;
        private Stopwatch lockTimer;
    }
}
