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

            if (upgradeable)
                rwl.EnterUpgradeableReadLock();
            else
                rwl.EnterReadLock();

            if (context != null)
                Trace.WriteLine($"Entered ReadLock '{context}'");
        }

        public void Dispose()
        {
            if (upgradeable)
                rwl.ExitUpgradeableReadLock();
            else
                rwl.ExitReadLock();

            if (context != null)
                Trace.WriteLine($"Exited ReadLock '{context}'");
        }

        private ReaderWriterLockSlim rwl;
        private bool upgradeable;
        private string context;
    }

    public class WriteLock : IDisposable
    {
        public WriteLock(ReaderWriterLockSlim rwl, string context = null)
        {
            this.rwl = rwl;
            this.context = context;

            rwl.EnterWriteLock();

            if (context != null)
                Trace.WriteLine($"Entered WriteLock '{context}'");
        }

        public void Dispose()
        {
            rwl.ExitWriteLock();

            if (context != null)
                Trace.WriteLine($"Exited WriteLock '{context}'");
        }

        private ReaderWriterLockSlim rwl;
        private string context;
    }
}
