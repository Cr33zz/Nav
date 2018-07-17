using System;
using System.Threading;

namespace Nav
{
    public class ReadLock : IDisposable
    {        
        public ReadLock(ReaderWriterLockSlim rwl, bool upgradeable = false)
        {
            this.rwl = rwl;
            this.upgradeable = upgradeable;

            if (upgradeable)
                rwl.EnterUpgradeableReadLock();
            else
                rwl.EnterReadLock();
        }

        public void Dispose()
        {
            if (upgradeable)
                rwl.ExitUpgradeableReadLock();
            else
                rwl.ExitReadLock();
        }

        private ReaderWriterLockSlim rwl;
        private bool upgradeable;
    }

    public class WriteLock : IDisposable
    {
        public WriteLock(ReaderWriterLockSlim rwl)
        {
            this.rwl = rwl;

            rwl.EnterWriteLock();
        }

        public void Dispose()
        {
            rwl.ExitWriteLock();
        }

        private ReaderWriterLockSlim rwl;
    }
}
