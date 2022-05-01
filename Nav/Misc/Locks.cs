using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;

namespace Nav
{
    public class LockInfo
    {
        public LockInfo(ushort id, string description, Int64 lockTime, Thread lockThread)
        {
            Id = id;
            Description = description;
            LockTime = lockTime;
            LockThread = lockThread;
        }

        public override string ToString()
        {
            return $"#{Id} {Description} ({LocksState.GetLockedDuration(LockTime)}ms)";
        }

        public string ToStringWithStackTrace()
        {
            return $"{ToString()}\n{GetStackTrace()}";
        }

        public string GetStackTrace()
        {
            return LocksState.GetStackTrace(LockThread)?.ToString() ?? "";
        }

        public readonly ushort Id;
        public readonly string Description;
        public readonly Int64 LockTime;
        public readonly Thread LockThread;
    }

    public static class LocksState
    {
        internal static Int64 GetLockedDuration(Int64 lockTime) => ExistanceTimer.ElapsedMilliseconds - lockTime;

        internal static void Reset()
        {
            lock (Locker)
            {
                NextId = 0;
                ActiveLocks.Clear();
                ExistanceTimer.Restart();
            }
        }

        internal static LockInfo OnLockEnter(string description)
        {
            lock (Locker)
            {
                var info = new LockInfo(NextId++, description, ExistanceTimer.ElapsedMilliseconds, Thread.CurrentThread);
                ActiveLocks.Add(info);
                return info;
            }
        }

        internal static void OnLockExit(LockInfo info)
        {
            lock (Locker)
                ActiveLocks.Remove(info);
        }

        public static List<LockInfo> GetActiveLocks()
        {
            lock (Locker)
                return ActiveLocks.ToList();
        }

#pragma warning disable CS0618
        public static StackTrace GetStackTrace(Thread targetThread)
        {
            //using (ManualResetEvent fallbackThreadReady = new ManualResetEvent(false), exitedSafely = new ManualResetEvent(false))
            //{
            //    Thread fallbackThread = new Thread(delegate ()
            //    {
            //        fallbackThreadReady.Set();
            //        while (!exitedSafely.WaitOne(200))
            //        {
            //            try
            //            {
            //                targetThread.Resume();
            //            }
            //            catch (Exception) {/*Whatever happens, do never stop to resume the target-thread regularly until the main-thread has exited safely.*/}
            //        }
            //    });
            //    fallbackThread.Name = "GetStackFallbackThread";
            //    try
            //    {
            //        fallbackThread.Start();
            //        fallbackThreadReady.WaitOne();
            //        //From here, you have about 200ms to get the stack-trace.
            //        targetThread.Suspend();
            //        StackTrace trace = null;
            //        try
            //        {
            //            trace = new StackTrace(targetThread, true);
            //        }
            //        catch (ThreadStateException)
            //        {
            //            //failed to get stack trace, since the fallback-thread resumed the thread
            //            //possible reasons:
            //            //1.) This thread was just too slow (not very likely)
            //            //2.) The deadlock ocurred and the fallbackThread rescued the situation.
            //            //In both cases just return null.
            //        }
            //        try
            //        {
            //            targetThread.Resume();
            //        }
            //        catch (ThreadStateException) {/*Thread is running again already*/}
            //        return trace;
            //    }
            //    finally
            //    {
            //        //Just signal the backup-thread to stop.
            //        exitedSafely.Set();
            //        //Join the thread to avoid disposing "exited safely" too early. And also make sure that no leftover threads are cluttering iis by accident.
            //        fallbackThread.Join();
            //    }
            //}
            return null;
        }
#pragma warning restore CS0618

        private static readonly Stopwatch ExistanceTimer = Stopwatch.StartNew();
        private static readonly HashSet<LockInfo> ActiveLocks = new HashSet<LockInfo>();
        private static readonly object Locker = new object();
        private static ushort NextId = 0;
    }

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

        protected LockInfo info;
    }

    public class ReadLock : EmptyLock
    {
        public ReadLock(ReaderWriterLockSlim rwl, bool upgradeable = false, string description = null)
        {
            this.rwl = rwl;
            this.upgradeable = upgradeable;

            if (upgradeable)
                rwl.EnterUpgradeableReadLock();
            else
                rwl.EnterReadLock();

            if (description != null)
                info = LocksState.OnLockEnter($"R{(upgradeable ? "U" : "")} {description}");
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                if (upgradeable)
                    rwl.ExitUpgradeableReadLock();
                else
                    rwl.ExitReadLock();

                if (info != null)
                    LocksState.OnLockExit(info);
            }

            base.Dispose(disposing);
        }

        private ReaderWriterLockSlim rwl;
        private bool upgradeable;
    }

    public class WriteLock : EmptyLock
    {
        public WriteLock(ReaderWriterLockSlim rwl, string description = null)
        {
            this.rwl = rwl;

            //if (description != null)
            //    Trace.WriteLine($"Wants WriteLock '{description}'");

            //Stopwatch timer = null;
            //if (description != null)
            //    timer = Stopwatch.StartNew();

            rwl.EnterWriteLock();

            //Trace.WriteLine($"Entered WriteLock '{description}' enter wait duration {(timer.ElapsedMilliseconds)}ms");

            if (description != null)
                info = LocksState.OnLockEnter($"W {description}");

        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                rwl.ExitWriteLock();

                if (info != null)
                    LocksState.OnLockExit(info);
            }
        }

        private ReaderWriterLockSlim rwl;
    }
}
