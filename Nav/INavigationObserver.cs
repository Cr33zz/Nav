using System;

namespace Nav
{
    public interface INavigationObserver
    {
        void OnHugeCurrentPosChange();
        void OnDestinationReached(destination dest);
        void OnDestinationReachFailed(destination dest);
    }
}
