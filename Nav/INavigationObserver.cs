using System;

namespace Nav
{
    public interface INavigationObserver
    {
        void OnHugeCurrentPosChange();
        void OnDestinationReached(DestType type, Vec3 dest, Object userData);
        void OnDestinationReachFailed(DestType type, Vec3 dest, Object userData);
    }
}
