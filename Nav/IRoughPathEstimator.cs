using System;
using System.Collections.Generic;

namespace Nav
{
    public interface IRoughPathEstimator
    {
        bool FindRoughPath(Vec3 from, Vec3 to, ref List<Vec3> path, out string debug_info);
        float GetRoughPathRecalcPrecision();
    }
}
