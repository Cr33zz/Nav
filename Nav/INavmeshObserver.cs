using System;

namespace Nav
{
    public interface INavmeshObserver
    {
        void OnGridCellAdded(GridCell grid_cell);
        void OnNavDataChanged(AABB affected_area);
        void OnNavBlockersChanged();
        void OnPatchesChanged();
        void OnNavDataCleared();
    }
}
