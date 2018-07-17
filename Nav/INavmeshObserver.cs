using System;

namespace Nav
{
    public interface INavmeshObserver
    {
        void OnGridCellAdded(GridCell grid_cell);
        void OnNavDataChanged();
        void OnNavDataCleared();
    }
}
