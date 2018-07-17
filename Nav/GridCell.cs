using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;

namespace Nav
{
    public class GridCell : Cell
    {
        public GridCell()
        {
            InitGridCell(-1);
        }

        public GridCell(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, int id = -1, int area_id = -1)
            : base(min_x, min_y, min_z, max_x, max_y, max_z, MovementFlag.None, id)
        {
            InitGridCell(area_id);
        }

        public GridCell(Vec3 min, Vec3 max, int id = -1, int area_id = -1)
            : base(min, max, MovementFlag.None, id)
        {
            InitGridCell(area_id);
        }

        private void InitGridCell(int area_id)
        {
            AreaId = area_id;
            Cells = new List<Cell>();
            GlobalId = LastGridCellGlobalId++;
        }

        public float MIN_CELL_AREA_TO_CONSIDER = 0;

        public bool Add(Cell cell)
        {
            if (cell.AABB.Area < MIN_CELL_AREA_TO_CONSIDER)
                return false;

            Vec3 border_point = null;

            foreach (Cell our_cell in Cells)
                our_cell.AddNeighbour(cell, out border_point);

            Cells.Add(cell);

            return true;
        }

        public bool Add(List<Cell> cells)
        {
            bool anything_added = false;

            foreach (Cell n_cell in cells)
                anything_added = anything_added || Add(n_cell);
            
            return anything_added;
        }

        // Try to connect grid_cell with this cell. Connection can be established only when any two cells of both grid cells are connected. This function should behave
        // properly when called multiple times for the same pair of grid cells!
        public void AddNeighbour(GridCell grid_cell)
        {
            if (GlobalId == grid_cell.GlobalId || AABB.Intersect(grid_cell.AABB, true) == null)
                return;

            // this is removed to properly handle merging
            //if (Neighbours.Exists(x => x.cell.Equals(grid_cell)))
            //    return;

            bool any_cells_connected = false;
            MovementFlag connection_flags = MovementFlag.None;

            foreach (Cell our_cell in Cells)
            {
                foreach (Cell other_cell in grid_cell.Cells)
                {
                    Vec3 border_point = null;

                    bool cells_connected = our_cell.AddNeighbour(other_cell, out border_point);

                    if (cells_connected)
                    {
                        any_cells_connected = true;
                        MovementFlag flags = our_cell.Flags & other_cell.Flags;

                        if (flags > connection_flags)
                            connection_flags = flags;
                    }
                }
            }

            if (any_cells_connected)
            {
                Neighbour n1 = Neighbours.FirstOrDefault(x => x.cell.GlobalId == grid_cell.GlobalId);

                // if they were not connected before, simply connect them
                if (n1 == null)
                {
                    Neighbours.Add(new Neighbour(grid_cell, null, null, connection_flags));
                    grid_cell.Neighbours.Add(new Neighbour(this, null, null, connection_flags));
                }
                // otherwise verify connection flags
                else if (n1.connection_flags < connection_flags)
                {
                    Neighbour n2 = grid_cell.Neighbours.FirstOrDefault(x => x.cell.GlobalId == GlobalId);

                    n1.connection_flags = connection_flags;
                    n2.connection_flags = connection_flags;
                }
            }
        }

        // Only replacement cells should be removed
        internal void Remove(Cell cell)
        {
            cell.Detach();
            Cells.RemoveAll(x => x.GlobalId == cell.GlobalId);
        }

        internal override void Serialize(BinaryWriter w)
        {
            base.Serialize(w);

            w.Write(Cells.Count);
            foreach(Cell cell in Cells)
                w.Write(cell.GlobalId);
        }

        internal void Deserialize(List<GridCell> grid_cells, List<Cell> all_cells, BinaryReader r)
        {
            base.Deserialize(grid_cells, r);

            int cells_count = r.ReadInt32();
            for (int i = 0; i < cells_count; ++i)
            {
                int cell_global_id = r.ReadInt32();
                Cells.Add(all_cells.Find(x => x.GlobalId == cell_global_id));
            }
        }

        public List<Cell> Cells { get; private set; }
        public int AreaId { get; private set; }

        internal static int LastGridCellGlobalId = 0;
    }
}
