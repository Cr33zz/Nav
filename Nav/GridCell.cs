using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Threading;
using System.Diagnostics;

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
            GlobalId = Interlocked.Increment(ref LastGridCellGlobalId);
        }

        public float MIN_CELL_AREA_TO_CONSIDER = 0;

        // add this cell to grid and try to connect it with already existing cells
        public bool Add(Cell cell)
        {
            if (cell.AABB.Area < MIN_CELL_AREA_TO_CONSIDER)
                return false;

            Vec3 border_point = default(Vec3);

            foreach (Cell our_cell in Cells)
                our_cell.TryAddNeighbour(cell, ref border_point);

            cell.ParentAABB = AABB;
            Cells.AddFirst(cell);

            return true;
        }

        public bool Add(IEnumerable<Cell> cells)
        {
            bool anything_added = false;

            foreach (Cell n_cell in cells)
                anything_added |= Add(n_cell);
            
            return anything_added;
        }

        // Try to connect grid_cell with this cell. Connection can be established only when any two cells of both grid cells are connected. This function should behave
        // properly when called multiple times for the same pair of grid cells!
        public void AddNeighbour(GridCell grid_cell)
        {
            if (GlobalId == grid_cell.GlobalId || !AABB.Overlaps2D(grid_cell.AABB, true))
                return;

            bool any_cells_connected = false;
            MovementFlag connection_flags = MovementFlag.None;

            foreach (Cell our_cell in Cells)
            {
                foreach (Cell other_cell in grid_cell.Cells)
                {
                    Vec3 border_point = default(Vec3);
                    bool cells_connected = our_cell.TryAddNeighbour(other_cell, ref border_point);

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
                    Neighbours.Add(new Neighbour(grid_cell, Vec3.ZERO, connection_flags));
                    grid_cell.Neighbours.Add(new Neighbour(this, Vec3.ZERO, connection_flags));
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

        // This function will return list of cells pairs that should be connected
        public List<(Cell, Cell, Vec3)> CheckNeighbour(GridCell grid_cell, out MovementFlag connection_flags)
        {
            var result = new List<(Cell, Cell, Vec3)>();
            connection_flags = MovementFlag.None;

            if (GlobalId == grid_cell.GlobalId || !AABB.Overlaps2D(grid_cell.AABB, true))
                return result;

            foreach (Cell our_cell in Cells)
            {
                foreach (Cell other_cell in grid_cell.Cells)
                {
                    Vec3 border_point = default(Vec3);
                    bool should_be_connected = our_cell.ShouldBecomeNeighbours(other_cell, ref border_point);

                    if (should_be_connected)
                    {
                        result.Add((our_cell, other_cell, border_point));

                        MovementFlag flags = our_cell.Flags & other_cell.Flags;

                        if (flags > connection_flags)
                            connection_flags = flags;
                    }
                }
            }

            return result;
        }

        internal override void Serialize(BinaryWriter w)
        {
            base.Serialize(w);

            w.Write(Cells.Count);
            foreach(Cell cell in Cells)
                w.Write(cell.GlobalId);

            w.Write(ReplacementCells.Count);
            foreach (Cell cell in ReplacementCells)
                w.Write(cell.GlobalId);
        }

        internal void Deserialize(HashSet<GridCell> all_grid_cells, HashSet<Cell> all_cells, Dictionary<int, Cell> id_to_cell, BinaryReader r)
        {
            base.Deserialize(all_grid_cells, null, r);

            int cells_count = r.ReadInt32();
            for (int i = 0; i < cells_count; ++i)
            {
                int cell_global_id = r.ReadInt32();
                Cells.AddFirst(id_to_cell != null ? id_to_cell[cell_global_id] : all_cells.First(x => x.GlobalId == cell_global_id));
            }

            int replacement_cells_count = r.ReadInt32();
            for (int i = 0; i < replacement_cells_count; ++i)
            {
                int cell_global_id = r.ReadInt32();
                ReplacementCells.Add(all_cells.First(x => x.GlobalId == cell_global_id));
            }
        }

        public List<Cell> GetCells(Func<Cell, bool> predicate, bool allow_replacement_cells = true)
        {
            var result = Cells.Where(predicate).ToList();

            if (allow_replacement_cells)
                result.AddRange(ReplacementCells.Where(predicate));
            
            return result;
        }

        public IEnumerable<Cell> GetCellsAt(Vec3 pos, bool test_2d, float z_tolerance)
        {
            return Cells.Where(x => (test_2d ? x.Contains2D(pos) : x.Contains(pos, z_tolerance))).
                   Concat(ReplacementCells.Where(x => (test_2d ? x.Contains2D(pos) : x.Contains(pos, z_tolerance))));
        }

        public List<Cell> GetCells(bool allow_replacement_cells = true)
        {
            IEnumerable<Cell> result = Cells;

            if (allow_replacement_cells)
                result = result.Concat(ReplacementCells);

            return result.ToList();
        }

        public Int32 GetCellsCount(bool count_replacement_cells = true)
        {
            return Cells.Count + (count_replacement_cells ? ReplacementCells.Count : 0);
        }

        internal void AddReplacementCell(Cell cell)
        {
            ReplacementCells.Add(cell);
        }

        internal void RemoveReplacementCell(Cell cell)
        {
            cell.Detach();
            ReplacementCells.Remove(cell);
        }

        private LinkedList<Cell> Cells = new LinkedList<Cell>();
        private List<Cell> ReplacementCells  = new List<Cell>();
        public int AreaId { get; private set; }

        internal static int LastGridCellGlobalId = 0;
    }
}
