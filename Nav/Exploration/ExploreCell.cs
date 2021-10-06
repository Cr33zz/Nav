using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;

namespace Nav
{
    public class ExploreCell : Cell
    {
        public ExploreCell()
        {
            InitExploreCell();
            Cells = new List<Cell>();
            GridCellsId = new List<int>();
        }

        public ExploreCell(AABB aabb, List<Cell> cells, List<int> grid_cells_id, int id = -1)
            : base(aabb.Min.X, aabb.Min.Y, 0, aabb.Max.X, aabb.Max.Y, 0, MovementFlag.None, id)
        {
            InitExploreCell();

            Cells = cells;
            GridCellsId = grid_cells_id;

            foreach (Cell c in Cells)
            {
                CellsArea += c.AABB.Area;
                CellsAABB = CellsAABB.Extend(c.AABB);
            }

            AABB clampedCellsAABB = default(AABB);
            CellsAABB.Intersect(this.AABB, ref clampedCellsAABB);
            CellsAABB = clampedCellsAABB;

            var nearest_cell = cells.OrderBy(x => CellsAABB.Center.Distance2DSqr(x.AABB.Align(CellsAABB.Center))).First();
            Position = nearest_cell.AABB.Align(CellsAABB.Center);
        }

        private void InitExploreCell()
        {
            Explored = false;
            Delayed = false;
            Small = false;
            GlobalId = LastExploreCellGlobalId++;
        }

        public bool AddNeighbour(ExploreCell explore_cell)
        {
            if (!Overlaps2D(explore_cell, true))
                return false;

            foreach (Cell cell in Cells)
            {
                foreach (Cell other_cell in explore_cell.Cells)
                {
                    if (cell.Equals(other_cell) || cell.Neighbours.Exists(x => x.cell.Equals(other_cell)))
                    {
                        Neighbours.Add(new Neighbour(explore_cell, Vec3.ZERO, MovementFlag.None));
                        explore_cell.Neighbours.Add(new Neighbour(this, Vec3.ZERO, MovementFlag.None));
                        
                        return true;
                    }
                }
            }

            return false;
        }

        public IEnumerable<Cell> GetCellsAt(Vec3 pos, bool test_2d, float z_tolerance)
        {
            return Cells.Where(x => (test_2d ? x.Contains2D(pos) : x.Contains(pos, z_tolerance)));
        }

        internal override void Serialize(BinaryWriter w)
        {
            base.Serialize(w);

            w.Write(Explored);
            w.Write(Delayed);
            w.Write(Small);
            Position.Serialize(w);
            CellsAABB.Serialize(w);
            w.Write(CellsArea);

            w.Write(Cells.Count);
            foreach (Cell cell in Cells)
                w.Write(cell.GlobalId);

            w.Write(GridCellsId.Count);
            foreach (int g_cell_id in GridCellsId)
                w.Write(g_cell_id);
        }

        internal void Deserialize(HashSet<ExploreCell> explore_cells, HashSet<Cell> all_cells, Dictionary<int, Cell> id_to_cell, BinaryReader r)
        {
            base.Deserialize(explore_cells, null, r);

            Explored = r.ReadBoolean();
            Delayed = r.ReadBoolean();
            Small = r.ReadBoolean();
            Position = new Vec3(r);
            CellsAABB = new AABB(r);
            CellsArea = r.ReadSingle();

            int cells_count = r.ReadInt32();
            for (int i = 0; i < cells_count; ++i)
            {
                int cell_global_id = r.ReadInt32();
                var cell = id_to_cell != null ? id_to_cell[cell_global_id] : all_cells.FirstOrDefault(x => x.GlobalId == cell_global_id);
                Cells.Add(cell);
            }

            int grid_cells_id_count = r.ReadInt32();
            for (int i = 0; i < grid_cells_id_count; ++i)
            {
                int grid_cell_id = r.ReadInt32();
                GridCellsId.Add(grid_cell_id);
            }
        }

        public bool CellsContains2D(Vec3 p)
        {
            if (!Contains2D(p))
                return false;

            return Cells.Exists(c => c.Contains2D(p));
        }

        public List<Cell> Cells { get; private set; }
        public List<int> GridCellsId { get; private set; }
        public Vec3 Position { get; private set; }
        public override Vec3 Center { get { return Position; } }
        public AABB CellsAABB { get; private set; }
        public float CellsArea { get; private set; }

        public bool Explored;

        // Exploration of this cell has been delayed until all non-small unexplored cells has been visited
        public bool Delayed;

        // Small explore cells will be visited as last
        public bool Small;

        internal static int LastExploreCellGlobalId = 0;
    }
}
