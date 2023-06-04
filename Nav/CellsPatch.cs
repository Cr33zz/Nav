using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Threading;

namespace Nav
{
    public class CellsPatch : Cell
    {
        public CellsPatch(HashSet<Cell> cells, Dictionary<AABB, List<Cell>> cellsGrid, MovementFlag movement_flags)
        {
            Flags = movement_flags;
            Cells = cells;
            CellsGrid = cellsGrid;
            GlobalId = Interlocked.Increment(ref LastCellsPatchGlobalId);
        }

        internal List<Cell> GetCellsWithin(Vec3 p, float radius, MovementFlag flags)
        {
            var result_cells = new List<Cell>();

            if (p.IsZero())
                return result_cells;

            foreach (var cellsG in CellsGrid)
            {
                if (cellsG.Key.Distance2D(p) > radius)
                    continue;

                result_cells.AddRange(Algorihms.GetCellsWithin(cellsG.Value, p, radius, flags, allow_disabled: true));
            }

            return result_cells;
        }

        internal List<Cell> GetCellsWithin(AABB area, MovementFlag flags)
        {
            var result_cells = new List<Cell>();

            foreach (var cellsG in CellsGrid)
            {
                if (!cellsG.Key.Overlaps2D(area))
                    continue;

                result_cells.AddRange(Algorihms.GetCellsWithin(cellsG.Value, area, flags, allow_disabled: true));
            }

            return result_cells;
        }

        internal bool AnyCellWithin(Vec3 p, float radius, MovementFlag flags)
        {
            if (p.IsZero())
                return false;

            foreach (var cellsG in CellsGrid)
            {
                if (cellsG.Key.Distance2D(p) > radius)
                    continue;

                if (Algorihms.AnyCellWithin(cellsG.Value, p, radius, flags, allow_disabled: true))
                    return true;
            }

            return false;
        }

        public override void Serialize(BinaryWriter w)
        {
            base.Serialize(w);

            w.Write(Cells.Count);
            foreach (Cell cell in Cells)
                w.Write(cell.GlobalId);

            w.Write(CellsGrid.Count);
            foreach (var entry in CellsGrid)
            {
                entry.Key.Serialize(w);
                w.Write(entry.Value.Count);
                foreach (var cell in entry.Value)
                    w.Write(cell.GlobalId);
            }
        }

        public void Deserialize(HashSet<Cell> all_cells, Dictionary<int, Cell> id_to_cell, BinaryReader r)
        {
            base.Deserialize(all_cells, null, r);

            int cells_count = r.ReadInt32();
            for (int i = 0; i < cells_count; ++i)
            {
                int cell_global_id = r.ReadInt32();
                Cells.Add(id_to_cell != null ? id_to_cell[cell_global_id] : all_cells.First(x => x.GlobalId == cell_global_id));
            }

            int cells_grid_count = r.ReadInt32();
            for (int i = 0; i < cells_grid_count; ++i)
            {
                var key = new AABB(r);
                int cells_list_count = r.ReadInt32();
                var cells = new List<Cell>();
                for (int j = 0; j < cells_list_count; ++j)
                {
                    int cell_global_id = r.ReadInt32();
                    cells.Add(id_to_cell != null ? id_to_cell[cell_global_id] : all_cells.First(x => x.GlobalId == cell_global_id));
                }
                CellsGrid.Add(key, cells);
            }
        }

        public HashSet<Cell> Cells { get; private set; }
        public Dictionary<AABB, List<Cell>> CellsGrid;

        internal static int LastCellsPatchGlobalId = 0;
    }
}
