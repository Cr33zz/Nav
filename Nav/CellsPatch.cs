using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;

namespace Nav
{
    public class CellsPatch : Cell
    {
        public CellsPatch(HashSet<Cell> cells, MovementFlag movement_flags)
        {
            Flags = movement_flags;
            Cells = cells;
            GlobalId = LastCellsPatchGlobalId++;
        }

        internal override void Serialize(BinaryWriter w)
        {
            base.Serialize(w);

            w.Write(Cells.Count);
            foreach (Cell cell in Cells)
                w.Write(cell.GlobalId);
        }

        internal void Deserialize(List<Cell> all_cells, BinaryReader r)
        {
            base.Deserialize(all_cells, r);

            int cells_count = r.ReadInt32();
            for (int i = 0; i < cells_count; ++i)
            {
                int cell_global_id = r.ReadInt32();
                Cells.Add(all_cells.Find(x => x.GlobalId == cell_global_id));
            }
        }

        public HashSet<Cell> Cells { get; private set; }

        internal static int LastCellsPatchGlobalId = 0;
    }
}
