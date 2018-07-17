using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
using Enigma.D3;

namespace Nav.D3
{
    class SceneSnoNavData
    {
        private enum NavCellFlags
        {
            AllowWalk = 0x1,
            AllowFlier = 0x2,
            AllowSpider = 0x4,
            LevelAreaBit0 = 0x8,
            LevelAreaBit1 = 0x10,
            NoNavMeshIntersected = 0x20,
            NoSpawn = 0x40,
            Special0 = 0x80,
            Special1 = 0x100,
            SymbolNotFound = 0x200,
            AllowProjectile = 0x400,
            AllowGhost = 0x800,
            RoundedCorner0 = 0x1000,
            RoundedCorner1 = 0x2000,
            RoundedCorner2 = 0x4000,
            RoundedCorner3 = 0x8000
        }

        public SceneSnoNavData(Enigma.D3.Assets.Scene sno_scene)
        {
            scene_sno_id = (int)sno_scene.x000_Header.x00_SnoId;

            Enigma.D3.Assets.Scene.NavCell[] nav_cells = sno_scene.x180_NavZoneDefinition.x08_NavCells;

            if (nav_cells == null)
                return;

            foreach (Enigma.D3.Assets.Scene.NavCell nav_cell in nav_cells)
            {
                MovementFlag flags = MovementFlag.None;

                if ((nav_cell.x18 & (short)NavCellFlags.AllowWalk) != 0)
                    flags |= MovementFlag.Walk;
                if ((nav_cell.x18 & (short)NavCellFlags.AllowFlier) != 0)
                    flags |= MovementFlag.Fly;

                // skip not walkable/flyable
                if (flags == MovementFlag.None)
                    continue;

                float min_x = nav_cell.x00.X;
                float min_y = nav_cell.x00.Y;
                float min_z = nav_cell.x00.Z;

                float max_x = nav_cell.x0C.X;
                float max_y = nav_cell.x0C.Y;
                float max_z = nav_cell.x0C.Z;

                cells.Add(new Nav.Cell(min_x, min_y, min_z, max_x, max_y, max_z, flags));
            }
        }

        public SceneSnoNavData(int scene_sno_id)
        {
            this.scene_sno_id = scene_sno_id;

            Load();
        }

        public int SceneSnoId
        {
            get { return scene_sno_id; }
        }

        public List<Cell> Cells
        {
            get { return cells; }
        }

        private void Load()
        {
            if (!File.Exists(FileName))
                return;

            using (FileStream fs = File.OpenRead(FileName))
            using (BinaryReader br = new BinaryReader(fs))
            {
                int cells_count = br.ReadInt32();

                for (int i = 0; i < cells_count; ++i)
                    cells.Add(new Cell(br.ReadSingle(), br.ReadSingle(), br.ReadSingle(), br.ReadSingle(), br.ReadSingle(), br.ReadSingle(), (MovementFlag)br.ReadInt32()));
            }
        }

        public void Save()
        {
            if (File.Exists(FileName))
                return;

            using (FileStream fs = File.Create(FileName))
            using (BinaryWriter bw = new BinaryWriter(fs))
            {
                bw.Write(cells.Count);

                foreach (Cell cell in cells)
                {
                    bw.Write(cell.Min.X);
                    bw.Write(cell.Min.Y);
                    bw.Write(cell.Min.Z);

                    bw.Write(cell.Max.X);
                    bw.Write(cell.Max.Y);
                    bw.Write(cell.Max.Z);

                    bw.Write((int)cell.Flags);
                }
            }
        }

        private string FileName
        {
            get { return Navmesh.SceneSnoCacheDir + scene_sno_id; }
        }

        private int scene_sno_id = -1;
        List<Cell> cells = new List<Cell>();
    }
}
