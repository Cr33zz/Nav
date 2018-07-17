using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.IO;
using System.Diagnostics;
using System.Threading;
using System.Globalization;

namespace Nav
{
    public class region_data : IEquatable<region_data>
    {
        public region_data(AABB area, float move_cost_mult) { this.area = area; this.move_cost_mult = move_cost_mult; }
        public region_data(region_data region) { this.area = new AABB(region.area); this.move_cost_mult = region.move_cost_mult; }
        public region_data(BinaryReader r) { Deserialize(r); }

        public AABB area;
        public float move_cost_mult;

        public bool Equals(region_data region)
        {
            return move_cost_mult == region.move_cost_mult && area.Equals(region.area);
        }

        public override bool Equals(Object obj)
        {
            if (obj == null)
                return false;

            region_data r = obj as region_data;

            return Equals(r);
        }

        public override int GetHashCode()
        {
            return area.GetHashCode();
        }

        public void Serialize(BinaryWriter w)
        {
            area.Serialize(w);
            w.Write(move_cost_mult);
        }

        public void Deserialize(BinaryReader r)
        {
            area = new AABB(r);
            move_cost_mult = r.ReadSingle();
        }
    }

    public class Navmesh : IDisposable
    {
        public Navmesh(bool verbose = false)
        {
            Log("[Nav] Creating navmesh...");

            Verbose = verbose;

            Init();

            UpdatesThread = new Thread(Updates);
            UpdatesThread.Name = "Navmesh-UpdatesThread";
            UpdatesThread.Start();

            Log("[Nav] Navmesh created");
        }

        protected virtual void Init()
        {
            RegionsMoveCostMode = RegionsMode.Mult;
            RegionsEnabled = true;
        }

        // when true will print additional data to output
        public bool Verbose { get; set; }

        public bool Add(GridCell g_cell, bool trigger_nav_data_change)
        {
            if (g_cell == null || g_cell.Cells.Count == 0)
                return false;

            using (new WriteLock(DataLock))
            {
                // check if same grid is not already defined (merge then)
                GridCell base_grid_cell = m_GridCells.Find(x => x.AABB.Equals(g_cell.AABB));

                if (base_grid_cell != null)
                {
                    base_grid_cell.Add(g_cell.Cells);
                    Log("[Nav] Grid cell (" + g_cell.Id + " " + g_cell.Min + ") with " + g_cell.Cells.Count + " cell(s) merged with grid cell (" + base_grid_cell.Id + " " + base_grid_cell.Min + ")");

                    foreach (GridCell grid_cell in m_GridCells)
                        grid_cell.AddNeighbour(base_grid_cell);
                }
                else
                {
                    foreach (GridCell grid_cell in m_GridCells)
                        grid_cell.AddNeighbour(g_cell);

                    m_GridCells.Add(g_cell);
                    Log("[Nav] Grid cell (" + g_cell.Id + " " + g_cell.Min + ") with " + g_cell.Cells.Count + " cell(s) added");
                }

                m_AllCells.AddRange(g_cell.Cells);
                UpdateCellsPatches(g_cell.Cells);

                NotifyOnGridCellAdded(g_cell);
            }

            NotifyOnNavDataChanged();

            return true;
        }

        private void UpdateCellsPatches(List<Cell> cells)
        {
            //using (new Profiler("[Nav] Cells patches refreshed [{t}]"))
            {
                foreach (Cell c in cells)
                {
                    if (!c.HasFlags(MovementFlag.Walk))
                        continue;

                    HashSet<Cell> merged_cells = new HashSet<Cell>{c};

                    // find all patches, this cell neighbours belongs to
                    foreach (var neighbour in c.Neighbours)
                    {
                        CellsPatch connected_patch = m_CellsPatches.FirstOrDefault(x => x.Cells.Contains(neighbour.cell));

                        if (connected_patch != null)
                        {
                            merged_cells.UnionWith(connected_patch.Cells);
                            m_CellsPatches.Remove(connected_patch);
                        }
                    }

                    m_CellsPatches.Add(new CellsPatch(merged_cells, c.Flags));                
                }
            }
        }

        private const int MAX_CELLS_CACHE_SIZE = 6;

        private static ReaderWriterLockSlim CellsCacheLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        internal static List<Cell> m_CellsCache = new List<Cell>();

        // Returns true when position is on navmesh. Aquires DataLock (read)
        internal bool GetCellContaining(Vec3 p, out Cell c, MovementFlag flags = MovementFlag.Walk, bool allow_disabled = false, bool nearest = false, float nearest_tolerance = -1, bool test_2d = false, float z_tolerance = 0, HashSet<Cell> exclude_cells = null)
        {
            using (new ReadLock(DataLock))
            {
                c = null;

                if (p.IsEmpty)
                    return false;

                // check cache first
                using (new ReadLock(CellsCacheLock))
                {
                    foreach (Cell cell in m_CellsCache.FindAll(x => x.HasFlags(flags)))
                    {
                        if ((allow_disabled || !cell.Disabled) && (exclude_cells == null || !exclude_cells.Contains(cell)) && (test_2d ? cell.Contains2D(p) : cell.Contains(p, z_tolerance)))
                        {
                            c = cell;
                            return true;
                        }
                    }
                }

                float min_dist = float.MaxValue;

                foreach (GridCell grid_cell in m_GridCells)
                {
                    List<Cell> cells = grid_cell.Cells.FindAll(x => (allow_disabled || !x.Disabled) && x.HasFlags(flags));

                    if (grid_cell.Contains2D(p))
                    {
                        foreach (Cell cell in cells)
                        {
                            if ((exclude_cells == null || !exclude_cells.Contains(cell)) && (test_2d ? cell.Contains2D(p) : cell.Contains(p, z_tolerance)))
                            {
                                c = cell;

                                using (new WriteLock(CellsCacheLock))
                                {
                                    m_CellsCache.Add(cell);

                                    if (m_CellsCache.Count > MAX_CELLS_CACHE_SIZE)
                                        m_CellsCache.RemoveAt(0);
                                }

                                return true;
                            }
                        }
                    }

                    if (nearest)
                    {
                        foreach (Cell cell in cells)
                        {
                            if (exclude_cells != null && exclude_cells.Contains(cell))
                                continue;

                            float dist = cell.Distance(p);

                            if ((nearest_tolerance < 0 || dist <= nearest_tolerance) && dist < min_dist)
                            {
                                min_dist = dist;
                                c = cell;
                            }
                        }
                    }
                }

                return false;
            }
        }

        public int GetGridCellId(Vec3 pos)
        {
            using (new ReadLock(DataLock))
            {
                GridCell grid_cell = GetGridCell(pos);

                return grid_cell != null ? grid_cell.Id : -1;
            }
        }

        // Assume DataLock (read)
        internal GridCell GetGridCell(Vec3 pos)
        {
            List<GridCell> grid_cells = GetGridCellsContaining(pos);

            if (grid_cells.Count > 0)
            {
                float min_grid_cell_area = grid_cells.Min(x => x.AABB.Area);
                GridCell smallest_grid_cell = grid_cells.Find(x => x.AABB.Area.Equals(min_grid_cell_area));

                return smallest_grid_cell;
            }

            return null;
        }

        public HashSet<region_data> Regions
        {
            get
            {
                using (new ReadLock(InputLock))
                    return new HashSet<region_data>(m_Regions);
            }

            set
            {
                using (new WriteLock(InputLock))
                {
                    if (!value.SetEquals(m_Regions))
                        m_Regions = value;
                }
            }
        }

        public bool RegionsEnabled
        {
            get;
            set;
        }

        public enum RegionsMode
        {
            Mult,
            Max,
        }

        public RegionsMode RegionsMoveCostMode
        {
            get;
            set;
        }

        private class overlapped_cell_data
        {
            public overlapped_cell_data(Cell replaced_cell) { this.replaced_cell = replaced_cell; }
            public overlapped_cell_data(List<Cell> all_cells, BinaryReader r) { Deserialize(all_cells, r); }

            public Cell replaced_cell;
            public List<Cell> replacement_cells = new List<Cell>();
            public HashSet<region_data> last_areas = new HashSet<region_data>();
            public HashSet<region_data> areas = new HashSet<region_data>();

            public void Serialize(BinaryWriter w)
            {
                w.Write(replaced_cell.GlobalId);

                w.Write(replacement_cells.Count);
                foreach (Cell cell in replacement_cells)
                    w.Write(cell.GlobalId);

                w.Write(last_areas.Count);
                foreach (region_data region in last_areas)
                    region.Serialize(w);

                w.Write(areas.Count);
                foreach (region_data region in areas)
                    region.Serialize(w);
            }

            public void Deserialize(List<Cell> all_cells, BinaryReader r)
            {
                int replaced_cell_global_id = r.ReadInt32();
                replaced_cell = all_cells.Find(x => x.GlobalId == replaced_cell_global_id);

                int replacement_cells_count = r.ReadInt32();
                for (int i = 0; i < replacement_cells_count; ++i)
                {
                    int replacement_cell_global_id = r.ReadInt32();
                    replacement_cells.Add(all_cells.Find(x => x.GlobalId == replacement_cell_global_id));
                }

                int last_areas_count = r.ReadInt32();
                for (int i = 0; i < last_areas_count; ++i)
                    last_areas.Add(new region_data(r));

                int areas_count = r.ReadInt32();
                for (int i = 0; i < areas_count; ++i)
                    areas.Add(new region_data(r));
            }
        }

        private Dictionary<int, overlapped_cell_data> m_CellsOverlappedByRegions = new Dictionary<int, overlapped_cell_data>(); // @ DataLock

        private void Updates()
        {
            Stopwatch timer = new Stopwatch();
            timer.Start();

            while (!m_ShouldStopUpdates)
            {
                OnUpdate(timer.ElapsedMilliseconds);
                Thread.Sleep(50);
            }
        }

        // Controls updated thread execution
        private volatile bool m_ShouldStopUpdates = false;

        protected virtual void OnUpdate(Int64 time)
        {
            if (time - m_LastUpdateRegionsTime > 250)
            {
                UpdateRegions();
                m_LastUpdateRegionsTime = time;
            }
        }

        private Int64 m_LastUpdateRegionsTime = 0;

        private void UpdateRegions()
        {
            // copy current regions to avoid acquiring lock later
            HashSet<region_data> regions_copy = Regions;

            using (new WriteLock(DataLock))
            {
                foreach (var data in m_CellsOverlappedByRegions)
                    data.Value.areas.Clear();
            
                if (RegionsEnabled)
                {
                    // update cells overlapped by avoid areas
                    foreach (region_data region in regions_copy)
                    {
                        List<GridCell> g_cells = m_GridCells.FindAll(x => x.AABB.Overlaps2D(region.area));
                        List<Cell> cells = new List<Cell>();

                        // do not ignore disabled cells on purpose so we don't have to iterate separately over
                        foreach (GridCell g_cell in g_cells)
                            cells.AddRange(g_cell.Cells.FindAll(x => !x.Replacement && x.HasFlags(MovementFlag.Walk) && x.AABB.Overlaps2D(region.area)));

                        foreach (Cell cell in cells)
                        {
                            overlapped_cell_data data = null;
                            if (!m_CellsOverlappedByRegions.TryGetValue(cell.GlobalId, out data))
                                m_CellsOverlappedByRegions[cell.GlobalId] = data = new overlapped_cell_data(cell);

                            data.areas.Add(new region_data(region));
                        }
                    }
                }

                List<int> inactive_keys = new List<int>();
                bool anything_changed = false;

                // perform rectangulation
                foreach (var item in m_CellsOverlappedByRegions)
                {
                    // no longer overlapped
                    if (item.Value.areas.Count == 0)
                        inactive_keys.Add(item.Key);

                    overlapped_cell_data data = item.Value;

                    if (!data.areas.SetEquals(data.last_areas))
                    {
                        anything_changed = true;

                        //using (new Profiler("[Nav] Avoid areas updated [{t}]"))
                        {
                            GridCell parent_grid_cell = GetGridCell(data.replaced_cell.Center);

                            // grid cell containing this replacement has been removed
                            if (parent_grid_cell == null)
                                continue;

                            List<Cell> potential_neighbors = data.replaced_cell.Neighbours.Select(x => x.cell).ToList();

                            {
                                // disable original cell
                                if (data.last_areas.Count == 0)
                                    data.replaced_cell.Disabled = true;

                                // remove previous replacement cells
                                foreach (Cell replacement_cell in data.replacement_cells)
                                {
                                    parent_grid_cell.Remove(replacement_cell);
                                    m_AllCells.RemoveAll(x => x.GlobalId == replacement_cell.GlobalId);
                                }

                                data.replacement_cells.Clear();
                            }

                            if (data.areas.Count > 0)
                            {
                                // generate new replacement cells
                                data.replacement_cells.Add(data.replaced_cell);

                                foreach (region_data region in data.areas)
                                {
                                    List<Cell> cells_to_check = new List<Cell>(data.replacement_cells);

                                    foreach (Cell c in cells_to_check)
                                    {
                                        AABB[] extracted = c.AABB.Extract2D(region.area);

                                        if (extracted != null)
                                        {
                                            data.replacement_cells.Remove(c);

                                            for (int k = 0; k < extracted.Length; ++k)
                                            {
                                                // in case of negative movement cost treat this region as dynamic obstacle
                                                if (region.move_cost_mult < 0 && k == 0)
                                                    continue;

                                                float movement_cost_mult = c.MovementCostMult;

                                                // first cell is always the one overlapped by region
                                                if (k == 0)
                                                {
                                                    if (RegionsMoveCostMode == RegionsMode.Mult)
                                                        movement_cost_mult *= region.move_cost_mult;
                                                    else if (RegionsMoveCostMode == RegionsMode.Max)
                                                    {
                                                        if (movement_cost_mult < 1 && region.move_cost_mult < 1)
                                                            movement_cost_mult = Math.Min(region.move_cost_mult, movement_cost_mult);
                                                        else
                                                            movement_cost_mult = Math.Max(region.move_cost_mult, movement_cost_mult);
                                                    }
                                                }

                                                Cell r_cell = new Cell(extracted[k], data.replaced_cell.Flags, movement_cost_mult);
                                                r_cell.Replacement = true;
                                                data.replacement_cells.Add(r_cell);
                                            }
                                        }
                                    }
                                }

                                // try to connect new replacement cells with potential neighbors
                                foreach (Cell replacement_cell in data.replacement_cells)
                                {
                                    Vec3 border_point = null;

                                    foreach (Cell potential_neighbor in potential_neighbors)
                                        replacement_cell.AddNeighbour(potential_neighbor, out border_point);

                                    parent_grid_cell.Cells.Add(replacement_cell);
                                    m_AllCells.Add(replacement_cell);

                                    // this cell is now potential neighbor as well, because can be interconnected with other replacement cells!
                                    potential_neighbors.Add(replacement_cell);
                                }
                            }

                            item.Value.last_areas = new HashSet<region_data>(item.Value.areas);

                            // enable original cell when no longer overlapped
                            if (data.areas.Count == 0)
                                data.replaced_cell.Disabled = false;
                        }

                        m_CellsCache.Clear();
                    }
                }

                if (anything_changed)
                {
                    NotifyOnNavDataChanged();
                }

                // remove inactive data
                foreach (int key in inactive_keys)
                    m_CellsOverlappedByRegions.Remove(key);
            }
        }

        public bool IsNavDataAvailable
        {
            get
            {
                using (new ReadLock(DataLock))
                    return m_GridCells.Count > 0;
            }
        }

        public virtual void Clear()
        {
            using (new WriteLock(DataLock))
            using (new WriteLock(InputLock))
            {
                m_CellsCache.Clear();
                m_AllCells.Clear();
                m_GridCells.Clear();
                m_Regions.Clear();
                m_CellsOverlappedByRegions.Clear();
                m_CellsPatches.Clear();
                CellsPatch.LastCellsPatchGlobalId = 0;
                Cell.LastCellGlobalId = 0;
                GridCell.LastGridCellGlobalId = 0;

                m_LastGridCellId = 0;
                m_LastCellId = 0;

                foreach (INavmeshObserver observer in m_Observers)
                    observer.OnNavDataCleared();
            }

            Log("[Nav] Navmesh cleared!");
        }

        public bool Load(string filename, bool clear = true, bool force_update_align_plane = false)
        {
            if (filename == null)
                return false;

            //using (new Profiler("[Nav] Loaded nav data [{t}]"))
            {
                if (clear)
                    Clear();

                try
                {
                    StreamReader stream = new StreamReader(filename);

                    if (stream != null)
                    {
                        string line;
                        GridCell g_cell = null;
                        Vec3 cell_shrink_size = Vec3.Empty;
                        HashSet<region_data> avoid_areas = new HashSet<region_data>();

                        CultureInfo inv = (CultureInfo)CultureInfo.CurrentCulture.Clone();
                        inv.NumberFormat.CurrencyDecimalSeparator = ",";
                        inv.NumberFormat.NumberDecimalSeparator = ",";

                        while ((line = stream.ReadLine()) != null)
                        {
                            string[] data = line.Split(' ');

                            if (data[0] == "g")
                            {
                                // add previous grid cell
                                Add(g_cell, false);

                                m_LastCellId = 0;

                                g_cell = new GridCell(float.Parse(data[1], inv), float.Parse(data[2], inv), float.Parse(data[3], inv), float.Parse(data[4], inv), float.Parse(data[5], inv), float.Parse(data[6], inv), (data.Length > 7 ? int.Parse(data[7]) : m_LastGridCellId++));
                            }
                            else if (data[0] == "n")
                            {
                                MovementFlag flags = MovementFlag.Walk | MovementFlag.Fly;

                                if (data.Length > 7)
                                    flags = (MovementFlag)int.Parse(data[7]);

                                Cell n_cell = new Cell(new Vec3(float.Parse(data[1], inv), float.Parse(data[2], inv), float.Parse(data[3], inv)) - cell_shrink_size, new Vec3(float.Parse(data[4], inv), float.Parse(data[5], inv), float.Parse(data[6], inv)) - cell_shrink_size, flags, m_LastCellId++);
                                g_cell.Add(n_cell);
                            }
                            else if (data[0] == "r")
                            {
                                avoid_areas.Add(new region_data(new AABB(float.Parse(data[1], inv), float.Parse(data[2], inv), float.Parse(data[3], inv), float.Parse(data[4], inv), float.Parse(data[5], inv), float.Parse(data[6], inv)), float.Parse(data[7], inv)));
                            }
                        }

                        // add last grid cell
                        Add(g_cell, false);

                        if (force_update_align_plane)
                        {
                            using (new ReadLock(DataLock))
                            {
                                foreach (Cell c in m_AllCells)
                                    c.UpdateAlignPlane();
                            }
                        }

                        Random rng = new Random();
                        GridCell random_grid_cell = null;

                        random_grid_cell = m_GridCells[rng.Next(m_GridCells.Count)];

                        NotifyOnNavDataChanged();
                        
                        Regions = avoid_areas;
                        
                        Log("[Nav] Navmesh loaded.");

                        stream.Close();

                        return true;
                    }
                }
                catch (Exception)
                {
                }

                Log("[Nav] Navmesh load failed!");

                return false;
            }
        }

        public Vec3 GetRandomPos()
        {
            using (new ReadLock(DataLock))
            {
                GridCell g_cell = m_GridCells[Rng.Next(m_GridCells.Count)];
                List<Cell> enabled_cells = g_cell.Cells.FindAll(x => !x.Disabled);
                return enabled_cells[Rng.Next(enabled_cells.Count)].AABB.GetRandomPos();
            }
        }

        public Vec3 GetCenter()
        {
            using (new ReadLock(DataLock))
            {
                Vec3 avg_pos = Vec3.Empty;

                foreach (GridCell g_cell in m_GridCells)
                    avg_pos += g_cell.AABB.Center;

                return avg_pos / (float)m_GridCells.Count;
            }
        }

        public List<Cell> GetCellsInside(AABB area)
        {
            List<Cell> result = new List<Cell>();

            using (new ReadLock(DataLock))
            {
                List<GridCell> g_cells = m_GridCells.FindAll(x => x.AABB.Overlaps2D(area));
                List<Cell> cells = new List<Cell>();

                // do not ignore disabled cells on purpose so we don't have to iterate separately over
                foreach (GridCell g_cell in g_cells)
                    cells.AddRange(g_cell.Cells.FindAll(x => !x.Replacement && x.HasFlags(MovementFlag.Walk) && x.AABB.Overlaps2D(area)));

                foreach (var cell in cells)
                    result.Add(cell.CreateSimplifiedClone());
            }

            return result;
        }

        public bool RayTrace(Vec3 from, Vec3 to, MovementFlag flags, out Vec3 intersection)
        {
            using (new ReadLock(DataLock))
            {
                intersection = Vec3.Empty;

                Vec3 ray_dir = to - from;
                ray_dir.Normalize();

                foreach (Cell c in m_AllCells)
                {
                    if (!c.HasFlags(flags))
                        continue;

                    Vec3 new_intersection = null;
                    if (c.AABB.RayTest(from, ray_dir, out new_intersection) && from.DistanceSqr(new_intersection) < from.DistanceSqr(intersection))
                        intersection = new_intersection;
                }

                return !intersection.IsEmpty;
            }
        }

        public bool RayCast(Vec3 from, Vec3 to, MovementFlag flags, bool ignore_movement_cost = true)
        {
            HashSet<Cell> ignored_cells = new HashSet<Cell>();
            return RayCast(from, null, to, flags, false, ignore_movement_cost, ref ignored_cells);
        }

        public bool RayCast2D(Vec3 from, Vec3 to, MovementFlag flags, bool ignore_movement_cost = true)
        {
            HashSet<Cell> ignored_cells = new HashSet<Cell>();
            return RayCast(from, null, to, flags, true, ignore_movement_cost, ref ignored_cells);
        }

        // Aquires DataLock (read)
        private bool RayCast(Vec3 from, Cell from_cell, Vec3 to, MovementFlag flags, bool test_2d, bool ignore_movement_cost, ref HashSet<Cell> ignored_cells)
        {
            using (new ReadLock(DataLock))
            {
                if (to.IsEmpty)
                    return false;

                if (from_cell == null && !GetCellContaining(from, out from_cell, flags, false, false, -1, test_2d, 2, ignored_cells))
                    return false;

                if (test_2d ? from_cell.Contains2D(to) : from_cell.Contains(to, 2))
                    return true;

                ignored_cells.Add(from_cell);

                Vec3 ray_dir = to - from;
                if (test_2d)
                    ray_dir.Normalize2D();
                else
                    ray_dir.Normalize();

                Vec3 ray_origin = new Vec3(from);

                // raycast through neighbours
                foreach (Cell.Neighbour neighbour in from_cell.Neighbours)
                {
                    if (neighbour.cell.Disabled || (neighbour.connection_flags & flags) != flags || (!ignore_movement_cost && neighbour.cell.MovementCostMult > from_cell.MovementCostMult))
                        continue;

                    Cell neighbour_cell = neighbour.cell;

                    if (ignored_cells.Contains(neighbour_cell))
                        continue;

                    Vec3 intersection = null;

                    bool ray_test_result = test_2d ? neighbour_cell.AABB.RayTest2D(ray_origin, ray_dir, out intersection) :
                                                     neighbour_cell.AABB.RayTest(ray_origin, ray_dir, out intersection);

                    // ray intersects on connection plane
                    if (ray_test_result)
                    {
                        AABB shared_aabb = test_2d ? from_cell.AABB.Intersect2D(neighbour_cell.AABB, true) :
                                                     from_cell.AABB.Intersect(neighbour_cell.AABB, true);

                        if (shared_aabb != null)
                        {
                            bool accepted = test_2d ? shared_aabb.Contains2D(intersection) :
                                                      shared_aabb.Contains(intersection);

                            if (accepted && RayCast(intersection, neighbour_cell, to, flags, test_2d, ignore_movement_cost, ref ignored_cells))
                                return true;
                        }
                    }
                }

                return false;
            }
        }

        public bool AreConnected(Vec3 pos1, Vec3 pos2, MovementFlag flags, float nearest_tolerance)
        {
            using (new ReadLock(DataLock))
            {
                Cell pos1_cell = null;
                Cell pos2_cell = null;

                // do not ignore disabled as cells patches to not include replacement cells!
                GetCellContaining(pos1, out pos1_cell, flags, true, true, nearest_tolerance, false, 2);

                if (pos1_cell == null)
                    return false;

                // do not ignore disabled as cells patches to not include replacement cells!
                GetCellContaining(pos2, out pos2_cell, flags, true, true, nearest_tolerance, false, 2);

                if (pos2_cell == null)
                    return false;

                IEnumerable<CellsPatch> pos1_patches = m_CellsPatches.Where(x => x.Cells.Contains(pos1_cell));

                foreach (var p in pos1_patches)
                {
                    if (p.Cells.Contains(pos2_cell))
                        return true;
                }

                return false;
            }
        }

        // Operations on this list are not thread safe! Use AquireReadDataLock to make it thread safe
        public List<GridCell> dbg_GetGridCells()
        {
            return m_GridCells;
        }

        public void dbg_GenerateRandomAvoidAreas(float move_cost_mult)
        {
            Random rng = new Random();
            HashSet<region_data> regions = new HashSet<region_data>();

            using (new ReadLock(DataLock))
            {
                for (int i = 0; i < 20; ++i)
                {
                    Vec3 pos = GetRandomPos();
                    float size = 20 + (float)rng.NextDouble() * 10;
                    regions.Add(new region_data(new AABB(pos - new Vec3(size * 0.5f, size * 0.5f, 0), pos + new Vec3(size * 0.5f, size * 0.5f, 0)), move_cost_mult));
                }
            }

            Regions = regions;
        }

        // Aquire DataLock (read)
        public List<GridCell> GetGridCellsContaining(Vec3 pos, bool check_2d = true)
        {
            using (new ReadLock(DataLock))
                return check_2d ? m_GridCells.FindAll(x => x.Contains2D(pos)) : m_GridCells.FindAll(x => x.Contains(pos));
        }

        public Vec3 GetRandomPosInRing(Vec3 ring_center, float min_radius, float max_radius)
        {
            List<GridCell> potential_grid_cells = GetCellsOverlappedByCircle(m_GridCells, ring_center, max_radius);

            List<Cell> cells = new List<Cell>();
            foreach (GridCell grid_cell in potential_grid_cells)
                cells.AddRange(grid_cell.Cells.FindAll(x => x.Overlaps(ring_center, max_radius) && !x.AABB.Inside(ring_center, min_radius)));

            Cell random_cell = cells[Rng.Next(cells.Count)];
            Vec3 random_pos = random_cell.AABB.GetRandomPos();

            float dist = random_pos.Distance(ring_center);
            if (dist > min_radius && dist <= max_radius)
                return random_pos;

            //align position to ring
            Vec3 dir_to_random_pos = random_pos - ring_center;
            dir_to_random_pos.Normalize();

            random_pos = ring_center + dir_to_random_pos * (min_radius + (float)Rng.NextDouble() * (max_radius - min_radius));
            return random_cell.AABB.Align(random_pos);
        }

        public void Dump(string filename)
        {
            System.IO.StreamWriter file = new System.IO.StreamWriter(filename);

            using (new ReadLock(DataLock))
            using (new ReadLock(InputLock))
            {
                foreach (GridCell grid_cell in m_GridCells)
                {
                    file.WriteLine("g {0} {1} {2} {3} {4} {5} {6}", grid_cell.Min.X, grid_cell.Min.Y, grid_cell.Min.Z, grid_cell.Max.X, grid_cell.Max.Y, grid_cell.Max.Z, grid_cell.Id);

                    foreach (Cell cell in grid_cell.Cells)
                        file.WriteLine("n {0} {1} {2} {3} {4} {5} {6}", cell.Min.X, cell.Min.Y, cell.Min.Z, cell.Max.X, cell.Max.Y, cell.Max.Z, (int)cell.Flags);
                }
            
                foreach (region_data region in Regions)
                    file.WriteLine("r {0} {1} {2} {3} {4} {5} {6}", region.area.Min.X, region.area.Min.Y, region.area.Min.Z, region.area.Max.X, region.area.Max.Y, region.area.Max.Z, region.move_cost_mult);
            }

            file.Close();

            Log("[Nav] Navmesh dumped.");
        }

        // Extension will be automatically added
        public void Serialize(string name)
        {
            using (FileStream fs = File.OpenWrite(name + ".navmesh"))
            using (BinaryWriter w = new BinaryWriter(fs))
            {
                OnSerialize(w);
            }
        }

        protected virtual void OnSerialize(BinaryWriter w)
        {
            using (new ReadLock(DataLock))
            using (new ReadLock(InputLock))
            {
                // write all cells global IDs
                w.Write(m_AllCells.Count);
                foreach (Cell cell in m_AllCells)
                    w.Write(cell.GlobalId);

                foreach (Cell cell in m_AllCells)
                {
                    cell.Serialize(w);

                    foreach (var neighbour in cell.Neighbours)
                    {
                        if (m_AllCells.Find(x => x.GlobalId == neighbour.cell.GlobalId) == null)
                            Log("[Nav] Cell neighbour not on all cells list! Cell info [" +  neighbour.cell.ToString() + "]", true);
                    }
                }

                w.Write(Cell.LastCellGlobalId);

                // write all grid cells global IDs
                w.Write(m_GridCells.Count);
                foreach (GridCell grid_cell in m_GridCells)
                    w.Write(grid_cell.GlobalId);

                foreach (GridCell grid_cell in m_GridCells)
                    grid_cell.Serialize(w);

                w.Write(GridCell.LastGridCellGlobalId);

                // write all patches global IDs
                w.Write(m_CellsPatches.Count);
                foreach (CellsPatch patch in m_CellsPatches)
                    w.Write(patch.GlobalId);

                foreach (CellsPatch patch in m_CellsPatches)
                    patch.Serialize(w);

                w.Write(CellsPatch.LastCellsPatchGlobalId);

                w.Write(m_Regions.Count);
                foreach (region_data region in m_Regions)
                    region.Serialize(w);

                w.Write(m_CellsOverlappedByRegions.Count);
                foreach (var entry in m_CellsOverlappedByRegions)
                {
                    w.Write(entry.Key);
                    entry.Value.Serialize(w);
                }
            }

            Log("[Nav] Navmesh serialized.");
        }

        // Extension will be automatically added
        public void Deserialize(string name)
        {
            using (FileStream fs = File.OpenRead(name + ".navmesh"))
            using (BinaryReader r = new BinaryReader(fs))
            {
                OnDeserialize(r);
            }
        }

        protected virtual void OnDeserialize(BinaryReader r)
        {
            using (new WriteLock(DataLock))
            using (new WriteLock(InputLock))
            {
                //using (new Profiler("Navmesh deserialization took {t}"))
                {
                    m_AllCells.Clear();
                    m_GridCells.Clear();
                    m_Regions.Clear();
                    m_CellsOverlappedByRegions.Clear();

                    Cell.CompareByGlobalId comp_by_global_id = new Cell.CompareByGlobalId();

                    int all_cells_count = r.ReadInt32();

                    // pre-allocate cells
                    for (int i = 0; i < all_cells_count; ++i)
                    {
                        Cell cell = new Cell(0, 0, 0, 0, 0, 0, MovementFlag.None);
                        cell.GlobalId = r.ReadInt32();
                        m_AllCells.Add(cell);
                    }

                    m_AllCells.Sort(comp_by_global_id);

                    foreach (Cell cell in m_AllCells)
                        cell.Deserialize(m_AllCells, r);

                    Cell.LastCellGlobalId = r.ReadInt32();

                    int grid_cells_count = r.ReadInt32();

                    // pre-allocate grid cells
                    for (int i = 0; i < grid_cells_count; ++i)
                    {
                        GridCell grid_cell = new GridCell(0, 0, 0, 0, 0, 0);
                        grid_cell.GlobalId = r.ReadInt32();
                        m_GridCells.Add(grid_cell);
                    }

                    m_GridCells.Sort(comp_by_global_id);

                    foreach (GridCell grid_cell in m_GridCells)
                        grid_cell.Deserialize(m_GridCells, m_AllCells, r);

                    GridCell.LastGridCellGlobalId = r.ReadInt32();

                    List<CellsPatch> patches = new List<CellsPatch>();
                    int patches_count = r.ReadInt32();

                    // pre-allocate patches
                    for (int i = 0; i < patches_count; ++i)
                    {
                        CellsPatch patch = new CellsPatch(new HashSet<Cell>(), MovementFlag.None);
                        patch.GlobalId = r.ReadInt32();
                        patches.Add(patch);
                    }

                    foreach (CellsPatch patch in patches)
                        patch.Deserialize(m_AllCells, r);

                    m_CellsPatches = new HashSet<CellsPatch>(patches);

                    CellsPatch.LastCellsPatchGlobalId = r.ReadInt32();

                    int regions_count = r.ReadInt32();
                    for (int i = 0; i < regions_count; ++i)
                        m_Regions.Add(new region_data(r));

                    int cells_overlapped_by_regions_count = r.ReadInt32();
                    for (int i = 0; i < cells_overlapped_by_regions_count; ++i)
                    {
                        int key = r.ReadInt32();
                        m_CellsOverlappedByRegions.Add(key, new overlapped_cell_data(m_AllCells, r));
                    }
                }
            }

            Log("[Nav] Navmesh deserialized.");
        }

        public virtual void Dispose()
        {
            m_ShouldStopUpdates = true;
            UpdatesThread.Join();
        }

        protected internal void Log(string msg, bool force = false)
        {
            if (Verbose || force)
            {
                DateTime d = DateTime.Now;
                Trace.WriteLine(d.Hour.ToString().PadLeft(2, '0') + ":" + d.Minute.ToString().PadLeft(2, '0') + ":" + d.Second.ToString().PadLeft(2, '0') + ":" + d.Millisecond.ToString().PadLeft(3, '0') + " " + msg);
            }
        }

        internal T GetNearestCell<T>(List<T> cells, Vec3 p) where T : Cell
        {
            float min_dist = float.MaxValue;
            T nearest_cell = null;

            foreach (T cell in cells)
            {
                if (cell.Disabled)
                    continue;

                float dist = cell.Distance(p);

                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest_cell = cell;
                }
            }

            return nearest_cell;
        }

        internal List<T> GetCellsOverlappedByCircle<T>(List<T> cells, Vec3 circle_center, float radius) where T : Cell
        {
            return cells.FindAll(x => x.Overlaps(circle_center, radius));
        }

        internal List<T> GetCellsInsideCircle<T>(List<T> cells, Vec3 circle_center, float radius) where T : Cell
        {
            return cells.FindAll(x => x.AABB.Inside(circle_center, radius));
        }

        public void AddObserver(INavmeshObserver observer)
        {
            using (new WriteLock(InputLock))
            {
                if (m_Observers.Contains(observer))
                    return;

                m_Observers.Add(observer);
            }
        }

        public void RemoveObserver(INavmeshObserver observer)
        {
            using (new WriteLock(InputLock))
                m_Observers.Remove(observer);
        }

        protected void NotifyOnNavDataChanged()
        {
            List<INavmeshObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavmeshObserver observer in observers_copy)
                observer.OnNavDataChanged();
        }

        protected void NotifyOnGridCellAdded(GridCell g_cell)
        {
            List<INavmeshObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavmeshObserver observer in observers_copy)
                observer.OnGridCellAdded(g_cell);
        }

        private int m_LastGridCellId = 0;
        private int m_LastCellId = 0;
        private Random Rng = new Random();

        private Thread UpdatesThread = null;

        private ReaderWriterLockSlim DataLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private ReaderWriterLockSlim InputLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);

        public ReadLock AcquireReadDataLock() { return new ReadLock(DataLock); }
        
        internal List<Cell> m_AllCells = new List<Cell>(); //@ DataLock
        private HashSet<CellsPatch> m_CellsPatches = new HashSet<CellsPatch>(); //@ DataLock
        internal List<GridCell> m_GridCells = new List<GridCell>(); //@ DataLock
        private HashSet<region_data> m_Regions = new HashSet<region_data>(); //@ InputLock
        private List<INavmeshObserver> m_Observers = new List<INavmeshObserver>(); //@ InputLock
    }
}
