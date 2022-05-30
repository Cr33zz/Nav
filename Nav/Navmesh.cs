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
    public struct Region : IEquatable<Region>
    {
        public Region(AABB area, float move_cost_mult, float threat = 0)
        {
            Area = area;
            MoveCostMult = move_cost_mult;
            Threat = threat;
        }

        public Region(Region region)
        {
            Area = new AABB(region.Area);
            MoveCostMult = region.MoveCostMult;
            Threat = region.Threat;
        }

        public Region(BinaryReader r) :  this()
        {
            Deserialize(r);
        }

        public AABB Area;
        // movement cost multiplier is used by navigation part of the system
        public float MoveCostMult;
        // threat level is used by avoidance to find safe spot
        public float Threat;

        public bool IsNavBlocker => MoveCostMult < 0;

        public override bool Equals(Object obj)
        {
            if (obj == null)
                return false;

            return (obj is Region) && Equals((Region)obj);
        }

        public bool Equals(Region region)
        {
            return Threat == region.Threat && MoveCostMult == region.MoveCostMult && Area.Equals(region.Area);
        }

        public override int GetHashCode()
        {
            return Area.GetHashCode() ^ Threat.GetHashCode() ^ MoveCostMult.GetHashCode();
        }

        public void Serialize(BinaryWriter w)
        {
            Area.Serialize(w);
            w.Write(MoveCostMult);
            w.Write(Threat);
        }

        public void Deserialize(BinaryReader r)
        {
            Area = new AABB(r);
            MoveCostMult = r.ReadSingle();
            Threat = r.ReadSingle();
        }
    }

    public struct RayCastResult
    {
        public bool Successful;
        public Vec3 End;
        public Cell EndCell;

        public static implicit operator bool(RayCastResult value)
        {
            return value.Successful;
        }
    }

    public class Navmesh : IDisposable
    {
        public Navmesh(bool verbose = false)
        {
            // quick tests
            //test_RayTrace();

            Log("[Nav] Creating navmesh...");

            Verbose = verbose;

            Init();
            LocksState.Reset();

            UpdatesThread = new Thread(Updates);
            UpdatesThread.Name = "Navmesh-UpdatesThread";
            UpdatesThread.Start();

            Log("[Nav] Navmesh created");
        }

        protected virtual void Init()
        {
            RegionsMoveCostMode = RegionsMode.Add;
            RegionsEnabled = true;
        }

        // when true will print additional data to output
        public bool Verbose { get; set; }

        public bool Add(GridCell g_cell, bool trigger_nav_data_change)
        {
            if (g_cell == null || g_cell.GetCellsCount() == 0)
                return false;

            List<Cell> incoming_cells = null;

            using (new WriteLock(DataLock, "DataLock - Navmesh.Add - add to grid cells"))
            {
                incoming_cells = g_cell.GetCells(false);
                
                // check if same grid is not already defined (merge then)
                GridCell base_grid_cell = m_GridCells.FirstOrDefault(x => x.AABB.Equals(g_cell.AABB));

                if (base_grid_cell != null)
                {
                    base_grid_cell.Add(incoming_cells);
                    g_cell = base_grid_cell;
                    //Log("[Nav] Grid cell (" + g_cell.Id + " " + g_cell.Min + ") with " + g_cell.GetCellsCount() + " cell(s) merged with grid cell (" + base_grid_cell.Id + " " + base_grid_cell.Min + ")");
                }
                //else
                //    Log("[Nav] Grid cell (" + g_cell.Id + " " + g_cell.Min + ") with " + g_cell.GetCellsCount() + " cell(s) added");

                m_GridCells.Add(g_cell);
            }
            
            var all_cells_neighbours = new List<(Cell, Cell, Vec3)>();
            var grid_cell_neighbours = new List<(GridCell, MovementFlag)>();

            using (new ReadLock(DataLock))
            {
                foreach (GridCell grid_cell in m_GridCells)
                {
                    var cells_neighbours = grid_cell.CheckNeighbour(g_cell, out var movement_flags);

                    if (cells_neighbours.Any())
                    {
                        all_cells_neighbours.AddRange(cells_neighbours);
                        grid_cell_neighbours.Add((grid_cell, movement_flags));
                    }
                }
            }

            using (new WriteLock(DataLock, "DataLock - Navmesh.Add - add to all cells"))
            {
                foreach (var grid_cell_neighbour in grid_cell_neighbours)
                {
                    var n1 = g_cell.Neighbours.FirstOrDefault(x => x.cell.GlobalId == grid_cell_neighbour.Item1.GlobalId);

                    // if they were not connected before, simply connect them
                    if (n1 == null)
                    {
                        float distance = g_cell.Center.Distance2D(grid_cell_neighbour.Item1.Center);
                        g_cell.Neighbours.Add(new GridCell.Neighbour(grid_cell_neighbour.Item1, Vec3.ZERO, grid_cell_neighbour.Item2, distance));
                        grid_cell_neighbour.Item1.Neighbours.Add(new GridCell.Neighbour(g_cell, Vec3.ZERO, grid_cell_neighbour.Item2, distance));
                    }
                    // otherwise verify connection flags
                    else if (n1.connection_flags < grid_cell_neighbour.Item2)
                    {
                        var n2 = grid_cell_neighbour.Item1.Neighbours.FirstOrDefault(x => x.cell.GlobalId == g_cell.GlobalId);

                        n1.connection_flags = grid_cell_neighbour.Item2;
                        n2.connection_flags = grid_cell_neighbour.Item2;
                    }
                }

                foreach (var cells_neighbours in all_cells_neighbours)
                {
                    cells_neighbours.Item1.MakeNeighbours(cells_neighbours.Item2, cells_neighbours.Item3);
                }

                m_AllCells.UnionWith(incoming_cells);
                Interlocked.Exchange(ref ForcePatchesUpdate , 1);                
            }

            NotifyOnGridCellAdded(g_cell);

            if (trigger_nav_data_change)
                NotifyOnNavDataChanged(g_cell.AABB);

            return true;
        }

        private const int MAX_CELLS_CACHE_SIZE = 6;

        private static ReaderWriterLockSlim CellsCacheLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        internal static List<Cell> m_CellsCache = new List<Cell>();

        // Returns true when position is on navmesh. Acquires DataLock (read)
        public bool GetCellAt(Vec3 p, out Cell result_cell, MovementFlag flags = MovementFlag.Walk, bool allow_disabled = false, bool allow_replacement = true, bool nearest = false, float nearest_tolerance = -1, bool test_2d = true, float z_tolerance = 0, HashSet<Cell> exclude_cells = null)
        {
            using (new ReadLock(DataLock))
            //using (new ReadLock(DataLock, context: "GetCellAt"))
            {
                result_cell = null;

                if (p.IsZero())
                    return false;

                float min_dist = float.MaxValue;

                foreach (GridCell grid_cell in m_GridCells)
                {
                    if (grid_cell.Contains2D(p))
                    {
                        var cells = grid_cell.GetCellsAt(p, test_2d, z_tolerance, allow_replacement);
                        foreach (Cell cell in cells.Where(x => (allow_disabled || !x.Disabled) && (allow_replacement || !x.Replacement) && (exclude_cells == null || !exclude_cells.Contains(x)) && x.HasFlags(flags) && x.MovementCostMult != -1))
                        {
                            result_cell = cell;

                            return true;
                        }
                    }

                    if (nearest)
                    {
                        var cells = grid_cell.GetCells(x => (allow_disabled || !x.Disabled) && (allow_replacement || !x.Replacement) && x.HasFlags(flags) && x.MovementCostMult != -1);

                        foreach (Cell cell in cells)
                        {
                            if (exclude_cells != null && exclude_cells.Contains(cell))
                                continue;

                            float dist = test_2d ? cell.Distance2D(p) : cell.Distance(p);

                            if ((nearest_tolerance < 0 || dist <= nearest_tolerance) && dist < min_dist)
                            {
                                min_dist = dist;
                                result_cell = cell;
                            }
                        }
                    }
                }

                return false;
            }
        }

        internal List<Cell> GetCellsWithin(Vec3 p, float radius, MovementFlag flags, bool allow_disabled = false, bool test_2d = true, float z_tolerance = 0)
        {
            using (new ReadLock(DataLock))
            //using (new ReadLock(DataLock, context: "GetCellsWithin"))
            {
                var result_cells = new List<Cell>();

                if (p.IsZero())
                    return result_cells;

                foreach (GridCell grid_cell in m_GridCells)
                {
                    if (grid_cell.Distance2D(p) > radius)
                        continue;

                    result_cells.AddRange(Algorihms.GetCellsWithin(grid_cell.GetCells(!allow_disabled), p, radius, flags, true, test_2d, z_tolerance));
                }

                // extract replacement cells for disabled cells
                foreach (var cell in result_cells.Where(x => x.Disabled).ToList())
                {
                    if (CellsOverlappedByRegions.TryGetValue(cell.GlobalId, out overlapped_cell_data data))
                        result_cells.AddRange(data.replacement_cells);
                }

                return result_cells.Where(x => allow_disabled || !x.Disabled).ToList();
            }
        }

        public int GetGridCellId(Vec3 pos)
        {
            using (new ReadLock(DataLock))
            //using (new ReadLock(DataLock, context: "GetGridCellId"))
            {
                return GetGridCell(pos)?.Id ?? -1;
            }
        }

        // Assume DataLock (read)
        internal GridCell GetGridCell(Vec3 pos)
        {
            var grid_cells = GetGridCellsContaining(pos);

            if (grid_cells.Count() > 0)
            {
                float min_grid_cell_area = grid_cells.Min(x => x.AABB.Area);
                GridCell smallest_grid_cell = grid_cells.FirstOrDefault(x => x.AABB.Area.Equals(min_grid_cell_area));

                return smallest_grid_cell;
            }

            return null;
        }

        public List<Region> Regions
        {
            get
            {
                using (new ReadLock(InputLock))
                    return new List<Region>(m_Regions);
            }

            set
            {
                using (new WriteLock(InputLock, "InputLock - Navmesh.Regions"))
                {
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
            Add,
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
            public overlapped_cell_data(Cell replaced_cell, GridCell parent_grid_cell) { this.replaced_cell = replaced_cell; this.parent_grid_cell = parent_grid_cell; }
            public overlapped_cell_data(HashSet<GridCell> all_grid_cells, HashSet<Cell> all_cells, BinaryReader r) { Deserialize(all_grid_cells, all_cells, r); }

            public GridCell parent_grid_cell;
            public Cell replaced_cell;
            public List<Cell> replacement_cells = new List<Cell>();
            public HashSet<Region> last_overlapping_regions = new HashSet<Region>();
            public HashSet<Region> overlapping_regions = new HashSet<Region>();

            public void Serialize(BinaryWriter w)
            {
                w.Write(parent_grid_cell.GlobalId);

                w.Write(replaced_cell.GlobalId);

                w.Write(replacement_cells.Count);
                foreach (Cell cell in replacement_cells)
                    w.Write(cell.GlobalId);

                w.Write(last_overlapping_regions.Count);
                foreach (Region region in last_overlapping_regions)
                    region.Serialize(w);

                w.Write(overlapping_regions.Count);
                foreach (Region region in overlapping_regions)
                    region.Serialize(w);
            }

            public void Deserialize(HashSet<GridCell> all_grid_cells, HashSet<Cell> all_cells, BinaryReader r)
            {
                int parent_grid_cell_global_id = r.ReadInt32();
                parent_grid_cell = all_grid_cells.First(x => x.GlobalId == parent_grid_cell_global_id);

                int replaced_cell_global_id = r.ReadInt32();
                replaced_cell = all_cells.First(x => x.GlobalId == replaced_cell_global_id);

                int replacement_cells_count = r.ReadInt32();
                for (int i = 0; i < replacement_cells_count; ++i)
                {
                    int replacement_cell_global_id = r.ReadInt32();
                    replacement_cells.Add(all_cells.First(x => x.GlobalId == replacement_cell_global_id));
                }

                int last_areas_count = r.ReadInt32();
                for (int i = 0; i < last_areas_count; ++i)
                    last_overlapping_regions.Add(new Region(r));

                int areas_count = r.ReadInt32();
                for (int i = 0; i < areas_count; ++i)
                    overlapping_regions.Add(new Region(r));
            }
        }

        internal bool AreNavBlockersPresent() { return LastBlockers.Count > 0; }

        private Dictionary<int, overlapped_cell_data> CellsOverlappedByRegions = new Dictionary<int, overlapped_cell_data>(); // @ DataLock
        private HashSet<AABB> LastBlockers = new HashSet<AABB>(); //@ DataLock

        private void Updates()
        {
            Stopwatch timer = new Stopwatch();
            timer.Start();

            while (!ShouldStopUpdates)
            {
                try
                {
                    OnUpdate(timer.ElapsedMilliseconds);
                    Thread.Sleep(15);
                }
                catch (Exception ex)
                {
                    Trace.WriteLine($"Navmesh exception {ex.Message}\n{ex.StackTrace}");
                }                
            }
        }

        // Controls updated thread execution
        private volatile bool ShouldStopUpdates = false;

        protected virtual void OnUpdate(Int64 time)
        {
            if (time - LastUpdateRegionsTime > UpdateRegionsInterval)
            {
                UpdateRegions();
                LastUpdateRegionsTime = time;
            }

            if (time - LastUpdatePatchesTime > UpdatePatchesInterval)
            {
                if (UpdatePatches())
                    NotifyOnPatchesChanged();
                LastUpdatePatchesTime = time;
            }
        }

        public uint UpdatePatchesInterval { get; set; } = 350;
        private Int64 LastUpdatePatchesTime = 0;

        internal int ForcePatchesUpdate = 0;

        private bool UpdatePatches(bool force = false, bool is_data_locked = false)
        {
            if (Interlocked.CompareExchange(ref ForcePatchesUpdate, 0, 1) == 1 || force)
            {
                //Console.WriteLine("patches update start");
                var cells_patches = new HashSet<CellsPatch>();
                var all_patches_cells = new HashSet<Cell>();
                var all_patches_cells_grid = new Dictionary<AABB, List<Cell>>();

                using (new Profiler($"Updating cell patches (incl. lock) took %t", 50))
                using (is_data_locked ? new EmptyLock() : new ReadLock(DataLock, description: "DataLock - Navmesh.UpdatePatches"))
                using (new Profiler($"Updating cell patches took %t", 50))
                {
                    var cells_copy = new HashSet<Cell>();

                    foreach (var cell in m_AllCells)
                    {
                        if (cell.Disabled || (cell.Flags & MovementFlag.Walk) == 0 || cell.MovementCostMult < 0)
                            continue;

                        cells_copy.Add(cell);
                    }

                    while (cells_copy.Count > 0)
                    {
                        var start_cell = cells_copy.FirstOrDefault();

                        if (start_cell == null)
                            break;

                        var patchVisitor = new Algorihms.PatchVisitor();
                        HashSet<Cell> visited = new HashSet<Cell>();

                        Algorihms.Visit(start_cell, ref visited, MovementFlag.Walk, visit_disabled: false, allowed_cells: cells_copy, visitor: patchVisitor);

                        cells_patches.Add(new CellsPatch(patchVisitor.cells, patchVisitor.cells_grid, MovementFlag.Walk));
                        cells_copy.ExceptWith(patchVisitor.cells);
                        all_patches_cells.UnionWith(patchVisitor.cells);
                    }

                    foreach (var cell in all_patches_cells)
                    {
                        AABB gridAABB = cell.ParentAABB;
                        if (!all_patches_cells_grid.ContainsKey(gridAABB))
                            all_patches_cells_grid.Add(gridAABB, new List<Cell>() { cell });
                        else
                            all_patches_cells_grid[gridAABB].Add(cell);
                    }
                }

                using (new WriteLock(PatchesDataLock, "PatchesDataLock - Navmesh.UpdatePatches"))
                {
                    m_CellsPatches = cells_patches;
                    m_UberCellsPatch = new CellsPatch(all_patches_cells, all_patches_cells_grid, MovementFlag.Walk);
                }

                return true;
            }

            return false;
        }

        public uint UpdateRegionsInterval { get; set; } = 100;

        private Int64 LastUpdateRegionsTime = 0;

        private void UpdateRegions()
        {
            // copy current regions to avoid acquiring lock later
            var regions_copy = Regions;

            AABB affected_area = AABB.ZERO;
            var blockers = new HashSet<AABB>();

            bool force_rebuild = false;
            if (force_rebuild)
            {
                using (new WriteLock(DataLock, "DataLock - Navmesh.UpdateRegions - force rebuild"))
                {
                    foreach (var cell in m_AllCells.Where(x => x.Replacement).ToList())
                    {
                        cell.Detach();
                        m_AllCells.Remove(cell);
                    }

                    foreach (var cell in m_AllCells)
                        cell.Disabled = false;

                    foreach (var g_cell in m_GridCells)
                        g_cell.ResetReplacementCells();

                    CellsOverlappedByRegions.Clear();
                    Interlocked.Exchange(ref ForcePatchesUpdate, 1);
                }
            }

            using (new ReadLock(DataLock, true, "DataLock - Navmesh.Add - update overlapping regions"))
            //using (new Profiler($"update cells overlapped by regions %t"))
            {
                if (CellsOverlappedByRegions.Count > 0)
                {
                    using (new WriteLock(DataLock, "DataLock - Navmesh.Add - clear overlapping regions"))
                    {
                        foreach (var data in CellsOverlappedByRegions)
                            data.Value.overlapping_regions.Clear();
                    }
                }

                if (RegionsEnabled)
                {
                    // update cells overlapped by avoid areas
                    foreach (Region region in regions_copy)
                    {
                        affected_area = affected_area.Extend(region.Area);
                        var grid_cells = m_GridCells.Where(x => x.AABB.Overlaps2D(region.Area));

                        // do not ignore disabled cells on purpose so we don't have to iterate separately over
                        foreach (GridCell g_cell in grid_cells)
                        {
                            // we only care about overlapping 'original' cells (ignore replacements)
                            var overlapped_cells = g_cell.GetCells(x => x.HasFlags(MovementFlag.Walk) && x.AABB.Overlaps2D(region.Area), false);

                            if (overlapped_cells.Count > 0)
                            {
                                using (new WriteLock(DataLock, "DataLock - Navmesh.Add - add to overlapping regions"))
                                {
                                    foreach (Cell cell in overlapped_cells)
                                    {
                                        // this requires write lock!
                                        if (!CellsOverlappedByRegions.TryGetValue(cell.GlobalId, out overlapped_cell_data data))
                                            CellsOverlappedByRegions[cell.GlobalId] = data = new overlapped_cell_data(cell, g_cell);

                                        data.overlapping_regions.Add(new Region(region));
                                    }
                                }
                            }
                        }
                    }

                    blockers = new HashSet<AABB>(regions_copy.Where(x => x.MoveCostMult < 0).Select(x => x.Area));
                }
            }

            List<int> no_longer_overlapped_cells_ids = new List<int>();
            bool nav_data_changed = false;
            bool anyReplacementCellWithMovementCost = false; // different than 1

            //DEBUG
            //var orphanedReplacementCellsBefore = new List<Cell>();
            //using (new ReadLock(DataLock))
            //    orphanedReplacementCellsBefore = m_AllCells.Where(x => x.Replacement).Where(x => !CellsOverlappedByRegions.Values.Any(r => r.replacement_cells.Any(y => y.GlobalId == x.GlobalId))).ToList();

            if (CellsOverlappedByRegions.Any())
            {
                bool blockersChanged = false;

                using (new WriteLock(DataLock, "DataLock - Navmesh.Add - update regions"))
                //using (new WriteLock(DataLock, context: "UpdateRegions"))
                using (new Profiler($"[Nav] Updating {regions_copy.Count} regions took %t", 50))
                {
                    // we need to go over every 'original' cell that is overlapped by at least one region and perform its 'rectangulation'
                    foreach (var item in CellsOverlappedByRegions)
                    {
                        // no longer overlapped
                        if (item.Value.overlapping_regions.Count == 0)
                            no_longer_overlapped_cells_ids.Add(item.Key);

                        overlapped_cell_data cell_data = item.Value;

                        // do nothing if overlapping regions didn't change
                        if (!cell_data.overlapping_regions.SetEquals(cell_data.last_overlapping_regions))
                        {
                            nav_data_changed = true;

                            //GridCell parent_grid_cell = GetGridCell(data.replaced_cell.Center);
                            GridCell parent_grid_cell = cell_data.parent_grid_cell;

                            // grid cell containing this replacement has been removed
                            if (parent_grid_cell == null)
                                continue;

                            // neighbors of replaced cell are first candidates for neighbors of replacement cells
                            List<Cell> potential_neighbors = cell_data.replaced_cell.Neighbours.Select(x => x.cell).ToList();

                            cell_data.replaced_cell.Disabled = (cell_data.overlapping_regions.Count > 0);

                            // remove previous replacement cells
                            foreach (Cell replacement_cell in cell_data.replacement_cells)
                            {
                                parent_grid_cell.RemoveReplacementCell(replacement_cell);
                                m_AllCells.Remove(replacement_cell);
                            }

                            cell_data.replacement_cells.Clear();

                            // generate new replacement cells
                            if (cell_data.overlapping_regions.Count > 0)
                            {
                                // full cell is first to be dissected and replaced (it will be removed as it is overlapped by region for sure)
                                cell_data.replacement_cells.Add(cell_data.replaced_cell);

                                // apply every region to all current replacement cells
                                foreach (Region region in cell_data.overlapping_regions)
                                {
                                    List<Cell> cells_to_check = new List<Cell>(cell_data.replacement_cells);

                                    foreach (Cell c in cells_to_check)
                                    {
                                        AABB[] extracted = c.AABB.Extract2D(region.Area);

                                        if (extracted != null)
                                        {
                                            cell_data.replacement_cells.Remove(c);

                                            for (int k = 0; k < extracted.Length; ++k)
                                            {
                                                // in case of negative movement cost treat this region as a dynamic obstacle (impassable area)
                                                if (region.MoveCostMult < 0 && k == 0)
                                                    continue;

                                                float movement_cost_mult = c.MovementCostMult;
                                                float threat = c.Threat;

                                                // first cell is always the one overlapped by region (because of how AABB.Extract2D is implemented)!
                                                if (k == 0)
                                                {
                                                    if (RegionsMoveCostMode == RegionsMode.Add)
                                                    {
                                                        movement_cost_mult += region.MoveCostMult;
                                                    }
                                                    else if (RegionsMoveCostMode == RegionsMode.Mult)
                                                    {
                                                        movement_cost_mult *= region.MoveCostMult;
                                                    }
                                                    else if (RegionsMoveCostMode == RegionsMode.Max)
                                                    {
                                                        if (movement_cost_mult < 1 && region.MoveCostMult < 1)
                                                            movement_cost_mult = Math.Min(region.MoveCostMult, movement_cost_mult);
                                                        else
                                                            movement_cost_mult = Math.Max(region.MoveCostMult, movement_cost_mult);
                                                    }

                                                    threat += region.Threat;
                                                }

                                                Cell r_cell = new Cell(extracted[k], cell_data.replaced_cell.Flags, movement_cost_mult) { ParentAABB = cell_data.replaced_cell.ParentAABB };
                                                r_cell.Replacement = true;
                                                r_cell.BlockerReplacement = c.BlockerReplacement || region.IsNavBlocker;
                                                r_cell.Threat = threat;
                                                cell_data.replacement_cells.Add(r_cell);
                                            }
                                        }
                                    }
                                }

                                // try to manually (without using GridCell methods) connect new replacement cells with potential neighbors
                                foreach (Cell replacement_cell in cell_data.replacement_cells)
                                {
                                    anyReplacementCellWithMovementCost |= replacement_cell.MovementCostMult != 1;

                                    Vec3 border_point = default(Vec3);

                                    foreach (Cell potential_neighbor in potential_neighbors)
                                        replacement_cell.TryAddNeighbour(potential_neighbor, out border_point);

                                    parent_grid_cell.AddReplacementCell(replacement_cell);
                                    m_AllCells.Add(replacement_cell);

                                    // this cell is now potential neighbor as well, because can be interconnected with other replacement cells!
                                    potential_neighbors.Add(replacement_cell);
                                }
                            }

                            item.Value.last_overlapping_regions = new HashSet<Region>(item.Value.overlapping_regions);

                            // enable original cell when no longer overlapped
                            if (cell_data.overlapping_regions.Count == 0)
                                cell_data.replaced_cell.Disabled = false;

                            m_CellsCache.Clear();

                            // request patches update at the earliest convenience when regions changed [no need for that, as only blockers can change patches]
                            //Interlocked.Exchange(ref ForcePatchesUpdate, 1);
                            //Console.WriteLine("regions changed!");
                        }
                    }

                    // when blockers changed patches needs to be updated immediately to have consistency between connection checks and path finding
                    if (!blockers.SetEquals(LastBlockers))
                    {
                        blockersChanged = true;
                        LastBlockers = blockers;
                        UpdatePatches(true, is_data_locked: true);
                        //Console.WriteLine("blockers changed!");
                    }
                }

                if (blockersChanged)
                {
                    NotifyOnNavBlockersChanged();
                    NotifyOnPatchesChanged();
                }
                
                //only when movement costs are affected
                if (nav_data_changed && anyReplacementCellWithMovementCost)
                {
                    //Trace.WriteLine($"{DateTime.Now.ToString("HH:mm:ss:ffff")} regions nav data changed");
                    NotifyOnNavDataChanged(affected_area);
                }

                // remove inactive data
                foreach (int key in no_longer_overlapped_cells_ids)
                    CellsOverlappedByRegions.Remove(key);

                //DEBUG
                //var orphanedReplacementCellsAfter = new List<Cell>();
                //using (new ReadLock(DataLock))
                //    orphanedReplacementCellsAfter = m_AllCells.Where(x => x.Replacement).Where(x => !CellsOverlappedByRegions.Values.Any(r => r.replacement_cells.Any(y => y.GlobalId == x.GlobalId))).ToList();

                //if (orphanedReplacementCellsAfter.Count > orphanedReplacementCellsBefore.Count)
                //    Trace.WriteLine($"{orphanedReplacementCellsAfter.Count - orphanedReplacementCellsBefore.Count} orphaned replacement cells detected!");

                //if (blockersChanged)
                //    Console.WriteLine("regions update done");
            }
        }

        public bool IsNavDataAvailable
        {
            get
            {
                //using (new ReadLock(DataLock))
                    return m_GridCells.Count > 0;
            }
        }

        public virtual void Clear()
        {
            using (new WriteLock(DataLock, "DataLock - Navmesh.Clear"))
            using (new WriteLock(PatchesDataLock, "PatchesDataLock - Navmesh.Clear"))
            using (new WriteLock(InputLock, "InputLock - Navmesh.Clear"))
            {
                m_CellsCache.Clear();
                m_AllCells.Clear();
                m_GridCells.Clear();
                m_Regions.Clear();
                CellsOverlappedByRegions.Clear();
                LastBlockers.Clear();
                m_UberCellsPatch = null;
                m_CellsPatches.Clear();
                CellsPatch.LastCellsPatchGlobalId = 0;
                Cell.LastCellGlobalId = 0;
                GridCell.LastGridCellGlobalId = 0;

                m_LastGridCellId = 0;
                m_LastCellId = 0;

                Trace.WriteLine("navmesh - cleared");
            }

            foreach (INavmeshObserver observer in m_Observers)
                observer.OnNavDataCleared();

            Log("[Nav] Navmesh cleared!");
        }

        public bool Load(string filename, bool clear = true, bool force_update_align_plane = false)
        {
            if (filename == null)
                return false;

            Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

            //using (new Profiler("[Nav] Loaded nav data [%t]"))
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
                        Vec3 cell_shrink_size = Vec3.ZERO;
                        var avoid_areas = new List<Region>();

                        while ((line = stream.ReadLine()) != null)
                        {
                            string[] data = line.Split(' ');

                            if (data[0] == "g")
                            {
                                // add previous grid cell
                                Add(g_cell, false);

                                m_LastCellId = 0;

                                g_cell = new GridCell(float.Parse(data[1]), float.Parse(data[2]), float.Parse(data[3]), float.Parse(data[4]), float.Parse(data[5]), float.Parse(data[6]), (data.Length > 7 ? int.Parse(data[7]) : m_LastGridCellId++));
                            }
                            else if (data[0] == "n")
                            {
                                MovementFlag flags = MovementFlag.Walk | MovementFlag.Fly;

                                if (data.Length > 7)
                                    flags = (MovementFlag)int.Parse(data[7]);

                                Cell n_cell = new Cell(new Vec3(float.Parse(data[1]), float.Parse(data[2]), float.Parse(data[3])) - cell_shrink_size, new Vec3(float.Parse(data[4]), float.Parse(data[5]), float.Parse(data[6])) - cell_shrink_size, flags, 1, m_LastCellId++);
                                g_cell.Add(n_cell);
                            }
                            else if (data[0] == "r")
                            {
                                avoid_areas.Add(new Region(new AABB(float.Parse(data[1]), float.Parse(data[2]), float.Parse(data[3]), float.Parse(data[4]), float.Parse(data[5]), float.Parse(data[6])), float.Parse(data[7]), float.Parse(data[8])));
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

                        random_grid_cell = m_GridCells.ElementAt(rng.Next(m_GridCells.Count));

                        NotifyOnNavDataChanged();

                        Regions = avoid_areas;

                        Log("[Nav] Navmesh loaded.");

                        stream.Close();

                        return true;
                    }
                }
                catch (Exception ex)
                {
                    Trace.WriteLine($"{ex.Message}");
                }

                Log("[Nav] Navmesh load failed!");

                return false;
            }
        }

        public Vec3 GetRandomPos(Random rng = null)
        {
            using (new ReadLock(DataLock))
            //using (new ReadLock(DataLock, context: "GetRandomPos"))
            {
                rng = rng ?? Rng;
                GridCell g_cell = m_GridCells.ElementAt(rng.Next(m_GridCells.Count));
                var enabled_cells = g_cell.GetCells(false).ToArray();
                return enabled_cells[rng.Next(enabled_cells.Count())].AABB.GetRandomPos(rng);
            }
        }

        public Vec3 GetCenter()
        {
            using (new ReadLock(DataLock))
            {
                Vec3 avg_pos = Vec3.ZERO;

                foreach (GridCell g_cell in m_GridCells)
                    avg_pos += g_cell.AABB.Center;

                return avg_pos / (float)m_GridCells.Count;
            }
        }

        public List<Cell> GetCellsInside(AABB area)
        {
            List<Cell> result = new List<Cell>();

            using (new ReadLock(DataLock))
            //using (new ReadLock(DataLock, context: "GetCellsInside"))
            {
                var g_cells = m_GridCells.Where(x => x.AABB.Overlaps2D(area));
                List<Cell> cells = new List<Cell>();

                // do not ignore disabled cells on purpose so we don't have to iterate separately over
                foreach (GridCell g_cell in g_cells)
                    cells.AddRange(g_cell.GetCells(x => x.HasFlags(MovementFlag.Walk) && x.AABB.Overlaps2D(area), false));

                foreach (var cell in cells)
                    result.Add(cell.CreateSimplifiedClone());
            }

            return result;
        }

        public bool RayTrace(Vec3 from, Vec3 to, MovementFlag flags, ref Vec3 intersection)
        {
            using (new ReadLock(DataLock))
            {
                intersection = Vec3.ZERO;

                Vec3 ray_dir = to - from;
                ray_dir.Normalize();
                bool ray_hit = false;

                foreach (Cell c in m_AllCells)
                {
                    if (!c.HasFlags(flags))
                        continue;

                    Vec3 new_intersection = default(Vec3);
                    if (c.AABB.RayTest(from, ray_dir, ref new_intersection) && from.DistanceSqr(new_intersection) < from.DistanceSqr(intersection))
                    {
                        ray_hit = true;
                        intersection = new_intersection;
                    }
                }

                return ray_hit;
            }
        }

        public RayCastResult RayCast(Vec3 from, Vec3 to, MovementFlag flags, float max_movement_cost_mult = float.MaxValue)
        {
            HashSet<Cell> ignored_cells = new HashSet<Cell>();
            return RayCast(from, null, to, flags, false, max_movement_cost_mult, ref ignored_cells);
        }

        public RayCastResult RayCast(Vec3 from, Cell from_cell, Vec3 to, MovementFlag flags, float max_movement_cost_mult = float.MaxValue)
        {
            HashSet<Cell> ignored_cells = new HashSet<Cell>();
            return RayCast(from, from_cell, to, flags, false, max_movement_cost_mult, ref ignored_cells);
        }

        public RayCastResult RayCast2D(Vec3 from, Vec3 to, MovementFlag flags, float max_movement_cost_mult = float.MaxValue)
        {
            HashSet<Cell> ignored_cells = new HashSet<Cell>();
            return RayCast(from, null, to, flags, true, max_movement_cost_mult, ref ignored_cells);
        }

        public RayCastResult RayCast2D(Vec3 from, Cell from_cell, Vec3 to, MovementFlag flags, float max_movement_cost_mult = float.MaxValue)
        {
            HashSet<Cell> ignored_cells = new HashSet<Cell>();
            return RayCast(from, from_cell, to, flags, true, max_movement_cost_mult, ref ignored_cells);
        }

        private RayCastResult RayCast(Vec3 from, Cell from_cell, Vec3 to, MovementFlag flags, bool test_2d, float max_movement_cost_mult, ref HashSet<Cell> ignored_cells)
        {
            bool force_failed = false;

            // when we don't care about regions, we can run ray cast on unmodified navmesh and clamp end position to nearest intersected nav blocker
            if (max_movement_cost_mult == float.MaxValue)
            {
                using (new ReadLock(DataLock))
                {
                    float dist_to_nearest_intersection = -1;
                    Vec3 nearest_intersection = to;
                    Vec3 intersection = Vec3.ZERO;
                    foreach (var blocker in LastBlockers)
                    {
                        bool success;
                        if (test_2d)
                            success = blocker.SegmentTest2D(from, to, ref intersection);
                        else
                            success = blocker.SegmentTest(from, to, ref intersection);

                        if (success)
                        {
                            float dist_to_intersection = from.DistanceSqr(intersection);
                            if (dist_to_nearest_intersection < 0 || dist_to_intersection < dist_to_nearest_intersection)
                            {
                                nearest_intersection = intersection;
                                dist_to_nearest_intersection = dist_to_intersection;
                            }
                        }
                    }

                    if (dist_to_nearest_intersection >= 0)
                    {
                        to = nearest_intersection;
                        force_failed = true;
                    }
                }
            }

            var result = RayCastInternal(from, from_cell, to, flags, test_2d, max_movement_cost_mult, ref ignored_cells);
            result.Successful &= !force_failed;
            return result;
        }

        // Aquires DataLock (read); returns true when there is no obstacle
        private RayCastResult RayCastInternal(Vec3 from, Cell from_cell, Vec3 to, MovementFlag flags, bool test_2d, float max_movement_cost_mult, ref HashSet<Cell> ignored_cells)
        {
            using (new ReadLock(DataLock))
            //using (new ReadLock(DataLock, context: "RayCast"))
            {
                // when we don't care about movement cost mult we can greatly improve cast when there is lots of regions by running it on base navmesh and doing extra check against nav blockers (regions with negative movement cost mult)
                bool check_non_replacement_only = max_movement_cost_mult == float.MaxValue;

                if (from_cell == null && !GetCellAt(from, out from_cell, flags, check_non_replacement_only, !check_non_replacement_only, false, -1, test_2d, 2, ignored_cells))
                {
                    return new RayCastResult { Successful = false, End = from, EndCell = null };
                }

                if (test_2d ? from_cell.Contains2D(to) : from_cell.Contains(to, 2))
                {
                    return new RayCastResult { Successful = true, End = to, EndCell = from_cell };
                }

                ignored_cells.Add(from_cell);

                Vec3 ray_dir = to - from;
                if (test_2d)
                    ray_dir.Normalize2D();
                else
                    ray_dir.Normalize();

                Vec3 ray_origin = new Vec3(from);

                bool any_neighbour_accepted = false;

                RayCastResult result = default(RayCastResult);

                // check if intersection in
                foreach (Cell.Neighbour neighbour in from_cell.Neighbours)
                {
                    if ((neighbour.connection_flags & flags) == 0)
                        continue;

                    if (check_non_replacement_only)
                    {
                        if (neighbour.cell.Replacement)
                            continue;
                    }
                    else
                    {
                        if (neighbour.cell.Disabled || (neighbour.cell.MovementCostMult > max_movement_cost_mult))
                            continue;
                    }

                    Cell neighbour_cell = neighbour.cell;

                    if (ignored_cells.Contains(neighbour_cell))
                        continue;

                    Vec3 intersection = default(Vec3);

                    bool ray_test_result = test_2d ? neighbour_cell.AABB.RayTest2D(ray_origin, ray_dir, ref intersection) :
                                                     neighbour_cell.AABB.RayTest(ray_origin, ray_dir, ref intersection);

                    if (ray_test_result)
                    {
                        AABB shared_aabb = default(AABB);

                        bool instersects = test_2d ? from_cell.AABB.Intersect2D(neighbour_cell.AABB, ref shared_aabb, true) :
                                                     from_cell.AABB.Intersect(neighbour_cell.AABB, ref shared_aabb, true);

                        // ray intersects on connection plane
                        if (instersects)
                        {
                            bool accepted = test_2d ? shared_aabb.Contains2D(intersection) :
                                                      shared_aabb.Contains(intersection);

                            any_neighbour_accepted |= accepted;

                            if (accepted)
                            {
                                result = RayCastInternal(intersection, neighbour_cell, to, flags, test_2d, max_movement_cost_mult, ref ignored_cells);

                                if (result.Successful)
                                    return result;
                            }
                        }
                    }
                }

                // ray cast doesn't hit any neighbour we need to figure out where is intersects this aabb, but since origin is on the edge and we want to find "exit" point
                // we need to offset origin a tiny bit
                if (!any_neighbour_accepted)
                {
                    if (test_2d)
                    {
                        // it is possible to miss this ray test close to the corner
                        if (!from_cell.AABB.RayTest2D(ray_origin + ray_dir * 0.1f, ray_dir, ref result.End))
                            result.End = ray_origin;
                    }
                    else
                    {
                        if (!from_cell.AABB.RayTest(ray_origin + ray_dir * 0.1f, ray_dir, ref result.End))
                            result.End = ray_origin;
                    }

                    result.EndCell = from_cell;
                }

                return result;
            }
        }

        public HashSet<int> GetPatchesIds(Vec3 pos, MovementFlag flags, float nearest_tolerance_pos)
        {
            using (new ReadLock(PatchesDataLock))
            {
                if (m_UberCellsPatch == null)
                    return new HashSet<int>();

                var near_cells = m_UberCellsPatch.GetCellsWithin(pos, nearest_tolerance_pos, flags);

                var ids = new HashSet<int>();
                foreach (var patch in m_CellsPatches)
                {
                    if (near_cells.Where(x => patch.Cells.Contains(x)).Any())
                        ids.Add(patch.GlobalId);
                }
                return ids;
            }
        }

        public bool AreConnected(HashSet<int> patchesIds1, HashSet<int> patchesIds2)
        {
            return patchesIds1.Overlaps(patchesIds2);
        }

        public bool AreConnected(Vec3 pos1, Vec3 pos2, MovementFlag flags, float nearest_tolerance_pos1, float nearest_tolerance_pos2)
        {
            return AreConnected(pos1, pos2, flags, nearest_tolerance_pos1, nearest_tolerance_pos2, out var pos1_on_navmesh, out var pos2_on_navmesh);
        }

        public bool AreConnected(Vec3 pos1, Vec3 pos2, MovementFlag flags, float nearest_tolerance_pos1, float nearest_tolerance_pos2, out Vec3 pos1_on_navmesh, out Vec3 pos2_on_navmesh)
        {
            return AreConnected(pos1, pos2, flags, nearest_tolerance_pos1, nearest_tolerance_pos2, out pos1_on_navmesh, out pos2_on_navmesh, false, out var is_pos1_near_navmesh, out var is_pos2_near_navmesh);
        }

        public bool AreConnected(Vec3 pos1, Vec3 pos2, MovementFlag flags, float nearest_tolerance_pos1, float nearest_tolerance_pos2, out Vec3 pos1_on_navmesh, out Vec3 pos2_on_navmesh, bool check_near_navmesh, out bool is_pos1_near_navmesh, out bool is_pos2_near_navmesh)
        {
            is_pos2_near_navmesh = is_pos1_near_navmesh = false;

            using (new Profiler("AreConnected (incl. lock) took %t", 100))
            using (new ReadLock(PatchesDataLock))
            //using (new ReadLock(DataLock, context: "AreConnected"))
            using (new Profiler("AreConnected took %t", 10))
            {
                pos1_on_navmesh = pos1;
                pos2_on_navmesh = pos2;

                if (m_UberCellsPatch == null)
                    return false;

                var near_pos1_cells = m_UberCellsPatch.GetCellsWithin(pos1, nearest_tolerance_pos1, flags).ToHashSet();

                if (!check_near_navmesh && !near_pos1_cells.Any())
                    return false;

                var near_pos2_cells = m_UberCellsPatch.GetCellsWithin(pos2, nearest_tolerance_pos2, flags).ToHashSet();

                if (!check_near_navmesh && !near_pos2_cells.Any())
                    return false;

                foreach (var patch in m_CellsPatches)
                {
                    List<Cell> pos1_cells = 
                    //patch.GetCellsWithin(pos1, nearest_tolerance_pos1, flags);
                    near_pos1_cells.Where(x => patch.Cells.Contains(x)).ToList();
                    //near_pos1_cells.Intersect(patch.Cells).ToList();

                    if (pos1_cells.Count == 0)
                    {
                        if (!check_near_navmesh)
                            continue;
                    }
                    else
                        is_pos1_near_navmesh = true;

                    List<Cell> pos2_cells =
                    //patch.GetCellsWithin(pos2, nearest_tolerance_pos2, flags);
                    near_pos2_cells.Where(x => patch.Cells.Contains(x)).ToList();
                    //near_pos2_cells.Intersect(patch.Cells).ToList();

                    if (pos2_cells.Count == 0)
                        continue;
                    else
                        is_pos2_near_navmesh = true;

                    if (check_near_navmesh && pos1_cells.Count == 0)
                        continue;

                    var pos1_nearest_cell = pos1_cells.OrderBy(x => pos1.Distance2DSqr(x.AABB.Align(pos1))).First();
                    var pos2_nearest_cell = pos2_cells.OrderBy(x => pos2.Distance2DSqr(x.AABB.Align(pos2))).First();

                    pos1_on_navmesh = pos1_nearest_cell.AABB.Align(pos1);
                    pos2_on_navmesh = pos2_nearest_cell.AABB.Align(pos2);
                    return true;
                }

                return false;
            }
        }

        public bool SnapToNavmesh(Vec3 pos, float tolerance, MovementFlag flags, out Vec3 snapped_pos)
        {
            snapped_pos = pos;
            float snapped_pos_dist = -1;

            using (new ReadLock(PatchesDataLock))
            {
                if (m_UberCellsPatch == null)
                    return false;

                var pos_cells = m_UberCellsPatch.GetCellsWithin(pos, tolerance, flags);

                if (pos_cells.Count == 0)
                    return false;

                var pos_nearest_cell = pos_cells.OrderBy(x => pos.Distance2DSqr(x.AABB.Align(pos))).First();

                var aligned_pos = pos_nearest_cell.AABB.Align(pos);
                var dist = pos.Distance2DSqr(aligned_pos);

                if (snapped_pos_dist < 0 || dist < snapped_pos_dist)
                {
                    snapped_pos = aligned_pos;
                    snapped_pos_dist = dist;
                }

                return snapped_pos_dist >= 0;
            }
        }

        public bool IsOnNavmesh(Vec3 pos, MovementFlag flags, bool check_2d = true)
        {
            using (new ReadLock(DataLock))
            {
                var gridCells = (check_2d ? m_GridCells.Where(x => x.Contains2D(pos)) : m_GridCells.Where(x => x.Contains(pos))).ToList();

                if (!gridCells.Any())
                    return false;

                foreach (var gridCell in gridCells)
                    if (gridCell.GetCellsAt(pos, check_2d, 0, false).Where(x => x.HasFlags(flags)).Any())
                        return true;

                return false;
            }
        }

        // Operations on this list are not thread safe! Use AquireReadDataLock to make it thread safe
        public HashSet<GridCell> dbg_GetGridCells()
        {
            return m_GridCells;
        }

        public int GridCellsCount => m_GridCells.Count;

        public int CellsCount => m_AllCells.Count;

        private bool test_RayTrace()
        {
            Vec3 result = default(Vec3);

            AABB a = new AABB(-5, -5, -5, 5, 5, 5);

            // tangent along Y
            if (!a.RayTest(new Vec3(-5, -100, 0), new Vec3(0, 1, 0), ref result) || !result.Equals(new Vec3(-5, -5, 0)))
                return false;
            if (!a.RayTest(new Vec3(-5, 0, 0), new Vec3(0, 1, 0), ref result) || !result.Equals(new Vec3(-5, 0, 0)))
                return false;
            if (a.RayTest(new Vec3(-5, 100, 0), new Vec3(0, 1, 0), ref result))
                return false;

            if (a.RayTest(new Vec3(-5, -100, 0), new Vec3(0, -1, 0), ref result))
                return false;
            if (!a.RayTest(new Vec3(-5, 0, 0), new Vec3(0, -1, 0), ref result) || !result.Equals(new Vec3(-5, 0, 0)))
                return false;
            if (!a.RayTest(new Vec3(-5, 100, 0), new Vec3(0, -1, 0), ref result) || !result.Equals(new Vec3(-5, 5, 0)))
                return false;

            // tangent along X
            if (!a.RayTest(new Vec3(-100, 5, 0), new Vec3(1, 0, 0), ref result) || !result.Equals(new Vec3(-5, 5, 0)))
                return false;
            if (!a.RayTest(new Vec3(0, 5, 0), new Vec3(1, 0, 0), ref result) || !result.Equals(new Vec3(0, 5, 0)))
                return false;
            if (a.RayTest(new Vec3(100, 5, 0), new Vec3(1, 0, 0), ref result))
                return false;

            // tangent along Z
            if (!a.RayTest(new Vec3(0, 5, 100), new Vec3(0, 0, -1), ref result) || !result.Equals(new Vec3(0, 5, 5)))
                return false;
            if (!a.RayTest(new Vec3(0, 5, 0), new Vec3(0, 0, -1), ref result) || !result.Equals(new Vec3(0, 5, 0)))
                return false;
            if (a.RayTest(new Vec3(0, 5, -100), new Vec3(0, 0, -1), ref result))
                return false;

            // touching corners
            if (!a.RayTest(new Vec3(-5, -5, 0), new Vec3(1, 0, 0), ref result) || !result.Equals(new Vec3(-5, -5, 0)))
                return false;
            if (!a.RayTest(new Vec3(-5, -5, 0), new Vec3(0, 1, 0), ref result) || !result.Equals(new Vec3(-5, -5, 0)))
                return false;
            if (!a.RayTest(new Vec3(-5, -5, 0), new Vec3(0, 0, 1), ref result) || !result.Equals(new Vec3(-5, -5, 0)))
                return false;

            return true;
        }

        public void dbg_GenerateRandomAvoidAreas(int areas_num, float move_cost_mult, int approx_size, float threat = 0)
        {
            Random rng = new Random();
            List<Region> regions = new List<Region>();

            using (new ReadLock(DataLock))
            {
                for (int i = 0; i < areas_num; ++i)
                {
                    Vec3 pos = GetRandomPos();
                    float size = approx_size + (float)rng.NextDouble() * (approx_size * 0.5f);
                    regions.Add(new Region(new AABB(pos - new Vec3(size * 0.5f, size * 0.5f, 0), pos + new Vec3(size * 0.5f, size * 0.5f, 0)), move_cost_mult, threat));
                }
            }

            Regions = regions;
        }

        public void dbg_GenerateBlockOfAvoidAreas(int area_size, int grid_size, int threat_min = 0, int threat_max = 20)
        {
            Random rng = new Random();
            List<Region> regions = new List<Region>();

            using (new ReadLock(DataLock))
            {
                Vec3 pos = GetRandomPos();

                for (int x = 0; x < grid_size; ++x)
                for (int y = 0; y < grid_size; ++y)
                {
                    regions.Add(new Region(new AABB(pos + new Vec3(x * area_size, y * area_size, 0), pos + new Vec3((x+1) * area_size, (y+1) * area_size, 0)), 2, rng.Next(threat_min, threat_max)));
                }
            }

            Regions = regions;
        }

        // Aquire DataLock (read)
        public IEnumerable<GridCell> GetGridCellsContaining(Vec3 pos, bool check_2d = true)
        {
            using (new ReadLock(DataLock))
                return check_2d ? m_GridCells.Where(x => x.Contains2D(pos)) : m_GridCells.Where(x => x.Contains(pos));
        }

        public Vec3 GetRandomPosInRing(Vec3 ring_center, float min_radius, float max_radius)
        {
            var potential_grid_cells = GetCellsOverlappedByCircle(m_GridCells, ring_center, max_radius);

            List<Cell> cells = new List<Cell>();
            foreach (GridCell grid_cell in potential_grid_cells)
                cells.AddRange(grid_cell.GetCells(x => x.Overlaps(ring_center, max_radius) && !x.AABB.Inside(ring_center, min_radius)));

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

            Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

            using (new ReadLock(DataLock))
            using (new ReadLock(InputLock))
            {
                foreach (GridCell grid_cell in m_GridCells)
                {
                    file.WriteLine("g {0} {1} {2} {3} {4} {5} {6}", grid_cell.Min.X, grid_cell.Min.Y, grid_cell.Min.Z, grid_cell.Max.X, grid_cell.Max.Y, grid_cell.Max.Z, grid_cell.Id);

                    foreach (Cell cell in grid_cell.GetCells(false))
                        file.WriteLine("n {0} {1} {2} {3} {4} {5} {6}", cell.Min.X, cell.Min.Y, cell.Min.Z, cell.Max.X, cell.Max.Y, cell.Max.Z, (int)cell.Flags);
                }

                foreach (Region region in Regions)
                    file.WriteLine("r {0} {1} {2} {3} {4} {5} {6} {7}", region.Area.Min.X, region.Area.Min.Y, region.Area.Min.Z, region.Area.Max.X, region.Area.Max.Y, region.Area.Max.Z, region.MoveCostMult, region.Threat);
            }

            file.Close();

            Log("[Nav] Navmesh dumped.");
        }

        // Extension will be automatically added
        public void Serialize(string name)
        {
            using (BinaryWriter w = new BinaryWriter(File.OpenWrite(name + ".navmesh")))
            {
                OnSerialize(w);
            }
        }

        protected virtual void OnSerialize(BinaryWriter w)
        {
            using (new ReadLock(DataLock))
            using (new ReadLock(PatchesDataLock))
            using (new ReadLock(InputLock))
            {
                var all_cells_ids = new HashSet<int>();

                // write all cells global IDs
                w.Write(m_AllCells.Count);
                foreach (Cell cell in m_AllCells)
                {
                    w.Write(cell.GlobalId);

                    if (Verbose)
                        all_cells_ids.Add(cell.GlobalId);
                }

                // cell patches may contain old cells that are no longer in all cells (this is becuase patches only refresh when nav blockers change)
                // write all patch-only cells global IDs
                var patch_cells = m_CellsPatches.SelectMany(x => x.Cells).Except(m_AllCells).ToList();
                w.Write(patch_cells.Count);
                foreach (Cell cell in patch_cells)
                {
                    w.Write(cell.GlobalId);

                    if (Verbose)
                        all_cells_ids.Add(cell.GlobalId);
                }

                foreach (Cell cell in m_AllCells.Concat(patch_cells))
                {
                    cell.Serialize(w);

                    if (Verbose)
                    {
                        foreach (var neighbour in cell.Neighbours)
                        {
                            if (!all_cells_ids.Contains(neighbour.cell.GlobalId))
                                Log("[Nav] Cell neighbour not on all cells list! Cell info [" + neighbour.cell.ToString() + "]", true);
                        }
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

                w.Write(m_UberCellsPatch?.GlobalId ?? -1);
                m_UberCellsPatch?.Serialize(w);

                w.Write(CellsPatch.LastCellsPatchGlobalId);

                w.Write(m_Regions.Count);
                foreach (Region region in m_Regions)
                    region.Serialize(w);

                w.Write(CellsOverlappedByRegions.Count);
                foreach (var entry in CellsOverlappedByRegions)
                {
                    w.Write(entry.Key);
                    entry.Value.Serialize(w);
                }

                w.Write(LastBlockers.Count);
                foreach (var blocker in LastBlockers)
                    blocker.Serialize(w);
            }

            Log("[Nav] Navmesh serialized.");
        }

        // Extension will be automatically added
        public void Deserialize(string name)
        {
            using (BinaryReader r = new BinaryReader(File.OpenRead(name + ".navmesh")))
            {
                OnDeserialize(r);
            }
        }

        protected virtual void OnDeserialize(BinaryReader r)
        {
            using (new WriteLock(DataLock, "DataLock - Navmesh.OnDeserialize"))
            using (new WriteLock(PatchesDataLock, "PatchesDataLock - Navmesh.OnDeserialize"))
            using (new WriteLock(InputLock, "InputLock - Navmesh.OnDeserialize"))
            {
                //using (new Profiler("Navmesh deserialization took %t"))
                {
                    m_AllCells.Clear();
                    m_IdToCell.Clear();
                    m_GridCells.Clear();
                    m_Regions.Clear();
                    CellsOverlappedByRegions.Clear();

                    Cell.CompareByGlobalId comp_by_global_id = new Cell.CompareByGlobalId();

                    int all_cells_count = r.ReadInt32();

                    // pre-allocate cells
                    for (int i = 0; i < all_cells_count; ++i)
                    {
                        Cell cell = new Cell(0, 0, 0, 0, 0, 0, MovementFlag.None);
                        cell.GlobalId = r.ReadInt32();
                        m_IdToCell[cell.GlobalId] = cell;
                        m_AllCells.Add(cell);
                    }

                    int patch_cells_count = r.ReadInt32();
                    var patch_cells = new HashSet<Cell>();

                    // pre-allocate patch-only cells
                    for (int i = 0; i < patch_cells_count; ++i)
                    {
                        Cell cell = new Cell(0, 0, 0, 0, 0, 0, MovementFlag.None);
                        cell.GlobalId = r.ReadInt32();
                        m_IdToCell[cell.GlobalId] = cell;
                        patch_cells.Add(cell);
                    }

                    foreach (Cell cell in m_AllCells)
                        cell.Deserialize(m_AllCells, m_IdToCell, r);

                    var patch_and_all_cells = m_AllCells.Union(patch_cells).ToHashSet();
                    foreach (Cell cell in patch_cells)
                        cell.Deserialize(patch_and_all_cells, m_IdToCell, r);

                    Cell.LastCellGlobalId = r.ReadInt32();

                    int grid_cells_count = r.ReadInt32();

                    // pre-allocate grid cells
                    for (int i = 0; i < grid_cells_count; ++i)
                    {
                        GridCell grid_cell = new GridCell(0, 0, 0, 0, 0, 0);
                        grid_cell.GlobalId = r.ReadInt32();
                        m_GridCells.Add(grid_cell);
                    }

                    foreach (GridCell grid_cell in m_GridCells)
                        grid_cell.Deserialize(m_GridCells, m_AllCells, m_IdToCell, r);

                    GridCell.LastGridCellGlobalId = r.ReadInt32();

                    List<CellsPatch> patches = new List<CellsPatch>();
                    int patches_count = r.ReadInt32();

                    // pre-allocate patches
                    for (int i = 0; i < patches_count; ++i)
                    {
                        CellsPatch patch = new CellsPatch(new HashSet<Cell>(), new Dictionary<AABB, List<Cell>>(), MovementFlag.None);
                        patch.GlobalId = r.ReadInt32();
                        patches.Add(patch);
                    }

                    foreach (CellsPatch patch in patches)
                        patch.Deserialize(m_AllCells, m_IdToCell, r);

                    var uber_patch_global_id = r.ReadInt32();
                    if (uber_patch_global_id != -1)
                    {
                        m_UberCellsPatch = new CellsPatch(new HashSet<Cell>(), new Dictionary<AABB, List<Cell>>(), MovementFlag.None);
                        m_UberCellsPatch.GlobalId = uber_patch_global_id;
                        m_UberCellsPatch.Deserialize(m_AllCells, m_IdToCell, r);
                    }
                    else
                        m_UberCellsPatch = null;

                    m_CellsPatches = new HashSet<CellsPatch>(patches);

                    CellsPatch.LastCellsPatchGlobalId = r.ReadInt32();

                    int regions_count = r.ReadInt32();
                    for (int i = 0; i < regions_count; ++i)
                        m_Regions.Add(new Region(r));

                    int cells_overlapped_by_regions_count = r.ReadInt32();
                    for (int i = 0; i < cells_overlapped_by_regions_count; ++i)
                    {
                        int key = r.ReadInt32();
                        CellsOverlappedByRegions.Add(key, new overlapped_cell_data(m_GridCells, m_AllCells, r));
                    }

                    int blockers_count = r.ReadInt32();
                    for (int i = 0; i < blockers_count; ++i)
                        LastBlockers.Add(new AABB(r));
                }
            }

            Log("[Nav] Navmesh deserialized.");
        }

        public virtual void Dispose()
        {
            ShouldStopUpdates = true;
            if (!UpdatesThread.Join(3000))
                UpdatesThread.Interrupt();
        }

        internal protected void Log(string msg, bool force = false)
        {
            if (Verbose || force)
            {
                DateTime d = DateTime.Now;
                Trace.WriteLine(d.Hour.ToString().PadLeft(2, '0') + ":" + d.Minute.ToString().PadLeft(2, '0') + ":" + d.Second.ToString().PadLeft(2, '0') + ":" + d.Millisecond.ToString().PadLeft(3, '0') + " " + msg);
            }
        }

        internal IEnumerable<T> GetCellsOverlappedByCircle<T>(HashSet<T> cells, Vec3 circle_center, float radius) where T : Cell
        {
            return cells.Where(x => x.Overlaps(circle_center, radius));
        }

        internal List<T> GetCellsInsideCircle<T>(List<T> cells, Vec3 circle_center, float radius) where T : Cell
        {
            return cells.FindAll(x => x.AABB.Inside(circle_center, radius));
        }

        public void AddObserver(INavmeshObserver observer)
        {
            using (new WriteLock(InputLock, "InputLock - Navmesh.AddObserver"))
            {
                if (m_Observers.Contains(observer))
                    return;

                m_Observers.Add(observer);
            }
        }

        public void RemoveObserver(INavmeshObserver observer)
        {
            using (new WriteLock(InputLock, "InputLock - Navmesh.RemoveObserver"))
                m_Observers.Remove(observer);
        }

        protected void NotifyOnNavDataChanged(AABB affected_area = default(AABB))
        {
            List<INavmeshObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavmeshObserver observer in observers_copy)
                observer.OnNavDataChanged(affected_area);
        }

        protected void NotifyOnNavBlockersChanged()
        {
            List<INavmeshObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavmeshObserver observer in observers_copy)
                observer.OnNavBlockersChanged();
        }

        protected void NotifyOnPatchesChanged()
        {
            List<INavmeshObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavmeshObserver observer in observers_copy)
                observer.OnPatchesChanged();
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
        protected Random Rng = new Random();

        private Thread UpdatesThread = null;

        internal ReaderWriterLockSlim DataLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        internal ReaderWriterLockSlim PatchesDataLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private ReaderWriterLockSlim InputLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);

        public ReadLock AcquireReadDataLock(string description = null)
        {
            return new ReadLock(DataLock, description: $"Navmesh.DataLock - {description}");
        }

        internal HashSet<Cell> m_AllCells = new HashSet<Cell>(); //@ DataLock
        internal Dictionary<int, Cell> m_IdToCell = new Dictionary<int, Cell>(); //@ DataLock
        // cell patches are interconnected groups of cells allowing ultra fast connection checks
        internal HashSet<CellsPatch> m_CellsPatches = new HashSet<CellsPatch>(); //@ PatchesDataLock
        // all cells used to build past patches snapshot
        internal CellsPatch m_UberCellsPatch; //@ PatchesDataLock
        internal HashSet<GridCell> m_GridCells = new HashSet<GridCell>(); //@ DataLock
        private List<Region> m_Regions = new List<Region>(); //@ InputLock        
        private List<INavmeshObserver> m_Observers = new List<INavmeshObserver>(); //@ InputLock
    }
}
