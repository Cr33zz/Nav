using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.IO;
using System.Linq;

namespace Nav
{
    public abstract class ExplorationEngine : IDisposable, INavigationObserver, INavmeshObserver, IRoughPathEstimator
    {
        public ExplorationEngine(Navmesh navmesh, NavigationEngine navigator, int explore_cell_size = 90)
        {
            ExploreCellSize = explore_cell_size;

            m_Navmesh = navmesh;
            m_Navmesh.AddObserver(this);

            m_Navigator = navigator;
            m_Navigator.AddObserver(this);
            m_Navigator.m_RoughtPathEstimator = m_Navigator.m_RoughtPathEstimator ?? this;

            // generate exploration data from already existing grid cells
            Reset();

            UpdatesThread = new Thread(Updates);
            UpdatesThread.Name = "Explorator-UpdatesThread";
            UpdatesThread.Start();
        }

        // called every update when heading towards an explore cell. parameters are destination explore cell and current position
        public Func<ExploreCell, Vec3, bool> AlternativeExploredCondition { get; set; }

        public float MaxAreaToMarkAsSmall { get; set; } = 0 * 0;
        public float MaxDimensionToMarkAsSmall { get; set; } = 0;

        // precision with explore destination will be accepted as reached
        public float ExploreDestPrecision { get; set; } = 20;

        // change it via ChangeExploreCellSize function
        public int ExploreCellSize { get; private set; }

        public AABB ExploreConstraint
        {
            get
            {
                return m_ExploreConstraint;
            }
            set
            {
                if (m_ExploreConstraint.Equals(value))
                    return;

                m_ExploreConstraint = value;
                lock (UpdateLocker)
                {
                    UpdateExploration(true);
                }
            }
        }

        public void ChangeExploreCellSize(int size)
        {
            if (size <= 0)
                return;

            ExploreCellSize = size;
            Clear();
        }

        public virtual void Clear()
        {
            using (new WriteLock(DataLock))
            using (new WriteLock(InputLock))
            {
                m_ExploreCells.Clear();
                m_ExploredCellsCount = 0;
                m_CellsToExploreCount = 0;

                ExploreCell.LastExploreCellGlobalId = 0;
                m_LastExploreCellId = 0;

                m_HintPos = Vec3.ZERO;
            }
        }

        public void ResetExploration(AABB area = default)
        {
            using (new WriteLock(DataLock))
            using (new WriteLock(InputLock))
            {
                if (!area.IsZero())
                {
                    foreach (var ex_cell in m_ExploreCells)
                        ex_cell.Explored = false;
                    m_ExploredCellsCount = 0;
                    m_CellsToExploreCount = 0;
                }
                else
                {
                    var exploreCellsInArea = m_ExploreCells.Where(x => x.AABB.Overlaps2D(area)).ToList();
                    var alreadyExploredCellsCount = exploreCellsInArea.Count(x => x.Explored);
                    m_ExploredCellsCount -= alreadyExploredCellsCount;
                    m_CellsToExploreCount += alreadyExploredCellsCount;

                    foreach (var ex_cell in exploreCellsInArea)
                        ex_cell.Explored = false;
                }
            }
        }

        public virtual void Reset()
        {
            Clear();

            // generate exploration data from already existing grid cells
            HashSet<GridCell> grid_cells_copy;
            using (m_Navmesh.AcquireReadDataLock())
                grid_cells_copy = new HashSet<GridCell>(m_Navmesh.m_GridCells);

            foreach (GridCell g_cell in grid_cells_copy)
                OnGridCellAdded(g_cell);
        }

        public virtual float GetExploredPercent()
        {
            // need to run expensive version when exploration is disabled and number of explore cells has changed (because numbers used are not refreshed then)
            if (!Enabled && m_ForceRefreshExploredPercent)
            {
                m_ForceRefreshExploredPercent = false;

                ExploreCell current_explore_cell;
                using (new ReadLock(DataLock))
                    current_explore_cell = GetCurrentExploreCell();

                if (current_explore_cell == null)
                    return 100;

                GetUnexploredCells(current_explore_cell);
            }
        
            return m_CellsToExploreCount > 0 ? (float)Math.Round(m_ExploredCellsCount / (float)m_CellsToExploreCount * 100, 1) : 0;
        }

        public virtual void Dispose()
        {
            m_ShouldStopUpdates = true;
            UpdatesThread.Join();

            m_Navmesh.RemoveObserver(this);
            m_Navigator.RemoveObserver(this);
        }

        // Extension will be automatically added
        public void Serialize(string name)
        {
            using (BinaryWriter w = new BinaryWriter(File.OpenWrite(name + ".explorer")))
            {
                OnSerialize(w);
            }
        }

        protected virtual void OnSerialize(BinaryWriter w)
        {
            using (new ReadLock(DataLock))
            using (new ReadLock(InputLock))
            using (m_Navmesh.AcquireReadDataLock())
            {
                w.Write(m_Enabled);

                // write all cells global IDs
                w.Write(m_ExploreCells.Count);

                foreach (ExploreCell explore_cell in m_ExploreCells)
                    w.Write(explore_cell.GlobalId);

                foreach (ExploreCell explore_cell in m_ExploreCells)
                    explore_cell.Serialize(w);

                w.Write(m_DestCell?.GlobalId ?? -1);
                w.Write(ExploreCell.LastExploreCellGlobalId);

                w.Write(m_ExploredCellsCount);
                w.Write(m_CellsToExploreCount);

                ExploreConstraint.Serialize(w);
                //w.Write(IsContraintExplored);

                m_HintPos.Serialize(w);
            }
        }

        // Extension will be automatically added
        public void Deserialize(string name)
        {
            using (BinaryReader r = new BinaryReader(File.OpenRead(name + ".explorer")))
            using (m_Navmesh.AcquireReadDataLock())
            {                
                OnDeserialize(m_Navmesh.m_AllCells, m_Navmesh.m_IdToCell, r);
            }
        }

        protected virtual void OnDeserialize(HashSet<Cell> all_cells, Dictionary<int, Cell> id_to_cell, BinaryReader r)
        {
            using (new WriteLock(DataLock))
            using (new WriteLock(InputLock))
            {
                m_ExploreCells.Clear();

                m_Enabled = r.ReadBoolean();

                int explore_cells_count = r.ReadInt32();

                // pre-allocate explore cells
                for (int i = 0; i < explore_cells_count; ++i)
                {
                    ExploreCell explore_cell = new ExploreCell();
                    explore_cell.GlobalId = r.ReadInt32();
                    m_ExploreCells.Add(explore_cell);
                }

                foreach (ExploreCell explore_cell in m_ExploreCells)
                    explore_cell.Deserialize(m_ExploreCells, all_cells, id_to_cell, r);

                var dest_cell_global_id = r.ReadInt32();
                if (dest_cell_global_id >= 0)
                    m_DestCell = m_ExploreCells.FirstOrDefault(x => x.GlobalId == dest_cell_global_id);

                ExploreCell.LastExploreCellGlobalId = r.ReadInt32();

                m_ExploredCellsCount = r.ReadInt32();
                m_CellsToExploreCount = r.ReadInt32();

                ExploreConstraint.Deserialize(r);
                //IsContraintExplored = r.ReadBoolean();

                m_HintPos = new Vec3(r);
            }
        }

        public virtual void OnHugeCurrentPosChange()
        {
            RequestReevaluation();
            m_ForceRefreshExploredPercent = true;
        }

        internal virtual ExploreCell GetDestinationCell(ExploreCell curr_explore_cell)
        {
            return null;
        }

        internal virtual bool IsDataAvailable
        {
            get { return m_ExploreCells.Count > 0; }
        }
        
        public bool Enabled
        {
            get
            {
                return m_Enabled;
            }

            set
            {
                if (m_Enabled == value)
                    return;

                m_Enabled = CanBeEnabled() && value;

                if (value)
                    RequestReevaluation();
                else
                    m_Navigator.ClearDestination(DestType.Explore);
            }
        }

        // Operations on this list are not thread safe! Use dbg_ReadLockGridCells to make it thread safe
        public HashSet<ExploreCell> dbg_GetExploreCells()
        {
            return m_ExploreCells;
        }

        public ReadLock AquireReadDataLock()
        {
            return new ReadLock(DataLock);
        }

        protected virtual bool CanBeEnabled() { return true; }

        protected Navmesh m_Navmesh;
        protected NavigationEngine m_Navigator;

        internal void RequestReevaluation()
        {
            m_ForceReevaluation = true;
        }

        public void OnDestinationReached(destination dest)
        {
            if (dest.type != DestType.Explore)
                return;

            ExploreCell dest_cell = dest.user_data as ExploreCell;

            //Trace.WriteLine($"marking explore cell id {dest_cell?.GlobalId ?? -1} as explored");

            if (dest_cell != null)
            {
                Trace.WriteLine($"Explore cell GID {m_DestCell.GlobalId} considered explored because it was reached.");
                OnCellExplored(dest_cell);
            }

            SelectNewDestinationCell(dest_cell);
        }

        private void SelectNewDestinationCell(ExploreCell curr_explore_cell)
        {
            //using (new Profiler("SelectNewDestinationCell [%t]"))
            {
                var prev_dest_cell = m_DestCell;
                m_DestCell = GetDestinationCell(curr_explore_cell);

                //Trace.WriteLine($"dest explore cell id {m_DestCell?.GlobalId ?? -1} pos {GetDestinationCellPosition()}");

                //using (new Profiler("set explore cell pos as dest [%t]"))
                if (Enabled)
                    m_Navigator.SetDestination(new destination(GetDestinationCellPosition(), DestType.Explore, ExploreDestPrecision, user_data: m_DestCell, stop: false));

                // force reevaluation so connectivity check is performed on selected cell (it is cheaper than checking connectivity for all unexplored cells)
                m_ValidateDestCell = m_DestCell != prev_dest_cell;
            }
        }

        public void OnDestinationReachFailed(destination dest)
        {
            OnDestinationReached(dest);
        }

        public virtual void OnGridCellAdded(GridCell grid_cell)
        {
            using (new ReadLock(DataLock, true))
            {
                // remove explore cells overlapping with grid cell
                var cells_to_validate = m_ExploreCells.Where(x => x.Overlaps2D(grid_cell)).ToList();

                using (new WriteLock(DataLock))
                {
                    foreach (ExploreCell explore_cell in cells_to_validate)
                    {
                        explore_cell.Detach();
                        m_ExploreCells.Remove(explore_cell);
                    }
                }

                // check if new explore cells should be added
                int x_min = (int)Math.Floor(grid_cell.Min.X / ExploreCellSize);
                int y_min = (int)Math.Floor(grid_cell.Min.Y / ExploreCellSize);

                int x_max = (int)Math.Ceiling(grid_cell.Max.X / ExploreCellSize);
                int y_max = (int)Math.Ceiling(grid_cell.Max.Y / ExploreCellSize);

                int explore_cells_generated = 0;

                for (int y_index = y_min; y_index < y_max; ++y_index)
                {
                    for (int x_index = x_min; x_index < x_max; ++x_index)
                    {
                        AABB cell_aabb = new AABB(x_index * ExploreCellSize, y_index * ExploreCellSize, -10000,
                                                  (x_index + 1) * ExploreCellSize, (y_index + 1) * ExploreCellSize, 10000);

                        //using (new Profiler("[Nav] Explore cells generated [%t]"))
                        {
                            explore_cells_generated += GenerateExploreCells(cell_aabb);
                        }
                    }
                }

                //if (explore_cells_generated > 0)
                //    m_Navmesh.Log("[Nav] " + explore_cells_generated + " explore cell(s) generated");
            }
        }

        public virtual void OnNavDataChanged(AABB affected_area)
        {
            RequestReevaluation();
        }

        public virtual void OnNavDataCleared()
        {
            Clear();
        }

        protected virtual void OnCellExplored(ExploreCell cell)
        {
            if (cell == null)
                return;

            cell.Explored = true; // this is safe as explore cells cannot be added/removed now
            m_Navmesh.Log("[Nav] Explored cell " + cell.GlobalId + " [progress: " + GetExploredPercent() + "%]!");
        }

        public virtual bool IsExplored()
        {
            if (!IsDataAvailable)
                return false;

            return GetExploredPercent() >= 100;
        }

        protected ExploreCell GetCurrentExploreCell()
        {
            Vec3 curr_pos = m_Navigator.CurrentPos;

            ExploreCell current_explore_cell = m_ExploreCells.FirstOrDefault(x => x.CellsContains2D(curr_pos));

            if (current_explore_cell == null)
            {
                // try explore cells containing current position first
                var potential_explore_cells = m_ExploreCells.Where(x => x.Contains2D(curr_pos));

                if (potential_explore_cells == null)
                    potential_explore_cells = m_ExploreCells; // find nearest explore cell

                if (potential_explore_cells != null)
                {
                    if (potential_explore_cells.Count() == 1)
                        current_explore_cell = potential_explore_cells.First();
                    else
                    {
                        float min_dist = float.MaxValue;

                        foreach (var explore_cell in potential_explore_cells)
                        {
                            float dist = explore_cell.Cells.Select(x => x.Distance(curr_pos)).Min();

                            if (dist < min_dist)
                            {
                                current_explore_cell = explore_cell;
                                min_dist = dist;
                            }
                        }
                    }
                }
            }

            return current_explore_cell;
        }

        private class UnexploredSelector : Algorihms.IVisitor<ExploreCell>
        {
            public UnexploredSelector(AABB contraint)
            {
                this.constraint = contraint;
            }

            public void Visit(ExploreCell cell)
            {
                if (!constraint.IsZero() && !cell.AABB.Overlaps2D(constraint))
                    return;

                ++all_cells_count;

                if (!cell.Explored)
                    unexplored_cells.Add(cell);
            }

            public AABB constraint;
            public int all_cells_count = 0;
            public HashSet<ExploreCell> unexplored_cells = new HashSet<ExploreCell>();
        }

        protected HashSet<ExploreCell> GetUnexploredCells(ExploreCell origin_cell)
        {
            using (new ReadLock(DataLock))
            {
                UnexploredSelector selector = new UnexploredSelector(ExploreConstraint);
                Algorihms.Visit<ExploreCell>(origin_cell, MovementFlag.None, -1, null, selector);

                m_CellsToExploreCount = selector.all_cells_count;
                m_ExploredCellsCount = m_CellsToExploreCount - selector.unexplored_cells.Count;

                // first try all big cells unvisited by anyone (undelayed)
                HashSet<ExploreCell> unexplored_cells_subset = new HashSet<ExploreCell>(selector.unexplored_cells.Where(x => !x.Small && !x.Delayed));

                if (unexplored_cells_subset.Count > 0)
                    return unexplored_cells_subset;

                // try all big cells unvisited by me 
                unexplored_cells_subset = new HashSet<ExploreCell>(selector.unexplored_cells.Where(x => !x.Small));

                if (unexplored_cells_subset.Count > 0)
                    return unexplored_cells_subset;

                // try all remaining cells
                return selector.unexplored_cells;
            }
        }

        private void Updates()
        {
            long last_update_time = 0;

            Stopwatch timer = new Stopwatch();
            timer.Start();

            while (!m_ShouldStopUpdates)
            {
                long time = timer.ElapsedMilliseconds;

                if (m_ForceReevaluation || (m_UpdateExplorationInterval > 0 && (time - last_update_time) > m_UpdateExplorationInterval) || (Enabled && m_Navigator.GetDestinationType() < DestType.Explore))
                {
                    last_update_time = time;

                    lock (UpdateLocker)
                    {
                        //using (new Profiler("[Nav] Nav updated [%t]"))
                        UpdateExploration(false);
                    }

                    m_ForceReevaluation = false;
                }
                else
                    Thread.Sleep(50);
            }
        }

        private object UpdateLocker = new object();

        // Controls updated thread execution
        private volatile bool m_ShouldStopUpdates = false;

        private void UpdateExploration(bool ignore_explored)
        {
            Vec3 current_pos = m_Navigator.CurrentPos;

            if (current_pos.IsZero())
                return;

            if (!IsDataAvailable)
            {
                m_Navmesh.Log("[Nav] Exploration data unavailable!");
                return;
            }

            if (!ignore_explored && IsExplored())
            {
                m_Navmesh.Log("[Nav] Exploration finished!");
                m_Navigator.ClearDestination(DestType.Explore);
                return;
            }

            if (m_DestCell != null)
            {
                bool mark_dest_cell_as_explored = false;
                // perform connection check only when reevaluation is forced (usually due to navigation data change)
                bool is_dest_cell_connected = (!m_ForceReevaluation && !m_ValidateDestCell) || m_Navmesh.AreConnected(GetDestinationCellPosition(), current_pos, MovementFlag.Walk, ExploreDestPrecision, 0, out var unused1, out var unused2);

                // delay exploration of currently unconnected explore cells, unless they are already delayed (mark them as explored in that case)
                if (!is_dest_cell_connected)
                {
                    if (!m_DestCell.Delayed)
                    {
                        m_DestCell.Delayed = true;
                        m_ForceReevaluation = true; // this is a change to find another un-delayed explore cell
                    }
                    else
                    {
                        // looks like there are no better cells to visit, and there is no way to reach this one... so sorry but we have to mark it as explored
                        mark_dest_cell_as_explored = true;
                    }
                }

                m_ValidateDestCell = false;

                // mark destination cell as explored when external function says so or destination cell is no longer connected (mostly due to nav blocker)
                mark_dest_cell_as_explored |= AlternativeExploredCondition?.Invoke(m_DestCell, current_pos) ?? false;

                if (mark_dest_cell_as_explored)
                {
                    Trace.WriteLine($"Explore cell GID {m_DestCell.GlobalId} considered explored by external logic.");
                    OnCellExplored(m_DestCell);
                }
            }

            using (new ReadLock(DataLock, true))
            {
                if ((Enabled && m_Navigator.GetDestinationType() < DestType.Explore) || (m_DestCell?.Explored ?? false) || m_ForceReevaluation)
                {
                    SelectNewDestinationCell(null);
                    //m_Navmesh.Log("[Nav] Explore dest changed.");
                }

                // mark cells as explored when passing by close enough
                ExploreCell current_explore_cell = m_ExploreCells.FirstOrDefault(x => !x.Explored && x.Position.Distance2D(current_pos) < ExploreDestPrecision);

                if (current_explore_cell != null)
                {
                    Trace.WriteLine($"Explore cell GID {current_explore_cell.GlobalId} considered explored because of passing by.");
                    OnCellExplored(current_explore_cell);
                }

                OnUpdateExploration();
            }
        }

        // DataLock is already acquired in read-upgradeable state
        protected virtual void OnUpdateExploration()
        {
        }

        private void Add(ExploreCell explore_cell)
        {
            using (m_Navmesh.AcquireReadDataLock())
            using (new WriteLock(DataLock))
            {
                foreach (ExploreCell e_cell in m_ExploreCells)
                {
                    e_cell.AddNeighbour(explore_cell);
                }

                m_ExploreCells.Add(explore_cell);
                m_ForceRefreshExploredPercent = true;
            }
        }

        // assume Navmesh data lock is aquired
        private int GenerateExploreCells(AABB cell_aabb)
        {
            // should not happen
            //if (m_ExploreCells.Exists(x => x.AABB.Equals(cell_aabb)))
            //    return;

            MovementFlag movement_flags = m_Navigator.MovementFlags;

            //using (new Profiler("[Nav] Nav data generated [%t]"))
            //using (m_Navmesh.AquireReadDataLock())
            {
                List<Cell> cells = new List<Cell>();

                // i'm interested in unique list
                HashSet<int> tmp_overlapping_grid_cells = new HashSet<int>();

                // find all cells inside cell_aabb
                using (m_Navmesh.AcquireReadDataLock())
                {
                    foreach (GridCell grid_cell in m_Navmesh.m_GridCells.Where(x => x.AABB.Overlaps2D(cell_aabb)))
                    {
                        tmp_overlapping_grid_cells.Add(grid_cell.Id);

                        cells.AddRange(grid_cell.GetCells(x => x.HasFlags(movement_flags) && cell_aabb.Overlaps2D(x.AABB), false));
                    }
                }

                // for traversing purposes list will be faster
                List<int> overlapping_grid_cells = tmp_overlapping_grid_cells.ToList();

                //create ExploreCell for each interconnected group of cells
                HashSet<Cell> cells_copy = new HashSet<Cell>(cells);
                int last_explore_cells_count = m_ExploreCells.Count;

                while (cells_copy.Count > 0)
                {
                    HashSet<Cell> visited = new HashSet<Cell>();

                    using (m_Navmesh.AcquireReadDataLock())
                        Algorihms.Visit(cells_copy.First(), ref visited, movement_flags, true, 1, -1, cells_copy);

                    List<AABB> intersections = new List<AABB>();
                    AABB intersections_aabb = new AABB();

                    AABB intersection = default(AABB);

                    foreach (Cell c in visited)
                    {
                        if (cell_aabb.Intersect(c.AABB, ref intersection))
                        {
                            intersections.Add(intersection);
                            intersections_aabb = intersections_aabb.Extend(intersection);
                        }
                    }

                    ExploreCell ex_cell = new ExploreCell(cell_aabb, visited.ToList(), overlapping_grid_cells, m_LastExploreCellId++);
                    Add(ex_cell);

                    ex_cell.Small = (ex_cell.CellsArea < MaxAreaToMarkAsSmall) || (ex_cell.CellsAABB.Dimensions.Max() < MaxDimensionToMarkAsSmall);

                    cells_copy.RemoveWhere(x => visited.Contains(x));
                }

                return m_ExploreCells.Count - last_explore_cells_count;
            }
        }

        internal bool GetCellAt(Vec3 p, out ExploreCell result_cell)
        {
            using (new ReadLock(DataLock))
            {
                result_cell = null;

                if (p.IsZero())
                    return false;

                foreach (var explore_cell in m_ExploreCells)
                {
                    if (explore_cell.Contains2D(p))
                    {
                        var cells = explore_cell.GetCellsAt(p, test_2d: true, z_tolerance: 0);

                        if (cells.Count() > 0)
                        {
                            result_cell = explore_cell;
                            return true;
                        }
                    }
                }

                return false;
            }
        }

        public bool FindRoughPath(Vec3 from, Vec3 to, ref List<Vec3> path)
        {
            if (from.IsZero() || to.IsZero())
                return false;

            List<path_pos> tmp_path = new List<path_pos>();

            bool start_on_nav_mesh = GetCellAt(from, out ExploreCell start);
            bool end_on_nav_mesh = GetCellAt(to, out ExploreCell end);

            using (new ReadLock(DataLock))
                Algorihms.FindPath(start, from, new Algorihms.DestinationPathFindStrategy<ExploreCell>(to, end), MovementFlag.None, ref tmp_path, use_cell_centers: true);

            path = tmp_path.Select(x => x.pos).ToList();
            return true;
        }

        public float GetRoughPathRecalcPrecision()
        {
            return ExploreCellSize * 1;
        }

        // Position toward which exploration should be conducted
        public Vec3 HintPos
        {
            get
            {
                using (new ReadLock(InputLock))
                    return new Vec3(m_HintPos);
            }

            set
            {
                using (new WriteLock(InputLock))
                    m_HintPos = value;
            }
        }

        public bool IsDestinationCellSmall()
        {
            return m_DestCell != null && m_DestCell.Small;
        }

        public Vec3 GetDestinationCellPosition()
        {
            var dest_cell = m_DestCell;
            if (dest_cell != null)
            {
                var constraint = m_ExploreConstraint;
                if (constraint.IsZero())
                    return dest_cell.Position;
                var pos_in_contraint = constraint.Align(dest_cell.Position);
                return new Vec3(pos_in_contraint.X, pos_in_contraint.Y, dest_cell.Position.Z);
            }
            return Vec3.ZERO;
        }

        private bool m_Enabled = true;
        private int m_LastExploreCellId = 0;

        protected HashSet<ExploreCell> m_ExploreCells = new HashSet<ExploreCell>(); //@ DataLock
        protected int m_ExploredCellsCount = 0;
        protected int m_CellsToExploreCount = 0;
        private bool m_ForceRefreshExploredPercent = true;
        private AABB m_ExploreConstraint = default;

        private Thread UpdatesThread = null;        

        private volatile bool m_ForceReevaluation = false;
        private bool m_ValidateDestCell = false;
        protected int m_UpdateExplorationInterval = 300;

        private ReaderWriterLockSlim InputLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        protected ReaderWriterLockSlim DataLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);

        protected ExploreCell m_DestCell;
        private Vec3 m_HintPos = Vec3.ZERO; //@ InputLock
    }
}
