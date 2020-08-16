using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.IO;
using System.Linq;

namespace Nav
{
    public abstract class ExplorationEngine : IDisposable, INavigationObserver, INavmeshObserver
    {
        public ExplorationEngine(Navmesh navmesh, NavigationEngine navigator, int explore_cell_size = 90)
        {
            ExploreCellSize = explore_cell_size;
            
            m_Navmesh = navmesh;
            m_Navmesh.AddObserver(this);

            m_Navigator = navigator;
            m_Navigator.AddObserver(this);

            // generate exploration data from already existing grid cells
            Reset();

            UpdatesThread = new Thread(Updates);
            UpdatesThread.Name = "Explorator-UpdatesThread";
            UpdatesThread.Start();
        }

        // called every update when heading towards an explore cell. parameters are destination explore cell and current position
        public Func<ExploreCell, Vec3, bool> AlternativeExploredCondition { get; set; }

        public float MaxAreaToMarkAsSmall { get; set; } = 50 * 50;

        // precision with explore destination will be accepted as reached
        public float ExploreDestPrecision { get; set; } = 20;

        // change it via ChangeExploreCellSize function
        public int ExploreCellSize { get; private set; }

        public void ChangeExploreCellSize(int size)
        {
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

        public virtual void Reset()
        {
            Clear();

            // generate exploration data from already existing grid cells
            using (m_Navmesh.AcquireReadDataLock())
            {
                foreach (GridCell g_cell in m_Navmesh.m_GridCells)
                    OnGridCellAdded(g_cell);
            }
        }

        public virtual float GetExploredPercent()
        {
            return (float)Math.Round(m_ExploredCellsCount / (float)m_CellsToExploreCount * 100, 1);
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

                m_HintPos.Serialize(w);
            }
        }

        // Extension will be automatically added
        public void Deserialize(string name)
        {
            using (BinaryReader r = new BinaryReader(File.OpenRead(name + ".explorer")))
            using (m_Navmesh.AcquireReadDataLock())
            {                
                OnDeserialize(m_Navmesh.m_AllCells, r);
            }
        }

        protected virtual void OnDeserialize(HashSet<Cell> all_cells, BinaryReader r)
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
                    explore_cell.Deserialize(m_ExploreCells, all_cells, r);

                var dest_cell_global_id = r.ReadInt32();
                if (dest_cell_global_id >= 0)
                    m_DestCell = m_ExploreCells.FirstOrDefault(x => x.GlobalId == dest_cell_global_id);

                ExploreCell.LastExploreCellGlobalId = r.ReadInt32();

                m_HintPos = new Vec3(r);
            }
        }

        public virtual void OnHugeCurrentPosChange()
        {
            RequestReevaluation();
        }

        internal virtual ExploreCell GetDestinationCell()
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

        public void OnDestinationReached(DestType type, Vec3 dest)
        {
            if (type != DestType.Explore)
                return;

            ExploreCell dest_cell = m_ExploreCells.FirstOrDefault(x => x.Position.Equals(dest));

            if (dest_cell != null)
                OnCellExplored(dest_cell);

            m_DestCell = GetDestinationCell();
            m_Navigator.SetDestination(GetDestinationCellPosition(), DestType.Explore, ExploreDestPrecision);
        }

        public void OnDestinationReachFailed(DestType type, Vec3 dest)
        {
            OnDestinationReached(type, dest);
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

                if (explore_cells_generated > 0)
                    m_Navmesh.Log("[Nav] " + explore_cells_generated + " explore cell(s) generated");
            }
        }

        public virtual void OnNavDataChanged()
        {
            RequestReevaluation();
        }

        public virtual void OnNavDataCleared()
        {
            Clear();
        }

        protected virtual void OnCellExplored(ExploreCell cell)
        {
            cell.Explored = true; // this is safe as explore cells cannot be added/removed now
            m_Navmesh.Log("[Nav] Explored cell " + cell.GlobalId + " [progress: " + GetExploredPercent() + "%]!");
        }

        public virtual bool IsExplored()
        {
            using (new ReadLock(DataLock))
            {
                if (!IsDataAvailable)
                    return false;

                ExploreCell current_explore_cell = GetCurrentExploreCell();

                if (current_explore_cell == null)
                    return true;

                if (!current_explore_cell.Explored)
                    return false;

                return GetUnexploredCells(current_explore_cell).Count == 0;
            }
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
            public void Visit(ExploreCell cell)
            {
                ++all_cells_count;

                if (!cell.Explored)
                    unexplored_cells.Add(cell);
            }

            public int all_cells_count = 0;
            public HashSet<ExploreCell> unexplored_cells = new HashSet<ExploreCell>();
        }

        protected HashSet<ExploreCell> GetUnexploredCells(ExploreCell origin_cell)
        {
            using (new ReadLock(DataLock))
            {
                UnexploredSelector selector = new UnexploredSelector();
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

                if (Enabled && (m_ForceReevaluation || (m_UpdateExplorationInterval > 0 && (time - last_update_time) > m_UpdateExplorationInterval) || m_Navigator.GetDestinationType() < DestType.Explore))
                {
                    last_update_time = time;
                    
                    //using (new Profiler("[Nav] Nav updated [%t]"))
                    UpdateExploration();

                    m_ForceReevaluation = false;
                }
                else
                    Thread.Sleep(50);
            }
        }

        // Controls updated thread execution
        private volatile bool m_ShouldStopUpdates = false;

        private void UpdateExploration()
        {
            Vec3 current_pos = m_Navigator.CurrentPos;

            if (current_pos.IsZero())
                return;

            if (!IsDataAvailable)
            {
                m_Navmesh.Log("[Nav] Exploration data unavailable!");
                return;
            }

            if (IsExplored())
            {
                m_Navmesh.Log("[Nav] Exploration finished!");
                m_Navigator.ClearDestination(DestType.Explore);
                return;
            }

            using (new ReadLock(DataLock, true))
            {
                if (m_Navigator.GetDestinationType() < DestType.Explore || (m_DestCell?.Explored ?? false) || m_ForceReevaluation)
                {
                    m_DestCell = GetDestinationCell();
                    m_Navigator.SetDestination(GetDestinationCellPosition(), DestType.Explore, ExploreDestPrecision);
                    //m_Navmesh.Log("[Nav] Explore dest changed.");
                }

                if (m_DestCell != null && (AlternativeExploredCondition?.Invoke(m_DestCell, current_pos) ?? false))
                    OnCellExplored(m_DestCell);

                // mark cells as explored when passing by close enough
                ExploreCell current_explore_cell = m_ExploreCells.FirstOrDefault(x => !x.Explored && x.Position.Distance2D(current_pos) < ExploreDestPrecision);

                if (current_explore_cell != null)
                    OnCellExplored(current_explore_cell);

                OnUpdateExploration();
            }
        }

        // DataLock is already aquired in read-upgradeable state
        protected virtual void OnUpdateExploration()
        {
        }

        private void Add(ExploreCell explore_cell)
        {
            using (new WriteLock(DataLock))
            {
                foreach (ExploreCell e_cell in m_ExploreCells)
                {
                    e_cell.AddNeighbour(explore_cell);
                }

                m_ExploreCells.Add(explore_cell);
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

                    Algorihms.Visit(cells_copy.First(), ref visited, movement_flags, true, 1, -1, cells_copy);

                    List<AABB> intersections = new List<AABB>();
                    AABB intersections_aabb = new AABB();

                    AABB intersection = default(AABB);

                    foreach (Cell c in visited)
                    {
                        if (cell_aabb.Intersect(c.AABB, ref intersection))
                        {
                            intersections.Add(intersection);
                            intersections_aabb.Extend(intersection);
                        }
                    }

                    Vec3 nearest_intersection_center = Vec3.ZERO;
                    float nearest_intersection_dist = float.MaxValue;

                    foreach (AABB inter_aabb in intersections)
                    {
                        float dist = inter_aabb.Center.Distance2D(intersections_aabb.Center);

                        if (dist < nearest_intersection_dist)
                        {
                            nearest_intersection_center = inter_aabb.Center;
                            nearest_intersection_dist = dist;
                        }
                    }

                    ExploreCell ex_cell = new ExploreCell(cell_aabb, visited.ToList(), overlapping_grid_cells, nearest_intersection_center, m_LastExploreCellId++);
                    Add(ex_cell);

                    ex_cell.Small = (ex_cell.CellsArea() < MaxAreaToMarkAsSmall);

                    cells_copy.RemoveWhere(x => visited.Contains(x));
                }

                return m_ExploreCells.Count - last_explore_cells_count;
            }
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
            return m_DestCell != null ? m_DestCell.Position : Vec3.ZERO;
        }

        private bool m_Enabled = true;
        private int m_LastExploreCellId = 0;

        protected HashSet<ExploreCell> m_ExploreCells = new HashSet<ExploreCell>(); //@ DataLock
        protected int m_ExploredCellsCount = 0;
        protected int m_CellsToExploreCount = 0;
        
        private Thread UpdatesThread = null;        

        private volatile bool m_ForceReevaluation = false;
        protected int m_UpdateExplorationInterval = 300;

        private ReaderWriterLockSlim InputLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        protected ReaderWriterLockSlim DataLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);

        protected ExploreCell m_DestCell;
        private Vec3 m_HintPos = Vec3.ZERO; //@ InputLock
    }
}
