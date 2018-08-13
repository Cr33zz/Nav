using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.IO;
using System.Diagnostics;

namespace Nav
{
    // the higher value the more important destination
    public enum DestType
    {
        None = 0x0000,
        Explore = 0x0001,
        Waypoint = 0x0002,
        Grid = 0x0004,
        Custom = 0x0008, // same as user but not cleared automatically
        User = 0x0010,
        BackTrack = 0x0020, // used for moving along historical destinations
        RunAway = 0x0040, // not used yet
        All = 0xFFFF,
    }

    public class NavigationEngine : IDisposable, INavmeshObserver
    {
        public NavigationEngine(Navmesh navmesh)
        {
            m_Navmesh = navmesh;
            m_Navmesh.AddObserver(this);

            UpdatesThread = new Thread(Updates);
            UpdatesThread.Name = "Navigator-UpdatesThread";
            UpdatesThread.Start();
        }

        public void AddObserver(INavigationObserver observer)
        {
            using (new WriteLock(InputLock))
            {
                if (m_Observers.Contains(observer))
                    return;

                m_Observers.Add(observer);
            }
        }

        public void RemoveObserver(INavigationObserver observer)
        {
            using (new WriteLock(InputLock))
                m_Observers.Remove(observer);
        }

        // defines how user can move through navmesh
        public MovementFlag MovementFlags { get; set; } = MovementFlag.Walk;

        // precision with each path node will be accepted as reached
        public float DefaultPrecision { get; set; } = 10;

        public float Precision { get; set; }

        // precision with grid destination will be accepted as reached
        public float GridDestPrecision { get; set; } = 40;

        // how much path will be randomized 0 by default
        public float PathRandomCoeff { get; set; } = 0;

        public float DistToKeepFromEdge { get; set; } = 0; // not supported

        // only initial part of path of this length will be moved away from edges (to minimize performance impact if there are frequent enough path recalculates)
        public float KeepAwayFromEdgesOnInitialLength { get; set; } = 50; // not supported

        public float KeepFromEdgePrecision { get; set; } = 5;

        // each point on path will be offseted in direction from previous point so bot will move along path more precisely even with high precision parameter
        public float PathNodesShiftDist { get; set; } = 10;

        // when new CurrentPos differ from last one by more than this value path update will be automatically requested
        public float CurrentPosDiffRecalcThreshold { set; get; } = 15;

        // path will be automatically recalculated with this interval (milliseconds)
        public int UpdatePathInterval { get; set; } = -1;

        public float PathSmoothingPrecision { get; set; } = 3;

        public bool EnableAntiStuck { get; set; } = false;

        // should be used when EnableAntiStuck is true to notify navigator that actor is not blocked by some obstacle but just standing
        public bool IsStandingOnPurpose { get; set; } = true;

        public List<int> DestinationGridsId
        {
            get
            {
                using (new ReadLock(InputLock))
                    return new List<int>(m_DestinationGridsId);
            }

            set
            {
                using (new WriteLock(InputLock))
                {
                    if (value == null)
                        m_DestinationGridsId.Clear();
                    else
                        m_DestinationGridsId = new List<int>(value);
                }
            }
        }

        public bool FindPath(Vec3 from, Vec3 to, ref List<Vec3> path, bool as_close_as_possible)
        {
            using (new Profiler("[Nav-Profiler] FindPath updated [%t]", 500))
            return FindPath(from, to, MovementFlags, ref path, PATH_NODES_MERGE_DISTANCE, as_close_as_possible, false, m_PathRandomCoeffOverride > 0 ? m_PathRandomCoeffOverride : PathRandomCoeff, m_PathBounce, PathNodesShiftDist);
        }

        public bool FindPath(Vec3 from, Vec3 to, MovementFlag flags, ref List<Vec3> path, float merge_distance = -1, bool as_close_as_possible = false, bool include_from = false, float random_coeff = 0, bool bounce = false, float shift_nodes_distance = 0, bool smoothen = true)
        {
            using (m_Navmesh.AcquireReadDataLock())
            {
                List<path_pos> tmp_path = new List<path_pos>();

                if (from.IsZero() || to.IsZero())
                    return false;

                Cell start = null;
                Cell end = null;

                bool start_on_nav_mesh = m_Navmesh.GetCellContaining(from, out start, flags, false, as_close_as_possible, -1, false, 2, null);
                bool end_on_nav_mesh = m_Navmesh.GetCellContaining(to, out end, flags, false, as_close_as_possible, -1, false, 2, null);

                if (bounce)
                {
                    Vec3 bounce_dir = start.AABB.GetBounceDir2D(from);
                    Vec3 new_from = from + bounce_dir * 10;
                    m_Navmesh.GetCellContaining(new_from, out start, flags, false, as_close_as_possible, -1, false, 2, null);

                    if (!Algorihms.FindPath<Cell>(start, ref end, new_from, to, flags, ref tmp_path, random_coeff, true))
                        return false;

                    tmp_path.Insert(0, new path_pos(start.AABB.Align(from), start));
                }
                else
                {
                    if (!Algorihms.FindPath<Cell>(start, ref end, from, to, flags, ref tmp_path, random_coeff, true))
                        return false;
                }

                if (smoothen && random_coeff == 0)
                    SmoothenPath(ref tmp_path, flags, bounce ? 1 : 0);

                if (DistToKeepFromEdge > 0 && random_coeff == 0)
                    KeepAwayFromEdges(ref tmp_path, flags);

                path = tmp_path.Select(x => x.pos).ToList();

                PostProcessPath(ref path, merge_distance, shift_nodes_distance);

                if (!include_from && start_on_nav_mesh)
                    path.RemoveAt(0);

                return true;
            }
        }

        private void SmoothenPath(ref List<path_pos> path, MovementFlag flags, int skip_first_count = 0)
        {
            int ray_start_index = skip_first_count;
            Vec3 intersection = default(Vec3);

            while (ray_start_index + 2 < path.Count)
            {
                path_pos ray_start_data = path[ray_start_index];
                path_pos intermediate_data = path[ray_start_index + 1];
                path_pos ray_end_data = path[ray_start_index + 2];

                // try remove middle point completely
                if (m_Navmesh.RayCast2D(ray_start_data.pos, ray_end_data.pos, flags, ref intersection, false))
                    path.RemoveAt(ray_start_index + 1);
                else
                    ++ray_start_index;
            }

            // perform second smoothing pass after getting rid of all unnecessary points to get better performance and results
            // this pass is basically "moving" from point to point and checking at which step (in-between points) I can already see the next point
            if (PathSmoothingPrecision > 0)
            {
                ray_start_index = skip_first_count;

                while (ray_start_index + 2 < path.Count)
                {
                    path_pos ray_start_data = path[ray_start_index];
                    path_pos intermediate_data = path[ray_start_index + 1];
                    path_pos ray_end_data = path[ray_start_index + 2];

                    Vec3 dir = intermediate_data.pos - ray_start_data.pos;
                    float length = dir.Normalize();
                    int steps = (int)(length / PathSmoothingPrecision);

                    for (int i = 1; i < steps; ++i) // checking 0 is unnecessary since this is the current path
                    {
                        if (m_Navmesh.RayCast2D(ray_start_data.pos + dir * (float)i * PathSmoothingPrecision, ray_end_data.pos, flags, ref intersection, false))
                        {
                            path[ray_start_index + 1] = new path_pos(ray_start_data.pos + dir * (float)i * PathSmoothingPrecision, intermediate_data.cell);
                            break;
                        }
                    }

                    ++ray_start_index;
                }
            }
        }

        private void KeepAwayFromEdges(ref List<path_pos> path, MovementFlag flags)
        {
            //WIP

            //Vec3[] ray_dirs = new Vec3[4] { new Vec3(1, 0, 0), new Vec3(-1, 0, 0) , new Vec3(0, 1, 0) , new Vec3(0, -1, 0) };
            //Vec3[] intersections = new Vec3[4] { null, null, null, null };
            //float[] distances = new float[4] { 0, 0, 0, 0 };

            //for (int i = 0; i < path.Count - 1; ++i)
            //{
            //    Vec3 start = path[i].pos;
            //    Vec3 end = path[i + 1].pos;
            //    Vec3 dir = end - start;
            //    float length = dir.Length2D();
            //    Vec3 dir_2d = dir.Normalized2D();
            //    dir.Normalize();
            //    int steps = (int)(length / KeepFromEdgePrecision);
            //    Vec3 perp_dir = new Vec3(dir_2d.Y, -dir_2d.X, 0);

            //    for (int s = 0; s <= steps; ++s) // checking 0 is unnecessary since this is the current path
            //    {
            //        Vec3 shift_dir = null;
            //        float shift_dist = 0;

            //        Vec3 ray_start = start + dir * (float)s * KeepFromEdgePrecision;

            //        for (int r = 0; r < 4; ++r)
            //        {
            //            m_Navmesh.RayCast2D(ray_start, ray_start + ray_dirs[r] * 2 * DistToKeepFromEdge, flags, ref intersections[r]);
            //            distances[r] = ray_start.Distance(intersections[r]);
            //        }



            //        //Vec3 intersection_1 = null;
            //        //bool test_1 = m_Navmesh.RayCast2D(ray_start, ray_start + perp_dir * 2 * DistToKeepFromEdge, flags, ref intersection_1);
            //        //float dist_to_1 = ray_start.Distance(intersection_1);

            //        //Vec3 intersection_2 = null;
            //        //bool test_2 = m_Navmesh.RayCast2D(ray_start, ray_start - perp_dir * 2 * DistToKeepFromEdge, flags, ref intersection_2);
            //        //float dist_to_2 = ray_start.Distance(intersection_2);

            //        //if (dist_to_1 < dist_to_2 && dist_to_1 < DistToKeepFromEdge)
            //        //{
            //        //    shift_dir = -perp_dir;
            //        //    if (dist_to_2 > DistToKeepFromEdge + DistToKeepFromEdge - dist_to_1) // if there is enough room on the other side
            //        //        shift_dist = DistToKeepFromEdge - dist_to_1;
            //        //    else
            //        //        shift_dist = (dist_to_1 + dist_to_2) * 0.5f - dist_to_1; // move to the middle so the distance from both edges is the same
            //        //}
            //        //else if (dist_to_2 < dist_to_1 && dist_to_2 < DistToKeepFromEdge)
            //        //{
            //        //    shift_dir = perp_dir;
            //        //    if (dist_to_1 > DistToKeepFromEdge + DistToKeepFromEdge - dist_to_2) // if there is enough room on the other side
            //        //        shift_dist = DistToKeepFromEdge - dist_to_2;
            //        //    else
            //        //        shift_dist = (dist_to_1 + dist_to_2) * 0.5f - dist_to_2; // move to the middle so the distance from both edges is the same
            //        //}

            //        if (shift_dir != null)
            //        {
            //            if (s == 0 && i > 0)
            //            {
            //                path[i].pos = new Vec3(start + shift_dir * shift_dist);
            //            }
            //            else
            //            {
            //                path.Insert(i + 1, new path_pos(new Vec3(ray_start + shift_dir * shift_dist), path[i].cell));
            //                break;
            //            }
            //        }
            //    }
            //}
        }

        private void PostProcessPath(ref List<Vec3> path, float merge_distance, float shift_nodes_distance)
        {
            if (merge_distance > 0)
            {
                for (int i = 0; i < path.Count - 1; ++i)
                {
                    int start_count = path.Count;
                    Vec3 merge_point = path[i];

                    while (i < path.Count - 1 && path[i].Distance2D(path[i + 1]) < merge_distance)
                    {
                        merge_point = merge_point + path[i + 1];
                        path.RemoveAt(i + 1);
                    }

                    if (path.Count != start_count)
                        path[i] = merge_point / (float)(start_count - path.Count + 1);
                }
            }

            // shift points to increase movement accuracy
            if (shift_nodes_distance > 0)
            {
                for (int i = path.Count - 2; i > 0; --i)
                {
                    Vec3 dir_to_next = path[i] - path[i - 1];
                    dir_to_next.Normalize();

                    path[i] += dir_to_next * shift_nodes_distance;
                }
            }
        }

        public Vec3 Destination
        {
            get
            {
                using (new ReadLock(InputLock))
                    return new Vec3(m_Destination);
            }

            set
            {
                SetDestination(value, DestType.User, DefaultPrecision);
            }
        }

        public void SetDestination(Vec3 pos, float precision)
        {
            SetDestination(pos, DestType.User, precision > 0 ? precision : DefaultPrecision);
        }

        public void ClearAllDestinations()
        {
            ClearDestination(DestType.All);
        }

        public void ClearGridDestination()
        {
            ClearDestination(DestType.Grid);
        }

        public void SetCustomDestination(Vec3 pos, float precision = -1)
        {
            SetDestination(pos, DestType.Custom, precision < 0 ? DefaultPrecision : precision);
        }

        public void ClearCustomDestination()
        {
            ClearDestination(DestType.Custom);
        }

        public bool TryGetPath(ref List<Vec3> p, ref DestType p_dest_type)
        {
            if (PathLock.TryEnterReadLock(0))
            {
                p = new List<Vec3>(m_Path);
                p_dest_type = m_PathDestType;
                PathLock.ExitReadLock();
                return true;
            }

            return false;
        }

        public float TryGetPathLength(ref DestType p_dest_type)
        {
            float length = 0;

            if (PathLock.TryEnterReadLock(0))
            {
                for (int i = 0; i < m_Path.Count - 1; ++i)
                    length += m_Path[i].Distance2D(m_Path[i + 1]);
                p_dest_type = m_PathDestType;
                PathLock.ExitReadLock();
            }

            return length;
        }

        public bool TryGetBackTrackPath(ref List<Vec3> p)
        {
            if (InputLock.TryEnterReadLock(0))
            {
                p = new List<Vec3>(m_DestinationsHistory);
                p.Reverse();
                InputLock.ExitReadLock();
                return true;
            }

            return false;
        }

        public bool TryGetDebugPositionsHistory(ref List<Vec3> p)
        {
            if (InputLock.TryEnterReadLock(0))
            {
                p = new List<Vec3>(m_DebugPositionsHistory);
                InputLock.ExitReadLock();
                return true;
            }

            return false;
        }

        public List<Vec3> GetPath()
        {
            using (new ReadLock(PathLock))
                return new List<Vec3>(m_Path);
        }

        public DestType GetDestinationType()
        {
            return m_DestinationType;
        }

        public Vec3 GoToPosition
        {
            get
            {
                using (new ReadLock(PathLock))
                    return m_Path.Count > 0 ? new Vec3(m_Path[0]) : Vec3.ZERO;
            }
        }

        public Vec3 CurrentPos
        {
            get
            {
                using (new ReadLock(InputLock))
                    return new Vec3(m_CurrentPos);
            }

            set
            {
                bool was_empty = false;
                float diff = 0;

                using (new WriteLock(InputLock))
                {
                    if (!m_CurrentPos.Equals(value))
                    {
                        was_empty = m_CurrentPos.IsZero();
                        diff = m_CurrentPos.Distance2D(value);
                        m_CurrentPos = value;

                        if (m_DebugPositionsHistory.Count == 0 || value.Distance2D(m_DebugPositionsHistory.Last()) > 15)
                            m_DebugPositionsHistory.Add(value);

                        // add initial position as history destination
                        if (was_empty && m_DestinationsHistory.Count == 0)
                            m_DestinationsHistory.Add(value);
                    }
                }

                bool path_empty = false;
                bool huge_current_pos_change = was_empty || diff > CurrentPosDiffRecalcThreshold;

                // reduce locking when not necessary
                if (!huge_current_pos_change)
                {
                    //using (new ReadLock(PathLock))
                        path_empty = m_Path.Count == 0;
                }

                if (huge_current_pos_change || path_empty)
                {
                    if (huge_current_pos_change)
                        NotifyOnHugeCurrentPosChange();

                    RequestPathUpdate();
                }

                if (was_empty)
                    ReorganizeWaypoints();

                if (!path_empty)
                    UpdatePathProgression(value);
            }
        }

        public List<Vec3> Waypoints
        {
            get
            {
                using (new ReadLock(InputLock))
                    return new List<Vec3>(m_Waypoints);
            }

            set
            {
                using (new WriteLock(InputLock))
                    m_Waypoints = value;

                ReorganizeWaypoints();
            }
        }

        public bool BackTrackEnabled
        {
            get
            {
                return m_HistoryDestId >= 0;
            }

            set
            {
                using (new ReadLock(InputLock))
                {
                    // already enabled
                    if (value && m_HistoryDestId >= 0)
                        return;

                    m_HistoryDestId = (value ? m_DestinationsHistory.Count - 1 : -1);
                }

                if (!value)
                    ClearDestination(DestType.BackTrack);
            }
        }

        public GridCell GetCurrentGridCell()
        {
            using (new ReadLock(InputLock))
                return m_Navmesh.GetGridCell(m_CurrentPos);
        }

        public Int64 dbg_GetAntiStuckPrecisionTimerTime()
        {
            return m_AntiStuckPrecisionTimer.ElapsedMilliseconds;
        }

        public Int64 dbg_GetAntiStuckPathingTimerTime()
        {
            return m_AntiStuckPathingTimer.ElapsedMilliseconds;
        }

        public Int64 dbg_GetDestReachFailedTimerTime()
        {
            return m_DestReachFailedTimer.ElapsedMilliseconds;
        }

        public void dbg_StressTestPathing()
        {
            int timeout = 6000 * 1000;
            m_Navmesh.Log("[Nav] Stress test started [" + timeout / 1000 + "sec long]...");

            Stopwatch timeout_watch = new Stopwatch();
            timeout_watch.Start();
            Vec3 intersection = default(Vec3);

            while (timeout_watch.ElapsedMilliseconds < timeout)
            {
                CurrentPos = m_Navmesh.GetRandomPos();
                Destination = m_Navmesh.GetRandomPos();
                m_Navmesh.RayCast2D(m_Navmesh.GetRandomPos(), m_Navmesh.GetRandomPos(), MovementFlag.Fly, ref intersection);
                m_Navmesh.Log("[Nav] Ray cast done!");
            }

            m_Navmesh.Log("[Nav] Stress test ended!");
        }

        public void Dispose()
        {
            m_ShouldStopUpdates = true;
            UpdatesThread.Join();

            m_Navmesh.RemoveObserver(this);
        }

        // Aquires InputLock (read -> write)
        public void ClearDestination(DestType type)
        {
            using (new ReadLock(InputLock, true))
            {
                if ((m_DestinationType & type) == 0)
                    return;

                using (new WriteLock(InputLock))
                {
                    //m_Navmesh.Log("[Nav] Dest [" + m_DestinationType + "] cleared using [" + type + "] flags!");

                    m_Destination = Vec3.ZERO;
                    m_DestinationType = DestType.None;
                }
            }
        }

        public void Clear()
        {
            using (new WriteLock(InputLock))
            using (new WriteLock(PathLock))
            {
                m_CurrentPos = Vec3.ZERO;
                m_Destination = Vec3.ZERO;
                m_DestinationType = DestType.None;
                m_Waypoints.Clear();
                m_DestinationsHistory.Clear();
                m_DebugPositionsHistory.Clear();
                m_HistoryDestId = -1;
                m_DestinationGridsId.Clear();
                m_Path.Clear();
                m_PathDestination = Vec3.ZERO;
                m_PathDestType = DestType.None;
            }

            ResetAntiStuckPrecition(Vec3.ZERO);
            ResetAntiStuckPathing(Vec3.ZERO);
        }

        // May enter InputLock (read -> write). Do not use ZERO pos to clear destination, use ClearDestination instead!
        internal void SetDestination(Vec3 pos, DestType type, float precision)
        {
            // refactor not longer clearing destination automatically since ZERO is valid position to go to
            //if (pos == null || pos.IsEmpty)
            //{
            //    ClearDestination(type);
            //    return;
            //}

            using (new ReadLock(InputLock, true))
            {
                if ((m_Destination.Equals(pos) && m_DestinationType == type) || (m_Destination.IsZero() && m_DestinationType > type))
                    return;

                using (new WriteLock(InputLock))
                {
                    m_Destination = pos;
                    m_DestinationType = type;
                    Precision = precision;
                }

                //m_Navmesh.Log("[Nav] Dest changed to " + pos + " [" + type + "] precision " + precision);

                ResetDestReachFailed(m_CurrentPos);
            }

            RequestPathUpdate();
        }

        internal bool IsDestinationReached(DestType type_filter)
        {
            Vec3 temp = Vec3.ZERO;
            return IsDestinationReached(type_filter, ref temp);
        }

        internal bool IsDestinationReached(DestType type_filter, ref Vec3 destination)
        {
            using (new ReadLock(InputLock))
            using (new ReadLock(PathLock))
            {
                if ((m_DestinationType & type_filter) == 0)
                    return false;

                destination = new Vec3(m_Destination);

                return (m_CurrentPos.IsZero() || m_Destination.IsZero() || !m_Destination.Equals(m_PathDestination)) ? false : (m_Path.Count == 0);
            }
        }

        public void RequestPathUpdate()
        {
            //m_Navmesh.Log("Path update requested!");
            ForcePathUpdate = true;
        }

        private void Updates()
        {
            Stopwatch timer = new Stopwatch();
            timer.Start();

            while (!m_ShouldStopUpdates)
            {
                OnUpdate(timer.ElapsedMilliseconds);
            }
        }

        protected virtual void OnUpdate(Int64 time)
        {
            UpdateWaypointDestination();
            UpdateGridDestination();
            UpdateBackTrackDestination();

            int update_path_interval = m_UpdatePathIntervalOverride > 0 ? m_UpdatePathIntervalOverride : UpdatePathInterval;

            if (ForcePathUpdate || (update_path_interval > 0 && (time - m_LastPathUpdateTime) > update_path_interval))
            {
                ForcePathUpdate = false;
                m_LastPathUpdateTime = time;
                UpdatePath();
            }

            bool path_empty = true;

            //using (new ReadLock(PathLock))
                path_empty = m_Path.Count == 0;

            if (!path_empty)
            {
                UpdateAntiStuck();
                UpdateDestReachFailed();
            }

            Thread.Sleep(50);
        }

        private Int64 m_LastPathUpdateTime = 0;

        // Controls updated thread execution
        private volatile bool m_ShouldStopUpdates = false;

        private void UpdatePath()
        {
            if (!m_Navmesh.IsNavDataAvailable)
                return;

            Vec3 destination = Vec3.ZERO;
            DestType dest_type = DestType.None;

            // make sure destination and its type are in sync
            using (new ReadLock(InputLock))
            {
                destination = m_Destination;
                dest_type = m_DestinationType;
            }

            Vec3 current_pos = CurrentPos;

            //refactoring both current position and destination can be ZERO
            //if (current_pos.IsZero() || destination.IsZero())
            //    return;

            List<Vec3> new_path = new List<Vec3>();

            FindPath(current_pos, destination, MovementFlags, ref new_path, PATH_NODES_MERGE_DISTANCE, true, false, m_PathRandomCoeffOverride > 0 ? m_PathRandomCoeffOverride : PathRandomCoeff, m_PathBounce, PathNodesShiftDist);

            // verify whenever some point of path was not already passed during its calculation (this may take place when path calculations took long time)
            // this is done by finding first path segment current position can be casted on and removing all points preceding this segment including segment origin
            current_pos = CurrentPos;

            while (new_path.Count > 1)
            {
                Vec3 segment = new_path[1] - new_path[0];
                Vec3 segment_origin_to_current_pos = current_pos - new_path[0];

                float segment_len = segment.Length2D();
                float projection_len = segment.Dot2D(segment_origin_to_current_pos) / segment_len;

                // current position is already 'after' segment origin so remove it from path
                if (projection_len > 0)
                {
                    float distance_from_segment = -1;

                    // additionally verify if current pos is close enough to segment
                    if (projection_len > segment_len)
                        distance_from_segment = current_pos.Distance2D(new_path[1]);
                    else
                        distance_from_segment = current_pos.Distance2D(segment.Normalized2D() * projection_len);

                    if (distance_from_segment < DefaultPrecision)
                        new_path.RemoveAt(0);
                    else
                        break;
                }
                else
                    break;
            }

            using (new WriteLock(PathLock))
            {
                // reset override when first destination from path changed
                if (m_Path.Count == 0 || (new_path.Count > 0 && !m_Path[0].Equals(new_path[0])))
                    ResetAntiStuckPrecition(current_pos);

                m_Path = new_path;
                m_PathDestination = destination;
                m_PathDestType = dest_type;
            }
        }

        private void UpdateGridDestination()
        {
            Vec3 destination = Vec3.ZERO;
            bool grid_dest_found = false;
            List<int> destination_grids_id = null;

            using (new ReadLock(InputLock))
                destination_grids_id = new List<int>(m_DestinationGridsId);

            if (destination_grids_id.Count > 0)
            {
                using (m_Navmesh.AcquireReadDataLock())
                {
                    GridCell current_grid = m_Navmesh.m_GridCells.FirstOrDefault(g => g.Contains2D(CurrentPos));

                    GridCell destination_grid = m_Navmesh.m_GridCells.FirstOrDefault(g => destination_grids_id.Contains(g.Id) && Algorihms.AreConnected(current_grid, ref g, MovementFlag.None));

                    if (destination_grid != null)
                    {
                        grid_dest_found = true;
                        destination = m_Navmesh.GetNearestCell(destination_grid.Cells, destination_grid.Center).Center;
                    }
                }
            }

            if (grid_dest_found)
                SetDestination(destination, DestType.Grid, GridDestPrecision);
        }

        private void UpdateBackTrackDestination()
        {
            Vec3 destination = Vec3.ZERO;
            bool backtrack_dest_found = false;

            using (new ReadLock(InputLock))
            {
                if (BackTrackEnabled)
                {
                    backtrack_dest_found = true;
                    destination = m_DestinationsHistory[m_HistoryDestId];
                }
            }

            if (backtrack_dest_found)
                SetDestination(destination, DestType.BackTrack, DefaultPrecision);
        }

        private void UpdateWaypointDestination()
        {
            Vec3 destination = Vec3.ZERO;
            bool waypoint_dest_found = false;

            using (new ReadLock(InputLock))
            {
                if (m_Waypoints.Count > 0)
                {
                    waypoint_dest_found = true;
                    destination = m_Waypoints[0];
                }
            }

            if (waypoint_dest_found)
                SetDestination(destination, DestType.Waypoint, DefaultPrecision);
        }

        private void UpdatePathProgression(Vec3 current_pos)
        {
            bool any_node_reached = false;
            Vec3 reached_pos = Vec3.ZERO;

            // check and removed reached nodes from path
            using (new ReadLock(PathLock, true))
            {
                while (m_Path.Count > 0)
                {
                    float precision = DefaultPrecision;

                    if (m_PrecisionOverride > 0)
                        precision = m_PrecisionOverride;
                    else if (m_Path.Count == 1)
                        precision = Precision;

                    if (current_pos.Distance2D(m_Path[0]) > precision)
                        break;

                    reached_pos = m_Path[0];

                    using (new WriteLock(PathLock))
                        m_Path.RemoveAt(0);

                    any_node_reached = true;

                    ResetAntiStuckPrecition(current_pos);
                }
            }

            Vec3 destination = Destination;

            // update destination arrived
            if (any_node_reached)
            {
                if (IsDestinationReached(DestType.All))
                {
                    using (new WriteLock(InputLock))
                    {
                        if (m_DestinationType != DestType.BackTrack &&
                            (m_DestinationsHistory.Count == 0 || (!m_DestinationsHistory[m_DestinationsHistory.Count - 1].Equals(reached_pos) &&
                                                                    m_DestinationsHistory[m_DestinationsHistory.Count - 1].Distance(reached_pos) > MIN_DEST_DIST_TO_ADD_TO_HISTORY)))
                        {
                            m_DestinationsHistory.Add(reached_pos);
                        }
                    }

                    NotifyOnDestinationReached(m_DestinationType, destination);

                    ResetAntiStuckPathing(current_pos);

                    //m_Navmesh.Log("[Nav] Dest " + m_Destination + " [" + m_DestinationType + "] reached!");
                }

                if (IsDestinationReached(DestType.Waypoint))
                {
                    using (new WriteLock(InputLock))
                        m_Waypoints.RemoveAt(0);

                    if (m_Waypoints.Count == 0)
                        ClearDestination(DestType.Waypoint);
                }

                if (IsDestinationReached(DestType.User))
                {
                    ClearDestination(DestType.User);
                }

                if (IsDestinationReached(DestType.BackTrack))
                {
                    --m_HistoryDestId;

                    if (!BackTrackEnabled)
                        ClearDestination(DestType.BackTrack);
                }
            }
        }

        private void UpdateDestReachFailed()
        {
            const float MIN_DIST_TO_RESET = 90;
            const float MIN_TIME_TO_FAIL_DESTINATION_REACH = 20000;

            Vec3 curr_pos = CurrentPos;

            if (m_DestReachFailedTestPos.Distance(curr_pos) > MIN_DIST_TO_RESET)
                ResetDestReachFailed(curr_pos);
            else if (IsStandingOnPurpose)
                m_DestReachFailedTimer.Stop();
            else
                m_DestReachFailedTimer.Start();

            if (m_DestReachFailedTimer.ElapsedMilliseconds > MIN_TIME_TO_FAIL_DESTINATION_REACH)
            {
                DestType dest_type = DestType.None;
                Vec3 dest = default(Vec3);

                using (new ReadLock(InputLock))
                {
                    dest_type = m_DestinationType;
                    dest = m_Destination;
                }

                //m_Navmesh.Log("[Nav] Destination " + dest + " [" + dest_type + "] reach failed!");

                NotifyOnDestinationReachFailed(dest_type, dest);

                ResetDestReachFailed(curr_pos);
            }
        }

        private void UpdateAntiStuck()
        {
            if (!EnableAntiStuck)
                return;

            const float MIN_DIST_TO_RESET_ANTI_STUCK_PRECISION = 10;
            const float MIN_DIST_TO_RESET_ANTI_STUCK_PATHING = 25;

            const float MIN_TIME_TO_RECALCULATE_PATH = 2000;
            const float MIN_TIME_TO_OVERRIDE_PRECISION = 4000;
            const float MIN_TIME_TO_BOUNCE = 6000;
            const float MIN_TIME_TO_OVERRIDE_PATH_RANDOM_COEFF = 9000;

            Vec3 curr_pos = CurrentPos;

            using (new ReadLock(AntiStuckLock, true))
            {
                if (m_AntiStuckPrecisionTestPos.IsZero() || m_AntiStuckPrecisionTestPos.Distance(curr_pos) > MIN_DIST_TO_RESET_ANTI_STUCK_PRECISION)
                    ResetAntiStuckPrecition(curr_pos);
                else if (IsStandingOnPurpose)
                    m_AntiStuckPrecisionTimer.Stop();
                else
                    m_AntiStuckPrecisionTimer.Start();

                if (m_AntiStuckPathingTestPos.IsZero() || m_AntiStuckPathingTestPos.Distance(curr_pos) > MIN_DIST_TO_RESET_ANTI_STUCK_PATHING)
                    ResetAntiStuckPathing(curr_pos);
                else if (IsStandingOnPurpose)
                    m_AntiStuckPathingTimer.Stop();
                else
                    m_AntiStuckPathingTimer.Start();

                // handle anti stuck precision management features
                if (m_AntiStuckPrecisionTimer.ElapsedMilliseconds > MIN_TIME_TO_OVERRIDE_PRECISION)
                    m_PrecisionOverride = 60;

                // handle anti stuck path management features

                // level 1
                if (m_AntiStuckPathingTimer.ElapsedMilliseconds > MIN_TIME_TO_RECALCULATE_PATH &&
                    m_AntiStuckPathingLevel == 0)
                {
                    m_AntiStuckPathingLevel = 1;
                    RequestPathUpdate();
                }
                // level 2
                else if (m_AntiStuckPathingTimer.ElapsedMilliseconds > MIN_TIME_TO_BOUNCE &&
                         m_AntiStuckPathingLevel == 1)
                {
                    ResetAntiStuckPrecition(curr_pos);
                    m_AntiStuckPathingLevel = 2;
                    m_PathBounce = true;
                    RequestPathUpdate();
                }
                // level 3
                else if (m_AntiStuckPathingTimer.ElapsedMilliseconds > MIN_TIME_TO_OVERRIDE_PATH_RANDOM_COEFF &&
                         m_AntiStuckPathingLevel == 2)
                {
                    ResetAntiStuckPrecition(curr_pos);
                    m_PathBounce = false;
                    m_AntiStuckPathingLevel = 3;
                    m_PathRandomCoeffOverride = 1.5f;
                    m_UpdatePathIntervalOverride = 3000;
                    RequestPathUpdate();
                }
            }
        }

        private void ResetAntiStuckPrecition(Vec3 curr_pos)
        {
            using (new WriteLock(AntiStuckLock))
            {
                m_PrecisionOverride = -1;
                m_AntiStuckPrecisionTestPos = curr_pos;
                m_AntiStuckPrecisionTimer.Reset();
            }
        }

        private void ResetAntiStuckPathing(Vec3 curr_pos)
        {
            using (new WriteLock(AntiStuckLock))
            {
                if (m_PathRandomCoeffOverride > 0)
                    RequestPathUpdate();
                m_PathRandomCoeffOverride = -1;
                m_UpdatePathIntervalOverride = -1;
                m_PathBounce = false;
                m_AntiStuckPathingTestPos = curr_pos;
                m_AntiStuckPathingTimer.Reset();
                m_AntiStuckPathingLevel = 0;
            }
        }

        private void ResetDestReachFailed(Vec3 curr_pos)
        {
            m_DestReachFailedTestPos = curr_pos;
            m_DestReachFailedTimer.Reset();
        }

        private void ReorganizeWaypoints()
        {
            Vec3 pos = CurrentPos;

            using (new ReadLock(InputLock, true))
            {
                if (m_Waypoints.Count == 0)
                    return;

                int nearest_waypoint_index = -1;
                float nearest_waypoint_dist = float.MaxValue;

                for (int i = 0; i < m_Waypoints.Count; ++i)
                {
                    float dist = m_Waypoints[i].Distance(pos);

                    if (dist < nearest_waypoint_dist)
                    {
                        nearest_waypoint_index = i;
                        nearest_waypoint_dist = dist;
                    }
                }

                using (new WriteLock(InputLock))
                {
                    for (int i = 0; i < nearest_waypoint_index; ++i)
                    {
                        m_Waypoints.Add(new Vec3(m_Waypoints[0]));
                        m_Waypoints.RemoveAt(0);
                    }
                }
            }
        }

        public virtual void OnGridCellAdded(GridCell grid_cell)
        {
        }

        public virtual void OnNavDataChanged()
        {
            RequestPathUpdate();
        }

        public virtual void OnNavDataCleared()
        {
            Clear();
        }

        protected void NotifyOnHugeCurrentPosChange()
        {
            List<INavigationObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavigationObserver observer in observers_copy)
                observer.OnHugeCurrentPosChange();
        }

        protected void NotifyOnDestinationReached(DestType type, Vec3 dest)
        {
            List<INavigationObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavigationObserver observer in observers_copy)
                observer.OnDestinationReached(type, dest);
        }

        protected void NotifyOnDestinationReachFailed(DestType type, Vec3 dest)
        {
            List<INavigationObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavigationObserver observer in observers_copy)
                observer.OnDestinationReachFailed(type, dest);
        }

        // Extension will be automatically added
        public void Serialize(string name)
        {
            using (BinaryWriter w = new BinaryWriter(File.OpenWrite(name + ".navigator")))
            {
                OnSerialize(w);
            }
        }

        protected virtual void OnSerialize(BinaryWriter w)
        {
            using (new ReadLock(InputLock))
            using (new ReadLock(PathLock))
            {
                w.Write(m_Waypoints.Count);
                foreach (Vec3 p in m_Waypoints)
                    p.Serialize(w);

                w.Write(m_Path.Count);
                foreach (Vec3 p in m_Path)
                    p.Serialize(w);
                m_PathDestination.Serialize(w);
                w.Write((int)m_PathDestType);

                w.Write(m_DestinationsHistory.Count);
                foreach (Vec3 p in m_DestinationsHistory)
                    p.Serialize(w);
                w.Write(m_HistoryDestId);

                w.Write(m_DebugPositionsHistory.Count);
                foreach (Vec3 p in m_DebugPositionsHistory)
                    p.Serialize(w);

                w.Write(m_DestinationGridsId.Count);
                foreach (int d in m_DestinationGridsId)
                    w.Write(d);

                m_CurrentPos.Serialize(w);
                m_Destination.Serialize(w);
                w.Write((int)m_DestinationType);

                w.Write(UpdatePathInterval);
                w.Write(CurrentPosDiffRecalcThreshold);
                w.Write(PathNodesShiftDist);
                w.Write(PathRandomCoeff);
                w.Write(DefaultPrecision);
                w.Write(Precision);
            }
        }

        // Extension will be automatically added
        public void Deserialize(string name)
        {
            using (BinaryReader r = new BinaryReader(File.OpenRead(name + ".navigator")))
            {
                OnDeserialize(r);
            }
        }

        protected virtual void OnDeserialize(BinaryReader r)
        {
            using (new WriteLock(InputLock))
            using (new WriteLock(PathLock))
            {
                m_Waypoints.Clear();
                m_DestinationGridsId.Clear();
                m_Path.Clear();
                m_DestinationsHistory.Clear();
                m_DebugPositionsHistory.Clear();

                int waypoints_count = r.ReadInt32();
                for (int i = 0; i < waypoints_count; ++i)
                    m_Waypoints.Add(new Vec3(r));

                int path_count = r.ReadInt32();
                for (int i = 0; i < path_count; ++i)
                    m_Path.Add(new Vec3(r));
                m_PathDestination = new Vec3(r);
                m_PathDestType = (DestType)r.ReadInt32();

                int destination_history_count = r.ReadInt32();
                for (int i = 0; i < destination_history_count; ++i)
                    m_DestinationsHistory.Add(new Vec3(r));
                m_HistoryDestId = r.ReadInt32();

                int debug_positions_history_count = r.ReadInt32();
                for (int i = 0; i < debug_positions_history_count; ++i)
                    m_DebugPositionsHistory.Add(new Vec3(r));

                int destination_grid_cells_id_count = r.ReadInt32();
                for (int i = 0; i < destination_grid_cells_id_count; ++i)
                    m_DestinationGridsId.Add(r.ReadInt32());

                m_CurrentPos = new Vec3(r);
                m_Destination = new Vec3(r);
                m_DestinationType = (DestType)r.ReadInt32();

                UpdatePathInterval = r.ReadInt32();
                CurrentPosDiffRecalcThreshold = r.ReadSingle();
                PathNodesShiftDist = r.ReadSingle();
                PathRandomCoeff = r.ReadSingle();
                DefaultPrecision = r.ReadSingle();
                Precision = r.ReadSingle();
            }

            Vec3 curr_pos = CurrentPos;

            ResetAntiStuckPrecition(curr_pos);
            ResetAntiStuckPathing(curr_pos);
        }

        private float PATH_NODES_MERGE_DISTANCE = -1;
        private float MIN_DEST_DIST_TO_ADD_TO_HISTORY = 75;

        private Thread UpdatesThread = null;
        private volatile bool ForcePathUpdate = false;

        private ReaderWriterLockSlim PathLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private ReaderWriterLockSlim InputLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private ReaderWriterLockSlim AntiStuckLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);

        private List<Vec3> m_Path = new List<Vec3>(); //@ PathLock
        private Vec3 m_PathDestination = Vec3.ZERO; //@ PathLock
        private DestType m_PathDestType = DestType.None; //@ PathLock
        private List<Vec3> m_Waypoints = new List<Vec3>(); //@ InputLock
        private List<Vec3> m_DestinationsHistory = new List<Vec3>(); //@ InputLock
        private List<Vec3> m_DebugPositionsHistory = new List<Vec3>(); //@ InputLock
        private int m_HistoryDestId = -1; //@ InputLock
        private List<int> m_DestinationGridsId = new List<int>(); //@ InputLock

        private float m_PrecisionOverride = -1;
        private Stopwatch m_AntiStuckPrecisionTimer = new Stopwatch();
        private float m_PathRandomCoeffOverride = -1;
        private int m_UpdatePathIntervalOverride = -1;
        private Stopwatch m_AntiStuckPathingTimer = new Stopwatch();
        private int m_AntiStuckPathingLevel = 0;
        private bool m_PathBounce = false;
        private Stopwatch m_DestReachFailedTimer = new Stopwatch();

        private Vec3 m_AntiStuckPrecisionTestPos = Vec3.ZERO; //@ AntiStuckLock
        private Vec3 m_AntiStuckPathingTestPos = Vec3.ZERO; //@ AntiStuckLock
        private Vec3 m_DestReachFailedTestPos = Vec3.ZERO;

        private Vec3 m_CurrentPos = Vec3.ZERO; //@ InputLock
        private Vec3 m_Destination = Vec3.ZERO; //@ InputLock
        private DestType m_DestinationType = DestType.None; //@ InputLock

        private List<INavigationObserver> m_Observers = new List<INavigationObserver>(); //@ InputLock

        private Navmesh m_Navmesh = null;
    }
}
