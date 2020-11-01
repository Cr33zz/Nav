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
        User = 0x0008,
        Custom = 0x0010, // same as user but not cleared automatically
        BackTrack = 0x0020, // used for moving along historical destinations
        RunAway = 0x0040, // used for threat avoidance
        All = 0xFFFF,
    }

    public struct destination : IEquatable<destination>
    {
        public destination(Vec3 pos, DestType type = DestType.User, float precision = 0, float precision_max = 0, Object user_data = null)
        {
            this.pos = pos;
            this.type = type;
            this.precision = precision;
            this.precision_max = precision_max;
            this.user_data = user_data;

            is_ring = false;
            shift = false;
        }

        public override bool Equals(Object obj)
        {
            return obj is destination d && Equals(d);
        }

        public bool Equals(destination d)
        {
            return pos.Equals(d.pos) && type == d.type && precision == d.precision && precision_max == d.precision_max && user_data == d.user_data && is_ring == d.is_ring;
        }

        public override int GetHashCode()
        {
            return pos.GetHashCode() ^ type.GetHashCode() ^ precision.GetHashCode() ^ user_data.GetHashCode() ^ is_ring.GetHashCode() ^ precision_max.GetHashCode();
        }

        public Vec3 pos;
        public DestType type;
        public float precision;
        public float precision_max;
        public Object user_data;
        internal bool is_ring;
        internal bool shift;
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

        public bool Navigate2D { get; set; } = true;

        // precision with each path node will be accepted as reached
        public float DefaultPrecision { get; set; } = 10;

        // precision with grid destination will be accepted as reached
        public float GridDestPrecision { get; set; } = 40;

        // how much path will be randomized 0 by default
        public float PathRandomCoeff { get; set; } = 0;

        public bool AlignGoToPositionToCurrentPosZWhenZero { get; set; } = false;

        public float DistToKeepFromEdge { get; set; } = 0; // not supported

        // only initial part of path of this length will be moved away from edges (to minimize performance impact if there are frequent enough path recalculates)
        public float KeepAwayFromEdgesOnInitialLength { get; set; } = 50; // not supported

        public float KeepFromEdgePrecision { get; set; } = 5;

        // each point on path will be offset-ed in direction from previous point so bot will move along path more precisely even with high precision parameter
        public float PathNodesShiftDist { get; set; } = 10;

        // distance to bounce away from wall (used by anti-stuck feature)
        public float BounceDist { get; set; } = 10;

        // when new CurrentPos differ from last one by more than this value path update will be automatically requested
        public float CurrentPosDiffRecalcThreshold { set; get; } = 15;

        public float MinDestDistToAddToHistory { set; get; } = 75;
        public float MinDestDistToAddToDebugHistory { set; get; } = 15;

        // path will be automatically recalculated with this interval (milliseconds)
        public int UpdatePathInterval { get; set; } = -1;

        public bool AllowRoughPath { get; set; } = false;
        public float PathSmoothingDistance { get; set; } = float.MaxValue;
        public float PathSmoothingPrecision { get; set; } = 3;

        public bool EnableAntiStuck { get; set; } = false;

        public bool AntiStuckActive => m_AntiStuckPathingLevel > 0;

        // position must change from last one by at least this to reset precision override
        public float MinDistToResetAntiStuckPrecision { get; set; } = 10;
        public float MinDistToResetAntiStuckPathing { get; set; } = 25;
        public float AntiStuckPrecisionOverride { get; set; } = 60;

        // turn on/off danger detection and danger avoidance
        public bool EnableThreatAvoidance { get; set; } = false;

        public float ThreatThreshold { get; set; } = 0;

        // when avoidance is enabled and current position is on a cell with threat level higher than MaxAllowedThreat
        public bool IsInThreat { get; private set; } = false;

        public bool IsThreatAt(Vec3 pos)
        {
            return m_Navmesh.Regions.Any(x => x.Threat > ThreatThreshold && x.Area.Contains2D(pos));
        }

        public bool IsThreatBetween(Vec3 start, Vec3 end)
        {
            Vec3 threat_pos = default(Vec3);

            foreach (var t in m_Navmesh.Regions.Where(x => x.Threat > ThreatThreshold).ToList())
            {
                if (t.Area.SegmentTest2D(start, end, ref threat_pos))
                    return true;
            }

            return false;
        }

        public bool IsThreatAhead { get; private set; } = false;

        // when not in threat but path leads through threats IsThreatAhead will be turned when agent is closer than ThreatDetectionRange from a threat ahead
        public float ThreatDetectionRange { get; set; } = 10;

        public float ThreatDetectionPrecision { get; set; } = 0.5f;

        // should be used when EnableAntiStuck is true to notify navigator that actor is not blocked by some obstacle but just standing
        public bool IsStandingOnPurpose { get; set; } = true;

        // is current path for currently requested destination type (may be false when destination change has been requested but path is not updated yet)
        public bool IsPathUpToDate => m_PathDestination.type == m_Destination.type;

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

        public bool FindPath(Vec3 from, Vec3 to, ref List<Vec3> path, bool as_close_as_possible, bool allow_rought_path)
        {
            return FindPath(from, to, MovementFlags, ref path, out var path_recalc_trigger_position, out var path_recalc_trigger_dist, PATH_NODES_MERGE_DISTANCE, as_close_as_possible, false, m_PathRandomCoeffOverride > 0 ? m_PathRandomCoeffOverride : PathRandomCoeff, m_PathBounce, PathNodesShiftDist, false, PathSmoothingDistance, AllowRoughPath);
        }

        public bool FindPath(Vec3 from, Vec3 to, MovementFlag flags, ref List<Vec3> path, out Vec3 path_recalc_trigger_position, out float path_recalc_trigger_precision, float merge_distance = -1, bool as_close_as_possible = false, bool include_from = false, float random_coeff = 0, bool bounce = false, float shift_nodes_distance = 0, bool shift_dest = false, float smoothen_distance = float.MaxValue, bool allow_rough_path = false)
        {
            using (m_Navmesh.AcquireReadDataLock())
            //using (new Profiler("[Nav] Path finding took %t", 20))
            {
                path_recalc_trigger_position = Vec3.ZERO;
                path_recalc_trigger_precision = 0;

                if (from.IsZero() || to.IsZero())
                    return false;

                bool start_on_nav_mesh = m_Navmesh.GetCellAt(from, out Cell start, flags, false, as_close_as_possible);
                bool end_on_nav_mesh = m_Navmesh.GetCellAt(to, out Cell end, flags, false, as_close_as_possible);

                // align to position to closest cell
                if (!end_on_nav_mesh)
                {
                    if (end == null)
                        return false;

                    to = end.AABB.Align(to);
                }

                List<Vec3> rought_path = new List<Vec3>();

                if (m_RoughtPathEstimator != null && allow_rough_path && !m_Navmesh.AreNavBlockersPresent())
                    m_RoughtPathEstimator.FindRoughPath(from, to, ref rought_path);

                if (rought_path.Count > 4)
                {
                    to = rought_path[4];
                    m_Navmesh.GetCellAt(to, out end, flags, false, as_close_as_possible);

                    // ideally this point should be set somewhere along the path to rough path node (but it would required going over the path to find it afterwards)
                    path_recalc_trigger_position = to;
                    path_recalc_trigger_precision = m_RoughtPathEstimator.GetRoughPathRecalcPrecision();
                    rought_path.RemoveRange(0, 4);
                }
                else
                    rought_path.Clear();

                List<path_pos> tmp_path = new List<path_pos>();

                if (bounce)
                {
                    Vec3 bounce_dir = start.AABB.GetBounceDir2D(from);
                    Vec3 new_from = from + bounce_dir * BounceDist;
                    m_Navmesh.GetCellAt(new_from, out start, flags, false, as_close_as_possible);

                    if (!Algorihms.FindPath(start, new_from, new Algorihms.DestinationPathFindStrategy<Cell>(to, end), flags, ref tmp_path, random_coeff, true))
                        return false;

                    tmp_path.Insert(0, new path_pos(start.AABB.Align(from), start));
                }
                else
                {
                    if (!Algorihms.FindPath(start, from, new Algorihms.DestinationPathFindStrategy<Cell>(to, end), flags, ref tmp_path, random_coeff, true))
                        return false;
                }

                if (smoothen_distance > 0 && random_coeff == 0)
                    SmoothenPath(ref tmp_path, flags, smoothen_distance, ref path_recalc_trigger_position, ref path_recalc_trigger_precision, bounce ? 1 : 0);

                if (DistToKeepFromEdge > 0 && random_coeff == 0)
                    KeepAwayFromEdges(ref tmp_path, flags);

                path = tmp_path.Select(x => x.pos).Concat(rought_path).ToList();

                PostProcessPath(ref path, merge_distance, shift_nodes_distance, shift_dest);

                if (!include_from && start_on_nav_mesh)
                    path.RemoveAt(0);

                return true;
            }
        }

        public bool FindAvoidancePath(Vec3 from, float max_allowed_threat, MovementFlag flags, ref List<Vec3> path, Vec3 hint_pos = default(Vec3), bool include_from = false, float shift_nodes_distance = 0, float smoothen_distance = float.MaxValue)
        {
            using (m_Navmesh.AcquireReadDataLock())
            {
                if (from.IsZero())
                    return false;

                m_Navmesh.GetCellAt(from, out Cell start, flags, false, true);

                List<path_pos> tmp_path = new List<path_pos>();

                if (!Algorihms.FindPath(start, from, new Algorihms.AvoidancePathFindStrategy<Cell>(max_allowed_threat, hint_pos), flags, ref tmp_path))
                    return false;

                //m_Navmesh.Log($"original path has {tmp_path.Count} nodes");

                if (smoothen_distance > 0)
                {
                    Vec3 path_recalc_trigger_position = Vec3.ZERO;
                    float path_recalc_trigger_precision = 0;
                    SmoothenPath(ref tmp_path, flags, smoothen_distance, ref path_recalc_trigger_position, ref path_recalc_trigger_precision);
                }

                //m_Navmesh.Log($"smoothened path has {tmp_path.Count} nodes");

                // direct towards safe cell center so user is not stuck on the edge of a threat
                if (tmp_path.Count > 0)
                {
                    var last_node = tmp_path.Last();
                    tmp_path.Add(new path_pos(last_node.cell.Center, last_node.cell));
                }

                path = tmp_path.Select(x => x.pos).ToList();

                PostProcessPath(ref path, -1, shift_nodes_distance, true);

                if (!include_from)
                    path.RemoveAt(0);

                return true;
            }
        }

        private void SmoothenPath(ref List<path_pos> path, MovementFlag flags, float smoothen_distance, ref Vec3 path_recalc_trigger_position, ref float path_recalc_trigger_precision, int skip_first_count = 0)
        {
            int ray_start_index = skip_first_count;
            float distanceCovered = 0;
            int force_stop_index = -1;

            while (ray_start_index + 2 < path.Count)
            {
                path_pos ray_start_data = path[ray_start_index];
                path_pos intermediate_data = path[ray_start_index + 1];
                path_pos ray_end_data = path[ray_start_index + 2];
                float max_move_cost_mult = Math.Min(ray_start_data.cell.MovementCostMult, intermediate_data.cell.MovementCostMult);

                // try remove middle point completely
                if (m_Navmesh.RayCast2D(ray_start_data.pos, ray_start_data.cell, ray_end_data.pos, flags, max_move_cost_mult))
                    path.RemoveAt(ray_start_index + 1);
                else
                {
                    ++ray_start_index;
                    distanceCovered += ray_start_data.pos.Distance2D(ray_end_data.pos);
                }

                if (smoothen_distance > 0 && distanceCovered > smoothen_distance)
                {
                    path_recalc_trigger_position = ray_end_data.pos;
                    path_recalc_trigger_precision = PathSmoothingDistance * 0.3f;
                    force_stop_index = ray_start_index;
                    break;
                }
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

                    Vec3 dir = ray_end_data.pos - intermediate_data.pos;
                    float length = dir.Normalize2D();
                    int steps = (int)(length / PathSmoothingPrecision);
                    float max_movement_cost_mult = Math.Min(ray_start_data.cell.MovementCostMult, intermediate_data.cell.MovementCostMult);

                    RayCastResult last_unobstucted_result = default(RayCastResult);
                    for (int i = 1; i <= steps; ++i) // checking 0 is unnecessary since this is the current path
                    {
                        var result = m_Navmesh.RayCast2D(ray_start_data.pos, ray_start_data.cell, intermediate_data.pos + dir * (float)i * PathSmoothingPrecision, flags, max_movement_cost_mult);
                        if (result)
                            last_unobstucted_result = result;
                        else
                            break;
                    }

                    if (last_unobstucted_result)
                        intermediate_data = path[ray_start_index + 1] = new path_pos(last_unobstucted_result.End, last_unobstucted_result.EndCell);

                    dir = ray_start_data.pos - intermediate_data.pos;
                    length = dir.Normalize2D();
                    steps = (int)(length / PathSmoothingPrecision);
                    max_movement_cost_mult = Math.Min(ray_start_data.cell.MovementCostMult, intermediate_data.cell.MovementCostMult);

                    last_unobstucted_result = default(RayCastResult);
                    for (int i = 1; i <= steps; ++i) // checking 0 is unnecessary since this is the current path
                    {
                        var result = m_Navmesh.RayCast2D(ray_end_data.pos, ray_end_data.cell, intermediate_data.pos + dir * (float)i * PathSmoothingPrecision, flags, max_movement_cost_mult);
                        if (result)
                            last_unobstucted_result = result;                        
                        else
                            break;
                    }

                    if (last_unobstucted_result)
                        intermediate_data = path[ray_start_index + 1] = new path_pos(last_unobstucted_result.End, last_unobstucted_result.EndCell);

                    ++ray_start_index;

                    if (force_stop_index > 0 && ray_start_index >= force_stop_index)
                        break;
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

        private void PostProcessPath(ref List<Vec3> path, float merge_distance, float shift_nodes_distance, bool shift_dest = false)
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
                    {
                        Vec3 new_point = merge_point / (float)(start_count - path.Count + 1);
                        new_point.Z = path[i].Z;
                        path[i] = new_point;
                    }
                }
            }

            // shift points to increase movement accuracy
            if (shift_nodes_distance > 0)
            {
                for (int i = path.Count - (shift_dest ? 1 : 2); i > 0; --i)
                {
                    bool is_dest = (i == path.Count - 1);

                    Vec3 dir_to_next = path[i] - path[i - 1];
                    float dist = dir_to_next.Normalize2D();

                    if (dist > 0.01)
                        path[i] += dir_to_next * shift_nodes_distance * (is_dest ? 0.8f : 1.0f);
                }
            }
        }

        public destination RingDestination
        {
            get
            {
                using (new ReadLock(InputLock))
                    return m_RingDestination;
            }
        }

        public destination Destination
        {
            get
            {
                using (new ReadLock(InputLock))
                    return m_Destination;
            }

            set
            {
                SetDestination(new destination(value.pos, value.type, value.precision < 0 ? DefaultPrecision : value.precision, value.precision_max, value.user_data));
            }
        }

        public void SetDestination(Vec3 pos, float precision, float precision_max = -1, Object user_data = null)
        {
            SetDestination(new destination(pos, DestType.User, precision > 0 ? precision : DefaultPrecision, precision_max, user_data));
        }

        public void ClearRingDestinations()
        {
            using (new WriteLock(InputLock))
            {
                if (m_Destination.is_ring)
                    m_Destination = default(destination);

                m_RingDestination = default(destination);

            }
        }

        public void ClearAllDestinations()
        {
            ClearRingDestinations();
            ClearDestination(DestType.All);
        }

        public void ClearGridDestination()
        {
            ClearDestination(DestType.Grid);
        }

        public void SetCustomDestination(Vec3 pos, float precision = -1, float precision_max = -1, Object user_data = null)
        {
            SetDestination(new destination(pos, DestType.Custom, precision < 0 ? DefaultPrecision : precision, precision_max, user_data));
        }

        public void ClearCustomDestination()
        {
            ClearDestination(DestType.Custom);
        }

        public bool TryGetPath(ref List<Vec3> path, ref destination path_dest)
        {
            if (PathLock.TryEnterReadLock(0))
            {
                path = new List<Vec3>(Path);
                path_dest = m_PathDestination;
                PathLock.ExitReadLock();
                return true;
            }

            return false;
        }

        public float TryGetPathLength(ref destination path_dest)
        {
            float length = 0;

            if (PathLock.TryEnterReadLock(0))
            {
                length = Algorihms.GetPathLength(Path, CurrentPos);
                path_dest = m_PathDestination;
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
                return new List<Vec3>(Path);
        }

        public DestType GetDestinationType()
        {
            return m_Destination.type;
        }

        public Vec3 GoToPosition
        {
            get
            {
                using (new ReadLock(PathLock))
                {
                    if (Path.Count > 0)
                    {
                        var pos = Path[0];
                        if (AlignGoToPositionToCurrentPosZWhenZero && pos.Z == 0)
                            pos.Z = m_CurrentPos.Z;
                        return pos;
                    }

                    return Vec3.ZERO;
                }
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

                        if (m_DebugPositionsHistory.Count == 0 || value.Distance2D(m_DebugPositionsHistory.Last()) > MinDestDistToAddToDebugHistory)
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
                        path_empty = Path.Count == 0;
                }

                if (huge_current_pos_change || path_empty)
                {
                    if (huge_current_pos_change)
                        NotifyOnHugeCurrentPosChange();

                    RequestPathUpdate();
                }

                if (!PathRecalcTriggerPosition.IsZero() && PathRecalcTriggerPosition.Distance2D(value) < PathRecalcTriggerPrecision)
                    RequestPathUpdate();

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
            
            while (timeout_watch.ElapsedMilliseconds < timeout)
            {
                CurrentPos = m_Navmesh.GetRandomPos();
                Destination = new destination(m_Navmesh.GetRandomPos());
                m_Navmesh.RayCast2D(m_Navmesh.GetRandomPos(), m_Navmesh.GetRandomPos(), MovementFlag.Fly);
                m_Navmesh.Log("[Nav] Ray cast done!");
            }

            m_Navmesh.Log("[Nav] Stress test ended!");
        }

        public void Dispose()
        {
            ShouldStopUpdates = true;
            UpdatesThread.Join();

            m_Navmesh.RemoveObserver(this);
        }

        // Acquires InputLock (read -> write)
        public void ClearDestination(DestType type)
        {
            using (new ReadLock(InputLock, true))
            {
                if ((m_Destination.type & type) == 0)
                    return;

                using (new WriteLock(InputLock))
                {
                    //m_Navmesh.Log("[Nav] Dest [" + m_DestinationType + "] cleared using [" + type + "] flags!");

                    m_Destination = default(destination);
                }

                using (new WriteLock(PathLock))
                {
                    m_PathDestination = default(destination);
                    Path.Clear();
                }
            }
        }

        public void Clear()
        {
            using (new WriteLock(InputLock))
            using (new WriteLock(PathLock))
            {
                m_CurrentPos = Vec3.ZERO;
                m_Destination = default(destination);
                m_Waypoints.Clear();
                m_DestinationsHistory.Clear();
                m_DebugPositionsHistory.Clear();
                m_HistoryDestId = -1;
                m_DestinationGridsId.Clear();
                Path.Clear();
                m_PathDestination = default(destination);
            }

            ResetAntiStuckPrecition(Vec3.ZERO);
            ResetAntiStuckPathing(Vec3.ZERO);
        }

        // May enter InputLock (read -> write).
        internal void SetDestination(destination dest)
        {
            if (dest.pos.IsZero())
            {
                ClearDestination(dest.type);
                return;
            }

            using (new ReadLock(InputLock, true))
            {
                m_Destination.user_data = dest.user_data;

                if ((m_Destination.pos.Equals(dest.pos) && m_Destination.type == dest.type) || (!m_Destination.pos.IsZero() && m_Destination.type > dest.type))
                    return;

                using (new WriteLock(InputLock))
                {
                    if (dest.precision_max > 0)
                        m_RingDestination = dest;
                    else
                        m_Destination = dest;

                    // clear ring destination when overwritten by any other destination that is not coming from ring destination update
                    if (!dest.is_ring && dest.precision_max <= 0)
                        m_RingDestination = default(destination);
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

        internal bool IsDestinationReached(DestType type_filter, ref Vec3 dest_pos)
        {
            using (new ReadLock(InputLock))
            using (new ReadLock(PathLock))
            {
                if ((m_Destination.type & type_filter) == 0)
                    return false;

                dest_pos = new Vec3(m_Destination.pos);

                return (!m_CurrentPos.IsZero() && !m_Destination.pos.IsZero() && m_Destination.pos.Equals(m_PathDestination.pos)) && (Path.Count == 0);
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

            while (!ShouldStopUpdates)
            {
                OnUpdate(timer.ElapsedMilliseconds);
            }
        }

        protected virtual void OnUpdate(Int64 time)
        {
            UpdateRingDestination();
            UpdateWaypointDestination();
            UpdateGridDestination();
            UpdateBackTrackDestination();
            UpdateThreatAvoidance();

            int update_path_interval = m_UpdatePathIntervalOverride > 0 ? m_UpdatePathIntervalOverride : UpdatePathInterval;

            if (ForcePathUpdate || (update_path_interval > 0 && (time - LastPathUpdateTime) > update_path_interval))
            {
                ForcePathUpdate = false;
                LastPathUpdateTime = time;
                UpdatePath();
            }

            if (Path.Count > 0)
            {
                UpdateAntiStuck();
                UpdateDestReachFailed();
            }

            Thread.Sleep(50);
        }

        private Int64 LastPathUpdateTime = 0;

        // Controls updated thread execution
        private volatile bool ShouldStopUpdates = false;

        private void UpdatePath()
        {
            if (!m_Navmesh.IsNavDataAvailable)
                return;

            var dest = Destination;
            Vec3 current_pos = CurrentPos;

            if (current_pos.IsZero() || dest.pos.IsZero())
                return;

            var new_path = new List<Vec3>();
            var new_path_recalc_trigger_position = Vec3.ZERO;
            float new_path_recalc_trigger_precision = 0;

            if (dest.type == DestType.RunAway)
            {
                FindAvoidancePath(current_pos, ThreatThreshold, MovementFlags, ref new_path, Vec3.ZERO, false, PathNodesShiftDist);
                //m_Navmesh.Log($"avoidance path calculated {new_path.Count} nodes");
            }
            else
                FindPath(current_pos, dest.pos, MovementFlags, ref new_path, out new_path_recalc_trigger_position, out new_path_recalc_trigger_precision, PATH_NODES_MERGE_DISTANCE, as_close_as_possible: true, include_from: false, random_coeff: m_PathRandomCoeffOverride > 0 ? m_PathRandomCoeffOverride : PathRandomCoeff, bounce: m_PathBounce, shift_nodes_distance: PathNodesShiftDist, shift_dest: dest.shift, smoothen_distance: PathSmoothingDistance, allow_rough_path: AllowRoughPath);

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
                if (Path.Count == 0 || (new_path.Count > 0 && !Path[0].Equals(new_path[0])))
                    ResetAntiStuckPrecition(current_pos);

                //m_Navmesh.Log($"final path has {new_path.Count} nodes");

                Path = new_path;
                PathRecalcTriggerPosition = new_path_recalc_trigger_position;
                PathRecalcTriggerPrecision = new_path_recalc_trigger_precision;
                m_PathDestination = dest;
            }
        }

        private void UpdateGridDestination()
        {
            Vec3 dest_pos = Vec3.ZERO;
            bool grid_dest_found = false;
            List<int> destination_grids_id = null;

            using (new ReadLock(InputLock))
                destination_grids_id = new List<int>(m_DestinationGridsId);

            if (destination_grids_id.Count > 0)
            {
                using (m_Navmesh.AcquireReadDataLock())
                {
                    GridCell current_grid = m_Navmesh.m_GridCells.FirstOrDefault(x => x.Contains2D(CurrentPos));

                    GridCell destination_grid = m_Navmesh.m_GridCells.FirstOrDefault(x => destination_grids_id.Contains(x.Id) && Algorihms.AreConnected(current_grid, ref x, MovementFlag.None));

                    if (destination_grid != null)
                    {
                        grid_dest_found = true;
                        dest_pos = m_Navmesh.GetNearestCell(destination_grid.GetCells(), destination_grid.Center).Center;
                    }
                }
            }

            if (grid_dest_found)
                SetDestination(new destination(dest_pos, DestType.Grid, GridDestPrecision));
        }

        private void UpdateBackTrackDestination()
        {
            Vec3 dest_pos = Vec3.ZERO;
            bool backtrack_dest_found = false;

            using (new ReadLock(InputLock))
            {
                if (BackTrackEnabled)
                {
                    backtrack_dest_found = true;
                    dest_pos = m_DestinationsHistory[m_HistoryDestId];
                }
            }

            if (backtrack_dest_found)
                SetDestination(new destination(dest_pos, DestType.BackTrack, DefaultPrecision));
        }

        private void UpdateRingDestination()
        {
            var dest = RingDestination;

            if (dest.precision_max <= 0)
                return;

            var current_pos = CurrentPos;
            float distance = current_pos.Distance2D(dest.pos);
            bool too_far = distance > dest.precision_max;
            bool too_close = distance < dest.precision;

            if (distance < dest.precision || distance > dest.precision_max || !m_Navmesh.RayCast2D(dest.pos, current_pos, MovementFlag.Walk))
            {
                const int RAYS_COUNT = 16;
                const float ROTATE_STEP_ANGLE = 360 / RAYS_COUNT;
                Vec3 ROTATE_AXIS = new Vec3(0, 0, 1);

                Vec3[] DESTINATIONS = new Vec3[RAYS_COUNT + 1];
                Vec3 start_dir = new Vec3(1, 0, 0);
                for (int i = 0; i < RAYS_COUNT; ++i)
                    DESTINATIONS[i] = dest.pos + Vec3.Rotate(start_dir, i * ROTATE_STEP_ANGLE, ROTATE_AXIS) * dest.precision_max;
                DESTINATIONS[RAYS_COUNT] = dest.pos + (current_pos - dest.pos).Normalized2D() * dest.precision_max;

                DESTINATIONS = DESTINATIONS.OrderBy(x => x.Distance2D(current_pos)).ToArray();

                Vec3 best_dest = Vec3.ZERO;
                Vec3 furthest_dest = current_pos;
                float furthest_dest_dist = -1;

                //find best visible spot round the destination
                foreach (var dest_to_test in DESTINATIONS)
                {
                    var dest_pos = m_Navmesh.RayCast2D(dest.pos, dest_to_test, MovementFlag.Walk).End;
                    var dist = dest_pos.Distance2D(dest.pos);

                    if (IsThreatAt(dest_pos))
                        continue;

                    if (dist >= dest.precision && dist <= dest.precision_max)
                    {
                        best_dest = dest_pos;
                        break;
                    }

                    if (furthest_dest_dist < 0 || dist > furthest_dest_dist)
                    {
                        furthest_dest = dest_pos;
                        furthest_dest_dist = dist;
                    }
                }

                if (best_dest.IsZero())
                    best_dest = furthest_dest;

                SetDestination(new destination(best_dest, dest.type, DefaultPrecision * 0.8f, 0, dest.user_data) { is_ring = true/*, shift = true*/ });
            }
            else
                SetDestination(new destination(current_pos, dest.type, DefaultPrecision * 0.8f, 0, dest.user_data) { is_ring = true });
        }

        private void UpdateThreatAvoidance()
        {
            if (EnableThreatAvoidance)
            {
                Vec3 current_pos = CurrentPos;
                var threats = m_Navmesh.Regions.Where(x => x.Threat > ThreatThreshold).ToList();
                var current_threats = threats.Where(x => x.Area.Contains2D(current_pos));

                bool was_in_threat = IsInThreat;
                IsInThreat = current_threats.Any();

                if (IsInThreat)
                {
                    SetDestination(new destination(new Vec3(6,6,6), DestType.RunAway, DefaultPrecision * 0.3f));

                    //if (!was_in_threat)
                    //    m_Navmesh.Log("now in threat");

                    IsThreatAhead = false; // don't make user stop
                }
                else
                {
                    ClearDestination(DestType.RunAway);
                    if (was_in_threat)
                    {
                        RequestPathUpdate();
                        //m_Navmesh.Log("no longer in threat");
                    }

                    Vec3 go_to_pos = GoToPosition;

                    // if we are heading to point on a path we have to check if there is a threat-region on our path
                    if (!go_to_pos.IsZero())
                    {
                        Vec3 threat_pos = default(Vec3);
                        bool threat_ahead = false;
                        
                        // since often times we will be traveling along the regions we need to test with slightly minimized region
                        foreach (var t in threats)
                        {
                            if (t.Area.Resized(-ThreatDetectionPrecision).SegmentTest2D(current_pos, go_to_pos, ref threat_pos))
                            {
                                threat_ahead = current_pos.Distance2D(threat_pos) <= ThreatDetectionRange;
                                break;
                            }
                        }

                        IsThreatAhead = threat_ahead;
                    }
                }
            }
            else
            {
                IsInThreat = false;
                IsThreatAhead = false;
                ClearDestination(DestType.RunAway);
            }
        }

        private void UpdateWaypointDestination()
        {
            Vec3 dest_pos = Vec3.ZERO;
            bool waypoint_dest_found = false;

            using (new ReadLock(InputLock))
            {
                if (m_Waypoints.Count > 0)
                {
                    waypoint_dest_found = true;
                    dest_pos = m_Waypoints[0];
                }
            }

            if (waypoint_dest_found)
                SetDestination(new destination(dest_pos, DestType.Waypoint, DefaultPrecision));
        }

        private void UpdatePathProgression(Vec3 current_pos)
        {
            bool any_node_reached = false;
            Vec3 reached_pos = Vec3.ZERO;

            // check and removed reached nodes from path
            using (new ReadLock(PathLock, true))
            {
                while (Path.Count > 0)
                {
                    float precision = DefaultPrecision;

                    if (m_PrecisionOverride > 0)
                        precision = m_PrecisionOverride;
                    else if (Path.Count == 1)
                        precision = m_PathDestination.precision;

                    bool destination_reached = false;

                    Vec3 dest_pos = Path.Last();
                    // if there are no obstacles between current position and destination when in arrival radius, consider reached
                    if (current_pos.Distance2D(dest_pos) <= m_PathDestination.precision && m_Navmesh.RayCast2D(current_pos, dest_pos, MovementFlag.Walk))
                        destination_reached = true;

                    if (current_pos.Distance2D(Path[0]) > precision && !destination_reached)
                        break;

                    using (new WriteLock(PathLock))
                    {
                        if (destination_reached)
                        {
                            reached_pos = Path.Last();
                            Path.Clear();
                        }
                        else
                        {
                            reached_pos = Path[0];
                            Path.RemoveAt(0);
                        }
                    }

                    any_node_reached = true;

                    ResetAntiStuckPrecition(current_pos);
                }
            }

            // update destination arrived
            if (any_node_reached)
            {
                if (IsDestinationReached(DestType.All))
                {
                    using (new WriteLock(InputLock))
                    {
                        if (m_Destination.type != DestType.BackTrack &&
                            (m_DestinationsHistory.Count == 0 || (!m_DestinationsHistory[m_DestinationsHistory.Count - 1].Equals(reached_pos) &&
                                                                    m_DestinationsHistory[m_DestinationsHistory.Count - 1].Distance(reached_pos) > MinDestDistToAddToHistory)))
                        {
                            m_DestinationsHistory.Add(reached_pos);
                        }
                    }

                    NotifyOnDestinationReached(m_PathDestination);

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
                //m_Navmesh.Log("[Nav] Destination " + dest + " [" + dest_type + "] reach failed!");

                NotifyOnDestinationReachFailed(Destination);
                ResetDestReachFailed(curr_pos);
            }
        }

        private void UpdateAntiStuck()
        {
            if (!EnableAntiStuck)
                return;

            const float MIN_TIME_TO_RECALCULATE_PATH = 2000;
            const float MIN_TIME_TO_OVERRIDE_PRECISION = 4000;
            const float MIN_TIME_TO_BOUNCE = 6000;
            const float MIN_TIME_TO_OVERRIDE_PATH_RANDOM_COEFF = 9000;

            Vec3 curr_pos = CurrentPos;

            using (new ReadLock(AntiStuckLock, true))
            {
                if (m_AntiStuckPrecisionTestPos.IsZero() || m_AntiStuckPrecisionTestPos.Distance(curr_pos) > MinDistToResetAntiStuckPrecision)
                    ResetAntiStuckPrecition(curr_pos);
                else if (IsStandingOnPurpose)
                    m_AntiStuckPrecisionTimer.Stop();
                else
                    m_AntiStuckPrecisionTimer.Start();

                if (m_AntiStuckPathingTestPos.IsZero() || m_AntiStuckPathingTestPos.Distance(curr_pos) > MinDistToResetAntiStuckPathing)
                    ResetAntiStuckPathing(curr_pos);
                else if (IsStandingOnPurpose)
                    m_AntiStuckPathingTimer.Stop();
                else
                    m_AntiStuckPathingTimer.Start();

                // handle anti stuck precision management features
                if (m_AntiStuckPrecisionTimer.ElapsedMilliseconds > MIN_TIME_TO_OVERRIDE_PRECISION)
                    m_PrecisionOverride = AntiStuckPrecisionOverride;

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

            if (pos.IsZero())
                return;

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

        public virtual void OnNavDataChanged(AABB affected_area)
        {
            //m_Navmesh.Log($"[Nav] Request path update OnNavDataChanged.");
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

        protected void NotifyOnDestinationReached(destination dest)
        {
            List<INavigationObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavigationObserver observer in observers_copy)
                observer.OnDestinationReached(dest);
        }

        protected void NotifyOnDestinationReachFailed(destination dest)
        {
            List<INavigationObserver> observers_copy = null;

            using (new ReadLock(InputLock))
                observers_copy = m_Observers.ToList();

            foreach (INavigationObserver observer in observers_copy)
                observer.OnDestinationReachFailed(dest);
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

                w.Write(Path.Count);
                foreach (Vec3 p in Path)
                    p.Serialize(w);
                m_PathDestination.pos.Serialize(w);
                w.Write((int)m_PathDestination.type);

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
                m_Destination.pos.Serialize(w);
                w.Write((int)m_Destination.type);

                w.Write(PathRecalcTriggerPrecision);
                PathRecalcTriggerPosition.Serialize(w);

                w.Write(UpdatePathInterval);
                w.Write(CurrentPosDiffRecalcThreshold);
                w.Write(PathNodesShiftDist);
                w.Write(PathRandomCoeff);
                w.Write(DefaultPrecision);
                w.Write(AllowRoughPath);
                w.Write(PathSmoothingDistance);
                w.Write(PathSmoothingPrecision);
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
                Path.Clear();
                m_DestinationsHistory.Clear();
                m_DebugPositionsHistory.Clear();

                int waypoints_count = r.ReadInt32();
                for (int i = 0; i < waypoints_count; ++i)
                    m_Waypoints.Add(new Vec3(r));

                int path_count = r.ReadInt32();
                for (int i = 0; i < path_count; ++i)
                    Path.Add(new Vec3(r));
                m_PathDestination.pos = new Vec3(r);
                m_PathDestination.type = (DestType)r.ReadInt32();

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
                m_Destination.pos = new Vec3(r);
                m_Destination.type = (DestType)r.ReadInt32();

                PathRecalcTriggerPrecision = r.ReadSingle();
                PathRecalcTriggerPosition = new Vec3(r);

                UpdatePathInterval = r.ReadInt32();
                CurrentPosDiffRecalcThreshold = r.ReadSingle();
                PathNodesShiftDist = r.ReadSingle();
                PathRandomCoeff = r.ReadSingle();
                DefaultPrecision = r.ReadSingle();
                AllowRoughPath = r.ReadBoolean();
                PathSmoothingDistance = r.ReadSingle();
                PathSmoothingPrecision = r.ReadSingle();
            }

            Vec3 curr_pos = CurrentPos;

            ResetAntiStuckPrecition(curr_pos);
            ResetAntiStuckPathing(curr_pos);
        }

        private float PATH_NODES_MERGE_DISTANCE = -1;

        private Thread UpdatesThread = null;
        private volatile bool ForcePathUpdate = false;

        private ReaderWriterLockSlim PathLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private ReaderWriterLockSlim InputLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private ReaderWriterLockSlim AntiStuckLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);

        private List<Vec3> Path = new List<Vec3>(); //@ PathLock
        internal Vec3 PathRecalcTriggerPosition = Vec3.ZERO; //@ PathLock
        internal float PathRecalcTriggerPrecision = 0; //@ PathLock
        private destination m_PathDestination = default(destination); //@ PathLock
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
        private destination m_Destination = default(destination); //@ InputLock
        private destination m_RingDestination = default(destination); //@ InputLock
        private List<INavigationObserver> m_Observers = new List<INavigationObserver>(); //@ InputLock

        public IRoughPathEstimator m_RoughtPathEstimator;

        private Navmesh m_Navmesh = null;
    }
}
