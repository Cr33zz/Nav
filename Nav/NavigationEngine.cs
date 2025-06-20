﻿using System;
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
        public destination(Vec3 pos, DestType type = DestType.User, float precision = 0, bool stop = false, bool stop_at_edge = false, bool as_close_as_possible = true, bool keep_moving_while_repath = true, Object user_data = null, string debug_annotation = null)
        {
            this.pos = pos;
            this.type = type;
            this.precision = precision;
            this.stop = stop;
            this.stop_at_edge = stop_at_edge;
            this.debug_annotation = debug_annotation;
            this.user_data = user_data;
            this.as_close_as_possible = as_close_as_possible;
            this.keep_moving_while_repath = keep_moving_while_repath;
            this.shift = false;
        }

        public override bool Equals(Object obj)
        {
            return obj is destination d && Equals(d);
        }

        public bool Equals(destination d)
        {
            return pos.Equals(d.pos) && type == d.type && precision == d.precision && user_data == d.user_data && as_close_as_possible == d.as_close_as_possible && keep_moving_while_repath == d.keep_moving_while_repath && stop == d.stop && stop_at_edge == d.stop_at_edge;
        }

        public override int GetHashCode()
        {
            return pos.GetHashCode() ^ type.GetHashCode() ^ precision.GetHashCode() ^ user_data.GetHashCode() ^ as_close_as_possible.GetHashCode() ^ keep_moving_while_repath.GetHashCode() ^ stop.GetHashCode() ^ stop_at_edge.GetHashCode();
        }

        public void Serialize(BinaryWriter w)
        {
            pos.Serialize(w);
            w.Write((Int32)type);
            w.Write(precision);
            w.Write(stop);
            w.Write(stop_at_edge);
            w.Write(as_close_as_possible);
            w.Write(keep_moving_while_repath);
            w.Write(shift);
        }

        public void Deserialize(BinaryReader r)
        {
            pos = new Vec3(r);
            type = (DestType)r.ReadInt32();
            precision = r.ReadSingle();
            stop = r.ReadBoolean();
            stop_at_edge = r.ReadBoolean();
            as_close_as_possible = r.ReadBoolean();
            keep_moving_while_repath = r.ReadBoolean();
            shift = r.ReadBoolean();
        }

        public Vec3 pos;
        public DestType type;
        public float precision;
        public bool stop;
        public bool stop_at_edge;
        public bool as_close_as_possible;
        public bool keep_moving_while_repath;
        public string debug_annotation;
        public Object user_data;
        internal bool shift;
    }

    public struct goto_data
    {
        public goto_data(Vec3 pos, float precision, bool stop = true, destination path_destination = default(destination), Vec3 destination_pos = default(Vec3))
        {
            this.pos = pos;
            this.precision = precision;
            this.stop = stop;
            this.path_destination = path_destination;
            this.destination_pos = path_destination.pos;
        }

        public Vec3 pos;
        public float precision;
        public bool stop;
        public destination path_destination;
        public Vec3 destination_pos; // can be different from path destination when stopping at edge is enabled, can be used for distance checks to destination

        public bool is_going_to_dest => pos.Distance2D(destination_pos) < precision;
    }

    public struct path_data
    {
        internal path_data(List<Vec3> path, destination path_destination, Vec3 path_recalc_trigger_pos, float path_recalc_trigger_precision, Vec3 rough_path_destination)
        {
            this.path = path.ToList();
            this.path_destination = path_destination;
            this.path_recalc_trigger_pos = path_recalc_trigger_pos;
            this.path_recalc_trigger_precision = path_recalc_trigger_precision;
            this.rough_path_destination = rough_path_destination;
        }

        internal void Clear()
        {
            path.Clear();
            path_destination = default;
            path_recalc_trigger_pos = Vec3.ZERO;
            path_recalc_trigger_precision = 0;
            rough_path_destination = Vec3.ZERO;
        }

        internal List<Vec3> path;
        internal destination path_destination;
        internal Vec3 path_recalc_trigger_pos;
        internal float path_recalc_trigger_precision;
        internal Vec3 rough_path_destination;
    }

    public abstract class IPathFollowStrategy
    {
        public abstract void UpdatePathProgress(Vec3 current_pos, ref List<Vec3> path, destination path_destination, Navmesh navmesh);
        public abstract (Vec3, float) UpdateThreatAhead(Vec3 current_pos, IEnumerable<Region> threats);
        public abstract bool IncludePathStart();
        public virtual void OnPathReset()
        {
            m_Path?.Clear();
            m_PathDestination = default;
            m_GoToData = new goto_data(Vec3.ZERO, 0);
        }
        
        public virtual void GetPath(out List<Vec3> path, out destination path_dest)
        {
            path = m_Path;
            path_dest = m_PathDestination;
        }

        public void Connect(NavigationEngine owner)
        {
            m_Owner = owner;
        }

        public goto_data m_GoToData { get; protected set; }
        protected List<Vec3> m_Path;
        protected destination m_PathDestination;
        protected NavigationEngine m_Owner;
    }

    public class SimplePathFollow : IPathFollowStrategy
    {
        public override void UpdatePathProgress(Vec3 current_pos, ref List<Vec3> path, destination path_destination, Navmesh navmesh)
        {
            m_Path = path;
            m_PathDestination = path_destination;

            Vec3 dest_pos = m_Path.Last();
            // if there are no obstacles between current position and destination when in arrival radius, consider reached

            if (current_pos.Distance2D(dest_pos) <= path_destination.precision && m_Path.Count > 1 && navmesh.DataLock.TryEnterReadLock(10))
            {
                if (navmesh.RayCast2D(current_pos, dest_pos, MovementFlag.Walk))
                {
                    path.Clear();
                    m_GoToData = new goto_data(Vec3.ZERO, 0);
                }
                navmesh.DataLock.ExitReadLock();
            }

            while (path.Count > 0)
            {
                float precision = m_Owner.GetPrecision();

                if (current_pos.Distance2D(path[0]) > precision)
                    break;

                path.RemoveAt(0);

                m_Owner.ResetAntiStuckPrecition(current_pos);
            }

            if (path.Count > 0)
            {
                var pos = path[0];
                if (m_Owner.AlignGoToPositionToCurrentPosZWhenZero && pos.Z == 0)
                    pos.Z = current_pos.Z;
                m_GoToData = new goto_data(pos, m_Owner.GetPrecision(), stop: path.Count == 1 && path_destination.stop, path_destination: path_destination, path_destination.pos);
            }
            else 
                m_GoToData = new goto_data(Vec3.ZERO, 0);
        }

        public override (Vec3, float) UpdateThreatAhead(Vec3 current_pos, IEnumerable<Region> threats)
        {
            float threat_ahead = m_Owner.GetThreatBetween(current_pos, m_GoToData.pos, out var threat_pos, m_Owner.ThreatAheadThreshold, regions_cache: threats);

            if (current_pos.Distance2D(threat_pos) <= m_Owner.ThreatDetectionRange)
            {
                return (threat_pos, threat_ahead);
            }

            return (Vec3.ZERO, 0);
        }

        public override bool IncludePathStart()
        {
            return false;
        }
    }

    public class NavigationEngine : IDisposable, INavmeshObserver
    {
        public NavigationEngine(Navmesh navmesh, IPathFollowStrategy pathFollowStrategy)
        {
            m_Navmesh = navmesh;
            m_Navmesh.AddObserver(this);

            m_PathFollowStrategy = pathFollowStrategy;
            m_PathFollowStrategy.Connect(this);

            UpdatesThread = new Thread(Updates);
            UpdatesThread.Name = "Navigator-UpdatesThread";
            UpdatesThread.Start();
        }

        public void AddObserver(INavigationObserver observer)
        {
            using (new WriteLock(InputLock, "InputLock - NavigationEngine.AddObserver"))
            {
                if (m_Observers.Contains(observer))
                    return;

                m_Observers.Add(observer);
            }
        }

        public void RemoveObserver(INavigationObserver observer)
        {
            using (new WriteLock(InputLock, "InputLock - NavigationEngine.RemoveObserver"))
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
        public Int64 PathFindingTimeLimit { get; set; } = -1;

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

        // consider future threats as current threats for the following milliseconds since last actual threat avoidance was triggered
        public Int64 FutureThreatAvoidanceDuration { get; set; } = 2000;

        public float ThreatAheadThreshold { get; set; } = 0;
        public float ThreatThreshold { get; set; } = 0;

        // when avoidance is enabled and current position is on a cell with threat level higher than MaxAllowedThreat
        public bool IsInThreat { get; private set; } = false;
        public float Threat { get; private set; } = 0;
        public bool IsAvoidingFutureThreats { get; private set; } = false;

        // this function is using ThreatThreshold
        public bool IsThreatAt(Vec3 pos, float radius = 0, bool consider_future_threats = false, IEnumerable<Region> regions_cache = null)
        {
            regions_cache = regions_cache ?? m_Navmesh.Regions;

            if (radius <= 0)
                return regions_cache.Any(x => (consider_future_threats ? Math.Abs(x.Threat) : x.Threat) >= ThreatThreshold && x.Area.Contains2D(pos));

            return regions_cache.Any(x => (consider_future_threats ? Math.Abs(x.Threat) : x.Threat) >= ThreatThreshold && x.Area.Overlaps2D(pos, radius));
        }

        public List<Region> GetRegions(AABB area)
        {
            AABB output = default(AABB);
            return m_Navmesh.Regions.Where(x => x.Area.Intersect2D(area, ref output)).ToList();
        }

        public float GetThreatAt(Vec3 pos)
        {
            return m_Navmesh.Regions.FirstOrDefault(x => x.Area.Contains2D(pos)).Threat;
        }

        public float GetThreatBetween(Vec3 start, Vec3 end, out Vec3 closest_threat_pos, float minThreat = 0, bool consider_future_threats = false, IEnumerable<Region> regions_cache = null)
        {
            regions_cache = regions_cache ?? m_Navmesh.Regions;
            var threat_pos = default(Vec3);
            closest_threat_pos = default(Vec3);

            float closest_dist = -1;
            float threat_ahead = 0;

            // since often times we will be traveling along the regions we need to test with slightly minimized region
            foreach (var t in regions_cache)
            {
                var threat = consider_future_threats ? Math.Abs(t.Threat) : t.Threat;

                if (threat < minThreat)
                    continue;

                if (t.Area.Resized(-ThreatDetectionPrecision).SegmentTest2D(start, end, ref threat_pos))
                {
                    var dist = start.Distance2D(threat_pos);

                    if (closest_dist < 0 || dist < closest_dist)
                    {
                        threat_ahead = threat;
                        closest_dist = dist;
                        closest_threat_pos = threat_pos;
                    }
                }
            }

            return threat_ahead;
        }

        // this function is using ThreatThreshold
        public bool IsThreatBetween(Vec3 start, Vec3 end)
        {
            Vec3 threat_pos = default(Vec3);

            foreach (var t in m_Navmesh.Regions.Where(x => x.Threat >= ThreatThreshold).ToList())
            {
                if (t.Area.SegmentTest2D(start, end, ref threat_pos))
                    return true;
            }

            return false;
        }

        public float ThreatBetween(Vec3 start, Vec3 end)
        {
            Vec3 threat_pos = default(Vec3);
            float threat = 0;

            foreach (var t in m_Navmesh.Regions.Where(x => x.Threat >= ThreatThreshold).ToList())
            {
                if (t.Area.SegmentTest2D(start, end, ref threat_pos))
                    threat = Math.Max(threat, t.Threat);
            }

            return threat;
        }

        public (Vec3, float) ThreatAhead { get; private set; } = (Vec3.ZERO, 0);

        // when not in threat but path leads through threats IsThreatAhead will be turned when agent is closer than ThreatDetectionRange from a threat ahead
        public float ThreatDetectionRange { get; set; } = 10;

        public float ThreatDetectionPrecision { get; set; } = 0.5f;

        // should be used when EnableAntiStuck is true to notify navigator that actor is not blocked by some obstacle but just standing
        public bool IsStandingOnPurpose { get; set; } = true;

        // is current path for currently requested destination type (may be false when destination change has been requested but path is not updated yet)
        public bool IsPathUpToDate => m_Path.path_destination.pos == m_Destination.pos;

        public bool UseCellsCenters { get; set; } = false;

        public List<int> DestinationGridsId
        {
            get
            {
                using (new ReadLock(InputLock))
                    return new List<int>(m_DestinationGridsId);
            }

            set
            {
                using (new WriteLock(InputLock, "InputLock - NavigationEngine.DestinationGridsId"))
                {
                    if (value == null)
                        m_DestinationGridsId.Clear();
                    else
                        m_DestinationGridsId = new List<int>(value);
                }
            }
        }

        public bool FindPath(Vec3 from, Vec3 to, ref List<Vec3> path, out bool timed_out, bool as_close_as_possible, bool allow_rought_path, float random_coeff = 0, float nodes_shift_dist = 0, float smoothen_distance = float.MaxValue, bool ignore_movement_cost = false, bool can_time_out = true)
        {
            return FindPath(from, to, MovementFlags, m_Path, out var __unused, ref path, out timed_out, out var path_recalc_trigger_position, out var path_recalc_trigger_dist, out var unused, PATH_NODES_MERGE_DISTANCE, as_close_as_possible, false, random_coeff, m_PathBounce, nodes_shift_dist, false, smoothen_distance, allow_rought_path, ignore_movement_cost, can_time_out);
        }

        public bool FindPath(Vec3 from, Vec3 to, MovementFlag flags, path_data current_path, out string __tmp_crash_dbg, ref List<Vec3> path, out bool timed_out, out Vec3 path_recalc_trigger_position, out float path_recalc_trigger_precision, out Vec3 rough_path_destination, float merge_distance = -1, bool as_close_as_possible = false, bool include_from = false, float random_coeff = 0, bool bounce = false, float shift_nodes_distance = 0, bool shift_dest = false, float smoothen_distance = float.MaxValue, bool allow_rough_path = false, bool ignore_movement_cost = false, bool can_time_out = true)
        {
            __tmp_crash_dbg = "0";
            using (new Profiler("Path finding (incl. lock) took %t", 100))
            using (m_Navmesh.AcquireReadDataLock("NavigationEngine.FindPath"))
            using (new Profiler($"Path finding took %t [from: {from}, to: {to}]", 100))
            {
                __tmp_crash_dbg = "1";
                path_recalc_trigger_position = Vec3.ZERO;
                path_recalc_trigger_precision = 0;
                rough_path_destination = Vec3.ZERO;
                timed_out = false;

                if (from.IsZero() || to.IsZero())
                    return false;

                __tmp_crash_dbg = "2";

                var original_to = to;

                bool start_on_nav_mesh = m_Navmesh.GetCellAt(from, out Cell start, flags, false, true, true);
                bool end_on_nav_mesh = m_Navmesh.GetCellAt(to, out Cell end, flags, false, true, as_close_as_possible);

                __tmp_crash_dbg = "3";

                // align from position to closest cell
                if (!start_on_nav_mesh)
                {
                    if (start == null)
                        return false;

                    from = start.AABB.Align(from);
                }

                __tmp_crash_dbg = "4";

                // align to position to closest cell
                if (!end_on_nav_mesh)
                {
                    if (end == null)
                        return false;

                    to = end.AABB.Align(to);
                }

                __tmp_crash_dbg = "5";

                int are_connected = -1;

                if (!as_close_as_possible)
                {
                    are_connected = m_Navmesh.AreConnected(from, to, MovementFlag.Walk, 0, 0) ? 1 : 0;
                    if (are_connected == 0)
                        return false;
                }

                __tmp_crash_dbg = "6";

                List<Vec3> rough_path = new List<Vec3>();
                string rough_path_debug_info = "";

                if (m_RoughtPathEstimator != null && allow_rough_path)
                {
                    are_connected = are_connected == -1 ? (m_Navmesh.AreConnected(from, to, MovementFlag.Walk, 10, 0) ? 1 : 0) : are_connected;
                    if (are_connected == 1)
                        m_RoughtPathEstimator.FindRoughPath(from, to, ref rough_path, out rough_path_debug_info);
                }

                __tmp_crash_dbg = "7";

                bool use_time_limit = can_time_out && current_path.path_destination.pos == original_to && current_path.path.Count > 0;

                __tmp_crash_dbg = "8";

                // check if current rough path destination is still valid
                // sometimes path is not aligned with rough path and it may lead to some going back and forth stuck
                // by continue to use already selected rough path destination we can avoid that problem
                bool keep_using_rough_destination =
                    allow_rough_path &&
                    are_connected == 1 &&
                    current_path.path_destination.pos == original_to &&
                    !current_path.rough_path_destination.IsZero() &&
                    CurrentPos.Distance2D(current_path.rough_path_destination) > m_RoughtPathEstimator.GetRoughPathRecalcPrecision();

                __tmp_crash_dbg = "9";

                if (keep_using_rough_destination)
                {
                    __tmp_crash_dbg = "9a";
                    rough_path.Clear();
                    rough_path.Add(to);

                    to = current_path.rough_path_destination;
                    m_Navmesh.GetCellAt(to, out end, flags, false, true, as_close_as_possible);

                    rough_path_destination = current_path.rough_path_destination;
                    path_recalc_trigger_position = current_path.rough_path_destination;
                    path_recalc_trigger_precision = m_RoughtPathEstimator.GetRoughPathRecalcPrecision();
                }
                else if (rough_path.Count > 4)
                {
                    __tmp_crash_dbg = "9b";
                    rough_path_destination = rough_path[4];

                    //Trace.WriteLine($"overriding path destination {original_to} with rough destination {rough_path_destination} (path: {string.Join(" - ", rough_path)}, {rough_path_debug_info})");

                    // purely for debug
                    //if ()

                    rough_path.Clear();
                    rough_path.Add(to);

                    to = rough_path_destination;
                    m_Navmesh.GetCellAt(to, out end, flags, false, true, as_close_as_possible);

                    // ideally this point should be set somewhere along the path to rough path node (but it would required going over the path to find it afterwards)
                    path_recalc_trigger_position = to;
                    path_recalc_trigger_precision = m_RoughtPathEstimator.GetRoughPathRecalcPrecision();
                }
                else
                {
                    __tmp_crash_dbg = "9c";
                    rough_path.Clear();
                }

                __tmp_crash_dbg = "10";

                List<path_pos> tmp_path = new List<path_pos>();

                bool bounced = false;
                
                if (bounce)
                {
                    __tmp_crash_dbg = "10b";

                    Vec3 bounce_dir = start.AABB.GetBounceDir2D((to - from).Normalized2D(), m_Navmesh.Rng);
                    Vec3 new_from = from + bounce_dir * BounceDist;
                    m_Navmesh.GetCellAt(new_from, out var startBounce, flags, false, true, as_close_as_possible);

                    if (startBounce != null)
                    {
                        new_from = startBounce.AABB.Align(new_from);

                        if (!Algorihms.FindPath(startBounce, new_from, new Algorihms.DestinationPathFindStrategy<Cell>(to, end, m_Navmesh), flags, ref tmp_path, out timed_out, random_coeff, allow_disconnected: as_close_as_possible, use_cell_centers: UseCellsCenters, ignore_movement_cost: ignore_movement_cost, time_limit: use_time_limit ? PathFindingTimeLimit : -1))
                        {
                            Trace.WriteLine($"Failed to find bounce path between {from} and {to}, are connected {are_connected}, timed out {timed_out}, use time limit {use_time_limit}");
                            return false;
                        }

                        tmp_path.Insert(0, new path_pos(startBounce.AABB.Align(from), startBounce));

                        bounced = true;
                    }
                }

                if (!bounced)
                {
                    __tmp_crash_dbg = "11";
                    using (new Profiler($"Path finding algorithm took %t [rough: {rough_path.Count}, connected: {are_connected}]", 100))
                    {
                        if (!Algorihms.FindPath(start, from, new Algorihms.DestinationPathFindStrategy<Cell>(to, end, m_Navmesh), flags, ref tmp_path, out timed_out, random_coeff, allow_disconnected: as_close_as_possible, use_cell_centers: UseCellsCenters, ignore_movement_cost: ignore_movement_cost, time_limit: use_time_limit ? PathFindingTimeLimit : -1))
                        {
                            Trace.WriteLine($"Failed to find path between {from} and {to}, as_close_as_possible {as_close_as_possible}, original_to {original_to}, bounce {bounce}, start_cell {start?.GlobalId ?? -1}, start_on_nav_mesh {start_on_nav_mesh}, end_cell {end?.GlobalId ?? -1}, end_on_nav_mesh {end_on_nav_mesh}, are_connected {are_connected}, rough_path_dest {rough_path_destination}, rough_path_size {rough_path.Count}, keep_using_rough_destination {keep_using_rough_destination}, timed_out {timed_out}, use_time_limit {use_time_limit}");
                            return false;
                        }
                    }
                }

                __tmp_crash_dbg = "12";

                if (smoothen_distance > 0 && random_coeff == 0)
                    SmoothenPath(ref tmp_path, flags, smoothen_distance, ref path_recalc_trigger_position, ref path_recalc_trigger_precision, bounce ? 1 : 0);

                __tmp_crash_dbg = "13";

                if (DistToKeepFromEdge > 0 && random_coeff == 0)
                    KeepAwayFromEdges(ref tmp_path, flags);

                __tmp_crash_dbg = "14";

                path = tmp_path.Select(x => x.pos).Concat(rough_path).ToList();

                __tmp_crash_dbg = "15";

                PostProcessPath(ref path, merge_distance, shift_nodes_distance, shift_dest);

                __tmp_crash_dbg = "16";

                if (!include_from && start_on_nav_mesh)
                    path.RemoveAt(0);

                __tmp_crash_dbg = "17";

                return true;
            }
        }

        public bool FindAvoidancePath(Vec3 from, float threat_threshold, MovementFlag flags, ref List<Vec3> path, Vec3 hint_pos = default(Vec3), bool include_from = false, float shift_nodes_distance = 0, float smoothen_distance = float.MaxValue)
        {
            using (m_Navmesh.AcquireReadDataLock("NavigationEngine.FindAvoidancePath"))
            {
                if (from.IsZero())
                    return false;

                bool start_on_nav_mesh = m_Navmesh.GetCellAt(from, out Cell start, flags, false, true, true);

                if (!start_on_nav_mesh)
                {
                    if (start == null)
                        return false;

                    from = start.AABB.Align(from);
                }

                List<path_pos> tmp_path = new List<path_pos>();

                if (!Algorihms.FindPath(start, from, new Algorihms.AvoidancePathFindStrategy<Cell>(threat_threshold, hint_pos), flags, ref tmp_path, out var timedOut))
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
                    var dir_to_center = last_node.cell.Center - last_node.pos;
                    var dist_to_center = dir_to_center.Normalize();
                    //var last_point = last_node.pos + dir_to_center * Math.Min(m_AvoidanceDestination.precision, dist_to_center);
                    var last_point = last_node.pos + dir_to_center * m_AvoidanceDestination.precision;
                    tmp_path.Add(new path_pos(last_point, last_node.cell));
                }

                path = tmp_path.Select(x => x.pos).ToList();

                PostProcessPath(ref path, -1, shift_nodes_distance, true);

                if (!include_from && start_on_nav_mesh)
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

        //public destination RingDestination
        //{
        //    get
        //    {
        //        using (new ReadLock(InputLock))
        //            return m_RingDestination;
        //    }
        //}

        public destination Destination
        {
            get
            {
                using (new ReadLock(InputLock))
                    return m_Destination;
            }

            set
            {
                SetDestination(new destination(value.pos, value.type, value.precision <= 0 ? DefaultPrecision : value.precision, value.stop, value.stop_at_edge, value.as_close_as_possible, value.keep_moving_while_repath, value.user_data, value.debug_annotation));
            }
        }

        public void SetDestination(Vec3 pos, float precision, bool stop = false, bool stop_at_edge = false, bool as_close_as_possible = true, bool keep_moving_while_repath = true, Object user_data = null, string debug_annotation = null)
        {
            SetDestination(new destination(pos, DestType.User, precision <= 0 ? DefaultPrecision : precision, stop, stop_at_edge, as_close_as_possible, keep_moving_while_repath, user_data, debug_annotation));
        }

        //public void ClearRingDestinations()
        //{
        //    using (new WriteLock(InputLock, "InputLock - NavigationEngine.ClearRingDestinations"))
        //    {
        //        if (m_Destination.is_ring)
        //            m_Destination = default(destination);

        //        m_RingDestination = default(destination);

        //    }
        //}

        public void ClearAllDestinations()
        {
            ClearDestination(DestType.All);
        }

        public void ClearGridDestination()
        {
            ClearDestination(DestType.Grid);
        }

        public void SetCustomDestination(Vec3 pos, float precision = -1, bool stop = false, bool stop_at_edge = false, bool as_close_as_possible = true, bool keep_moving_while_repath = true, Object user_data = null, string debug_annotation = null)
        {
            SetDestination(new destination(pos, DestType.Custom, precision < 0 ? DefaultPrecision : precision, stop, stop_at_edge, as_close_as_possible, keep_moving_while_repath, user_data, debug_annotation));
        }

        public void ClearCustomDestination()
        {
            ClearDestination(DestType.Custom);
        }

        public bool TryGetPath(ref List<Vec3> path, out destination path_dest)
        {
            path_dest = default;

            if (PathLock.TryEnterReadLock(0))
            {
                path = new List<Vec3>(m_Path.path);
                path_dest = m_Path.path_destination;
                PathLock.ExitReadLock();
                return true;
            }

            return false;
        }

        public float TryGetPathLength(out destination path_dest)
        {
            float length = 0;
            path_dest = default;

            if (PathLock.TryEnterReadLock(0))
            {
                length = Algorihms.GetPathLength(m_Path.path, CurrentPos);
                path_dest = m_Path.path_destination;
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

        public Vec3 GetHistoricPosition(Vec3 referencePos, float minDistance, float connectedTolerance)
        {
            var refPosPatches = m_Navmesh.GetPatchesIds(referencePos, MovementFlag.Walk, connectedTolerance);

            using (new ReadLock(InputLock))
            {
                for (int i = m_DebugPositionsHistory.Count - 1; i >= 0; --i)
                {
                    var historicPos = m_DebugPositionsHistory[i];
                    var historicPosPatches = m_Navmesh.GetPatchesIds(historicPos, MovementFlag.Walk, connectedTolerance);

                    if (!m_Navmesh.AreConnected(refPosPatches, historicPosPatches))
                        break;

                    if (referencePos.Distance(historicPos) >= minDistance)
                        return historicPos;
                }

                return Vec3.ZERO;
            }
        }

        public List<Vec3> GetPath()
        {
            using (new ReadLock(PathLock))
                return new List<Vec3>(m_Path.path);
        }

        public DestType GetDestinationType()
        {
            return m_Destination.type;
        }

        public goto_data GoToPosition => m_PathFollowStrategy.m_GoToData;
        
        public Vec3 CurrentPos
        {
            get
            {
                return m_CurrentPos;
            }

            set
            {
                bool was_empty = false;
                float diff = 0;

                //using (new Profiler("curr pos [set] [%t]", 10))
                using (new WriteLock(InputLock, "InputLock - NavigationEngine.CurrentPos"))
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
                        path_empty = m_Path.path.Count == 0;
                }

                if (huge_current_pos_change || path_empty)
                {
                    if (huge_current_pos_change)
                        NotifyOnHugeCurrentPosChange();

                    RequestPathUpdate();
                }

                if (!m_Path.path_recalc_trigger_pos.IsZero() && 
                    m_Path.path_recalc_trigger_pos.Distance2D(value) < m_Path.path_recalc_trigger_precision &&
                    PathRecalcTriggerCooldown.ElapsedMilliseconds > 500)
                {
                    RequestPathUpdate();
                    PathRecalcTriggerCooldown.Restart();
                }

                if (was_empty)
                    ReorganizeWaypoints();

                //using (new Profiler("curr pos [path progress] [%t]", 10))
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
                using (new WriteLock(InputLock, "InputLock - NavigationEngine.Waypoints"))
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
            if (!UpdatesThread.Join(3000))
                UpdatesThread.Interrupt();

            m_Navmesh.RemoveObserver(this);
        }

        // Acquires InputLock (read -> write)
        public void ClearDestination(DestType type)
        {
            using (new ReadLock(InputLock, true, description: "InputLock - NavigationEngine.ClearDestination"))
            {
                if ((m_Destination.type & type) == 0)
                    return;

                using (new WriteLock(InputLock, "InputLock - NavigationEngine.ClearDestination"))
                {
                    //m_Navmesh.Log("[Nav] Dest [" + m_DestinationType + "] cleared using [" + type + "] flags!");

                    m_Destination = default(destination);
                }

                using (new WriteLock(PathLock, "PathLock - NavigationEngine.ClearDestination"))
                {
                    m_Path.Clear();
                    m_PathFollowStrategy.OnPathReset();
                }
            }
        }

        public void Clear()
        {
            using (new WriteLock(InputLock, "InputLock - NavigationEngine.Clear"))
            using (new WriteLock(PathLock, "PathLock - NavigationEngine.Clear"))
            {
                m_CurrentPos = Vec3.ZERO;
                m_Destination = default(destination);
                m_Waypoints.Clear();
                m_DestinationsHistory.Clear();
                m_DebugPositionsHistory.Clear();
                m_HistoryDestId = -1;
                m_DestinationGridsId.Clear();
                m_Path.Clear();
                m_PathFollowStrategy.OnPathReset();
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

            using (new WriteLock(InputLock, "InputLock - NavigationEngine.SetDestination"))
            //using (new WriteLock(InputLock, context: "SetDestination"))
            {
                m_Destination.user_data = dest.user_data;
                m_Destination.debug_annotation = dest.debug_annotation;

                if ((m_Destination.pos.Equals(dest.pos) && m_Destination.precision == dest.precision && m_Destination.type == dest.type) ||
                    (!m_Destination.pos.IsZero() && m_Destination.type > dest.type))
                {
                    return;
                }

                m_Destination = dest;

                //if (dest.type == DestType.Explore)
                //    Trace.WriteLine($"explore dest set to {dest.pos} ({dest.debug_annotation})");

                // clear ring destination when overwritten by any other destination that is not coming from ring destination update
                //if (!dest.is_ring && dest.precision_max <= 0)
                //    m_RingDestination = default(destination);
             
                //m_Navmesh.Log("[Nav] Dest changed to " + pos + " [" + type + "] precision " + precision);

                ResetDestReachFailed(m_CurrentPos);
            }

            RequestPathUpdate();
        }

        public void RequestPathUpdate()
        {
            //m_Navmesh.Log("Path update requested!");
            Interlocked.Exchange(ref ForcePathUpdate, 1);
        }

        private void Updates()
        {
            Stopwatch timer = new Stopwatch();
            timer.Start();

            while (!ShouldStopUpdates)
            {
                try
                {
                    OnUpdate(timer.ElapsedMilliseconds);
                }
                catch (Exception ex)
                {
                    Trace.WriteLine($"NavEngine exception {ex.Message}\n{ex.StackTrace}");
                }
            }
        }

        protected virtual void OnUpdate(Int64 time)
        {
            //Trace.WriteLine($"navigator on update [force path update - {ForcePathUpdate}]");

            //using (new Profiler("update destinations [%t]"))
            {
                //UpdateRingDestination();
                UpdateWaypointDestination();
                UpdateGridDestination();
                UpdateBackTrackDestination();
                UpdateThreatAvoidance();
            }

            int update_path_interval = m_UpdatePathIntervalOverride > 0 ? m_UpdatePathIntervalOverride : UpdatePathInterval;

            bool needToUpdatePath = Interlocked.CompareExchange(ref ForcePathUpdate, 0, 1) == 1 || (update_path_interval > 0 && (time - LastPathUpdateTime) > update_path_interval);

            //Trace.WriteLine($"navigator on update [need update path - {needToUpdatePath}]");

            if (needToUpdatePath)
            {
                LastPathUpdateTime = time;
                using (new ScopedTimer(PathUpdateTimer))
                    UpdatePath();
            }

            //using (new Profiler("update anti stuck [%t]"))
            if (m_Path.path.Count > 0)
            {
                UpdateAntiStuck();
                UpdateDestReachFailed();
            }

            Thread.Sleep(10);
        }

        public Int64 PathUpdateDuration => PathUpdateTimer.ElapsedMilliseconds;
        public string PathUpdateStage { get; private set; } = "";

        private Stopwatch PathRecalcTriggerCooldown = Stopwatch.StartNew();
        private Stopwatch PathUpdateTimer = new Stopwatch();
        private Int64 LastPathUpdateTime = 0;

        // Controls updated thread execution
        private volatile bool ShouldStopUpdates = false;

        private void UpdatePath()
        {
            if (!m_Navmesh.IsNavDataAvailable)
                return;

            PathUpdateStage = "Prepare";

            var dest = m_AvoidanceDestination.type == DestType.RunAway ? m_AvoidanceDestination : Destination;
            
            //Trace.WriteLine($"update path - dest {dest.pos}");

            Vec3 current_pos = CurrentPos;

            if (current_pos.IsZero() || (dest.pos.IsZero() && dest.type != DestType.RunAway))
            {
                if (m_Path.path.Count > 0)
                {
                    using (new WriteLock(PathLock, "PathLock - NavigationEngine.UpdatePath - clear path"))
                    {
                        m_Path.Clear();
                        m_PathFollowStrategy.OnPathReset();
                    }
                }
                return;
            }

            var new_path = new List<Vec3>();
            var new_path_recalc_trigger_position = Vec3.ZERO;
            float new_path_recalc_trigger_precision = 0;
            var new_rough_path_destination = Vec3.ZERO;
            var timed_out = false;

            //Trace.WriteLine($"pathfinding to {dest.pos}");

            PathUpdateStage = "Find";

            if (dest.type == DestType.RunAway)
            {
                FindAvoidancePath(
                    current_pos, 
                    ThreatThreshold, 
                    MovementFlags, 
                    ref new_path, 
                    m_AvoidanceDestination.pos, 
                    include_from: m_PathFollowStrategy.IncludePathStart(), 
                    shift_nodes_distance: PathNodesShiftDist);

                if (new_path.Any())
                {
                    dest.pos = new_path.Last();
                    dest.stop = true;
                }
            }
            else
            {
                string tmp_debug = "";
                try
                {
                    if (
                    !FindPath(
                        current_pos,
                        dest.pos,
                        MovementFlags,
                        m_Path,
                        out tmp_debug,
                        ref new_path,
                        out timed_out,
                        out new_path_recalc_trigger_position,
                        out new_path_recalc_trigger_precision,
                        out new_rough_path_destination,
                        PATH_NODES_MERGE_DISTANCE,
                        as_close_as_possible: dest.as_close_as_possible,
                        include_from: m_PathFollowStrategy.IncludePathStart(),
                        random_coeff: m_PathRandomCoeffOverride > 0 ? m_PathRandomCoeffOverride : PathRandomCoeff,
                        bounce: m_PathBounce,
                        shift_nodes_distance: PathNodesShiftDist,
                        shift_dest: dest.shift,
                        smoothen_distance: PathSmoothingDistance,
                        allow_rough_path: AllowRoughPath)
                        )
                    {
                        Trace.WriteLine($"No path to {dest.pos} (type: {dest.type} precision: {dest.precision} movement flags: {MovementFlags} annotation: {dest.debug_annotation ?? ""} current path dest: {m_Path.path_destination.pos})");
                        new_rough_path_destination = Vec3.ZERO; // clear rough destination just in case it is causing issues
                    }
                }
                catch (Exception ex)
                {
                    Trace.WriteLine($"[{tmp_debug}] {ex.Message}\n{ex.StackTrace}\n{ex.InnerException?.Message ?? "no-inner"}");
                }
            }

            if (timed_out)
                return;

            PathUpdateStage = "Post-process";

            if (!m_PathFollowStrategy.IncludePathStart())
            {
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
            }

            PathUpdateStage = "Notify";

            bool notify_dest_reached = false;

            // we are already in precision distance, no movement needed
            if (new_path.Count == 1 && current_pos.Distance2D(dest.pos) <= dest.precision)
            {
                new_path.Clear();
                notify_dest_reached = true;
                m_PathFollowStrategy.OnPathReset();
            }

            //Trace.WriteLine($"path to {dest.pos} computed");

            using (new WriteLock(PathLock, "PathLock - NavigationEngine.UpdatePath - set computed path"))
            {
                // reset override when first destination from path changed
                if (m_Path.path.Count == 0 || (new_path.Count > 0 && !m_Path.path[0].Equals(new_path[0])))
                    ResetAntiStuckPrecition(current_pos);

                //if (m_Path.path_destination.pos != dest.pos)
                //    Trace.WriteLine($"new path found from {current_pos} to {dest.pos} ({new_path.Count} points)");
                m_Path = new path_data(new_path, dest, new_path_recalc_trigger_position, new_path_recalc_trigger_precision, new_rough_path_destination);

                //Trace.WriteLine($"path [{new_path.Count}]");
                //Trace.WriteLine($"path updated");
            }

            if (notify_dest_reached)
                NotifyOnDestinationReached(dest);

            PathUpdateStage = "";
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

                    GridCell destination_grid = m_Navmesh.m_GridCells.FirstOrDefault(x => destination_grids_id.Contains(x.Id) && Algorihms.AreConnected(current_grid, ref x, MovementFlag.None, m_Navmesh));

                    if (destination_grid != null)
                    {
                        grid_dest_found = true;
                        dest_pos = Algorihms.GetNearestCell(destination_grid.GetCells(), destination_grid.Center).Center;
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

        //private void UpdateRingDestination()
        //{
        //    var ring_dest = RingDestination;

        //    if (ring_dest.precision_max <= 0)
        //        return;

        //    var current_pos = CurrentPos;
        //    var dest = Destination;
        //    float distance = current_pos.Distance2D(ring_dest.pos);
        //    bool too_far = distance > ring_dest.precision_max;
        //    bool too_close = distance < ring_dest.precision;

        //    if (distance < ring_dest.precision || distance > ring_dest.precision_max || !m_Navmesh.RayCast2D(ring_dest.pos, current_pos, MovementFlag.Walk) || (ring_dest.precision > 0 && IsThreatAt(dest.pos, consider_future_threats: true)))
        //    {
        //        float RAY_DIST = (ring_dest.precision + ring_dest.precision_max) * 0.5f;
        //        const int RAYS_COUNT = 32;
        //        const float ROTATE_STEP_ANGLE = 360 / RAYS_COUNT;
        //        Vec3 ROTATE_AXIS = new Vec3(0, 0, 1);

        //        Vec3[] DESTINATIONS = new Vec3[RAYS_COUNT + 1];
        //        Vec3 start_dir = new Vec3(1, 0, 0);
        //        for (int i = 0; i < RAYS_COUNT; ++i)
        //            DESTINATIONS[i] = ring_dest.pos + Vec3.Rotate(start_dir, i * ROTATE_STEP_ANGLE, ROTATE_AXIS) * RAY_DIST;
        //        DESTINATIONS[RAYS_COUNT] = ring_dest.pos + (current_pos - ring_dest.pos).Normalized2D() * RAY_DIST;

        //        Vec3 best_dest = Vec3.ZERO;
        //        Vec3 furthest_dest = DESTINATIONS[RAYS_COUNT]; // by default use direct towards player (it will only be selected when no ray cast passes or all targets positions are in threat)
        //        bool threat_at_furthest_dest = true;
        //        float furthest_dest_dist = -1;

        //        var sortReferencePos = dest.is_ring ? dest.pos : current_pos;
        //        DESTINATIONS = DESTINATIONS.OrderBy(x => x.Distance2D(sortReferencePos)).ToArray();

        //        //find best visible spot round the destination
        //        foreach (var dest_to_test in DESTINATIONS)
        //        {
        //            var dest_pos = m_Navmesh.RayCast2D(ring_dest.pos, dest_to_test, MovementFlag.Walk).End;
        //            var dist = dest_pos.Distance2D(ring_dest.pos);

        //            var threat_at_dest = IsThreatAt(dest_pos, consider_future_threats: true);

        //            if (dist >= ring_dest.precision && dist <= ring_dest.precision_max)
        //            {
        //                best_dest = dest_pos;
        //                break;
        //            }

        //            if (furthest_dest_dist < 0 || dist > furthest_dest_dist || (threat_at_furthest_dest && !threat_at_dest))
        //            {
        //                furthest_dest = dest_pos;
        //                furthest_dest_dist = dist;
        //                threat_at_furthest_dest = threat_at_dest;
        //            }
        //        }

        //        if (best_dest.IsZero())
        //            best_dest = furthest_dest;

        //        SetDestination(new destination(best_dest, ring_dest.type, DefaultPrecision * 0.8f, 0, true, false, true, ring_dest.user_data) { is_ring = true/*, shift = true*/ });
        //    }
        //    else
        //        SetDestination(new destination(current_pos, ring_dest.type, DefaultPrecision * 0.8f, 0, true, false, true, ring_dest.user_data) { is_ring = true });
        //}

        private void UpdateThreatAvoidance()
        {
            if (EnableThreatAvoidance)
            {
                Vec3 current_pos = CurrentPos;
                var threats = m_Navmesh.Regions.Where(x => Math.Abs(x.Threat) >= ThreatThreshold).ToList();
                var threats_at_current_pos = threats.Where(x => x.Area.Contains2D(current_pos));
                var current_threats = threats_at_current_pos.Where(x => x.Threat > 0);

                IsAvoidingFutureThreats = false;
                if (current_threats.Any())
                {
                    m_FutureThreatAvoidanceTimer.Reset();
                    IsAvoidingFutureThreats = true;
                }
                else
                {
                    m_FutureThreatAvoidanceTimer.Start();
                    IsAvoidingFutureThreats = m_FutureThreatAvoidanceTimer.ElapsedMilliseconds < FutureThreatAvoidanceDuration;
                }
                
                bool was_in_threat = IsInThreat;
                var dest = Destination;
                IsInThreat = IsAvoidingFutureThreats ? threats_at_current_pos.Any() : current_threats.Any();
                Threat = IsInThreat ? threats_at_current_pos.Select(x => IsAvoidingFutureThreats ? Math.Abs(x.Threat) : x.Threat).Max() : 0;

                //if (IsInThreat && (dest.pos.IsZero() || IsThreatAt(dest.pos, consider_future_threats: IsAvoidingFutureThreats)))
                if (IsInThreat && dest.pos.IsZero()) // only run away when not going anywhere, when there is threat at destination it doesn't mean it is one huge danger area
                {
                    m_AvoidanceDestination = new destination(dest.pos, DestType.RunAway, DefaultPrecision * 0.3f);
                    if (!was_in_threat)
                        RequestPathUpdate();
                }
                else
                {
                    m_AvoidanceDestination = default(destination);

                    if (!IsInThreat && was_in_threat)
                        RequestPathUpdate();

                    Vec3 go_to_pos = GoToPosition.pos;

                    // if we are heading to point on a path we have to check if there is a threat-region on our path
                    if (!go_to_pos.IsZero() && !IsInThreat)
                    {
                        using (new ReadLock(PathLock))
                            ThreatAhead = m_PathFollowStrategy.UpdateThreatAhead(current_pos, threats);
                    }
                    else
                        ThreatAhead = (Vec3.ZERO, 0); // don't make user stop
                }

                if (IsInThreat)
                {
                    ThreatAhead = (Vec3.ZERO, 0); // don't make user stop
                }
            }
            else
            {
                m_AvoidanceDestination = default(destination);
                IsInThreat = false;
                Threat = 0;
                ThreatAhead = (Vec3.ZERO, 0);
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

        // assumes path lock active
        internal float GetPrecision()
        {
            float precision = DefaultPrecision;

            if (m_PrecisionOverride > 0)
                precision = m_PrecisionOverride;
            else if (m_Path.path.Count == 1)
                precision = m_Path.path_destination.precision;

            return precision;
        }

        private bool IsDestinationReached(destination dest, destination path_dest, DestType type_filter)
        {
            if ((dest.type & type_filter) == 0)
                return false;

            return !dest.pos.IsZero() && dest.pos.Equals(path_dest.pos);
        }

        private void UpdatePathProgression(Vec3 current_pos)
        {
            if (current_pos.IsZero())
                return;

            bool destination_reached = false;
            destination path_destination = default;

            var dest = Destination;

            // check and removed reached nodes from path
            //using (new Profiler("update path prog [update] [%t]", 10))
            using (new ReadLock(PathLock, true, description: "PathLock - NavigationEngine.UpdatePathProgression"))
            {
                if (m_Path.path.Count > 0)
                {
                    path_destination = m_Path.path_destination;

                    using (new WriteLock(PathLock, "PathLock - NavigationEngine.UpdatePathProgression"))
                    {
                        // when current destination doesn't match the one from the path we are following
                        bool force_clear_path = !path_destination.keep_moving_while_repath && !dest.Equals(path_destination);

                        if (!force_clear_path)
                        {
                            m_PathFollowStrategy.UpdatePathProgress(current_pos, ref m_Path.path, path_destination, m_Navmesh);
                            destination_reached = m_Path.path.Count == 0;
                        }

                        if (destination_reached || force_clear_path)
                        {
                            m_Path.Clear();
                            m_PathFollowStrategy.OnPathReset();
                        }
                    }
                }
                else
                    m_PathFollowStrategy.OnPathReset();
            }

            // update destination arrived
            //using (new Profiler("update path prog [dest reached] [%t]", 10))
            if (destination_reached)
            {
                if (IsDestinationReached(dest, path_destination, DestType.All))
                {
                    using (new WriteLock(InputLock, "InputLock - NavigationEngine.UpdatePathProgression - dest history update"))
                    {
                        if (m_Destination.type != DestType.BackTrack &&
                            (m_DestinationsHistory.Count == 0 || (!m_DestinationsHistory[m_DestinationsHistory.Count - 1].Equals(m_Destination.pos) &&
                                                                    m_DestinationsHistory[m_DestinationsHistory.Count - 1].Distance(m_Destination.pos) > MinDestDistToAddToHistory)))
                        {
                            m_DestinationsHistory.Add(m_Destination.pos);
                        }
                    }

                    //using (new Profiler("update path prog [notify] [%t]", 10))
                    if (path_destination.type != DestType.RunAway)
                        NotifyOnDestinationReached(path_destination);

                    ResetAntiStuckPathing(current_pos);

                    //m_Navmesh.Log("[Nav] Dest " + m_Destination + " [" + m_DestinationType + "] reached!");
                }

                if (IsDestinationReached(dest, path_destination, DestType.Waypoint))
                {
                    using (new WriteLock(InputLock, "InputLock - NavigationEngine.UpdatePathProgression - remove waypoint"))
                        m_Waypoints.RemoveAt(0);

                    if (m_Waypoints.Count == 0)
                        ClearDestination(DestType.Waypoint);
                }

                if (IsDestinationReached(dest, path_destination, DestType.User))
                {
                    ClearDestination(DestType.User);
                }

                if (IsDestinationReached(dest, path_destination, DestType.BackTrack))
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

            const float MIN_TIME_TO_RECALCULATE_PATH = 1500;
            const float MIN_TIME_TO_OVERRIDE_PRECISION = 2000;
            const float MIN_TIME_TO_BOUNCE = 3000;
            const float MIN_TIME_TO_OVERRIDE_PATH_RANDOM_COEFF = 4500;

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
                if (m_AntiStuckPrecisionTimer.ElapsedMilliseconds > MIN_TIME_TO_OVERRIDE_PRECISION &&
                    m_AntiStuckPathingLevel <= 0)
                {
                    m_PrecisionOverride = AntiStuckPrecisionOverride; // overriding precision on higher levels can disable bouncing!
                }

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
                    m_PathRandomCoeffOverride = 1.25f;
                    m_PrecisionOverride = -1;
                    m_UpdatePathIntervalOverride = 1250;
                    RequestPathUpdate();
                }
                // level 3
                else if (m_AntiStuckPathingTimer.ElapsedMilliseconds > MIN_TIME_TO_OVERRIDE_PATH_RANDOM_COEFF &&
                         m_AntiStuckPathingLevel == 2)
                {
                    ResetAntiStuckPrecition(curr_pos);
                    m_AntiStuckPathingLevel = 3;
                    m_PathBounce = true;
                    m_PathRandomCoeffOverride = 1.75f;
                    m_PrecisionOverride = -1;
                    m_UpdatePathIntervalOverride = 1500;
                    RequestPathUpdate();
                }
            }
        }

        internal void ResetAntiStuckPrecition(Vec3 curr_pos)
        {
            using (new WriteLock(AntiStuckLock, "AntiStuckLock - NavigationEngine.ResetAntiStuckPrecition"))
            {
                m_PrecisionOverride = -1;
                m_AntiStuckPrecisionTestPos = curr_pos;
                m_AntiStuckPrecisionTimer.Reset();
            }
        }

        private void ResetAntiStuckPathing(Vec3 curr_pos)
        {
            using (new WriteLock(AntiStuckLock, "AntiStuckLock - NavigationEngine.ResetAntiStuckPathing"))
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

                using (new WriteLock(InputLock, "InputLock - NavigationEngine.ReorganizeWaypoints"))
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
            using (new ReadLock(PathLock, true, "PathLock - NavigationEngine.OnGridCellAdded - check rough destination"))
            {
                if (!m_Path.rough_path_destination.IsZero())
                {
                    using (new WriteLock(PathLock, "PathLock - NavigationEngine.UpdatePath - set computed path"))
                    {
                        Trace.WriteLine($"cleared rough path destination {m_Path.rough_path_destination} beacuse grid cell #{grid_cell.GlobalId} was added!");
                        m_Path.rough_path_destination = Vec3.ZERO;
                    }
                }
            }
        }

        public virtual void OnNavDataChanged(AABB affected_area)
        {
            //m_Navmesh.Log($"[Nav] Request path update OnNavDataChanged.");
            RequestPathUpdate();
        }

        public virtual void OnNavBlockersChanged()
        {
            RequestPathUpdate();
        }

        public virtual void OnPatchesChanged()
        {
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
            using (new ReadLock(InputLock, description: "InputLock - NavigationEngine.OnSerialize"))
            using (new ReadLock(PathLock, description: "PathLock - NavigationEngine.OnSerialize"))
            {
                w.Write(m_Waypoints.Count);
                foreach (Vec3 p in m_Waypoints)
                    p.Serialize(w);

                w.Write(m_Path.path.Count);
                foreach (Vec3 p in m_Path.path)
                    p.Serialize(w);
                m_Path.path_destination.Serialize(w);
                
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
                default(destination).Serialize(w); // placeholder for back-compatibility remove when there are serialization changes

                w.Write(m_Path.path_recalc_trigger_precision);
                m_Path.path_recalc_trigger_pos.Serialize(w);

                w.Write(UpdatePathInterval);
                w.Write(CurrentPosDiffRecalcThreshold);
                w.Write(PathNodesShiftDist);
                w.Write(PathRandomCoeff);
                w.Write(DefaultPrecision);
                w.Write(AllowRoughPath);
                w.Write(PathSmoothingDistance);
                w.Write(PathSmoothingPrecision);
                w.Write(EnableAntiStuck);
                w.Write(EnableThreatAvoidance);
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
            using (new WriteLock(InputLock, "InputLock - NavigationEngine.OnDeserialize"))
            using (new WriteLock(PathLock, "PathLock - NavigationEngine.OnDeserialize"))
            {
                m_Waypoints.Clear();
                m_DestinationGridsId.Clear();
                m_Path.Clear();
                m_PathFollowStrategy.OnPathReset();
                m_DestinationsHistory.Clear();
                m_DebugPositionsHistory.Clear();

                int waypoints_count = r.ReadInt32();
                for (int i = 0; i < waypoints_count; ++i)
                    m_Waypoints.Add(new Vec3(r));

                int path_count = r.ReadInt32();
                for (int i = 0; i < path_count; ++i)
                    m_Path.path.Add(new Vec3(r));
                m_Path.path_destination.Deserialize(r);
                
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
                m_Destination.Deserialize(r);
                destination unusedDest = default;
                unusedDest.Deserialize(r);

                m_Path.path_recalc_trigger_precision = r.ReadSingle();
                m_Path.path_recalc_trigger_pos = new Vec3(r);

                UpdatePathInterval = r.ReadInt32();
                CurrentPosDiffRecalcThreshold = r.ReadSingle();
                PathNodesShiftDist = r.ReadSingle();
                PathRandomCoeff = r.ReadSingle();
                DefaultPrecision = r.ReadSingle();
                AllowRoughPath = r.ReadBoolean();
                PathSmoothingDistance = r.ReadSingle();
                PathSmoothingPrecision = r.ReadSingle();
                EnableAntiStuck = r.ReadBoolean();
                EnableThreatAvoidance = r.ReadBoolean();
            }

            Vec3 curr_pos = CurrentPos;

            ResetAntiStuckPrecition(curr_pos);
            ResetAntiStuckPathing(curr_pos);
        }

        private float PATH_NODES_MERGE_DISTANCE = -1;

        private Thread UpdatesThread = null;
        private int ForcePathUpdate = 0;

        private ReaderWriterLockSlim PathLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private ReaderWriterLockSlim InputLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private ReaderWriterLockSlim AntiStuckLock = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);

        private IPathFollowStrategy m_PathFollowStrategy;

        internal path_data m_Path = new path_data(new List<Vec3>(), default, Vec3.ZERO, 0, Vec3.ZERO); //@ PathLock
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
        private destination m_AvoidanceDestination = default(destination); //@ InputLock
        private List<INavigationObserver> m_Observers = new List<INavigationObserver>(); //@ InputLock

        private Stopwatch m_FutureThreatAvoidanceTimer = new Stopwatch();

        public IRoughPathEstimator m_RoughtPathEstimator;

        private Navmesh m_Navmesh = null;
    }
}
