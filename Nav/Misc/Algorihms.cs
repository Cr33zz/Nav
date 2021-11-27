﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Threading;

namespace Nav
{
    public struct path_pos
    {
        public path_pos(Vec3 p, Cell c)
        {
            pos = p;
            cell = c;
        }

        public Vec3 pos;
        public Cell cell;
    }

    public class Algorihms
    {
        private static double AcceptanceProbability(float energy, float new_energy, double temperature)
        {
            // If the new solution is better, accept it
            if (new_energy < energy)
                return 1.0;

            // If the new solution is worse, calculate an acceptance probability
            return Math.Exp((energy - new_energy) / temperature);
        }

        private static float GetTravelDistance(List<ExploreCell> tour, CellsDistancer distances)
        {
            float travel_distance = 0;
            if (tour.Count > 2)
            {
                for (int i = 0; i < tour.Count - 1; ++i)
                    travel_distance += distances.GetDistance(tour[i].GlobalId, tour[i + 1].GlobalId);
            }

            return travel_distance;
        }

        private static List<ExploreCell> Swap2Opt(List<ExploreCell> tour, int i, int k)
        {
            List<ExploreCell> new_tour = tour.GetRange(0, i); // get from 0 to i-1
            List<ExploreCell> middle_reversed_tour = tour.GetRange(i, k - i + 1); // get from i to k
            middle_reversed_tour.Reverse();
            new_tour.AddRange(middle_reversed_tour);

            if (k + 1 < tour.Count)
                new_tour.AddRange(tour.GetRange(k + 1, tour.Count - k - 1)); // get from k+1 to end

            return new_tour;
        }

        private static void Swap2Opt(ref List<ExploreCell> tour, int i, int k)
        {
            tour.Reverse(i, k - i + 1); // reverse from i to k (inclusive)
        }

        private static float Swap2OptDistance(List<ExploreCell> tour, CellsDistancer distances, int i, int k)
        {
            var tour_indices = tour.Select(x => x.GlobalId).ToList();
            tour_indices.Reverse(i, k - i + 1);

            float travel_distance = 0;
            if (tour_indices.Count > 2)
            {
                for (int n = 0; n < tour_indices.Count - 1; ++n)
                    travel_distance += distances.GetDistance(tour_indices[n], tour_indices[n + 1]);
            }

            return travel_distance;
        }

        public static void FindExplorePath2Opt(ExploreCell start_cell, List<ExploreCell> explore_cells, CellsDistancer distances, ref List<ExploreCell> path)
        {
            using (new Profiler($"2-Opt %t"))
            {
                path = path ?? new List<ExploreCell>();
                path.Clear();

                if (start_cell == null)
                    return;

                //List<int> cells_id_in_start_cell_network = distances.GetConnectedTo(start_cell.GlobalId);
                //List<ExploreCell> best_tour_old = explore_cells.FindAll(c => cells_id_in_start_cell_network.Contains(c.GlobalId));

                // reordering cell in breadth-first order seems to help with 2-opt backtracking
                var collect_cells_visitor = new CollectVisitor();
                Algorihms.VisitBreadth(start_cell, visitor: collect_cells_visitor);
                var best_tour = collect_cells_visitor.cells.Select(x => x as ExploreCell).Intersect(explore_cells).ToList();

                best_tour.RemoveAll(c => c.Explored || c.Neighbours.Count == 0);

                if (start_cell.Explored) // re-insert start cell if needed
                    best_tour.Insert(0, start_cell);

                if (best_tour.Count == 1)
                {
                    path.Add(best_tour[0]);
                    return;
                }

                float best_dist = GetTravelDistance(best_tour, distances);


                if (best_tour.Count < 90)
                {
                    // based on http://en.wikipedia.org/wiki/2-opt

                    while (true)
                    {
                        bool better_found = false;

                        for (int i = 1; i < best_tour.Count - 1; ++i)
                        {
                            for (int k = i + 1; k < best_tour.Count; ++k)
                            {
                                float new_dist = Swap2OptDistance(best_tour, distances, i, k);

                                if (new_dist < best_dist)
                                {
                                    Swap2Opt(ref best_tour, i, k);
                                    best_dist = new_dist;

                                    better_found = true;
                                    break;
                                }
                            }

                            if (better_found)
                                break;
                        }

                        if (!better_found)
                            break;
                    }
                }
                else // greedy
                {
                    // based on http://on-demand.gputechconf.com/gtc/2014/presentations/S4534-high-speed-2-opt-tsp-solver.pdf

                    while (true)
                    {
                        float min_change = 0;

                        int min_i = -1;
                        int min_j = -1;

                        for (int i = 0; i < best_tour.Count - 2; ++i)
                        {
                            for (int j = i + 2; j < best_tour.Count - 1; ++j)
                            {
                                int city_a = best_tour[i].GlobalId;
                                int city_b = best_tour[i + 1].GlobalId;
                                int city_c = best_tour[j].GlobalId;
                                int city_d = best_tour[j + 1].GlobalId;

                                float change = (distances.GetDistance(city_a, city_c) + distances.GetDistance(city_b, city_d)) -
                                               (distances.GetDistance(city_a, city_b) + distances.GetDistance(city_c, city_d));

                                if (change < min_change)
                                {
                                    min_change = change;
                                    min_i = i + 1;
                                    min_j = j;
                                }
                            }
                        }

                        if (min_change >= 0)
                            break;

                        // apply min_i/min_j move
                        ExploreCell t = best_tour[min_i];
                        best_tour[min_i] = best_tour[min_j];
                        best_tour[min_j] = t;
                    }
                }

                Trace.WriteLine("[FindExplorePath 2-Opt] Final solution distance: " + GetTravelDistance(best_tour, distances));

                foreach (ExploreCell cell in best_tour)
                    path.Add(cell);

                if (start_cell.Explored)
                    path.RemoveAt(0);
            }
        }

        public static void FindExplorePath(ExploreCell start_cell, List<ExploreCell> explore_cells, CellsDistancer distances, ref List<ExploreCell> path)
        {
            using (new Profiler($"Simulated annealing %t"))
            {
                //based on http://www.theprojectspot.com/tutorial-post/simulated-annealing-algorithm-for-beginners/6
                path = path ?? new List<ExploreCell>();
                path.Clear();

                if (start_cell == null)
                    return;

                List<int> cells_id_in_start_cell_network = distances.GetConnectedTo(start_cell.GlobalId);

                List<ExploreCell> best_solution = explore_cells.FindAll(c => cells_id_in_start_cell_network.Contains(c.Id));
                best_solution.RemoveAll(c => (c.Explored || c.Neighbours.Count == 0) && c.GlobalId != start_cell.GlobalId);
                best_solution.Remove(start_cell);
                best_solution.Insert(0, start_cell);

                if (best_solution.Count == 1)
                {
                    path.Add(best_solution[0]);
                    return;
                }

                float best_energy = GetTravelDistance(best_solution, distances);

                List<ExploreCell> current_solution = best_solution;
                float current_energy = best_energy;

                Random rng = new Random();

                double temp = 10000;
                double alpha = 0.999;
                double epsilon = 0.0001;
                int temp_iterations = 2;

                // Loop until system has cooled
                while (temp > epsilon)
                {
                    for (int iter = 0; iter < temp_iterations; ++iter)
                    {
                        // Create new neighbour tour
                        List<ExploreCell> new_solution = new List<ExploreCell>(current_solution);

                        // Get a random positions in the tour
                        int tour_pos_1 = rng.Next(1, new_solution.Count);
                        int tour_pos_2 = rng.Next(1, new_solution.Count);

                        // Swap them
                        ExploreCell t = new_solution[tour_pos_1];
                        new_solution[tour_pos_1] = new_solution[tour_pos_2];
                        new_solution[tour_pos_2] = t;

                        // Get energy of solutions
                        float new_energy = GetTravelDistance(new_solution, distances);

                        // Decide if we should accept the neighbour
                        if (AcceptanceProbability(current_energy, new_energy, temp) >= rng.NextDouble())
                        {
                            current_solution = new_solution;
                            current_energy = new_energy;
                        }

                        // Keep track of the best solution found
                        if (current_energy < best_energy)
                        {
                            best_solution = current_solution;
                            best_energy = current_energy;
                        }
                    }

                    // Cool system
                    temp *= alpha;
                }

                Trace.WriteLine("[FindExplorePath] Final solution distance: " + GetTravelDistance(best_solution, distances));

                foreach (ExploreCell cell in best_solution)
                    path.Add(cell);

                if (start_cell.Explored)
                    path.RemoveAt(0);
            }
        }

        public static bool AreConnected<T>(T start, ref T end, MovementFlag flags) where T : Cell
        {
            List<path_pos> path = null;
            return FindPath(start, Vec3.ZERO, new Algorihms.DestinationPathFindStrategy<T>(Vec3.ZERO, end), flags, ref path, out var timedOut);
        }

        public static float GetPathLength(List<Vec3> path, Vec3 pos)
        {
            float length = 0;

            for (int i = 0; i < path.Count - 1; ++i)
                length += path[i].Distance2D(path[i + 1]);

            return length + ((path.Count > 0 && !pos.IsZero()) ? path[0].Distance2D(pos) : 0);
        }

        public static void ProjectAndMovePosOnPath(Vec3 pos, List<Vec3> path, float move_distance, out Vec3 projection, out Vec3 moved_projection, out int projection_segment_start_idx, out float scalar_projection)
        {
            projection = ProjectPosOnPath(pos, path, out projection_segment_start_idx, out scalar_projection);
            float dist = pos.Distance2D(projection);

            moved_projection = projection;
            float move_dist_remaining = move_distance - dist; // if we are far away from path, we care mostly about getting close to it first
            int current_segment_start_idx = projection_segment_start_idx;

            while (move_dist_remaining > 0 && current_segment_start_idx < path.Count - 1)
            {
                float dist_to_segment_end = moved_projection.Distance2D(path[current_segment_start_idx + 1]);

                if (dist_to_segment_end > move_dist_remaining)
                {
                    var move_dir = path[current_segment_start_idx + 1] - path[current_segment_start_idx];
                    move_dir.Normalize2D();

                    moved_projection += move_dir * move_dist_remaining;
                    break;
                }

                moved_projection = path[current_segment_start_idx + 1];
                move_dist_remaining -= dist_to_segment_end;

                ++current_segment_start_idx;
            }
        }

        public static Vec3 ProjectPosOnPath(Vec3 pos, List<Vec3> path, out int projection_segment_start_idx, out float scalar_projection)
        {
            Vec3 projection = path[0];
            projection_segment_start_idx = 0;
            scalar_projection = 0;
            float dist = pos.Distance2D(projection);

            // find closest pos projection on path
            for (int i = 0; i < path.Count - 1; ++i)
            {
                var p = Vec3.ProjectPointOnSegment(path[i], path[i + 1], pos, out var sp, true);
                var d = pos.Distance2D(p);

                if (d < dist)
                {
                    projection = p;
                    dist = d;
                    projection_segment_start_idx = i;
                    scalar_projection = sp;
                }
            }

            return projection;
        }

        public static Cell GetCellAt(IEnumerable<Cell> cells, Vec3 p, MovementFlag flags, bool allow_disabled = false, bool test_2d = true, float z_tolerance = 0)
        {
            if (p.IsZero())
                return null;

            foreach (Cell cell in cells)
            {
                if ((!allow_disabled && cell.Disabled) || (cell.Flags & flags) != flags)
                    continue;

                if (test_2d ? cell.Contains2D(p) : cell.Contains(p, z_tolerance))
                    return cell;
            }

            return null;
        }

        public static List<Cell> GetCellsWithin(IEnumerable<Cell> cells, Vec3 p, float radius, MovementFlag flags, bool allow_disabled = false, bool test_2d = true, float z_tolerance = 0)
        {
            var result_cells = new List<Cell>();

            if (p.IsZero())
                return result_cells;

            float radiusSqr = radius * radius;

            foreach (Cell cell in cells)
            {
                if ((!allow_disabled && cell.Disabled) || (cell.Flags & flags) != flags)
                    continue;

                float distSqr = test_2d ? cell.Distance2DSqr(p) : cell.DistanceSqr(p);

                if (distSqr <= radiusSqr)
                    result_cells.Add(cell);
            }

            return result_cells;
        }

        public static T GetNearestCell<T>(IEnumerable<T> cells, Vec3 p) where T : Cell
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

        private static NodeInfo GetNodeInfoFromList(Cell node, Vec3 leading_point, List<NodeInfo> list)
        {
            for (int i = 0; i < list.Count; ++i)
            {
                if (list[i].cell == node /*&& list[i].leading_point.Equals(leading_point)*/)
                    return list[i];
            }

            return null;
        }

        private class NodeInfo : IComparable<NodeInfo>
        {
            public NodeInfo(Cell cell, Vec3 leading_point, NodeInfo parent = null, float g = 0, float h = 0)
            {
                this.leading_point = leading_point;
                this.cell = cell;
                this.parent = parent;
                this.g = g;
                this.h = h;
            }

            public int CompareTo(NodeInfo other)
            {
                if (other == null)
                    return 1;

                if (other == this)
                    return 0;

                if (TotalCost.Equals(other.TotalCost))
                    return 0;

                return TotalCost > other.TotalCost ? 1 : -1;
            }

            public float TotalCost { get { return g + h; } }

            public Cell cell;
            public Vec3 leading_point;
            public NodeInfo parent;
            public float g; //cost from start node
            public float h; //heuristic cost to end node
        }

        public abstract class PathFindStrategy<T>
            where T : Cell
        {
            public virtual bool IsValid() { return true; }
            public abstract float GetMinDistance(T cell);
            public virtual Vec3 GetDestination() { return Vec3.ZERO;  } // center of cell
            public virtual bool UseFinalCellEntranceAsDestination() { return false; }
            public abstract bool IsDestCell(T cell);
            public T FinalDestCell { get; set; } // for storing result
        }

        public class DestinationPathFindStrategy<T> : PathFindStrategy<T>
            where T : Cell
        {
            public DestinationPathFindStrategy(Vec3 dest, T dest_cell) { Dest = dest; DestCell = dest_cell; FinalDestCell = dest_cell; }

            public override bool IsValid() { return DestCell != null; }
            public override float GetMinDistance(T cell) { return cell.AABB.Distance2D(Dest); }
            public override Vec3 GetDestination() { return Dest; }
            public override bool IsDestCell(T cell) { return cell == DestCell; }

            private Vec3 Dest;
            private T DestCell;
        }

        public class AvoidancePathFindStrategy<T> : PathFindStrategy<T>
            where T : Cell
        {
            public AvoidancePathFindStrategy(float threat_threshold, Vec3 hint_pos = default(Vec3))
            {
                ThreatThreshold = threat_threshold;
                HintPos = hint_pos;
            }

            public override float GetMinDistance(T cell) { return HintPos.IsZero() ? 0 : cell.AABB.Distance2D(HintPos); }
            // while path finding consider future threat regions (those with negative threat) as actual threats
            public override bool IsDestCell(T cell) { return Math.Abs(cell.Threat) < ThreatThreshold; }
            public override bool UseFinalCellEntranceAsDestination() { return true; }

            private float ThreatThreshold;
            private Vec3 HintPos;
        }

        public static bool FindPath<T,S>(T start, Vec3 from, S strategy, MovementFlag flags, ref List<path_pos> path, out bool timedOut, float random_coeff = 0, bool allow_disconnected = false, bool use_cell_centers = false, bool ignore_movement_cost = false, Int64 time_limit = -1)
            where T : Cell
            where S : PathFindStrategy<T>
        {
            if (path != null)
                path.Clear();

            var time_limit_timer = Stopwatch.StartNew();
            timedOut = false;

            //based on http://en.wikipedia.org/wiki/A*_search_algorithm

            List<NodeInfo> open = new List<NodeInfo>();
            List<NodeInfo> closed = new List<NodeInfo>();

            Random rng = new Random();

            if (start == null || !strategy.IsValid())
                return false;

            NodeInfo s = new NodeInfo(start, from, null, 0, strategy.GetMinDistance(start));
            open.Add(s);

            using (new Profiler($"A* %t", 100))
            {
                while (open.Count > 0)
                {
                    if (time_limit > 0 && time_limit_timer.ElapsedMilliseconds > time_limit)
                    {
                        //Trace.WriteLine("Path finding timed out!");
                        timedOut = true;
                        return false;
                    }

                    //sort by cost
                    float min_total_cost = open.Min(x => x.TotalCost);
                    NodeInfo best = open.Find(x => x.TotalCost.Equals(min_total_cost));

                    open.Remove(best);

                    //take node with lower cost from open list
                    NodeInfo current_node = best;

                    if (strategy.IsDestCell((T)current_node.cell))
                    {
                        strategy.FinalDestCell = (T)current_node.cell;
                        BuildPath(start, current_node.cell, from, strategy.UseFinalCellEntranceAsDestination() ? current_node.leading_point : strategy.GetDestination(), current_node, ref path, use_cell_centers);
                        return true;
                    }

                    closed.Insert(0, current_node);

                    foreach (Cell.Neighbour neighbour in current_node.cell.Neighbours)
                    {
                        Cell cell_neighbour = neighbour.cell;

                        if (cell_neighbour.Disabled || (neighbour.connection_flags & flags) != flags)
                            continue;

                        Vec3 leading_point = use_cell_centers ? neighbour.cell.Center : neighbour.cell.AABB.Align(current_node.leading_point);

                        NodeInfo neighbour_node = GetNodeInfoFromList(cell_neighbour, leading_point, closed);

                        // if already processed then skip this neighbour
                        if (neighbour_node != null)
                            continue;

                        float random_dist_mod = -random_coeff + (2 * random_coeff) * (float)rng.NextDouble();

                        float new_g = current_node.g + current_node.leading_point.Distance(leading_point) * (1 + random_dist_mod) * (ignore_movement_cost ? 1 : current_node.cell.MovementCostMult);
                        float new_h = strategy.GetMinDistance((T)neighbour.cell);

                        neighbour_node = GetNodeInfoFromList(cell_neighbour, leading_point, open);

                        // if not in open list
                        if (neighbour_node == null)
                        {
                            neighbour_node = new NodeInfo(cell_neighbour, leading_point, current_node, new_g, new_h); // g and h will be set later on
                            open.Insert(0, neighbour_node);
                        }
                        else if ((new_g + new_h) < neighbour_node.TotalCost)
                        {
                            neighbour_node.parent = current_node;
                            neighbour_node.leading_point = leading_point;
                            neighbour_node.g = new_g;
                            neighbour_node.h = new_h;
                        }
                    }
                }
            }

            if (allow_disconnected && closed.Count > 0)
            {
                //Trace.WriteLine("No direct path found :(");

                var dest = strategy.GetDestination();
                NodeInfo best = closed.OrderBy(x => x.h).First();
                strategy.FinalDestCell = (T)best.cell;

                Vec3 nearest_to_dest = best.cell.Center;
                best.cell.AABB.SegmentTest(dest, best.leading_point, ref nearest_to_dest);

                BuildPath(start, best.cell, from, strategy.UseFinalCellEntranceAsDestination() ? best.leading_point : nearest_to_dest, best, ref path, use_cell_centers);
                return true;
            }

            return false;
        }

        private static void BuildPath<T>(T start, T end, Vec3 from, Vec3 to, NodeInfo info, ref List<path_pos> path, bool use_cell_centers = false) where T : Cell
        {
            // build result path
            if (path == null)
                return;

            List<Cell> cells_list = new List<Cell>();

            if (info != null)
                path.Add(new path_pos(to.IsZero() ? info.cell.Center : to, info.cell));

            while (info != null)
            {
                path.Add(new path_pos((use_cell_centers && info.parent != null) ? info.cell.Center : info.leading_point, info.cell));
                info = info.parent;
            }

            path.Reverse();
        }

        public interface IDistanceVisitor<T> where T : Cell
        {
            void Visit(T cell, float dist);
        }

        public interface IVisitor<T> where T : Cell
        {
            void Visit(T cell);
        }

        private class visit_node<T> where T : Cell
        {
            public visit_node(T cell, float dist, int depth) { this.cell = cell; this.distance = dist; this.depth = depth; }
            public T cell;
            public float distance;
            public int depth;
        }

        public static void VisitBreadth<T>(T cell, MovementFlag flags, int max_depth = -1, HashSet<T> allowed_cells = null, IDistanceVisitor<T> visitor = null) where T : Cell
        {
            HashSet<T> visited = new HashSet<T>();
            List<visit_node<T>> to_visit = new List<visit_node<T>>();
            to_visit.Add(new visit_node<T>(cell, 0, 0));

            while (to_visit.Count > 0)
            {
                visit_node<T> cell_data = to_visit.First();
                to_visit.RemoveAt(0);
                visited.Add(cell_data.cell);

                if (visitor != null && (allowed_cells == null || allowed_cells.Contains(cell_data.cell)))
                    visitor.Visit(cell_data.cell, cell_data.distance);

                if (max_depth < 0 || cell_data.depth < max_depth)
                {
                    foreach (Cell.Neighbour neighbour in cell_data.cell.Neighbours)
                    {
                        T neighbour_cell = (T)neighbour.cell;
                        float distance = cell_data.distance + cell_data.cell.Distance(neighbour_cell);

                        visit_node<T> neighbour_cell_data = to_visit.FirstOrDefault(x => x.cell.Equals(neighbour_cell));

                        if (neighbour_cell_data != null)
                        {
                            neighbour_cell_data.distance = Math.Min(neighbour_cell_data.distance, distance);
                            continue;
                        }

                        if ((neighbour.connection_flags & flags) != flags ||
                            visited.Contains(neighbour_cell))
                        {
                            continue;
                        }

                        to_visit.Add(new visit_node<T>(neighbour_cell, distance, cell_data.depth + 1));
                    }
                }
            }
        }

        public static void VisitBreadth<T>(T cell, HashSet<T> allowed_cells = null, IVisitor<T> visitor = null) where T : Cell
        {
            HashSet<int> processed = new HashSet<int>();
            List<T> to_visit = new List<T>();
            to_visit.Add(cell);
            processed.Add(cell.GlobalId);

            while (to_visit.Count > 0)
            {
                cell = to_visit.First();
                to_visit.RemoveAt(0);                

                if (visitor != null && (allowed_cells == null || allowed_cells.Contains(cell)))
                    visitor.Visit(cell);

                foreach (Cell.Neighbour neighbour in cell.Neighbours)
                {
                    T neighbour_cell = (T)neighbour.cell;
                    
                    if (processed.Contains(neighbour_cell.GlobalId))
                        continue;

                    to_visit.Add(neighbour_cell);
                    processed.Add(neighbour_cell.GlobalId);
                }
            }
        }

        public static void Visit<T>(T cell, MovementFlag flags, int max_depth = -1, HashSet<T> allowed_cells = null, IVisitor<T> visitor = null) where T : Cell
        {
            HashSet<T> visited = new HashSet<T>();
            Visit<T>(cell, ref visited, flags, true, 1, max_depth, null, visitor);
        }

        public static void Visit<T>(T cell, ref HashSet<T> visited, MovementFlag flags, bool visit_disabled, int depth = 1, int max_depth = -1, HashSet<T> allowed_cells = null, IVisitor<T> visitor = null) where T : Cell
        {
            if ((allowed_cells != null && !allowed_cells.Contains(cell)) || visited.Contains(cell))
                return;

            if (!visit_disabled && cell.Disabled)
                return;

            if (max_depth > 0 && depth >= max_depth)
                return;

            if (visitor != null)
                visitor.Visit(cell);

            visited.Add(cell);

            foreach (Cell.Neighbour neighbour in cell.Neighbours)
            {
                T neighbour_cell = (T)neighbour.cell;

                if ((neighbour.connection_flags & flags) != flags)
                    continue;

                Visit(neighbour_cell, ref visited, flags, visit_disabled, depth + 1, max_depth, allowed_cells, visitor);
            }
        }

        internal class PatchVisitor : Algorihms.IVisitor<Cell>
        {
            public void Visit(Cell cell)
            {
                cells.Add(cell);

                AABB gridAABB = cell.ParentAABB;
                if (!cellsGrid.ContainsKey(gridAABB))
                    cellsGrid.Add(gridAABB, new List<Cell>() { cell });
                else
                    cellsGrid[gridAABB].Add(cell);
            }

            public HashSet<Cell> cells = new HashSet<Cell>();
            public Dictionary<AABB, List<Cell>> cellsGrid = new Dictionary<AABB, List<Cell>>();
        }

        public class CollectVisitor : IVisitor<Cell>
        {
            public void Visit(Cell cell)
            {
                cells.Add(cell);
            }

            public readonly List<Cell> cells = new List<Cell>();
        }
    }
}
