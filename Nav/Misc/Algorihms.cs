using System;
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
            tour.Reverse(i, k - i + 1); // get from i to k
        }

        private static float Swap2OptDistance(List<ExploreCell> tour, CellsDistancer distances, int i, int k)
        {
            float travel_distance = 0;
            for (int n = 0; n < i - 1; ++n)
                travel_distance += distances.GetDistance(tour[n].GlobalId, tour[n + 1].GlobalId);

            travel_distance += distances.GetDistance(tour[i - 1].GlobalId, tour[k].GlobalId);

            for (int n = k; n > i; --n)
                travel_distance += distances.GetDistance(tour[n].GlobalId, tour[n - 1].GlobalId);

            if (k + 1 < tour.Count)
            {
                travel_distance += distances.GetDistance(tour[i].GlobalId, tour[k + 1].GlobalId);

                for (int n = k + 1; n < tour.Count - 1; ++n)
                    travel_distance += distances.GetDistance(tour[n].GlobalId, tour[n + 1].GlobalId);
            }

            return travel_distance;
        }

        public static void FindExplorePath2Opt(ExploreCell start_cell, List<ExploreCell> explore_cells, CellsDistancer distances, ref List<Vec3> path)
        {
            path.Clear();

            if (start_cell == null)
                return;

            List<int> cells_id_in_start_cell_network = distances.GetConnectedTo(start_cell.GlobalId);

            List<ExploreCell> best_tour = explore_cells.FindAll(c => cells_id_in_start_cell_network.Contains(c.GlobalId));
            best_tour.RemoveAll(c => c.Explored || c.Neighbours.Count == 0);
            if (start_cell.Explored)
                best_tour.Insert(0, start_cell);

            if (best_tour.Count == 1)
            {
                path.Add(best_tour[0].Position);
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

            //Navmesh.Log("[FindExplorePath 2-Opt] Final solution distance: " + GetTravelDistance(tour, distances));

            foreach (ExploreCell cell in best_tour)
                path.Add(cell.Position);

            if (start_cell.Explored)
                path.RemoveAt(0);
        }

        public static void FindExplorePath(ExploreCell start_cell, List<ExploreCell> explore_cells, CellsDistancer distances, ref List<Vec3> path)
        {
            //based on http://www.theprojectspot.com/tutorial-post/simulated-annealing-algorithm-for-beginners/6

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
                path.Add(best_solution[0].Position);
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

            //Navmesh.Log("[FindExplorePath] Final solution distance: " + best_energy);

            foreach (ExploreCell cell in best_solution)
                path.Add(cell.Position);

            if (start_cell.Explored)
                path.RemoveAt(0);
        }

        public static bool AreConnected<T>(T start, ref T end, MovementFlag flags) where T : Cell
        {
            List<path_pos> path = null;
            return FindPath(start, Vec3.ZERO, new Algorihms.DestinationPathFindStrategy<T>(Vec3.ZERO, end), flags, ref path);
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
            public abstract float GetMinDistance(Vec3 from);
            public virtual Vec3 GetDestination() { return Vec3.ZERO;  } // center of cell
            public abstract bool IsDestCell(T cell);
            public T FinalDestCell { get; set; } // for storing result
        }

        public class DestinationPathFindStrategy<T> : PathFindStrategy<T>
            where T : Cell
        {
            public DestinationPathFindStrategy(Vec3 dest, T dest_cell) { Dest = dest; DestCell = dest_cell; FinalDestCell = dest_cell; }

            public override bool IsValid() { return DestCell != null; }
            public override float GetMinDistance(Vec3 from) { return from.Distance(Dest); }
            public override Vec3 GetDestination() { return Dest; }
            public override bool IsDestCell(T cell) { return cell == DestCell; }

            private Vec3 Dest;
            private T DestCell;
        }

        public class AvoidancePathFindStrategy<T> : PathFindStrategy<T>
            where T : Cell
        {
            public AvoidancePathFindStrategy(float max_allowed_threat, Vec3 hint_pos = default(Vec3))
            {
                MaxAllowedThreat = max_allowed_threat;
                HintPos = hint_pos;
            }

            public override float GetMinDistance(Vec3 from) { return HintPos.IsZero() ? 0 : from.Distance(HintPos); }
            public override bool IsDestCell(T cell) { return cell.Threat <= MaxAllowedThreat; }

            private float MaxAllowedThreat;
            private Vec3 HintPos;
        }

        public static bool FindPath<T,S>(T start, Vec3 from, S strategy, MovementFlag flags, ref List<path_pos> path, float random_coeff = 0, bool allow_disconnected = false)
            where T : Cell
            where S : PathFindStrategy<T>
        {
            if (path != null)
                path.Clear();

            //based on http://en.wikipedia.org/wiki/A*_search_algorithm

            List<NodeInfo> open = new List<NodeInfo>();
            List<NodeInfo> closed = new List<NodeInfo>();

            Random rng = new Random();

            if (start == null || !strategy.IsValid())
                return false;

            NodeInfo s = new NodeInfo(start, from, null, 0, strategy.GetMinDistance(from));
            open.Add(s);

            while (open.Count > 0)
            {
                //sort by cost
                float min_total_cost = open.Min(x => x.TotalCost);
                NodeInfo best = open.Find(x => x.TotalCost.Equals(min_total_cost));

                open.Remove(best);

                //take node with lower cost from open list
                NodeInfo info = best;

                if (strategy.IsDestCell((T)info.cell))
                {
                    strategy.FinalDestCell = (T)info.cell;
                    BuildPath(start, info.cell, from, strategy.GetDestination(), info, ref path);
                    return true;
                }

                closed.Insert(0, info);

                foreach (Cell.Neighbour neighbour in info.cell.Neighbours)
                {
                    Cell cell_neighbour = neighbour.cell;

                    if (cell_neighbour.Disabled || (neighbour.connection_flags & flags) != flags)
                        continue;

                    Vec3 leading_point = neighbour.cell.AABB.Align(info.leading_point);

                    NodeInfo info_neighbour = GetNodeInfoFromList(cell_neighbour, leading_point, closed);

                    // if already processed then skip this neighbour
                    if (info_neighbour != null)
                        continue;

                    float random_dist_mod = -random_coeff + (2 * random_coeff) * (float)rng.NextDouble();

                    float new_g = info.g + info.leading_point.Distance(leading_point) * (1 + random_dist_mod) * info.cell.MovementCostMult;
                    float new_h = strategy.GetMinDistance(leading_point) * (1 + random_dist_mod) * info.cell.MovementCostMult;

                    info_neighbour = GetNodeInfoFromList(cell_neighbour, leading_point, open);

                    // if not in open list
                    if (info_neighbour == null)
                    {
                        info_neighbour = new NodeInfo(cell_neighbour, leading_point, info, new_g, new_h); // g and h will be set later on
                        open.Insert(0, info_neighbour);
                    }
                    else if ((new_g + new_h) < info_neighbour.TotalCost)
                    {
                        info_neighbour.parent = info;
                        info_neighbour.leading_point = leading_point;
                        info_neighbour.g = new_g;
                        info_neighbour.h = new_h;
                    }
                }
            }

            if (allow_disconnected && closed.Count > 0)
            {
                float min_total_cost = closed.Min(x => x.h);
                NodeInfo best = closed.Find(x => x.h.Equals(min_total_cost));
                strategy.FinalDestCell = (T)best.cell;

                BuildPath(start, best.cell, from, strategy.GetDestination() , best, ref path);
                return true;
            }

            return false;
        }

        private static void BuildPath<T>(T start, T end, Vec3 from, Vec3 to, NodeInfo info, ref List<path_pos> path) where T : Cell
        {
            // build result path
            if (path == null)
                return;

            List<Cell> cells_list = new List<Cell>();

            if (info != null)
                path.Add(new path_pos(to.IsZero() ? info.cell.Center : to, info.cell));

            while (info != null)
            {
                path.Add(new path_pos(info.leading_point, info.cell));
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

        public static void Visit<T>(T cell, MovementFlag flags, int max_depth = -1, HashSet<T> allowed_cells = null, IVisitor<T> visitor = null) where T : Cell
        {
            HashSet<T> visited = new HashSet<T>();
            Visit<T>(cell, ref visited, flags, true, 1, max_depth, null, visitor);
        }

        public static void Visit<T>(T cell, ref HashSet<T> visited, MovementFlag flags, bool visit_disabled, int depth = 1, int max_depth = -1, HashSet<T> allowed_cells = null, IVisitor<T> visitor = null) where T : Cell
        {
            visited.Add(cell);

            if (max_depth > 0 && depth >= max_depth)
                return;

            foreach (Cell.Neighbour neighbour in cell.Neighbours)
            {
                T neighbour_cell = (T)neighbour.cell;

                if ((!visit_disabled && neighbour_cell.Disabled) || (neighbour.connection_flags & flags) != flags)
                    continue;

                if (visitor != null)
                    visitor.Visit(neighbour_cell);

                if ((allowed_cells == null || allowed_cells.Contains(neighbour_cell)) && !visited.Contains(neighbour_cell))
                    Visit(neighbour_cell, ref visited, flags, visit_disabled, depth + 1, max_depth, allowed_cells, visitor);
            }
        }

        public static List<CellsPatch> GenerateCellsPatches(List<Cell> cells, MovementFlag movement_flags)
        {
            List<CellsPatch> patches = new List<CellsPatch>();

            //create cells patch for each interconnected group of cells
            HashSet<Cell> cells_copy = new HashSet<Cell>(cells);

            while (cells_copy.Count > 0)
            {
                HashSet<Cell> visited = new HashSet<Cell>();

                Algorihms.Visit(cells_copy.First(), ref visited, movement_flags, true, 1, -1, cells_copy);

                patches.Add(new CellsPatch(visited, movement_flags));

                cells_copy.RemoveWhere(x => visited.Contains(x));
            }

            return patches;
        }

        //public static Vec3 GetRunAwayPosition(List<GridCell> grid_cells, HashSet<Navmesh.region_data> regions, Vec3 p, float min_run_away_dist, MovementFlag flags = MovementFlag.Walk)
        //{
        //    Cell start = null;
        //    if (!GetCellContaining(grid_cells, p, out start, false, flags, 2))
        //        return Vec3.Empty;


        //    List<NodeInfo> open = new List<NodeInfo>();
        //    List<NodeInfo> closed = new List<NodeInfo>();

        //    NodeInfo s = new NodeInfo(start, p, null, 0, 0);
        //    open.Add(s);

        //    while (open.Count > 0)
        //    {
        //        //sort by cost
        //        float min_total_cost = open.Min(x => x.TotalCost);
        //        NodeInfo best = open.Find(x => x.TotalCost.Equals(min_total_cost));

        //        open.Remove(best);

        //        //take node with lower cost from open list
        //        NodeInfo info = best;

        //        if (info.g >= min_run_away_dist)
        //        {
        //            bool overlaped_by_avoid_area = false;
        //            foreach (var region in regions)
        //            {
        //                if (info.cell.AABB.Overlaps2D(region.area, true))
        //                {
        //                    overlaped_by_avoid_area = true;
        //                    break;
        //                }
        //            }

        //            if (!overlaped_by_avoid_area)
        //                return info.cell.Center;
        //        }

        //        closed.Insert(0, info);

        //        foreach (Cell.Neighbour neighbour in info.cell.Neighbours)
        //        {
        //            Cell cell_neighbour = neighbour.cell;

        //            if ((neighbour.connection_flags & flags) != flags)
        //                continue;

        //            Vec3 leading_point = cell_neighbour.AABB.Align(info.leading_point);

        //            NodeInfo info_neighbour = GetNodeInfoFromList(cell_neighbour, leading_point, closed);

        //            // if already processed then skip this neighbour
        //            if (info_neighbour != null)
        //                continue;

        //            float new_g = info.g + info.leading_point.Distance2D(leading_point);
        //            bool is_better = false;

        //            info_neighbour = GetNodeInfoFromList(cell_neighbour, leading_point, open);

        //            // if not in open list
        //            if (info_neighbour == null)
        //            {
        //                info_neighbour = new NodeInfo(cell_neighbour, leading_point, null, new_g, 0);
        //                is_better = true;

        //                open.Insert(0, info_neighbour);
        //            }
        //            else if (new_g < info_neighbour.g)
        //            {
        //                is_better = true;
        //            }

        //            if (is_better)
        //            {
        //                info_neighbour.parent = info;
        //                info_neighbour.g = new_g;
        //            }
        //        }
        //    }

        //    return Vec3.Empty;
        //}
    }
}
