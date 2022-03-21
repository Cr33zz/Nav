using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;

namespace Nav.ExploreEngine
{
    // This algorithm chooses nearest unexplored neighbor but prefer those with many visited neighbors to not leave unexplored islands
    public class Nearest : ExplorationEngine
    {
        public Nearest(Navmesh navmesh, NavigationEngine navigator, int explore_cell_size = 90, bool ignore_small = true)
            : base(navmesh, navigator, explore_cell_size, ignore_small)
        {
        }

        public float DistanceReductionExplorePct { get; set; } = 500;

        protected override ExploreCell GetDestinationCell(ExploreCell curr_explore_cell)
        {
            if (curr_explore_cell == null)
                curr_explore_cell = GetCurrentExploreCell();

            if (curr_explore_cell == null)
                return curr_explore_cell;

            if (!curr_explore_cell.Explored)
                return curr_explore_cell;

            HashSet<ExploreCell> unexplored_cells = GetUnexploredCells(curr_explore_cell);

            if (unexplored_cells.Count > 0)
            {
                ExploreCellSelector selector = CreateExploreCellSelector();

                using (new ReadLock(DataLock, description: "DataLock - Nearest.GetDestinationCell"))
                    Algorihms.VisitBreadth(curr_explore_cell, MovementFlag.None, -1, unexplored_cells, selector);

                if (selector.dest_cell != null)
                    return selector.dest_cell;
            }
            
            return null;
        }

        protected virtual ExploreCellSelector CreateExploreCellSelector()
        {
            return new ExploreCellSelector(this);
        }

        protected class ExploreCellSelector : Algorihms.IDistanceVisitor<ExploreCell>
        {
            public ExploreCellSelector(Nearest explorer)
            {
                this.explorer = explorer;

                if (!explorer.HintPos.IsZero())
                    this.hint_pos = explorer.HintPos;
            }

            public void Visit(ExploreCell cell, float distance)
            {
                float dist = EvaluateDist(cell, distance);

                if (dist < dest_cell_distance)
                {
                    dest_cell = cell;
                    dest_cell_distance = dist;
                }
            }

            protected virtual float EvaluateDist(ExploreCell cell, float distance)
            {
                // decrease distance based on number of explored neighbors (do not leave small unexplored fragments)
                float dist = distance;

                if (!hint_pos.IsZero())
                    dist += cell.Position.Distance(hint_pos);

                dist -= explorer.DistanceReductionExplorePct * GetExploredNeighboursPct(cell, 1);
                //dist -= DISTANCE_REDUCTION_PER_EXPLORED_NEIGHBOUR * (float)explored_neighbours_count;
                //dist -= DISTANCE_REDUCTION_PER_MISSING_NEIGHBOUR * Math.Max(0, AVG_NEIGHBOURS_COUNT - cell.Neighbours.Count);
                //dist -= base_dist * DISTANCE_PCT_REDUCTION_PER_EXPLORED_NEIGHBOUR * (float)explored_neighbours_count;
                //dist -= base_dist * DISTANCE_PCT_REDUCTION_PER_MISSING_NEIGHBOUR * (float)missing_neighbours_count;

                return dist;
            }

            private float GetExploredNeighboursPct(ExploreCell cell, int max_depth)
            {
                HashSet<ExploreCell> cells_group = new HashSet<ExploreCell>();

                Algorihms.Visit(cell, ref cells_group, MovementFlag.None, true, 0, max_depth);

                //treat missing cells as explored thus explore edges to possibly load new navmesh data
                int max_cells_num = (int)Math.Pow(9, max_depth);
                int missing_cells = Math.Max(0, max_cells_num - cells_group.Count);

                return (float)(cells_group.Count(x => ((ExploreCell)x).Explored) + missing_cells) / (float)max_cells_num;
            }

            //const float DISTANCE_REDUCTION_PER_EXPLORED_NEIGHBOUR = 15;
            //const float DISTANCE_REDUCTION_PER_MISSING_NEIGHBOUR = 10;
            //const float DISTANCE_PCT_REDUCTION_PER_EXPLORED_NEIGHBOUR = 0.06f;
            //const float DISTANCE_PCT_REDUCTION_PER_MISSING_NEIGHBOUR = 0.05f;
            //const int AVG_NEIGHBOURS_COUNT = 8;

            public ExploreCell dest_cell = null;
            private float dest_cell_distance = float.MaxValue;
            private Vec3 hint_pos = Vec3.ZERO;
            private Nearest explorer;
        }
    }
}
