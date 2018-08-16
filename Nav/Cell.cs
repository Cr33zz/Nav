using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.IO;

namespace Nav
{
    [Flags]
    public enum MovementFlag
    {
        None = 0x0000,
        Walk = 0x0001,
        Fly = 0x0002,
        All = Walk | Fly,
    }

    public class Cell : IEquatable<Cell>
    {
        public Cell()
        {
            InitCell(-1, new AABB(0, 0, 0, 0, 0, 0), MovementFlag.None, 1);
        }

        public Cell(bool init)
        {
            if (init)
                InitCell(-1, new AABB(0, 0, 0, 0, 0, 0), MovementFlag.None, 1);
        }

        public Cell(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, MovementFlag flags, int id = -1)
        {
            InitCell(id, new AABB(min_x, min_y, min_z, max_x, max_y, max_z), flags, 1);
        }

        public Cell(Vec3 min, Vec3 max, MovementFlag flags, int id = -1)
        {
            InitCell(id, new AABB(min, max), flags, 1);
        }

        public Cell(AABB aabb, MovementFlag flags, float movement_cost_mult = 1, int id = -1)
        {
            InitCell(id, new AABB(aabb), flags, movement_cost_mult);
        }

        public Cell CreateSimplifiedClone()
        {
            Cell c = new Cell(false);
            c.AABB = new AABB(AABB);
            c.AlignPlane = Plane != null ? new Plane(Plane) : null;
            c.AlignPlaneDirty = false;
            c.Flags = Flags;
            return c;
        }

        private void InitCell(int id, AABB aabb, MovementFlag flags, float movement_cost_mult)
        {
            Id = id;
            Flags = flags;
            MovementCostMult = movement_cost_mult;
            Replacement = false;
            Disabled = false;
            AABB = aabb;
            Neighbours = new List<Neighbour>();
            GlobalId = LastCellGlobalId++;
        }

        public override string ToString()
        {
            return $"Id: {Id} MovementCostMult: {MovementCostMult} Threat: {Threat} Replacement: {Replacement} Disabled: {Disabled} GlobalId: {GlobalId}";
        }

        public override bool Equals(Object obj)
        {
            Cell cell = obj as Cell;

            return Equals(cell);
        }

        public bool Equals(Cell cell)
        {
            return GlobalId == cell.GlobalId;
        }

        public override int GetHashCode()
        {
            return GlobalId.GetHashCode();
        }

        public bool Contains(Vec3 p, float z_tolerance = 0)
        {
            return AABB.Contains(p, z_tolerance);
        }

        public bool Contains2D(Vec3 p)
        {
            return AABB.Contains2D(p);
        }

        public bool Overlaps(Vec3 circle_center, float radius, bool tangential_ok = false)
        {
            return AABB.Overlaps(circle_center, radius, tangential_ok);
        }

        public bool Overlaps2D(Vec3 circle_center, float radius, bool tangential_ok = false)
        {
            return AABB.Overlaps2D(circle_center, radius, tangential_ok);
        }

        public bool Overlaps2D(Cell cell, bool tangential_ok = false)
        {
            return AABB.Overlaps2D(cell.AABB, tangential_ok);
        }

        //public void UpdateAlignPlane()
        //{
        //    if (!AlignPlaneDirty)
        //        return;

        //    AlignPlaneDirty = false;

        //    if (AABB.Dimensions.Z < 3 || Neighbours == null)
        //    {
        //        AlignPlane = new Plane(AABB.Center, new Vec3(0, 0, 1));
        //    }
        //    // special case for single neighbor
        //    else if (Neighbours.Count == 1)
        //    {
        //        if (Neighbours[0].border_segment != null)
        //            AlignPlane = new Plane(Neighbours[0].border_segment.Item1, Neighbours[0].border_segment.Item2, Center);
        //    }
        //    else
        //    {
        //        var borders = Neighbours.Where(x => (x.connection_flags & MovementFlag.Walk) != 0 && x.border_segment != null).Select(x => x.border_segment);

        //        if (borders.Count() == 0)
        //        {
        //            AlignPlane = new Plane(AABB.Center, new Vec3(0, 0, 1));
        //            return;
        //        }

        //        // corners starting from min CCW and then from corner above min CCW
        //        Vec3[] corners = new Vec3[] { AABB.Min, new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Min.Z), new Vec3(AABB.Max.X, AABB.Max.Y, AABB.Min.Z), new Vec3(AABB.Max.X, AABB.Min.Y, AABB.Min.Z),
        //                                      new Vec3(AABB.Min.X, AABB.Min.Y, AABB.Max.Z), new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Max.Z), AABB.Max, new Vec3(AABB.Max.X, AABB.Min.Y, AABB.Max.Z) };

        //        bool[] corner_connected = new bool[8];

        //        List<int> connected_upper_corners = new List<int>();
        //        List<int> connected_lower_corners = new List<int>();

        //        for (int i = 0; i < corners.Count(); ++i)
        //        {
        //            corner_connected[i] = borders.FirstOrDefault(x => Vec3.GetDistanceFromSegment(x.Item1, x.Item2, corners[i]) < 2) != null;

        //            if (corner_connected[i])
        //            {
        //                if (i <= 3)
        //                    connected_lower_corners.Add(i);
        //                else
        //                {
        //                    connected_upper_corners.Add(i);
        //                    // disable connected corner below this one, as upper corners has priority as there are situations where two stairscases can cross - 
        //                    // in this case there might be 4 connected lower corners but only 2 connected upper corners
        //                    connected_lower_corners.RemoveAll(x => x == (i - 4));
        //                }
        //            }
        //        }

        //        if (connected_lower_corners.Count == 1 && connected_upper_corners.Count == 1)
        //        {
        //            // this ain't perfect solution (when those 3 point lineup its non-determined)
        //            AlignPlane = new Plane(corners[connected_lower_corners[0]], corners[connected_upper_corners[0]], AABB.Center);
        //        }
        //        // check upper corners first because 
        //        else if (connected_upper_corners.Count > 1)
        //        {
        //            if (connected_lower_corners.Count > 0)
        //                AlignPlane = new Plane(corners[connected_upper_corners[0]], corners[connected_upper_corners[1]], AABB.Center);
        //            else
        //                AlignPlane = new Plane(corners[connected_upper_corners[0]], corners[connected_upper_corners[1]], new Vec3(AABB.Center.X, AABB.Center.Y, AABB.Max.Z));
        //        }
        //        else if (connected_lower_corners.Count > 1)
        //        {
        //            if (connected_upper_corners.Count > 0)
        //                AlignPlane = new Plane(corners[connected_lower_corners[0]], corners[connected_lower_corners[1]], AABB.Center);
        //            else
        //                AlignPlane = new Plane(corners[connected_lower_corners[0]], corners[connected_lower_corners[1]], new Vec3(AABB.Center.X, AABB.Center.Y, AABB.Min.Z));
        //        }
        //        else
        //        {
        //            //having 1 or non connected corners i'm unable to determine plane
        //            //Log("[Nav] Undetermined cell align plane too few corners connected!");
        //        }

        //        //Tuple<Vec3, Vec3> longest_border = null;
        //        //float longest_border_len_2 = 0;

        //        //foreach (Neighbour n in Neighbours)
        //        //{
        //        //    if (n.border_segment == null)
        //        //        continue;

        //        //    var border_segment = n.border_segment;
        //        //    float len_2 = border_segment.Item1.Distance2DSqr(border_segment.Item2);

        //        //    if (longest_border == null || longest_border_len_2 < len_2)
        //        //    {
        //        //        longest_border = border_segment;
        //        //        longest_border_len_2 = len_2;
        //        //    }
        //        //}

        //        //if (longest_border != null)
        //        //    AlignPlane = new Plane(longest_border.Item1, longest_border.Item2, Center);
        //    }
        //}

        public void UpdateAlignPlane()
        {
            if (!AlignPlaneDirty)
                return;

            AlignPlaneDirty = false;

            if (AABB.Dimensions.Z < 3 || Neighbours == null)
            {
                AlignPlane = new Plane(AABB.Center, new Vec3(0, 0, 1));
            }
            // special case for single neighbor
            else if (Neighbours.Count == 1)
            {
                Vec3 v1 = default(Vec3);
                Vec3 v2 = default(Vec3);

                if (GetBorderSegmentWith(Neighbours[0].cell.AABB, ref v1, ref v2))
                    AlignPlane = new Plane(v1, v2, Center);
            }
            else
            {
                // find plane from all possible plane with smallest average distance of all connection points from it
                var connect_points = Neighbours.Where(x => (x.connection_flags & MovementFlag.Walk) != 0).Select(x => x.border_point);

                //this is Diablo related hack, as most certainly won't work in general case :(
                Plane[] possible_align_planes = null;

                if (AABB.Dimensions.X > 30)
                {
                    possible_align_planes = new Plane[] { new Plane(AABB.Min, new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Min.Z), AABB.Center), // bottom-up in direction -X
                                                          new Plane(new Vec3(AABB.Min.X, AABB.Min.Y, AABB.Max.Z), new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Max.Z), AABB.Center), // up-bottom in direction -X
                                                          new Plane(AABB.Min, new Vec3(0, 0, 1)), //flat bottom
                                                          new Plane(AABB.Max, new Vec3(0, 0, 1)) }; // flat up
                }
                else if (AABB.Dimensions.Y > 30)
                {
                    possible_align_planes = new Plane[] { new Plane(new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Min.Z), new Vec3(AABB.Max.X, AABB.Max.Y, AABB.Min.Z), AABB.Center), // bottom-up in direction Y
                                                          new Plane(new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Max.Z), AABB.Max, AABB.Center), // up-bottom in direction Y
                                                          new Plane(AABB.Min, new Vec3(0, 0, 1)), //flat bottom
                                                          new Plane(AABB.Max, new Vec3(0, 0, 1)) }; // flat up
                }
                else
                {
                    possible_align_planes = new Plane[] { new Plane(AABB.Min, new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Min.Z), AABB.Center), // bottom-up in direction -X
                                                          new Plane(new Vec3(AABB.Min.X, AABB.Min.Y, AABB.Max.Z), new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Max.Z), AABB.Center), // up-bottom in direction -X
                                                          new Plane(new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Min.Z), new Vec3(AABB.Max.X, AABB.Max.Y, AABB.Min.Z), AABB.Center), // bottom-up in direction Y
                                                          new Plane(new Vec3(AABB.Min.X, AABB.Max.Y, AABB.Max.Z), AABB.Max, AABB.Center), // up-bottom in direction Y
                                                          new Plane(AABB.Min, new Vec3(0, 0, 1)), //flat bottom
                                                          new Plane(AABB.Max, new Vec3(0, 0, 1)) }; // flat up
                }

                float min_avg_dist = -1;

                foreach (Plane plane in possible_align_planes)
                {
                    float avg_dist = 0;
                    foreach (Vec3 p in connect_points)
                        avg_dist += plane.Distance(p);

                    avg_dist /= (float)connect_points.Count();

                    if (avg_dist < min_avg_dist || min_avg_dist < 0)
                    {
                        AlignPlane = plane;
                        min_avg_dist = avg_dist;
                    }
                }
            }
        }

        public Vec3 Align(Vec3 point)
        {
            if (Plane != null)
                return Plane.Align(point);

            return point;
        }

        private void AddNeighbour(Cell cell, Vec3 border_point)
        {
            Neighbours.Add(new Neighbour(cell, border_point, Flags & cell.Flags));
            AlignPlaneDirty = true;
        }

        // Try to add given cell as neighbour. Returns true when actually added.
        public bool AddNeighbour(Cell cell, ref Vec3 border_point)
        {
            // should not happen - removed due to performance impact - deep down it only checks global ID matching
            if (cell.Equals(this))
                return false;

            AABB intersection = default(AABB);

            if (AABB.Intersect(cell.AABB, ref intersection, true))
            {
                if (Neighbours.Exists(x => x.cell.GlobalId == cell.GlobalId))
                    return false;

                AddNeighbour(cell, intersection.Center);
                cell.AddNeighbour(this, intersection.Center);

                border_point = new Vec3(intersection.Center);
                
                return true;
            }

            return false;
        }

        bool GetBorderSegmentWith(AABB aabb, ref Vec3 v1, ref Vec3 v2)
        {
            AABB intersection = default(AABB);

            if (AABB.Intersect(aabb, ref intersection, true))
            {
                // find widest vector inside intersection AABB along X or Y axis
                Vec3 dimentions = intersection.Dimensions;
                Tuple<Vec3, Vec3> border_segment;

                //if (intersection.Dimensions.Z < 3 && (Math.Abs(intersection.Center.Z - cell.AABB.Max.Z) < 1f || Math.Abs(intersection.Center.Z - cell.AABB.Min.Z) < 1f))
                if (aabb.Dimensions.Z < 3 || AABB.Dimensions.Z < 3)
                {
                    Vec3 intersection_center = intersection.Center;

                    if (dimentions.X > dimentions.Y)
                    {
                        border_segment = new Tuple<Vec3, Vec3>(new Vec3(intersection.Min.X, intersection_center.Y, intersection_center.Z),
                                                               new Vec3(intersection.Max.X, intersection_center.Y, intersection_center.Z));
                    }
                    else
                    {
                        border_segment = new Tuple<Vec3, Vec3>(new Vec3(intersection_center.X, intersection.Min.Y, intersection_center.Z),
                                                               new Vec3(intersection_center.X, intersection.Max.Y, intersection_center.Z));
                    }

                    return true;
                }
            }

            return false;
        }

        public void Detach()
        {
            foreach (Neighbour neighbour in Neighbours)
            {
                Cell other_cell = neighbour.cell;
                for (int i = 0; i < other_cell.Neighbours.Count; ++i)
                {
                    if (other_cell.Neighbours[i].cell == this)
                    {
                        other_cell.Neighbours.RemoveAt(i);
                        break;
                    }
                }
            }

            Neighbours.Clear();
        }

        public float Distance(Cell cell)
        {
            return Center.Distance(cell.Center);
        }

        public float Distance(Vec3 p)
        {
            return AABB.Distance(p);
        }

        public float Distance2D(Vec3 p)
        {
            return AABB.Distance2D(p);
        }

        public bool HasFlags(MovementFlag flags)
        {
            return (Flags & flags) == flags;
        }

        public int GlobalId { get; set; }
        public int Id { get; private set; }
        public MovementFlag Flags { get; protected set; }
        public bool Replacement { get; set; }
        public bool Disabled { get; set; }
        public float MovementCostMult { get; set; }
        public float Threat { get; set; }
        public Int64 UserData { get; set; }
        public AABB AABB { get; protected set; }
        public Vec3 Center { get { return AABB.Center; } }
        public Vec3 Min { get { return AABB.Min; } }
        public Vec3 Max { get { return AABB.Max; } }
        public Plane Plane { get { UpdateAlignPlane(); return AlignPlane; } }

        private Plane AlignPlane = null;
        private bool AlignPlaneDirty = true;

        public class Neighbour
        {
            public Neighbour(Cell cell, Vec3 border_point, MovementFlag connection_flags)
            {
                this.cell = cell;
                this.border_point = border_point;
                this.connection_flags = connection_flags;
            }

            public Cell cell;
            public Vec3 border_point;
            public MovementFlag connection_flags;
        }

        public List<Neighbour> Neighbours { get; private set; }

        //public List<Cell> Neighbours { get; private set; }
        //public List<Vec3> NeighbourBorderPoints { get; protected set; }

        public class CompareByGlobalId : IComparer<Cell>
        {
            public int Compare(Cell x, Cell y)
            {
                return x.GlobalId.CompareTo(y.GlobalId);
            }

        }

        internal virtual void Serialize(BinaryWriter w)
        {
            w.Write(GlobalId);
            w.Write(Id);
            AABB.Serialize(w);
            w.Write((int)Flags);
            w.Write(Replacement);
            w.Write(Disabled);
            w.Write(MovementCostMult);
            w.Write(Threat);

            w.Write(Neighbours.Count);
            foreach (Neighbour neighbour in Neighbours)
            {
                w.Write(neighbour.cell.GlobalId);
                neighbour.border_point.Serialize(w);
                w.Write((int)neighbour.connection_flags);
            }
        }

        internal void Deserialize<T>(HashSet<T> all_cells, BinaryReader r) where T : Cell, new()
        {
            GlobalId = r.ReadInt32();
            Id = r.ReadInt32();
            AABB = new AABB(r);
            Flags = (MovementFlag)r.ReadInt32();
            Replacement = r.ReadBoolean();
            Disabled = r.ReadBoolean();
            MovementCostMult = r.ReadSingle();
            Threat = r.ReadSingle();

            int neighbours_num = r.ReadInt32();
            for (int i = 0; i < neighbours_num; ++i)
            {
                Neighbour neighbour = new Neighbour(null, Vec3.ZERO, MovementFlag.None);

                int neighbour_global_id = r.ReadInt32();
                neighbour.cell = all_cells.FirstOrDefault(x => x.GlobalId == neighbour_global_id);
                neighbour.border_point = new Vec3(r);
                neighbour.connection_flags = (MovementFlag)r.ReadInt32();

                if (neighbour.cell != null)
                    Neighbours.Add(neighbour);
                else
                    Console.WriteLine("Oops!");
            }

            AlignPlaneDirty = true;
        }

        internal static int LastCellGlobalId = 0;
    }
}
