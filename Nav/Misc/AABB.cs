using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;

namespace Nav
{
    public class AABB : IEquatable<AABB>
    {
        public AABB()
        {
            min = Vec3.Empty;
            max = Vec3.Empty;
            is_empty = true;
        }

        public AABB(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z)
        {
            min = new Vec3(min_x, min_y, min_z);
            max = new Vec3(max_x, max_y, max_z);
        }

        public AABB(Vec3 min, Vec3 max)
        {
            this.min = new Vec3(min);
            this.max = new Vec3(max);
        }

        public AABB(AABB aabb)
        {
            min = new Vec3(aabb.min);
            max = new Vec3(aabb.max);
        }

        public AABB(BinaryReader r)
        {
            Deserialize(r);
        }

        public static AABB operator +(AABB LHS, Vec3 RHS)
        {
            return new AABB(LHS.min + RHS, LHS.max + RHS);
        }

        public float Area
        {
            get { return (max.X - min.X) * (max.Y - min.Y); }
        }

        public float Radius
        {
            get { return Math.Min(max.X - min.X, max.Y - min.Y) * 0.5f; }
        }

        public Vec3 Dimensions
        {
            get { return new Vec3(max.X - min.X, max.Y - min.Y, max.Z - min.Z); }
        }

        public float Volume
        {
            get { return Area * (max.Z - min.Z); }
        }

        public Vec3 Center
        {
            get { return (min + max) * 0.5f; }
        }

        public Vec3 Min
        {
            get { return min; }
        }

        public Vec3 Max
        {
            get { return max; }
        }

        public bool IsEmpty
        {
            get { return is_empty; }
        }

        public override bool Equals(Object obj)
        {
            if (obj == null)
                return false;

            AABB aabb = obj as AABB;

            return Equals(aabb);
        }

        public bool Equals(AABB aabb)
        {
            if (aabb == null)
                return false;

            return min.Equals(aabb.min) && max.Equals(aabb.max);
        }

        public override int GetHashCode()
        {
            return min.GetHashCode() + max.GetHashCode();
        }

        public float Distance2D(Vec3 p)
        {
            float dx = Math.Max(0, Math.Max(min.X - p.X, p.X - max.X));
            float dy = Math.Max(0, Math.Max(min.Y - p.Y, p.Y - max.Y));
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        public float Distance(Vec3 p)
        {
            float dx = Math.Max(0, Math.Max(min.X - p.X, p.X - max.X));
            float dy = Math.Max(0, Math.Max(min.Y - p.Y, p.Y - max.Y));
            float dz = Math.Max(0, Math.Max(min.Z - p.Z, p.Z - max.Z));
            return (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        public bool Contains(Vec3 p, float z_tolerance = 0)
        {
            return p.X >= min.X && p.X <= max.X && p.Y >= min.Y && p.Y <= max.Y && (p.Z + z_tolerance) >= min.Z && (p.Z - z_tolerance) <= max.Z;
        }

        public bool Contains2D(Vec3 p)
        {
            return p.X >= min.X && p.X <= max.X && p.Y >= min.Y && p.Y <= max.Y;
        }

        public bool Overlaps(Vec3 circle_center, float radius, bool tangential_ok = false)
        {
            Vec3 center_aligned = Align(circle_center);
            float dist = circle_center.Distance(center_aligned);
            return tangential_ok ? dist < radius : dist <= radius;
        }

        public bool Overlaps2D(Vec3 circle_center, float radius, bool tangential_ok = false)
        {
            Vec3 center_aligned = Align(circle_center);
            float dist = circle_center.Distance2D(center_aligned);
            return tangential_ok ? dist < radius : dist <= radius;
        }

        public bool Overlaps2D(AABB aabb, bool tangential_ok = false)
        {
            return (tangential_ok && max.X >= aabb.min.X && min.X <= aabb.max.X && max.Y >= aabb.min.Y && min.Y <= aabb.max.Y) ||
                   (max.X > aabb.min.X && min.X < aabb.max.X && max.Y > aabb.min.Y && min.Y < aabb.max.Y);
        }

        public AABB Intersect2D(AABB aabb, bool tangential_ok = false)
        {
            if (!Overlaps2D(aabb, tangential_ok))
                return null;

            return new AABB(Vec3.Max2D(min, aabb.min), Vec3.Min2D(max, aabb.max));
        }

        public AABB Intersect(AABB aabb, bool tangential_ok = false)
        {
            if (!Overlaps2D(aabb, tangential_ok))
                return null;

            return new AABB(Vec3.Max(min, aabb.min), Vec3.Min(max, aabb.max));
        }

        public AABB[] Extract2D(AABB aabb)
        {
            AABB inter = Intersect2D(aabb);

            if (inter == null)
                return null;

            //float PRECISION = 0.01f;
            //Vec3 EXTENTS_INCREASE_VEC = new Vec3(PRECISION, PRECISION, PRECISION);

            inter.Min.Z = Min.Z;
            inter.Max.Z = Max.Z;

            List<AABB> result = new List<AABB>() { new AABB(inter.Min, inter.Max),
                                                   new AABB(Min, new Vec3(Max.X, inter.Min.Y, Max.Z)),
                                                   new AABB(new Vec3(Min.X, inter.Min.Y, Min.Z), new Vec3(inter.Min.X, inter.Max.Y, Max.Z)),
                                                   new AABB(new Vec3(inter.Max.X, inter.Min.Y, Min.Z), new Vec3(Max.X, inter.Max.Y, Max.Z)),
                                                   new AABB(new Vec3(Min.X, inter.Max.Y, Min.Z), Max) };

            result.RemoveAll(x => x.Max.X - x.Min.X == 0 || x.Max.Y - x.Min.Y == 0);

            //foreach (AABB r in result)
            //    r.Extend(new AABB(r.Min - EXTENTS_INCREASE_VEC, r.Max + EXTENTS_INCREASE_VEC));
            
            return result.ToArray();
        }

        public void Extend(AABB aabb)
        {
            if (is_empty)
            {
                min = new Vec3(aabb.min);
                max = new Vec3(aabb.max);
                is_empty = aabb.is_empty;
                return;
            }

            min = Vec3.Min(min, aabb.min);
            max = Vec3.Max(max, aabb.max);
        }

        public static AABB Maximum(AABB aabb1, AABB aabb2)
        {
            return new AABB(Vec3.Min(aabb1.min, aabb2.min), Vec3.Max(aabb1.max, aabb2.max));
        }

        public static AABB Mininum(AABB aabb1, AABB aabb2)
        {
            return new AABB(Vec3.Max(aabb1.min, aabb2.min), Vec3.Min(aabb1.max, aabb2.max));
        }

        public Vec3 Align(Vec3 p)
        {
            return Vec3.Min(Vec3.Max(Min, p), Max);
        }

        public void Translate(Vec3 v)
        {
            min += v;
            max += v;
        }

        public void Serialize(BinaryWriter w)
        {
            min.Serialize(w);
            max.Serialize(w);
            w.Write(is_empty);
        }

        public void Deserialize(BinaryReader r)
        {
            min = new Vec3(r);
            max = new Vec3(r);
            is_empty = r.ReadBoolean();
        }

        public bool Inside(Vec3 circle_center, float radius)
        {
            return min.Distance(circle_center) < radius &&
                   max.Distance(circle_center) < radius &&
                   new Vec3(min.X, max.Y, max.Z).Distance(circle_center) < radius &&
                   new Vec3(min.X, max.Y, min.Z).Distance(circle_center) < radius &&
                   new Vec3(max.X, min.Y, max.Z).Distance(circle_center) < radius &&
                   new Vec3(max.X, min.Y, min.Z).Distance(circle_center) < radius &&
                   new Vec3(min.X, min.Y, max.Z).Distance(circle_center) < radius &&
                   new Vec3(max.X, max.Y, min.Z).Distance(circle_center) < radius;
        }

        // method assumes that pos is inside AABB
        public Vec3 GetBounceDir2D(Vec3 pos)
        {
            float dist_1 = pos.X - min.X;
            float dist_2 = max.X - pos.X;
            float dist_3 = pos.Y - min.Y;
            float dist_4 = max.Y - pos.Y;

            float[] dists = new float[] { dist_1, dist_2, dist_3, dist_4 };

            float min_dist = dists.Min();

            if (min_dist.Equals(dist_1))
                return new Vec3(1, 0, 0);
            else if (min_dist.Equals(dist_2))
                return new Vec3(-1, 0, 0);
            else if (min_dist.Equals(dist_3))
                return new Vec3(0, 1, 0);
            else 
                return new Vec3(0, -1, 0);
        }

        public Vec3 GetRandomPos()
        {
            Random rng = new Random();
            return min + new Vec3((float)rng.NextDouble() * (max.X - min.X), (float)rng.NextDouble() * (max.Y - min.Y), (float)rng.NextDouble() * (max.Z - min.Z));
        }

        public bool RayTest(Vec3 ray_origin, Vec3 ray_dir, ref Vec3 result)
        {
            return InternalRayTest(ray_origin, ray_dir, ref result, 3);
        }

        public bool RayTest2D(Vec3 ray_origin, Vec3 ray_dir, ref Vec3 result)
        {
            return InternalRayTest(ray_origin, ray_dir, ref result, 2);
        }

        private bool InternalRayTest(Vec3 ray_origin, Vec3 ray_dir, ref Vec3 result, int num_dim)
        {
            // implementation based upon https://www.gamedev.net/forums/topic/495636-raybox-collision-intersection-point/
            Vec3 tmin = (Min - ray_origin) / ray_dir;
            Vec3 tmax = (Max - ray_origin) / ray_dir;

            Vec3 real_min = Vec3.Min(tmin, tmax);
            Vec3 real_max = Vec3.Max(tmin, tmax);

            float minmax = Math.Min(real_max.X, real_max.Y);
            float maxmin = Math.Max(real_min.X, real_min.Y);

            if (num_dim > 2)
            {
                minmax = Math.Min(minmax, real_max.Z);
                maxmin = Math.Max(maxmin, real_min.Z);
            }

            // when both values are negative origin is 'after' aabb when looking in ray direction
            if (minmax >= maxmin && minmax >= 0)
            {
                // when maxmin is negative we are inside the aabb
                result = new Vec3(ray_origin + ray_dir * (maxmin < 0 ? minmax : maxmin));
                return true;
            }

            return false;
        }	

        public static readonly AABB Empty = new AABB();
        
        protected Vec3 min = null;
        protected Vec3 max = null;
        private bool is_empty = false;
    }
}
