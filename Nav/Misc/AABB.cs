using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;

namespace Nav
{
    public struct AABB : IEquatable<AABB>
    {
        public static readonly AABB ZERO = default(AABB);

        public AABB(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z)
        {
            Min = new Vec3(min_x, min_y, min_z);
            Max = new Vec3(max_x, max_y, max_z);
        }

        public AABB(Vec3 min, Vec3 max)
        {
            Min = new Vec3(min);
            Max = new Vec3(max);
        }

        public AABB(AABB aabb)
        {
            Min = new Vec3(aabb.Min);
            Max = new Vec3(aabb.Max);
        }

        public AABB(BinaryReader r) : this()
        {
            Deserialize(r);
        }

        public static AABB operator +(AABB aabb1, Vec3 aabb2)
        {
            return new AABB(aabb1.Min + aabb2, aabb1.Max + aabb2);
        }

        public float Area
        {
            get { return (Max.X - Min.X) * (Max.Y - Min.Y); }
        }

        public float Radius
        {
            get { return Math.Min(Max.X - Min.X, Max.Y - Min.Y) * 0.5f; }
        }

        public Vec3 Dimensions
        {
            get { return new Vec3(Max.X - Min.X, Max.Y - Min.Y, Max.Z - Min.Z); }
        }

        public float Volume
        {
            get { return Area * (Max.Z - Min.Z); }
        }

        public Vec3 Center
        {
            get { return (Min + Max) * 0.5f; }
        }

        public bool IsZero() { return Min.IsZero() && Max.IsZero(); }

        public override bool Equals(Object obj)
        {
            return obj is AABB && Equals((AABB)obj);
        }

        public bool Equals(AABB aabb)
        {
            return this == aabb;
        }

        public static bool operator ==(AABB aabb1, AABB aabb2)
        {
            return aabb1.Min == aabb2.Min && aabb1.Max == aabb2.Max;
        }
        public static bool operator !=(AABB x, AABB y)
        {
            return !(x == y);
        }

        public override int GetHashCode()
        {
            return Min.GetHashCode() + Max.GetHashCode();
        }

        public float Distance2D(Vec3 p)
        {
            float dx = Math.Max(0, Math.Max(Min.X - p.X, p.X - Max.X));
            float dy = Math.Max(0, Math.Max(Min.Y - p.Y, p.Y - Max.Y));
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        public float Distance(Vec3 p)
        {
            float dx = Math.Max(0, Math.Max(Min.X - p.X, p.X - Max.X));
            float dy = Math.Max(0, Math.Max(Min.Y - p.Y, p.Y - Max.Y));
            float dz = Math.Max(0, Math.Max(Min.Z - p.Z, p.Z - Max.Z));
            return (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        public bool Contains(Vec3 p, float z_tolerance = 0)
        {
            return p.X >= Min.X && p.X <= Max.X && p.Y >= Min.Y && p.Y <= Max.Y && (p.Z + z_tolerance) >= Min.Z && (p.Z - z_tolerance) <= Max.Z;
        }

        public bool Contains2D(Vec3 p)
        {
            return p.X >= Min.X && p.X <= Max.X && p.Y >= Min.Y && p.Y <= Max.Y;
        }

        public bool Overlaps(Vec3 circle_center, float radius, bool tangential_ok = false)
        {
            float radius_sqr = radius * radius;
            Vec3 center_aligned = Align(circle_center);
            float dist_sqr = circle_center.DistanceSqr(center_aligned);
            return tangential_ok ? dist_sqr < radius_sqr : dist_sqr <= radius_sqr;
        }

        public bool Overlaps2D(Vec3 circle_center, float radius, bool tangential_ok = false)
        {
            float radius_sqr = radius * radius;
            Vec3 center_aligned = Align(circle_center);
            float dist_sqr = circle_center.Distance2DSqr(center_aligned);
            return tangential_ok ? dist_sqr < radius_sqr : dist_sqr <= radius_sqr;
        }

        public bool Overlaps2D(AABB aabb, bool tangential_ok = false)
        {
            return (tangential_ok && Max.X >= aabb.Min.X && Min.X <= aabb.Max.X && Max.Y >= aabb.Min.Y && Min.Y <= aabb.Max.Y) ||
                   (Max.X > aabb.Min.X && Min.X < aabb.Max.X && Max.Y > aabb.Min.Y && Min.Y < aabb.Max.Y);
        }

        public bool Intersect2D(AABB aabb, ref AABB output, bool tangential_ok = false)
        {
            if (!Overlaps2D(aabb, tangential_ok))
                return false;

            output.Min = Vec3.Max2D(Min, aabb.Min);
            output.Max = Vec3.Min2D(Max, aabb.Max);
            return true;
        }

        public bool Intersect(AABB aabb, ref AABB output, bool tangential_ok = false)
        {
            if (!Overlaps2D(aabb, tangential_ok))
                return false;

            output.Min = Vec3.Max(Min, aabb.Min);
            output.Max = Vec3.Min(Max, aabb.Max);
            return true;
        }

        public AABB[] Extract2D(AABB aabb)
        {
            AABB inter = default(AABB);

            if (!Intersect2D(aabb, ref inter))
                return null;

            inter.Min.Z = Min.Z;
            inter.Max.Z = Max.Z;

            List<AABB> result = new List<AABB>() { new AABB(inter.Min, inter.Max),
                                                   new AABB(Min, new Vec3(Max.X, inter.Min.Y, Max.Z)),
                                                   new AABB(new Vec3(Min.X, inter.Min.Y, Min.Z), new Vec3(inter.Min.X, inter.Max.Y, Max.Z)),
                                                   new AABB(new Vec3(inter.Max.X, inter.Min.Y, Min.Z), new Vec3(Max.X, inter.Max.Y, Max.Z)),
                                                   new AABB(new Vec3(Min.X, inter.Max.Y, Min.Z), Max) };

            result.RemoveAll(x => x.Max.X - x.Min.X == 0 || x.Max.Y - x.Min.Y == 0);

            return result.ToArray();
        }

        public void Extend(AABB aabb)
        {
            if (IsZero())
            {
                Min = new Vec3(aabb.Min);
                Max = new Vec3(aabb.Max);
                return;
            }

            Min = Vec3.Min(Min, aabb.Min);
            Max = Vec3.Max(Max, aabb.Max);
        }

        public static AABB Maximum(AABB aabb1, AABB aabb2)
        {
            return new AABB(Vec3.Min(aabb1.Min, aabb2.Min), Vec3.Max(aabb1.Max, aabb2.Max));
        }

        public static AABB Mininum(AABB aabb1, AABB aabb2)
        {
            return new AABB(Vec3.Max(aabb1.Min, aabb2.Min), Vec3.Min(aabb1.Max, aabb2.Max));
        }

        public Vec3 Align(Vec3 p)
        {
            return Vec3.Min(Vec3.Max(Min, p), Max);
        }

        public void Translate(Vec3 v)
        {
            Min += v;
            Max += v;
        }

        public void Serialize(BinaryWriter w)
        {
            Min.Serialize(w);
            Max.Serialize(w);
        }

        public void Deserialize(BinaryReader r)
        {
            Min = new Vec3(r);
            Max = new Vec3(r);
        }

        public bool Inside(Vec3 circle_center, float radius)
        {
            return Min.Distance(circle_center) < radius &&
                   Max.Distance(circle_center) < radius &&
                   new Vec3(Min.X, Max.Y, Max.Z).Distance(circle_center) < radius &&
                   new Vec3(Min.X, Max.Y, Min.Z).Distance(circle_center) < radius &&
                   new Vec3(Max.X, Min.Y, Max.Z).Distance(circle_center) < radius &&
                   new Vec3(Max.X, Min.Y, Min.Z).Distance(circle_center) < radius &&
                   new Vec3(Min.X, Min.Y, Max.Z).Distance(circle_center) < radius &&
                   new Vec3(Max.X, Max.Y, Min.Z).Distance(circle_center) < radius;
        }

        // method assumes that pos is inside AABB
        public Vec3 GetBounceDir2D(Vec3 pos)
        {
            float dist_1 = pos.X - Min.X;
            float dist_2 = Max.X - pos.X;
            float dist_3 = pos.Y - Min.Y;
            float dist_4 = Max.Y - pos.Y;

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

        public Vec3 GetRandomPos(Random rng = null)
        {
            rng = rng ?? new Random();
            return Min + new Vec3((float)rng.NextDouble() * (Max.X - Min.X), (float)rng.NextDouble() * (Max.Y - Min.Y), (float)rng.NextDouble() * (Max.Z - Min.Z));
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
            // + added custom support for rays tangent to sides of AABB
            Vec3 ray_dir_inv = 1 / ray_dir;
            Vec3 tmin = (Min - ray_origin) * ray_dir_inv;
            Vec3 tmax = (Max - ray_origin) * ray_dir_inv;

            Vec3 real_min = Vec3.Min(tmin, tmax);
            Vec3 real_max = Vec3.Max(tmin, tmax);

            float minmax = real_max.X < real_max.Y ? real_max.X : real_max.Y;
            float maxmin = real_min.X > real_min.Y ? real_min.X : real_min.Y;

            bool tangent = false;

            if (float.IsInfinity(maxmin))
            {
                tangent = true;
                maxmin = float.IsInfinity(real_min.X) ? real_min.Y : real_min.X;
            }

            if (num_dim > 2)
            {
                minmax = minmax < real_max.Z ? minmax : real_max.Z;
                maxmin = maxmin > real_min.Z ? maxmin : real_min.Z;
                
                float maxmin_tmp = maxmin > real_min.Z ? maxmin : real_min.Z;
                if (float.IsInfinity(maxmin_tmp))
                {
                    tangent = true;
                    maxmin = float.IsInfinity(maxmin) ? real_min.Z : maxmin;
                }
            }

            // for rays tangent to aabb, intersecting it and starting in the middle of a side we return origin since this the fisrt point of contact

            // when both values are negative origin is 'after' aabb when looking in ray direction
            if (minmax >= maxmin && minmax >= 0)
            {
                // when maxmin is negative we are inside the aabb
                result = (tangent && maxmin < 0) ? ray_origin : (ray_origin + ray_dir * (maxmin < 0 ? minmax : maxmin));
                return true;
            }

            return false;
        }

        public Vec3 Min;
        public Vec3 Max;
    }
}
