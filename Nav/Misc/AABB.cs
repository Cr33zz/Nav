using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;

namespace Nav
{
    [Serializable]
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

        public AABB(Vec3 center, float radius, bool use_2d = false)
        {
            Min = new Vec3(center.X - radius, center.Y - radius, use_2d ? 0 : (center.Z - radius));
            Max = new Vec3(center.X + radius, center.Y + radius, use_2d ? 0 : (center.Z + radius));
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

        public float Radius => MaxDimension2d;

        public float MaxDimension2d
        {
            get { return Math.Max(Max.X - Min.X, Max.Y - Min.Y) * 0.5f; }
        }

        public float MinDimension2d
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
            return (float)Math.Sqrt(Distance2DSqr(p));
        }

        public float Distance2DSqr(Vec3 p)
        {
            float dx = Math.Max(0, Math.Max(Min.X - p.X, p.X - Max.X));
            float dy = Math.Max(0, Math.Max(Min.Y - p.Y, p.Y - Max.Y));
            return dx * dx + dy * dy;
        }

        public float Distance(Vec3 p)
        {
            return (float)Math.Sqrt(DistanceSqr(p));
        }

        public float DistanceSqr(Vec3 p)
        {
            float dx = Math.Max(0, Math.Max(Min.X - p.X, p.X - Max.X));
            float dy = Math.Max(0, Math.Max(Min.Y - p.Y, p.Y - Max.Y));
            float dz = Math.Max(0, Math.Max(Min.Z - p.Z, p.Z - Max.Z));
            return dx * dx + dy * dy + dz * dz;
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

        public bool Overlaps(AABB aabb, bool tangential_ok = false)
        {
            return (tangential_ok && Max.X >= aabb.Min.X && Min.X <= aabb.Max.X && Max.Y >= aabb.Min.Y && Min.Y <= aabb.Max.Y && Max.Z >= aabb.Min.Z && Min.Z <= aabb.Max.Z) ||
                   (Max.X > aabb.Min.X && Min.X < aabb.Max.X && Max.Y > aabb.Min.Y && Min.Y < aabb.Max.Y && Max.Z > aabb.Min.Z && Min.Z < aabb.Max.Z);
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

        public AABB[] Extract2D(AABB aabb, bool splitAlongX = false)
        {
            AABB inter = default(AABB);

            if (!Intersect2D(aabb, ref inter))
                return null;

            inter.Min.Z = Min.Z;
            inter.Max.Z = Max.Z;

            List<AABB> result = null;

            if (splitAlongX)
            {
                result = new List<AABB>() { new AABB(inter.Min, inter.Max),
                                            new AABB(Min, new Vec3(inter.Min.X, Max.Y, Max.Z)),
                                            new AABB(new Vec3(inter.Min.X, inter.Max.Y, Min.Z), new Vec3(inter.Max.X, Max.Y, Max.Z)),
                                            new AABB(new Vec3(inter.Min.X, Min.Y, Min.Z), new Vec3(inter.Max.X, inter.Min.Y, Max.Z)),
                                            new AABB(new Vec3(inter.Max.X, Min.Y, Min.Z), Max) };
            }
            else
            {
                result = new List<AABB>() { new AABB(inter.Min, inter.Max),
                                            new AABB(Min, new Vec3(Max.X, inter.Min.Y, Max.Z)),
                                            new AABB(new Vec3(Min.X, inter.Min.Y, Min.Z), new Vec3(inter.Min.X, inter.Max.Y, Max.Z)),
                                            new AABB(new Vec3(inter.Max.X, inter.Min.Y, Min.Z), new Vec3(Max.X, inter.Max.Y, Max.Z)),
                                            new AABB(new Vec3(Min.X, inter.Max.Y, Min.Z), Max) };
            }

            result.RemoveAll(x => x.Max.X - x.Min.X == 0 || x.Max.Y - x.Min.Y == 0);

            return result.ToArray();
        }

        // returns new aabb that is evenly expanded/shrinked along all axis
        public AABB Resized(float value)
        {
            return new AABB(Min - value, Max + value);
        }

        public AABB ResizedByFactor(float factor)
        {
            return new AABB(Center + (Min - Center) * factor, Center + (Max - Center) * factor);
        }

        public AABB Extend(AABB aabb)
        {
            if (IsZero())
            {
                return aabb;
            }

            return new AABB(Vec3.Min(Min, aabb.Min), Vec3.Max(Max, aabb.Max));
        }

        public AABB Extend(Vec3 point)
        {
            return new AABB(Vec3.Min(Min, point), Vec3.Max(Max, point));
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

        public AABB Translated(Vec3 v)
        {
            return new AABB(Min + v, Max = Max + v);
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

        public bool SegmentTest(Vec3 start, Vec3 end, ref Vec3 result)
        {
            return InternalSegmentTest(start, end, ref result, 3);
        }

        public bool SegmentTest2D(Vec3 start, Vec3 end, ref Vec3 result)
        {
            return InternalSegmentTest(start, end, ref result, 2);
        }

        private bool InternalSegmentTest(Vec3 start, Vec3 end, ref Vec3 result, int num_dim)
        {
            Vec3 ray_dir = end - start;
            float length = num_dim > 2 ? ray_dir.Normalize() : ray_dir.Normalize2D();
            float length_2 = length * length;
            if (InternalRayTest(start, ray_dir, ref result, num_dim))
            {
                if ((num_dim > 2 ? result.DistanceSqr(start) : result.Distance2DSqr(start)) <= length_2)
                    return true;

                result = end;
                return false;
            }

            return false;
        }

        public bool RayTest(Vec3 ray_origin, Vec3 ray_dir, ref Vec3 result)
        {
            return InternalRayTest(ray_origin, ray_dir, ref result, 3);
        }

        public bool RayTest2D(Vec3 ray_origin, Vec3 ray_dir, ref Vec3 result)
        {
            return InternalRayTest(ray_origin, ray_dir, ref result, 2);
        }

        private Vec3 Bounds(int i)
        {
            return i == 0 ? Min : Max;
        }

        //private bool InternalRayTestFast(Vec3 ray_origin, Vec3 ray_dir, int num_dim)
        //{

        //}

        private bool InternalRayTest(Vec3 ray_origin, Vec3 ray_dir, ref Vec3 result, int num_dim)
        {
            // implementation based upon https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
            // + added custom support for rays tangent to sides of AABB
            Vec3 ray_dir_inv = 1 / ray_dir;
            int[] sign = new int[] { ray_dir_inv.X < 0 ? 1 : 0, ray_dir_inv.Y < 0 ? 1 : 0, ray_dir_inv.Z < 0 ? 1 : 0 };

            float tmin, tmax, tymin, tymax, tzmin, tzmax;

            tmin = (Bounds(sign[0]).X - ray_origin.X) * ray_dir_inv.X;
            tmax = (Bounds(1 - sign[0]).X - ray_origin.X) * ray_dir_inv.X;
            tymin = (Bounds(sign[1]).Y - ray_origin.Y) * ray_dir_inv.Y;
            tymax = (Bounds(1 - sign[1]).Y - ray_origin.Y) * ray_dir_inv.Y;

            if ((tmin > tymax) || (tymin > tmax))
                return false;

            bool tangent_x = float.IsNaN(tmin) || float.IsNaN(tmax);
            bool tangent_y = float.IsNaN(tymin) || float.IsNaN(tymax);

            if (tymin > tmin || tangent_x)
                tmin = tymin;
            if (tymax < tmax && !tangent_y)
                tmax = tymax;

            bool tangent_z = false;

            if (num_dim > 2)
            {
                tzmin = (Bounds(sign[2]).Z - ray_origin.Z) * ray_dir_inv.Z;
                tzmax = (Bounds(1 - sign[2]).Z - ray_origin.Z) * ray_dir_inv.Z;

                tangent_z = float.IsNaN(tzmin) || float.IsNaN(tzmax);

                if ((tmin > tzmax) || (tzmin > tmax))
                    return false;

                if (tzmin > tmin || float.IsNaN(tmin))
                    tmin = tzmin;
                if (tzmax < tmax && !float.IsNaN(tmax))
                    tmax = tzmax;
            }

            if (tmax >= 0)
            {
                if (tmin < 0 && (tangent_x || tangent_y || tangent_z))
                    result = ray_origin;
                else
                    result = ray_origin + ray_dir * (tmin < 0 ? tmax : tmin);
                return true;
            }

            return false;
        }

        public Vec3 Min;
        public Vec3 Max;
    }
}
