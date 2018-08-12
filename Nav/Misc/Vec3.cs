using System;
using System.IO;
using System.Drawing.Drawing2D;

namespace Nav
{
    public class Vec3 : IEquatable<Vec3>
    {
        public float X, Y, Z;

        public static readonly Vec3 Empty = new Vec3();

        public bool IsEmpty { get; private set; }

        public Vec3(float x, float y, float z)
        {
            this.X = x; 
            this.Y = y; 
            this.Z = z;
            IsEmpty = false;
        }

        public Vec3(Vec3 v)
        {
            X = v.X; 
            Y = v.Y; 
            Z = v.Z;
            IsEmpty = v.IsEmpty;
        }

        private Vec3()
        {
            X = Y = Z = 0;
            IsEmpty = true;
        }

        public Vec3(BinaryReader r)
        {
            Deserialize(r);
        }

        // http://en.wikipedia.org/wiki/File:3D_Spherical.svg
        // theta is the angle between the positive Z-axis and the vector in question (0 ≤ θ ≤ π)
        // phi is the angle between the projection of the vector onto the X-Y-plane and the positive X-axis (0 ≤ φ < 2π)        
        public static Vec3 FromSpherical(float theta, float phi)
        {
            return new Vec3((float)(Math.Sin(theta) * Math.Cos(phi)), (float)(Math.Sin(theta) * Math.Sin(phi)), (float)Math.Cos(theta));
        }

        public static Vec3 FromSpherical2D(float phi)
        {
            return new Vec3((float)Math.Cos(phi), (float)Math.Sin(phi), 0);
        }

        public override bool Equals(Object obj)
        {
            if (obj == null)
                return false;

            Vec3 v = obj as Vec3;

            return Equals(v);
        }

        public bool Equals(Vec3 v)
        {
            return X.Equals(v.X) && Y.Equals(v.Y) && Z.Equals(v.Z);
        }

        public bool Equals(Vec3 v, float epsilon)
        {
            return Math.Abs(X - v.X) < epsilon &&
                   Math.Abs(Y - v.Y) < epsilon && 
                   Math.Abs(Z - v.Z) < epsilon;
        }

        public override int GetHashCode()
        {
            // i know this is bullshit!
            return X.GetHashCode() + Y.GetHashCode() + Z.GetHashCode();
        }

        public static Vec3 operator +(Vec3 LHS, Vec3 RHS)
        {
            return new Vec3(LHS.X + RHS.X,
                            LHS.Y + RHS.Y,
                            LHS.Z + RHS.Z);
        }

        public static Vec3 operator -(Vec3 LHS, Vec3 RHS)
        {
            return new Vec3(LHS.X - RHS.X,
                            LHS.Y - RHS.Y,
                            LHS.Z - RHS.Z);
        }

        public static Vec3 operator -(Vec3 RHS)
        {
            return new Vec3(-RHS.X,
                            -RHS.Y,
                            -RHS.Z);
        }

        public static Vec3 operator *(Vec3 LHS, float RHS)
        {
            return new Vec3(LHS.X * RHS,
                            LHS.Y * RHS,
                            LHS.Z * RHS);
        }

        public static Vec3 operator *(Vec3 LHS, Vec3 RHS)
        {
            return new Vec3(LHS.X * RHS.X,
                            LHS.Y * RHS.Y,
                            LHS.Z * RHS.Z);
        }

        public static Vec3 operator /(Vec3 LHS, float RHS)
        {
            return new Vec3(LHS.X / RHS,
                            LHS.Y / RHS,
                            LHS.Z / RHS);
        }

        public static Vec3 operator /(Vec3 LHS, Vec3 RHS)
        {
            return new Vec3(LHS.X / RHS.X,
                            LHS.Y / RHS.Y,
                            LHS.Z / RHS.Z);
        }

        public static Vec3 Rotate(Vec3 v, float angle, Vec3 axis)
        {
            if (angle.Equals(0))
                return new Vec3(v);

            double DEG2RAD = Math.PI / 180;

            double c = Math.Cos(angle * DEG2RAD);
            double s = Math.Sin(angle * DEG2RAD);
            double C = 1.0 - c;

            double[,] Q = new double[3,3];
            Q[0,0] = axis.X * axis.X * C + c;
            Q[0,1] = axis.Y * axis.X * C + axis.Z * s;
            Q[0,2] = axis.Z * axis.X * C - axis.Y * s;

            Q[1,0] = axis.Y * axis.X * C - axis.Z * s;
            Q[1,1] = axis.Y * axis.Y * C + c;
            Q[1,2] = axis.Z * axis.Y * C + axis.X * s;

            Q[2,0] = axis.X * axis.Z * C + axis.Y * s;
            Q[2,1] = axis.Z * axis.Y * C - axis.X * s;
            Q[2,2] = axis.Z * axis.Z * C + c;

            return new Vec3((float)(v.X * Q[0, 0] + v.Y * Q[1, 0] + v.Z * Q[2, 0]),
                            (float)(v.X * Q[0, 1] + v.Y * Q[1, 1] + v.Z * Q[2, 1]),
                            (float)(v.X * Q[0, 2] + v.Y * Q[1, 2] + v.Z * Q[2, 2]));
        }

        public float Distance(Vec3 v)
        {
            Vec3 diff = this - v;
            return (float)Math.Sqrt((double)(diff.X * diff.X + diff.Y * diff.Y + diff.Z * diff.Z));
        }

        public float DistanceSqr(Vec3 v)
        {
            Vec3 diff = this - v;
            return diff.X * diff.X + diff.Y * diff.Y + diff.Z * diff.Z;
        }

        public float Distance2D(Vec3 v)
        {
            Vec3 diff = this - v;
            return (float)Math.Sqrt((double)(diff.X * diff.X + diff.Y * diff.Y));
        }

        public float Distance2DSqr(Vec3 v)
        {
            Vec3 diff = this - v;
            return diff.X * diff.X + diff.Y * diff.Y;
        }

        public float Length()
        {
            return (float)Math.Sqrt((double)(X * X + Y * Y + Z * Z));
        }

        public float LengthSqr()
        {
            return X * X + Y * Y + Z * Z;
        }

        public float Length2D()
        {
            return (float)Math.Sqrt((double)(X * X + Y * Y));
        }

        public float Length2DSqr()
        {
            return X * X + Y * Y;
        }

        public float Normalize()
        {
            float len = Length();
            X /= len;
            Y /= len;
            Z /= len;
            return len;
        }

        public Vec3 Normalized()
        {
            float len = Length();
            return new Vec3(X / len, Y / len, Z / len);
        }

        public float Normalize2D()
        {
            float len = Length2D();
            X /= len;
            Y /= len;
            Z = 0;
            return len;
        }

        public Vec3 Normalized2D()
        {
            float len = Length2D();
            return new Vec3(X / len, Y / len, 0);
        }

        public Vec3 Cross(Vec3 RHS)
        {
            return new Vec3(Y * RHS.Z - Z * RHS.Y,
                            Z * RHS.X - X * RHS.Z,
                            X * RHS.Y - Y * RHS.X);
        }

        public float Dot(Vec3 RHS)
        {
            return (this.X * RHS.X + this.Y * RHS.Y + this.Z * RHS.Z);
        }

        public float DotNorm(Vec3 RHS)
        {
            return (this.X * RHS.X + this.Y * RHS.Y + this.Z * RHS.Z) / (Length() * RHS.Length());
        }

        public float Dot2D(Vec3 RHS)
        {
            return (this.X * RHS.X + this.Y * RHS.Y);
        }

        public float Dot2DNorm(Vec3 RHS)
        {
            return (this.X * RHS.X + this.Y * RHS.Y) / (Length2D() * RHS.Length2D());
        }

        public Vec3 Blend(Vec3 RHS, float ratio)
        {
            float ratio2 = 1.0f - ratio;
            return new Vec3(X * ratio2 + RHS.X * ratio,
                            Y * ratio2 + RHS.Y * ratio,
                            Z * ratio2 + RHS.Z * ratio);
        }

        public static Vec3 Max(Vec3 LHS, Vec3 RHS)
        {
            return new Vec3(Math.Max(LHS.X, RHS.X), Math.Max(LHS.Y, RHS.Y), Math.Max(LHS.Z, RHS.Z));
        }

        public static Vec3 Max2D(Vec3 LHS, Vec3 RHS)
        {
            return new Vec3(Math.Max(LHS.X, RHS.X), Math.Max(LHS.Y, RHS.Y), 0);
        }

        public static Vec3 Min(Vec3 LHS, Vec3 RHS)
        {
            return new Vec3(Math.Min(LHS.X, RHS.X), Math.Min(LHS.Y, RHS.Y), Math.Min(LHS.Z, RHS.Z));
        }

        public static Vec3 Min2D(Vec3 LHS, Vec3 RHS)
        {
            return new Vec3(Math.Min(LHS.X, RHS.X), Math.Min(LHS.Y, RHS.Y), 0);
        }

        public void Serialize(BinaryWriter w)
        {
            w.Write(X);
            w.Write(Y);
            w.Write(Z);
            w.Write(IsEmpty);
        }

        public void Deserialize(BinaryReader r)
        {
            X = r.ReadSingle();
            Y = r.ReadSingle();
            Z = r.ReadSingle();
            IsEmpty = r.ReadBoolean();
        }

        public static float GetDistanceFromSegment(Vec3 start, Vec3 end, Vec3 point, bool check_2d = false)
        {
            Vec3 temp = ProjectPointOnLine(start, end - start, point);
            float startDist2 = start.DistanceSqr(temp);
            float endDist2 = end.DistanceSqr(temp);
            float len2 = end.DistanceSqr(start);
            if ((startDist2 > len2) || (endDist2 > len2))
            {
                if (startDist2 < endDist2)
                    return check_2d ? start.Distance2D(point) : start.Distance(point);
                else
                    return check_2d ? end.Distance2D(point) : end.Distance(point);
            }
            else
                return check_2d ? temp.Distance2D(point) : temp.Distance(point);
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        public static Vec3 ProjectPointOnLine(Vec3 origin, Vec3 dir, Vec3 point)
        {
            Vec3 result;
            ProjectPointOnLine(origin, dir, point, out result);
            return result;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        public static float ProjectPointOnLine(Vec3 origin, Vec3 dir, Vec3 point, out Vec3 result)
        {
            float dir_len = dir.Length();

            if (dir_len != 0.0f)
                dir *= (1.0f / dir_len);

            Vec3 dir_to_point = point - origin;
            float proj_len = dir_to_point.Dot(dir);

            result = origin + dir * proj_len;

            return proj_len;
        }

        public override string ToString() { return "[" + X.ToString("0.00") + " " + Y.ToString("0.00") + " " + Z.ToString("0.00") + "]"; }
    }
}
