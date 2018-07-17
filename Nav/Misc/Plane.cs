using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Nav
{
    public class Plane
    {
        public Plane(Vec3 p1, Vec3 p2, Vec3 p3)
        {
            Vec3 v1 = p2 - p1;
            Vec3 v2 = p3 - p1;
            Vec3 plane_norm = v1.Cross(v2);
            plane_norm.Normalize();

            A = plane_norm.X;
            B = plane_norm.Y;
            C = plane_norm.Z;
            D = -A * p1.X - B * p1.Y - C * p1.Z;
        }

        public Plane(Vec3 p1, Vec3 normal)
        {
            normal.Normalize();
            A = normal.X;
            B = normal.Y;
            C = normal.Z;
            D = -A * p1.X - B * p1.Y - C * p1.Z;
        }

        public Plane(Plane p)
        {
            A = p.A;
            B = p.B;
            C = p.C;
            D = p.D;
        }

        public Vec3 Align(Vec3 point)
        {
            return new Vec3(point.X, point.Y, (-D - A * point.X - B * point.Y) / C);
        }

        public float Distance(Vec3 point)
        {
            return (float)Math.Abs(A * point.X + B * point.Y + C * point.Z + D) / (float)Math.Sqrt(A * A + B * B + C * C);
        }

        public float A;
        public float B;
        public float C;
        public float D;

    }
}
