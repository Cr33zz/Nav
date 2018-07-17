using System;
using System.IO;

namespace Nav
{
    class CellIdPair : IEquatable<CellIdPair>
    {
        public CellIdPair(int id_1, int id_2)
        {
            Id1 = Math.Min(id_1, id_2);
            Id2 = Math.Max(id_1, id_2);
        }

        public CellIdPair(BinaryReader r)
        {
            Deserialize(r);
        }

        public override bool Equals(Object obj)
        {
            if (obj == null)
                return false;

            CellIdPair p = obj as CellIdPair;

            return Equals(p);
        }

        public bool Equals(CellIdPair p)
        {
            return Id1 == p.Id1 && Id2 == p.Id2;
        }

        public override int GetHashCode()
        {
            return (Id1 * Id2).GetHashCode();
        }

        public int Id1 { get; private set; }

        public int Id2 { get; private set; }

        public void Serialize(BinaryWriter w)
        {
            w.Write(Id1);
            w.Write(Id2);
        }

        public void Deserialize(BinaryReader r)
        {
            Id1 = r.ReadInt32();
            Id2 = r.ReadInt32();
        }
    }
}
