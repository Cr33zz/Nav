using System;
using System.IO;
using System.Collections.Generic;

namespace Nav.D3
{
    class SceneData
    {
        public SceneData(Enigma.D3.Scene scene)
        {
            scene_sno_id = scene.x0E8_SceneSnoId;
            area_sno_id = scene.x018_LevelAreaSnoId;

            min = new Vec3(scene.x0FC_MeshMinX, scene.x100_MeshMinY, scene.x104_MeshMinZ);
            max = new Vec3(scene.x174_MeshMaxX, scene.x178_MeshMaxY, scene.x104_MeshMinZ); //there is no max z, so consider all grid cells flat
        }

        public class uid : IEquatable<uid>
        {
            public uid(Vec3 min, int sno_id) { this.min = min; this.sno_id = sno_id; }
            public uid(BinaryReader r) { Deserialize(r); }

            public Vec3 min;
            public int sno_id;

            public bool Equals(uid s)
            {
                if (s == null)
                    return false;

                return min.Equals(s.min) && sno_id.Equals(s.sno_id);
            }

            public override bool Equals(Object obj)
            {
                if (obj == null)
                    return false;

                uid u = obj as uid;

                return Equals(u);
            }

            public override int GetHashCode()
            {
                return sno_id;
            }

            public void Serialize(BinaryWriter w)
            {
                min.Serialize(w);
                w.Write(sno_id);
            }

            public void Deserialize(BinaryReader r)
            {
                min = new Vec3(r);
                sno_id = r.ReadInt32();
            }
        }

        public uid SceneUid
        {
            get { return new uid(min, scene_sno_id); }
        }

        public int SceneSnoId
        {
            get { return scene_sno_id; }
        }

        public int AreaSnoId
        {
            get { return area_sno_id; }
        }

        public Vec3 Min
        {
            get { return min; }
        }

        public Vec3 Max
        {
            get { return max; }
        }

        private int scene_sno_id;
        private int area_sno_id;
        private Vec3 min;
        private Vec3 max;
    }
}
