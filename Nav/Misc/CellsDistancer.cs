using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;

namespace Nav
{
    public class CellsDistancer
    {
        public CellsDistancer()
        {
            Distances = new Dictionary<CellIdPair, float>();
        }

        public float GetDistance(int cell_id_1, int cell_id_2)
        {
            float dist = -1;
            Distances.TryGetValue(new CellIdPair(cell_id_1, cell_id_2), out dist);
            return dist;
        }

        public void Connect(int cell_id_1, int cell_id_2, float distance)
        {
            UpdateExploreCellsDistances(cell_id_1, cell_id_2, distance);
            UpdateExploreCellsDistances(cell_id_2, cell_id_1, distance);
        }

        public int GetNearest(int cell_id, int[] allowed_cell_id = null)
        {
            int nearest_id = -1;
            float nearest_dist = float.MaxValue;

            foreach (var pair in Distances.Where(pair => (pair.Key.Id1 == cell_id || pair.Key.Id2 == cell_id)).ToList())
            {
                int other_cell_id = pair.Key.Id1 == cell_id ? pair.Key.Id2 : pair.Key.Id1;

                if (allowed_cell_id != null && !allowed_cell_id.Contains(other_cell_id))
                    continue;
                
                if (pair.Value < nearest_dist)
                {
                    nearest_id = other_cell_id;
                    nearest_dist = pair.Value;
                }
            }

            return nearest_id;
        }

        public void Disconnect(int cell_id)
        {
            foreach (var pair in Distances.Where(pair => (pair.Key.Id1 == cell_id || pair.Key.Id2 == cell_id)).ToList())
                Distances.Remove(pair.Key);
        }

        public List<int> GetConnectedTo(int cell_id)
        {
            List<int> result = new List<int>();
            foreach (var pair in Distances.Where(pair => (pair.Key.Id1 == cell_id || pair.Key.Id2 == cell_id)).ToList())
                result.Add(pair.Key.Id1 == cell_id ? pair.Key.Id2 : pair.Key.Id1);

            result.Add(cell_id);
            return result;
        }

        public void Clear()
        {
            Distances.Clear();
        }

        private void UpdateExploreCellsDistances(int cell_id_1, int cell_id_2, float distance)
        {
            var matching_dists = Distances.Where(p => ((p.Key.Id1 == cell_id_1 || p.Key.Id2 == cell_id_1) && p.Key.Id1 != cell_id_2 && p.Key.Id2 != cell_id_2)).ToList();

            foreach (var pair in matching_dists)
            {
                int other_id = pair.Key.Id1 == cell_id_1 ? pair.Key.Id2 : pair.Key.Id1;

                UpdateDist(cell_id_2, other_id, pair.Value + distance);

                // try connect all new_id related network with other_id
                var new_dists = Distances.Where(p => ((p.Key.Id1 == cell_id_2 || p.Key.Id2 == cell_id_2) && p.Key.Id1 != other_id && p.Key.Id2 != other_id)).ToList();

                foreach (var pair2 in new_dists)
                {
                    int other_id_2 = pair2.Key.Id1 == cell_id_2 ? pair2.Key.Id2 : pair2.Key.Id1;

                    UpdateDist(other_id, other_id_2, pair2.Value + pair.Value + distance);
                }
            }

            UpdateDist(cell_id_1, cell_id_2, distance);
        }

        private void UpdateDist(int id_1, int id_2, float dist)
        {
            CellIdPair p = new CellIdPair(id_1, id_2);
            float existing_dist = 0;

            if (Distances.TryGetValue(p, out existing_dist))
                Distances[p] = Math.Min(existing_dist, dist);
            else
                Distances.Add(p, dist);
        }

        public void Serialize(BinaryWriter w)
        {
            w.Write(Distances.Count);

            foreach (var pair in Distances)
            {
                pair.Key.Serialize(w);
                w.Write(pair.Value);
            }
        }

        public void Deserialize(BinaryReader r)
        {
            Distances.Clear();
            int distances_count = r.ReadInt32();

            for (int i = 0; i < distances_count; ++i)
                Distances.Add(new CellIdPair(r), r.ReadSingle());
        }

        private Dictionary<CellIdPair, float> Distances { get; set; }
    }
}
