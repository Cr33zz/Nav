using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;

namespace Nav
{
    public class CellsDistancer
    {
        public float GetDistance(int cell_id_1, int cell_id_2)
        {
            if (Distances.TryGetValue(cell_id_1, out var distance_table) && distance_table.TryGetValue(cell_id_2, out var dist))
                return dist;
            return 0;
        }

        public void Connect(int cell_id_1, int cell_id_2, float distance)
        {
            UpdateExploreCellsDistances(cell_id_1, cell_id_2, distance);
        }

        public void Disconnect(int cell_id)
        {
            if (Distances.TryGetValue(cell_id, out var distance_table))
            {
                var distance_table_copy = distance_table.ToDictionary(x => x.Key, x => x.Value);

                // remove cell from all connected distances
                foreach (var dist_entry in distance_table_copy)
                    Distances[dist_entry.Key].Remove(cell_id);

                // remove distances for the cell 
                Distances.Remove(cell_id);
            }
        }

        public List<int> GetConnectedTo(int cell_id)
        {
            List<int> result = new List<int>();

            if (Distances.TryGetValue(cell_id, out var distance_table))
                result = distance_table.Keys.ToList();

            result.Add(cell_id);
            return result;
        }

        public void Clear()
        {
            Distances.Clear();
        }

        private void UpdateExploreCellsDistances(int cell_id_1, int cell_id_2, float distance)
        {
            Distances.TryGetValue(cell_id_1, out var cell_1_distance_table);
            Distances.TryGetValue(cell_id_2, out var cell_2_distance_table);

            //check if we are about to join sub graphs
            bool is_merging_graphs = false;
            if (cell_1_distance_table != null && cell_2_distance_table != null && cell_1_distance_table.Keys.Except(cell_2_distance_table.Keys).Any())
                is_merging_graphs = true;

            // make copies of connections before merging!
            //if (is_merging_graphs)
            {
                cell_1_distance_table = cell_1_distance_table?.ToDictionary(x => x.Key, x => x.Value);
                cell_2_distance_table = cell_2_distance_table?.ToDictionary(x => x.Key, x => x.Value);
            }

            UpdateDist(cell_id_1, cell_id_2, distance);

            // we need to update distance for all nodes connected to cell_id_1, because now these are also connected to cell_id_2
            if (cell_1_distance_table != null)
            {
                foreach (var dist_1_entry in cell_1_distance_table)
                    UpdateDist(dist_1_entry.Key, cell_id_2, dist_1_entry.Value + distance);
            }

            if (is_merging_graphs)
            {
                foreach (var dist_2_entry in cell_2_distance_table)
                {
                    UpdateDist(cell_id_1, dist_2_entry.Key, distance + dist_2_entry.Value); // connect to cell 1 (cell 1 may not be in it is own connections)

                    foreach (var dist_1_entry in cell_1_distance_table)
                        UpdateDist(dist_1_entry.Key, dist_2_entry.Key, dist_1_entry.Value + distance + dist_2_entry.Value);
                }
            }
        }

        private void UpdateDist(int id_1, int id_2, float dist)
        {
            if (id_1 == id_2)
                dist = 0;

            if (!Distances.ContainsKey(id_1))
                Distances.Add(id_1, new Dictionary<int, float>());

            if (!Distances.ContainsKey(id_2))
                Distances.Add(id_2, new Dictionary<int, float>());

            Distances.TryGetValue(id_1, out var distance_table_1);
            Distances.TryGetValue(id_2, out var distance_table_2);

            if (!distance_table_2.TryGetValue(id_1, out var current_dist))
                distance_table_2.Add(id_1, dist);
            else
                distance_table_2[id_1] = Math.Min(current_dist, dist);


            if (!distance_table_1.TryGetValue(id_2, out current_dist))
                distance_table_1.Add(id_2, dist);
            else
                distance_table_1[id_2] = Math.Min(current_dist, dist);
        }

        public void Serialize(BinaryWriter w)
        {
            w.Write(Distances.Count);
            foreach (var pair in Distances)
            {
                w.Write(pair.Key);
                w.Write(pair.Value.Count);
                foreach (var pair2 in pair.Value)
                {
                    w.Write(pair2.Key);
                    w.Write(pair2.Value);
                }
            }
        }

        public void Deserialize(BinaryReader r)
        {
            Distances.Clear();
            int distances_count = r.ReadInt32();
            for (int i = 0; i < distances_count; ++i)
            {
                int key1 = r.ReadInt32();
                var distance_table = new Dictionary<int, float>();
                Distances.Add(key1, distance_table);
                int distances2_count = r.ReadInt32();
                for (int j = 0; j < distances2_count; ++j)
                {
                    distance_table.Add(r.ReadInt32(), r.ReadSingle());
                }
            }
        }

        private readonly Dictionary<int, Dictionary<int, float>> Distances = new Dictionary<int, Dictionary<int, float>>();
    }
}
