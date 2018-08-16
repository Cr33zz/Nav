using System;
using System.Collections.Generic;
using System.Drawing;
using System.Timers;
using System.IO;
using System.Linq;
using System.Threading;
using Enigma.D3.MemoryModel;
using Enigma.D3.Enums;
using Enigma.D3.MemoryModel.Core;
using System.Diagnostics;

namespace Nav.D3
{
    public class Navmesh : Nav.Navmesh
    {
        public Navmesh(MemoryContext memCtx, bool verbose = false)
            : base(verbose)
        {
            m_MemoryContext = memCtx;

            if (memCtx != null)
                Log("[Nav.D3] Navmesh created!");
            else
                Log("[Nav.D3] Navmesh not properly created, engine is null!");
        }

        protected override void Init()
        {
            base.Init();

            using (new Profiler("[Nav.D3] Loading SNO data cache took %t."))
                LoadSnoCache();
        }

        public override void Clear()
        {
            base.Clear();

            using (new WriteLock(D3InputLock))
            {
                m_AllowedAreasSnoId.Clear();
                m_AllowedGridCellsId.Clear();
            }

            using (new WriteLock(ProcessedScenesLock))
            {
                m_ProcessedSceneId.Clear();
            }

            Log("[Nav.D3] Navmesh cleared!");
        }

        protected override void OnSerialize(BinaryWriter w)
        {
            using (new ReadLock(D3InputLock))
            using (new ReadLock(ProcessedScenesLock))
            {
                w.Write(m_AllowedAreasSnoId.Count);
                foreach (int area_sno_id in m_AllowedAreasSnoId)
                    w.Write(area_sno_id);

                w.Write(m_AllowedGridCellsId.Count);
                foreach (int grid_cell_id in m_AllowedGridCellsId)
                    w.Write(grid_cell_id);

                w.Write(m_ProcessedSceneId.Count);
                foreach (SceneData.uid scene_id in m_ProcessedSceneId)
                    scene_id.Serialize(w);

                base.OnSerialize(w);
            }
        }

        protected override void OnDeserialize(BinaryReader r)
        {
            using (new WriteLock(D3InputLock))
            using (new WriteLock(ProcessedScenesLock))
            {
                m_AllowedAreasSnoId.Clear();
                m_AllowedGridCellsId.Clear();

                int area_sno_id_count = r.ReadInt32();

                for (int i = 0; i < area_sno_id_count; ++i)
                {
                    int area_sno_id = r.ReadInt32();
                    m_AllowedAreasSnoId.Add(area_sno_id);
                }

                int grid_cell_id_count = r.ReadInt32();

                for (int i = 0; i < grid_cell_id_count; ++i)
                {
                    int grid_cell_id = r.ReadInt32();
                    m_AllowedGridCellsId.Add(grid_cell_id);
                }
            
                m_ProcessedSceneId.Clear();

                int scene_id_count = r.ReadInt32();

                for (int i = 0; i < scene_id_count; ++i)
                    m_ProcessedSceneId.Add(new SceneData.uid(r));

                base.OnDeserialize(r);
            }            
        }

        public static string SceneSnoCacheDir
        {
            get { return SCENE_SNO_CACHE_DIR; }
        }

        public List<int> AllowedAreasSnoId
        {
            get { using (new ReadLock(D3InputLock)) return new List<int>(m_AllowedAreasSnoId); }
            set { using (new WriteLock(D3InputLock)) m_AllowedAreasSnoId = new List<int>(value); }
        }

        public List<int> AllowedGridCellsId
        {
            get { using (new ReadLock(D3InputLock)) return new List<int>(m_AllowedGridCellsId); }
            set { using (new WriteLock(D3InputLock)) m_AllowedGridCellsId = new List<int>(value); }
        }

        public static Navmesh Create(MemoryContext engine, bool verbose = false)
        {
            return Current = new Navmesh(engine, verbose);
        }

        public static Navmesh Current { get; private set; }

        protected override void OnUpdate(Int64 time)
        {
            base.OnUpdate(time);

            if (time - m_LastFetchNavDataTime > m_FetchNavDataInterval)
            {
                FetchNavData();
                m_LastFetchNavDataTime = time;
            }

            if (m_SnoCacheDirty && time - m_LastSnoCacheSaveTime > m_SnoCacheSaveInterval)
            {
                SaveSnoCache();
                m_LastSnoCacheSaveTime = time;
            }
        }

        private void FetchNavData()
        {
            if (IsPlayerReady())
            {
                try
                {
                    FetchSceneSnoData();
                    using (new WriteLock(ProcessedScenesLock))
                    {
                        FetchSceneData();
                    }
                }
                catch (Exception)
                {
                }
            }
            else
                Log("[Nav.D3] No player!");
        }

        private void FetchSceneSnoData()
        {
            List<SceneSnoNavData> new_scene_sno_nav_data = new List<SceneSnoNavData>();

            //using (new Profiler("[Nav.D3.Navigation] Scene sno data aquired [%t]", 70))
            {
                var sno_scenes = m_MemoryContext.DataSegment.SNOGroupStorage[(int)SNOType.Scene].Cast<Enigma.D3.MemoryModel.Assets.SNOGroupStorage<Enigma.D3.Assets.Scene>>().
                                                                                                 Dereference().
                                                                                                 Container.Where(o => o != null && o.ID != -1 && o.SNOType == SNOType.Scene && !o.PtrValue.IsInvalid).
                                                                                                 Select(o => o.PtrValue.Cast<Enigma.D3.Assets.Scene>().Dereference()).
                                                                                                 ToList();

                foreach (Enigma.D3.Assets.Scene sno_scene in sno_scenes)
                {
                    if (sno_scene == null ||
                        sno_scene.x000_Header.x00_SnoId <= 0 ||
                        m_SnoCache.ContainsKey(sno_scene.x000_Header.x00_SnoId))
                    {
                        continue;
                    }

                    new_scene_sno_nav_data.Add(new SceneSnoNavData(sno_scene));
                }
            }

            //using (new Nav.Profiler("[Nav.D3.Navigation] Scene sno data added [%t]"))
            {
                // add and save new data later to reduce memory reading duration
                foreach (SceneSnoNavData data in new_scene_sno_nav_data)
                {
                    m_SnoCache.Add(data.SceneSnoId, data);
                    m_SnoCacheDirty = true;

                    Log("[Nav.D3] SceneSnoId " + data.SceneSnoId + " added to cache, now containing " + m_SnoCache.Count + " entries!");
                }
            }
        }

        private void FetchSceneData()
        {
            List<SceneData> new_scene_data = new List<SceneData>();

            //using (new Nav.Profiler("[Nav.D3.Navigation] Navmesh data aquired [%t]", 70))
            {
                int scenes_available = 0;

                foreach (var scene in m_MemoryContext.DataSegment.ObjectManager.Scenes)
                {
                    if (scene == null || scene.ID == -1)
                        continue;

                    ++scenes_available;
                    
                    SceneData scene_data = new SceneData(scene);

                    if (m_AllowedAreasSnoId.Count > 0 && !m_AllowedAreasSnoId.Contains(scene_data.AreaSnoId) && !m_AllowedGridCellsId.Contains(scene_data.SceneSnoId))
                        continue;

                    if (m_ProcessedSceneId.Contains(scene_data.SceneUid))
                        continue;

                    new_scene_data.Add(scene_data);
                }
            }

            //using (new Nav.Profiler("[Nav.D3.Navigation] Navmesh data added [%t]"))
            {
                int grid_cells_added = 0;

                foreach (SceneData scene_data in new_scene_data)
                {
                    SceneSnoNavData sno_nav_data = null;

                    m_SnoCache.TryGetValue(scene_data.SceneSnoId, out sno_nav_data);

                    if (sno_nav_data == null)
                    {
                        //wait for navigation data to be fetched before processing this scene
                        continue;
                    }

                    GridCell grid_cell = new GridCell(scene_data.Min, scene_data.Max, scene_data.SceneSnoId, scene_data.AreaSnoId);
                    grid_cell.UserData = scene_data.AreaSnoId;

                    if (sno_nav_data != null)
                    {
                        int cell_id = 0;

                        foreach (Cell cell in sno_nav_data.Cells)
                            grid_cell.Add(new Cell(cell.Min + scene_data.Min, cell.Max + scene_data.Min, cell.Flags, cell_id++));
                    }
                    else
                        Log("[Nav.D3] Couldn't find SNO data for scene " + scene_data.SceneSnoId);

                    if (Add(grid_cell, false))
                        ++grid_cells_added;
                        
                    m_ProcessedSceneId.Add(scene_data.SceneUid);
                }

                if (grid_cells_added > 0)
                {
                    Log("[Nav.D3] " + grid_cells_added + " grid cells added.");

                    NotifyOnNavDataChanged();
                }
                else if (new_scene_data.Count > 0)
                {
                    Log("[Nav.D3] New scene data found but no grid cells added!");
                }
            }
        }

        public bool IsPlayerReady()
        {
            try
            {
                if (m_MemoryContext == null)
                    return false;

                var playerData = m_MemoryContext.DataSegment.ObjectManager.PlayerDataManager[m_MemoryContext.DataSegment.ObjectManager.Player.LocalPlayerIndex];
                return playerData.ActorID != -1 && playerData.ACDID != -1;
            }
            catch (Exception)
            {
            }

            return false;
        }

        private void LoadSnoCache()
        {
            if (!USE_SNO_CACHE)
                return;

            if (!Directory.Exists(SCENE_SNO_CACHE_DIR))
            {
                Directory.CreateDirectory(SCENE_SNO_CACHE_DIR);
                return;
            }

            if (!File.Exists(SCENE_SNO_CACHE_DIR + SCENE_SNO_CACHE_FILE))
                return;

            try
            {
                using (FileStream fs = File.OpenRead(SCENE_SNO_CACHE_DIR + SCENE_SNO_CACHE_FILE))
                using (BinaryReader br = new BinaryReader(fs))
                {
                    int scenes_count = br.ReadInt32();

                    for (int i = 0; i < scenes_count; ++i)
                    {
                        int scene_sno_id = br.ReadInt32();
                        m_SnoCache[scene_sno_id] = new SceneSnoNavData(scene_sno_id, br);
                    }
                }
            }
            catch (Exception)
            {
                Log("Failed to load scene sno cache. Try removing sno cache directory.");
                m_SnoCache.Clear();
            }
        }

        private void SaveSnoCache()
        {
            if (!m_SnoCacheDirty)
                return;

            using (FileStream fs = File.Create(SCENE_SNO_CACHE_DIR + SCENE_SNO_CACHE_FILE))
            using (BinaryWriter bw = new BinaryWriter(fs))
            {
                bw.Write(m_SnoCache.Count);

                foreach (var entry in m_SnoCache)
                {
                    bw.Write(entry.Key);
                    entry.Value.Save(bw);
                }
            }

            m_SnoCacheDirty = false;
        }

        private static string SCENE_SNO_CACHE_DIR = "sno_cache/";
        private static string SCENE_SNO_CACHE_FILE = "scene_sno_cache";
        private static bool USE_SNO_CACHE = true;

        private ReaderWriterLockSlim ProcessedScenesLock = new ReaderWriterLockSlim();
        private ReaderWriterLockSlim D3InputLock = new ReaderWriterLockSlim();

        private Dictionary<int, SceneSnoNavData> m_SnoCache = new Dictionary<int, SceneSnoNavData>();
        private HashSet<SceneData.uid> m_ProcessedSceneId = new HashSet<SceneData.uid>(); // @ProcessedScenesLock
        private List<int> m_AllowedAreasSnoId = new List<int>(); //@D3InputLock
        private List<int> m_AllowedGridCellsId = new List<int>(); //@D3InputLock
        private MemoryContext m_MemoryContext;
        private bool m_SnoCacheDirty = false;
        private Int64 m_LastFetchNavDataTime = 0;
        protected int m_FetchNavDataInterval = 250;
        private Int64 m_LastSnoCacheSaveTime = 0;
        protected int m_SnoCacheSaveInterval = 10000;
    }
}
