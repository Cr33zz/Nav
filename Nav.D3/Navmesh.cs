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
        public Navmesh(MemoryContext engine, bool verbose = false)
            : base(verbose)
        {
            m_MemoryContext = engine;

            if (engine != null)
                Log("[Nav.D3] Navmesh created!");
            else
                Log("[Nav.D3] Navmesh not properly created, engine is null!");
        }

        protected override void Init()
        {
            base.Init();

            using (new Profiler("[Nav.D3] Loading SNO data cache took {t}."))
                LoadSnoCache();

            DangerRegionsEnabled = false;
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

        public bool DangerRegionsEnabled { get; set; }

        public bool IsUpdating
        {
            get { return IsLocalActorValid() && IsObjectManagerOnNewFrame(); }
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

            if (time - m_LastFetchNavDataTime > 500)
            {
                FetchNavData();
                m_LastFetchNavDataTime = time;
            }

            if (time - m_LastFetchDangerRegionsTime > 100)
            {
                FetchDangerRegions();
                m_LastFetchDangerRegionsTime = time;
            }
        }

        private Int64 m_LastFetchNavDataTime = 0;
        private Int64 m_LastFetchDangerRegionsTime = 0;

        private void FetchNavData()
        {
            if (IsUpdating)
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
                Log("[Nav.D3] No updating!");
        }

        private void FetchSceneSnoData()
        {
            List<SceneSnoNavData> new_scene_sno_nav_data = new List<SceneSnoNavData>();

            //using (new Profiler("[Nav.D3.Navigation] Scene sno data aquired [{t}]", 70))
            {
                var sno_scenes = m_MemoryContext.DataSegment.SNOGroupStorage[(int)SNOType.Scene].
                Cast<Enigma.D3.MemoryModel.Assets.SNOGroupStorage<Enigma.D3.Assets.Scene>>().
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

            //using (new Nav.Profiler("[Nav.D3.Navigation] Scene sno data added [{t}]"))
            {
                // add and save new data later to reduce memory reading duration
                foreach (SceneSnoNavData data in new_scene_sno_nav_data)
                {
                    m_SnoCache.Add(data.SceneSnoId, data);
                    data.Save();

                    Log("[Nav.D3] SceneSnoId " + data.SceneSnoId + " added to cache, now containing " + m_SnoCache.Count + " entries!");
                }
            }
        }

        private void FetchSceneData()
        {
            List<SceneData> new_scene_data = new List<SceneData>();

            //using (new Nav.Profiler("[Nav.D3.Navigation] Navmesh data aquired [{t}]", 70))
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

            //using (new Nav.Profiler("[Nav.D3.Navigation] Navmesh data added [{t}]"))
            {
                int grid_cells_added = 0;

                foreach (SceneData scene_data in new_scene_data)
                {
                    SceneSnoNavData sno_nav_data = null;

                    // allow empty grids
                    m_SnoCache.TryGetValue(scene_data.SceneSnoId, out sno_nav_data);
                    
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

        class danger_data
        {
            public danger_data(string name, float range, float move_cost_mult)
            {
                this.name = name;
                this.range = range;
                this.move_cost_mult = move_cost_mult;
            }

            public string name;
            public float range;
            public float move_cost_mult;
        }

        private static readonly List<danger_data> DANGERS = new List<danger_data>() { new danger_data("sporeCloud_emitter", 15, 3),
                                                                                      new danger_data("ChargedBolt_Projectile", 7, 3),
                                                                                      new danger_data("monsterAffix_Desecrator_damage_AOE", 10, 3),
                                                                                      new danger_data("monsterAffix_Plagued", 15, 3),
                                                                                      new danger_data("monsterAffix_Molten_trail", 7, 3),
                                                                                      new danger_data("monsterAffix_Molten_death", 20, 3),
                                                                                      new danger_data("arcaneEnchantedDummy_spawn", 35, 3),
                                                                                      new danger_data("MonsterAffix_ArcaneEnchanted_PetSweep", 35, 3),
                                                                                      new danger_data("monsterAffix_frozen_iceClusters", 20, 3),
                                                                                      new danger_data("MonsterAffix_Orbiter", 7, 3),
                                                                                      new danger_data("MonsterAffix_frozenPulse", 15, 3),
                                                                                      new danger_data("MonsterAffix_CorpseBomber", 15, 3),
                                                                                      new danger_data("MorluSpellcaster_Meteor_Pending", 25, 3),
                                                                                      new danger_data("_Generic_AOE_", 25, 3),
                                                                                      new danger_data("ZoltunKulle_EnergyTwister", 20, 3),
                                                                                      new danger_data("Gluttony_gasCloud", 25, 3),
                                                                                      new danger_data("UberMaghda_Punish_", 20, 3),
                                                                                      new danger_data("Random_FallingRocks", 40, 3),
                                                                                      new danger_data("ringofFire_damageArea", 35, 3),
                                                                                      new danger_data("BoneCage_Proxy", 20, 3),
                                                                                      new danger_data("Brute_leap_telegraph", 20, 3),
                                                                                      new danger_data("creepMobArm", 20, 3),
                                                                                      new danger_data("Morlu_GroundBomb", 40, 3),
                                                                                      new danger_data("grenadier_proj_trail", 40, 3),
                                                                                      new danger_data("orbOfAnnihilation", 40, 3),
                                                                                      //new danger_data("westmarchRanged_projectile", 15, 1.5f),
                                                                                      new danger_data("Corpulent_A", 25, 3) };

        private void FetchDangerRegions(object source = null, ElapsedEventArgs e = null)
        {
            if (DangerRegionsEnabled && IsUpdating)
            {
                try
                {
                    IEnumerable<ACD> objects = m_MemoryContext.DataSegment.ObjectManager.ACDManager.ActorCommonData.Where(x => (x.ActorType == ActorType.ServerProp || x.ActorType == ActorType.Monster || x.ActorType == ActorType.Projectile || x.ActorType == ActorType.CustomBrain) && DANGERS.Exists(d => x.Name.Contains(d.name)));

                    HashSet<region_data> dangers = new HashSet<region_data>();

                    foreach (ACD obj in objects)
                    {
                        danger_data data = DANGERS.Find(d => obj.Name.Contains(d.name));
                        if (data != null)
                        {
                            Vec3 pos = new Vec3(obj.Position.X, obj.Position.Y, obj.Position.Z);
                            AABB area = new AABB(pos - new Vec3(data.range, data.range, pos.Z - 100), pos + new Vec3(data.range, data.range, pos.Z + 100));
                            dangers.Add(new region_data(area, data.move_cost_mult));
                        }
                    }

                    Regions = dangers;
                }
                catch (Exception)
                {
                }
            }
        }

        private bool IsLocalActorValid()
        {
            try
            {
                if (m_MemoryContext == null)
                    return false;

                var valid = m_MemoryContext.DataSegment.ObjectManager.PlayerDataManager[m_MemoryContext.DataSegment.ObjectManager.Player.LocalPlayerIndex].ActorID != -1;

                if (valid)
                {
                    if (!m_IsLocalActorReady)
                        m_IsLocalActorReady = true;

                    return true;
                }
                else
                {
                    if (m_IsLocalActorReady)
                        m_IsLocalActorReady = false;

                    return false;
                }
            }
            catch (Exception)
            {
            }

            return false;
        }

        private bool IsObjectManagerOnNewFrame()
        {
            try
            {
                if (m_MemoryContext == null)
                    return false;

                m_ObjectManager = m_ObjectManager ?? m_MemoryContext.DataSegment.ObjectManager;

                // Don't do anything unless game updated frame.
                int currentFrame = m_ObjectManager.RenderTick;

                if (currentFrame == m_LastFrame)
                    return false;

                if (currentFrame < m_LastFrame)
                {
                    // Lesser frame than before = left game probably.
                    m_LastFrame = currentFrame;
                    return false;
                }

                m_LastFrame = currentFrame;
                return true;
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

            string[] file_paths = Directory.GetFiles(SCENE_SNO_CACHE_DIR);

            foreach (string sno_file in file_paths)
            {
                int scene_sno_id = int.Parse(Path.GetFileName(sno_file));
                m_SnoCache.Add(scene_sno_id, new SceneSnoNavData(scene_sno_id));
            }
        }

        private static string SCENE_SNO_CACHE_DIR = "sno_cache/";
        private static bool USE_SNO_CACHE = true;

        private ReaderWriterLockSlim ProcessedScenesLock = new ReaderWriterLockSlim();
        private ReaderWriterLockSlim D3InputLock = new ReaderWriterLockSlim();

        private Dictionary<int, SceneSnoNavData> m_SnoCache = new Dictionary<int, SceneSnoNavData>();
        private HashSet<SceneData.uid> m_ProcessedSceneId = new HashSet<SceneData.uid>(); // @ProcessedScenesLock
        private List<int> m_AllowedAreasSnoId = new List<int>(); //@D3InputLock
        private List<int> m_AllowedGridCellsId = new List<int>(); //@D3InputLock
        private MemoryContext m_MemoryContext;
        private int m_LastFrame;
        private ObjectManager m_ObjectManager;
        private bool m_IsLocalActorReady = false;
    }
}
