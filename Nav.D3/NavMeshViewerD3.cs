using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.IO;
using System.Runtime.InteropServices;
using Enigma.D3.MemoryModel;
using Enigma.D3.MemoryModel.Core;
using Enigma.D3.MemoryModel.Caching;
using Enigma.D3.MemoryModel.SymbolPatching;
using Nav;
using Nav.D3;

namespace Nav.D3
{
    public class NavMeshViewerD3 : NavMeshViewer
    {
        public NavMeshViewerD3(string[] args, MemoryContext memory_context = null, Navmesh navmesh = null, NavigationEngine navigator = null, ExplorationEngine explorer = null)
            : base(args, navmesh, navigator, explorer)
        {
            if (memory_context != null)
                m_MemoryContext = memory_context;
            
            m_AutoClearOnLocationChange = (navmesh == null);
        }

        private MemoryContext CreateMemoryContext()
        {
            var ctx = default(MemoryContext);
            while (true)
            {
                var processes = Process.GetProcessesByName("Diablo III64");
                if (processes.Any())
                {
                    var process = default(Process);
                    if (processes.Length == 1)
                    {
                        process = processes[0];
                    }

                    if (process != null)
                    {
                        ctx = MemoryContext.FromProcess(process);
                        break;
                    }
                }
                else
                {
                    Trace.WriteLine("Could not find any process.");
                }
                Thread.Sleep(1000);
            }
            Trace.WriteLine("Found a process.");

            while (true)
            {
                try
                {
                    SymbolPatcher64.UpdateSymbolTable(ctx);
                    Trace.WriteLine("Symbol table updated.");
                    return ctx;
                }
                catch (Exception exception)
                {
                    Trace.WriteLine($"Could not update symbol table, optimized for patch {SymbolPatcher64.VerifiedBuild}, running {ctx.MainModuleVersion.Revision}: " + exception.Message);
                    Thread.Sleep(5000);
                }
            }
        }

        protected override void CreateNavigation()
        {
            m_MemoryContext = m_MemoryContext ?? CreateMemoryContext();

            if (m_Navmesh == null)
            {
                m_Navmesh = Nav.D3.Navmesh.Create(m_MemoryContext, true);
                m_Navigator = new Nav.NavigationEngine(m_Navmesh);
                m_Navmesh.RegionsMoveCostMode = Nav.Navmesh.RegionsMode.Mult;
                m_Explorer = new Nav.ExploreEngine.Nearest(m_Navmesh, m_Navigator);
                m_Explorer.Enabled = false;
            }
        }

        protected override void LoadDebugConfig()
        {
            base.LoadDebugConfig();

            Nav.D3.Navmesh navmesh_d3 = (m_Navmesh as Nav.D3.Navmesh);

            if (navmesh_d3 == null)
                return;

            Ini.IniFile debug_ini = new Ini.IniFile("./debug.ini");

            string[] allowed_areas_sno_id_str = debug_ini.IniReadValue("Navmesh", "allowed_areas_sno_id").Split(',');
            List<int> allowed_areas_sno_id = new List<int>();
            foreach (string id in allowed_areas_sno_id_str)
            {
                if (id.Length > 0)
                    allowed_areas_sno_id.Add(int.Parse(id));
            }
            navmesh_d3.AllowedAreasSnoId = allowed_areas_sno_id;

            string[] allowed_grid_cells_id_str = debug_ini.IniReadValue("Navmesh", "allowed_grid_cells_id").Split(',');
            List<int> allowed_grid_cells_id = new List<int>();
            foreach (string id in allowed_grid_cells_id_str)
            {
                if (id.Length > 0)
                    allowed_grid_cells_id.Add(int.Parse(id));
            }
            navmesh_d3.AllowedGridCellsId = allowed_grid_cells_id;
        }

        protected override void ModifyRenderMatrix(ref Matrix m)
        {
            if (m_MemoryContext != null)
            {
                // display navmesh in the same manner as Diablo does
                m.Rotate(135);
                Matrix flip_x_m = new Matrix(1, 0, 0, -1, 0, 0);
                m.Multiply(flip_x_m);
            }
            else
                base.ModifyRenderMatrix(ref m);
        }

        protected override void OnRenderData(PaintEventArgs e)
        {
            try
            {
                int location = -1;

                PlayerData local_player_data = m_MemoryContext?.DataSegment.ObjectManager.PlayerDataManager[m_MemoryContext.DataSegment.ObjectManager.Player.LocalPlayerIndex];

                if (local_player_data != null)
                    location = local_player_data.LevelAreaSNO;
                
                if (m_LastLocation != location)
                {
                    if (m_AutoClearOnLocationChange)
                    {
                        m_Navmesh.Clear();
                        LoadDebugConfig();
                    }
                    m_LastLocation = location;
                }
            }
            catch (Exception)
            {
            }

            base.OnRenderData(e);

            RenderHelper.DrawPoint(e.Graphics, Pens.Black, m_RenderCenter, m_Explorer.HintPos);
        }

        protected override void OnRenderUI(PaintEventArgs e)
        {
            if (m_MemoryContext == null)
                e.Graphics.DrawString("Run viewer when Diablo 3 is running!", new Font("Arial", 16), Brushes.Black, Width / 2 - 190, Height / 2 - 50);
            else
            {
                base.OnRenderUI(e);
                e.Graphics.DrawString("LevelAreaSnoId [" + m_LastLocation + "]", STATS_FONT, Brushes.Black, Width - 150, Height - 55);
            }
        }

        protected override void OnRefresh(int interval)
        {
            base.OnRefresh(interval);

            Nav.D3.Navmesh navmesh_d3 = (m_Navmesh as Nav.D3.Navmesh);
            
            if (navmesh_d3.IsPlayerReady())
            {
                try
                {
                    var playerACDID = m_MemoryContext.DataSegment.ObjectManager.PlayerDataManager[m_MemoryContext.DataSegment.ObjectManager.Player.LocalPlayerIndex].ACDID;

                    ACD player = playerACDID != -1 ? m_MemoryContext.DataSegment.ObjectManager.ACDManager.ActorCommonData[(short)playerACDID] : null;

                    if (player == null)
                        return;

                    m_RenderCenter.X = player.Position.X;
                    m_RenderCenter.Y = player.Position.Y;

                    m_Navigator.CurrentPos = new Vec3(player.Position.X, player.Position.Y, player.Position.Z);
                }
                catch (Exception) {}
            }
        }

        protected override void OnKey(KeyEventArgs e)
        {
            base.OnKey(e);

            Nav.D3.Navmesh navmesh_d3 = (m_Navmesh as Nav.D3.Navmesh);

            if (e.Control)
            {
                if (e.KeyCode == System.Windows.Forms.Keys.D3)
                {
                    e.Handled = true;
                }
            }
            else
            {
                if (e.KeyCode == System.Windows.Forms.Keys.A)
                {
                    m_AutoClearOnLocationChange = !m_AutoClearOnLocationChange;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.H)
                {
                    Vec3 result = default(Vec3);
                    if (!m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                                            new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                                            MovementFlag.Walk,
                                            ref result))
                    {
                        result = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    }

                    m_Explorer.HintPos = result;
                    e.Handled = true;
                }
            }
        }

        protected override void AddLegendEntries(ref List<LegendEntry> legend)
        {
            base.AddLegendEntries(ref legend);

            Nav.D3.Navmesh navmesh_d3 = (m_Navmesh as Nav.D3.Navmesh);

            legend.Add(new LegendEntry("A: Toggle auto clear navmesh", true, m_AutoClearOnLocationChange));
        }

        private MemoryContext m_MemoryContext;
        private int m_LastLocation = -1;
        private bool m_AutoClearOnLocationChange = false;
    }
}
