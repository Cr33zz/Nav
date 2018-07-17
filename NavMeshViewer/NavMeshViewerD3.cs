using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using System.Diagnostics;
using System.Globalization;
using System.Threading;
using System.IO;
using System.Runtime.InteropServices;
using Enigma.D3;
using Enigma.D3.Memory;
using Enigma.D3.Helpers;
using Enigma.D3.Collections;
using Enigma.D3.Enums;
using Nav;
using Nav.D3;

namespace NavMeshViewer
{
    public class NavMeshViewerD3 : NavMeshViewer
    {
        public NavMeshViewerD3(string[] args)
            : base(args)
        {
        }

        protected override void CreateNavigation()
        {
            m_Enigma = Engine.Create();

            m_Navmesh = Nav.D3.Navmesh.Create(m_Enigma, true);
            m_Navigator = new Nav.NavigationEngine(m_Navmesh);
            m_Navmesh.RegionsMoveCostMode = Nav.Navmesh.RegionsMode.Mult;
            m_Explorer = new Nav.ExploreEngine.Nearest(m_Navmesh, m_Navigator);
            m_Explorer.Enabled = false;

            m_AutoClearOnLocationChange = true;
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
            if (m_Enigma != null)
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

                if (m_Enigma != null)
                {
                    LevelArea level_area = Engine.Current.LevelArea;
                    if (level_area != null)
                        location = level_area.x044_SnoId;
                }
                //else
                //    m_RenderAxis = false;
                
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
            //if (m_Enigma == null)
            //    e.Graphics.DrawString("Run viewer when Diablo 3 is running!", new Font("Arial", 16), Brushes.Black, Width / 2 - 190, Height / 2 - 50);
            //else
                base.OnRenderUI(e);
        }

        protected override void OnRefresh(int interval)
        {
            base.OnRefresh(interval);

            Nav.D3.Navmesh navmesh_d3 = (m_Navmesh as Nav.D3.Navmesh);

            if (navmesh_d3.IsUpdating)
            {
                Actor local_actor = ActorHelper.GetLocalActor();

                if (local_actor == null)
                    return;

                m_RenderCenter.X = local_actor.x0A8_WorldPosX;
                m_RenderCenter.Y = local_actor.x0AC_WorldPosY;

                m_Navigator.CurrentPos = new Vec3(local_actor.x0A8_WorldPosX, local_actor.x0AC_WorldPosY, local_actor.x0B0_WorldPosZ);
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
                    navmesh_d3.DangerRegionsEnabled = !navmesh_d3.DangerRegionsEnabled;
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
                    Vec3 result = null;
                    m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                                       new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                                       MovementFlag.Walk,
                                       out result);

                    if (result.IsEmpty)
                        result = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);

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
            legend.Add(new LegendEntry("Ctrl+3: Toggle danger regions", true, navmesh_d3.DangerRegionsEnabled));
        }

        private Enigma.D3.Engine m_Enigma = null;
        private int m_LastLocation = -1;
        private bool m_AutoClearOnLocationChange = false;
    }
}
