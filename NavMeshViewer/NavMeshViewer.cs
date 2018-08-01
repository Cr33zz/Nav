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
using Nav;

namespace NavMeshViewer
{
    // Accepted parameters:
    // -load <text-data-file> example: -load nav_new_tristram.txt
    // -deserialize <name-of-serialized-state> example: -deserialize nav_save_20151012
    public partial class NavMeshViewer : Form
    {
        public NavMeshViewer(string[] args)
        {
            InitializeComponent();

            BackColor = Color.LightGray;

            m_Params = new Params(args);

            CreateNavigation();
            LoadDebugConfig();

            if (m_Params.HasParam("load"))
            {
                string file;
                m_Params.GetParam("load", out file);
                LoadData(file);
            }

            if (m_Params.HasParam("deserialize"))
            {
                string file;
                m_Params.GetParam("deserialize", out file);
                m_Navmesh.Deserialize(file);
                m_Navigator.Deserialize(file);
                m_Explorer.Deserialize(file);

                Vec3 initial_pos = m_Navigator.CurrentPos;
                if (initial_pos.IsEmpty)
                    initial_pos = m_Navmesh.GetCenter();
                m_RenderCenter.X = initial_pos.X;
                m_RenderCenter.Y = initial_pos.Y;
            }
        }

        protected virtual void CreateNavigation()
        {
            m_Navmesh = new Nav.Navmesh(false);
            m_Navigator = new Nav.NavigationEngine(m_Navmesh);
            m_Navmesh.RegionsMoveCostMode = Nav.Navmesh.RegionsMode.Mult;
            m_Explorer = new Nav.ExploreEngine.Nearest(m_Navmesh, m_Navigator);
            m_Explorer.Enabled = false;
        }

        protected virtual void LoadDebugConfig()
        {
            Ini.IniFile debug_ini = new Ini.IniFile("./debug.ini");

            m_Navigator.UpdatePathInterval = int.Parse(debug_ini.IniReadValue("Navigator", "update_path_interval"));
            m_Navigator.MovementFlags = (MovementFlag)Enum.Parse(typeof(MovementFlag), debug_ini.IniReadValue("Navigator", "movement_flags"));
            m_Navigator.PathNodesShiftDist = float.Parse(debug_ini.IniReadValue("Navigator", "path_nodes_shift_dist"));
        }

        private void OnLoad(object sender, EventArgs e)
        {
            DoubleBuffered = true;
            this.Paint += new PaintEventHandler(Render);
        }

        private void OnClosing(object sender, FormClosingEventArgs e)
        {
            m_Navmesh.Dispose();
            m_Navigator.Dispose();
            m_Explorer.Dispose();
        }

        protected class LegendEntry
        {
            public LegendEntry(string text, bool toggleable, bool toggled = false) { this.text = text; this.toggleable = toggleable;  this.toggled = toggled; }

            public string text;
            public bool toggleable;
            public bool toggled;
        }

        protected virtual void ModifyRenderMatrix(ref Matrix m)
        {            
        }

        // Everything inside this method is rendered with transformation resulting from ModifyRenderMatrix.
        protected virtual void OnRenderData(PaintEventArgs e)
        {
            try
            {
                int cells_count = 0;
                int grid_cells_count = 0;

                if (m_RenderGrids || m_RenderCells)
                {
                    using (m_Navmesh.AcquireReadDataLock())
                    {
                        List<GridCell> grid_cells = m_Navmesh.dbg_GetGridCells();

                        if (m_RenderGrids)
                        {
                            foreach (Nav.GridCell grid_cell in grid_cells)
                                RenderHelper.Render(grid_cell, m_RenderCenter, e, m_RenderConnections, m_RenderIds);

                            grid_cells_count = grid_cells.Count;
                        }

                        if (m_RenderCells)
                        {
                            float max_move_cost_mult = 1;

                            foreach (Nav.GridCell grid_cell in grid_cells)
                            {
                                foreach (Nav.Cell cell in grid_cell.Cells)
                                {
                                    RenderHelper.Render(cell, m_RenderCenter, e, m_RenderConnections, m_RenderIds, m_LastMaxMoveCostMult);
                                    max_move_cost_mult = Math.Max(max_move_cost_mult, cell.MovementCostMult);
                                }

                                cells_count += grid_cell.Cells.Count;
                            }

                            m_LastMaxMoveCostMult = max_move_cost_mult;
                        }
                    }
                }

                if (m_RenderExploreCells || m_RenderExploreArea)
                {
                    using (m_Explorer.AquireReadDataLock())
                    {
                        List<ExploreCell> explore_cells = m_Explorer.dbg_GetExploreCells();

                        foreach (Nav.ExploreCell explore_cell in explore_cells)
                        {
                            RenderHelper.Render(explore_cell, m_Explorer.ExploreDestPrecision, m_RenderCenter, e, m_RenderConnections, m_RenderIds);

                            if (m_RenderExploreArea)
                                RenderHelper.DrawString(e.Graphics, Brushes.Black, m_RenderCenter, explore_cell.Position, explore_cell.CellsArea().ToString(), 5);
                        }
                    }
                }

                if (m_RenderRegions)
                {
                    var regions = m_Navmesh.Regions;

                    foreach (var region in regions)
                        RenderHelper.DrawRectangle(e.Graphics, Pens.Black, m_RenderCenter, region.area.Min, region.area.Max);

                    //Vec3 safe_point = m_Navigator.GetNearestGridCellOutsideAvoidAreas();

                    //if (!safe_point.IsEmpty)
                    //    RenderHelper.DrawPoint(e.Graphics, Pens.Green, render_center, safe_point);
                }

                if (m_RenderAxis)
                {
                    e.Graphics.DrawString("X", new Font("Arial", 6 / m_RenderScale), Brushes.Black, 25 / m_RenderScale, 0);
                    e.Graphics.DrawLine(RenderHelper.AXIS_PEN, -25 / m_RenderScale, 0, 25 / m_RenderScale, 0);
                    e.Graphics.DrawString("Y", new Font("Arial", 6 / m_RenderScale), Brushes.Black, 0, 25 / m_RenderScale);
                    e.Graphics.DrawLine(RenderHelper.AXIS_PEN, 0, -25 / m_RenderScale, 0, 25 / m_RenderScale);
                }

                if (!m_RenderOriginalPath && m_RenderPath)
                {
                    DestType last_path_dest_type = DestType.None;
                    if (m_Navigator.TryGetPath(ref m_LastPath, ref last_path_dest_type))
                        m_LastPath.Insert(0, m_Navigator.CurrentPos);
                    RenderHelper.DrawLines(e.Graphics, RenderHelper.PATH_PEN, m_RenderCenter, m_LastPath, 1);
                }

                if (m_RenderBacktrackPath)
                {
                    if (m_Navigator.TryGetBackTrackPath(ref m_LastBacktrackPath))
                        m_LastBacktrackPath.Insert(0, m_Navigator.CurrentPos);
                    RenderHelper.DrawLines(e.Graphics, Pens.Blue, m_RenderCenter, m_LastBacktrackPath, 1);
                }

                if (m_RenderPositionsHistory)
                {
                    m_Navigator.TryGetDebugPositionsHistory(ref m_LastPositionsHistory);
                    RenderHelper.DrawLines(e.Graphics, Pens.Green, m_RenderCenter, m_LastPositionsHistory, 1);
                }

                if (!m_Navigator.CurrentPos.IsEmpty)
                    RenderHelper.DrawPoint(e.Graphics, Pens.Blue, m_RenderCenter, m_Navigator.CurrentPos);
                if (!m_Navigator.Destination.IsEmpty)
                    RenderHelper.DrawPoint(e.Graphics, Pens.LightBlue, m_RenderCenter, m_Navigator.Destination);

                {
                    Vec3 curr = m_Navigator.CurrentPos;
                    Vec3 dest = m_Navigator.Destination;

                    if (!curr.IsEmpty && !dest.IsEmpty)
                    {
                        if (m_RenderOriginalPath)
                        {
                            List<Vec3> path = new List<Vec3>();
                            m_Navigator.FindPath(curr, dest, MovementFlag.Walk, ref path, -1, false, false, 0, false, 0, false);
                            path.Insert(0, curr);
                            RenderHelper.DrawLines(e.Graphics, Pens.Black, m_RenderCenter, path, 1);
                        }

                        if (m_RenderRayCast)
                            RenderHelper.DrawLine(e.Graphics, m_Navmesh.RayCast2D(curr, dest, MovementFlag.Walk) ? Pens.Green : Pens.Red, m_RenderCenter, curr, dest);
                    }
                }

                if (m_WaypointsPaths.Count > 0)
                {
                    int waypoint_id = 1;
                    foreach (List<Vec3> p in m_WaypointsPaths)
                    {
                        if (p.Count > 0)
                        {
                            RenderHelper.DrawCircle(e.Graphics, Pens.Black, m_RenderCenter, p[0], 3);
                            RenderHelper.DrawString(e.Graphics, Brushes.Black, m_RenderCenter, p[0], waypoint_id.ToString(), 10);
                        }
                        RenderHelper.DrawLines(e.Graphics, Pens.Red, m_RenderCenter, p, 1);
                        ++waypoint_id;
                    }
                }

                if (m_Bot != null)
                {
                    //if (!m_Bot.Paused && m_CenterOnBot)
                    //    m_RenderCenter = new PointF(m_Bot.Position.X, m_Bot.Position.Y);
                    m_Bot.Render(e.Graphics, m_RenderCenter);
                }
            }
            catch (Exception)
            {
            }
        }

        protected virtual void OnRenderUI(PaintEventArgs e)
        {
            TextRenderer.DrawText(e.Graphics, "L: Toggle render legend", LEGEND_FONT, new Point(10, 10), m_RenderLegend ? Color.White : Color.Black, m_RenderLegend ? Color.Black : Color.Transparent);

            if (m_RenderLegend)
            {
                List<LegendEntry> legend = new List<LegendEntry>();
                AddLegendEntries(ref legend);

                const int Y_offset = 15;

                for (int i = 0; i < legend.Count; ++i)
                {
                    int Y = 10 + (i + 1) * Y_offset;
                    if (legend[i].toggleable)
                        TextRenderer.DrawText(e.Graphics, legend[i].text, LEGEND_FONT, new Point(10, Y), legend[i].toggled ? Color.White : Color.Black, legend[i].toggled ? Color.Black : Color.Transparent);
                    else
                        e.Graphics.DrawString(legend[i].text, LEGEND_FONT, Brushes.Black, 10, Y);
                }
            }

            e.Graphics.DrawString("[" + m_RenderCenter.X + ", " + m_RenderCenter.Y + "]", STATS_FONT, Brushes.Black, 10, Height - 55);
        }

        protected virtual void OnRefresh(int interval)
        {
            if (m_Bot != null)
                m_Bot.Update(interval * 0.001f);
        }

        protected virtual void OnKey(KeyEventArgs e)
        {
            if (e.Control)
            {
                if (e.KeyCode == System.Windows.Forms.Keys.D1)
                {
                    m_RenderPath = !m_RenderPath;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D2)
                {
                    m_Navmesh.RegionsEnabled = !m_Navmesh.RegionsEnabled;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D4)
                {
                    m_RenderPositionsHistory = !m_RenderPositionsHistory;
                    e.Handled = true;
                }
            }
            else
            {
                if (e.KeyCode == System.Windows.Forms.Keys.S)
                {
                    Vec3 result = null;
                    m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                                       new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                                       MovementFlag.Walk,
                                       out result);

                    if (result.IsEmpty)
                        result = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);

                    m_Navigator.CurrentPos = result;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.E)
                {
                    Vec3 result = null;
                    m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                                       new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                                       MovementFlag.Walk,
                                       out result);

                    if (result.IsEmpty)
                        result = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);

                    m_Navigator.Destination = result;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.L)
                {
                    m_RenderLegend = !m_RenderLegend;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D1)
                {
                    m_RenderGrids = !m_RenderGrids;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D2)
                {
                    m_RenderCells = !m_RenderCells;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D3)
                {
                    m_RenderExploreCells = !m_RenderExploreCells;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D4)
                {
                    m_RenderConnections = !m_RenderConnections;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D5)
                {
                    m_RenderIds = !m_RenderIds;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D6)
                {
                    m_RenderAxis = !m_RenderAxis;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D7)
                {
                    //render_explore_dist = !render_explore_dist;
                    m_RenderRegions = !m_RenderRegions;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D8)
                {
                    m_RenderOriginalPath = !m_RenderOriginalPath;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D9)
                {
                    m_RenderRayCast = !m_RenderRayCast;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D0)
                {
                    m_RenderBacktrackPath = !m_RenderBacktrackPath;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.F1)
                {
                    LoadWaypoints(m_LastWaypointsFile);
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.F2)
                {
                    LoadData(m_LastDataFile);
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.F3)
                {
                    m_Navmesh.Dump("nav_dump.txt");
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.F4)
                {
                    m_Navmesh.Clear();
                    m_Navigator.Clear();
                    m_Explorer.Clear();
                    LoadDebugConfig();
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.F5)
                {
                    m_Navmesh.Serialize("nav_save");
                    m_Navigator.Serialize("nav_save");
                    m_Explorer.Serialize("nav_save");
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.F6)
                {
                    m_Navmesh.Deserialize("nav_save");
                    m_Navigator.Deserialize("nav_save");
                    m_Explorer.Deserialize("nav_save");

                    Vec3 initial_pos = m_Navigator.CurrentPos;
                    if (initial_pos.IsEmpty)
                        initial_pos = m_Navmesh.GetCenter();
                    m_RenderCenter.X = initial_pos.X;
                    m_RenderCenter.Y = initial_pos.Y;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.F7)
                {
                    LoadDebugConfig();
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.F10)
                {
                    //Thread t = new Thread(dbg_ContiniousSerialize);
                    //t.Start();

                    Thread t = new Thread(dbg_MoveRegions);
                    t.Start();

                    //m_Navmesh.dbg_GenerateRandomAvoidAreas();

                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.B)
                {
                    Vec3 result = null;
                    m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                                       new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                                       MovementFlag.Walk,
                                       out result);

                    if (m_Bot != null)
                        m_Bot.Dispose();
                    m_Bot = new TestBot(m_Navmesh, m_Navigator, m_Explorer, result, m_Navigator.Destination, true, false);
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.C)
                {
                    m_CenterOnBot = !m_CenterOnBot;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.D)
                {
                    if (m_Bot != null)
                        m_Bot.Destination = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.H)
                {
                    if (m_Explorer != null)
                        m_Explorer.HintPos = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.X)
                {
                    m_Navigator.CurrentPos = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.Space)
                {
                    if (m_Bot != null)
                        m_Bot.Paused = !m_Bot.Paused;
                    e.Handled = true;
                }
                else if (e.KeyCode == System.Windows.Forms.Keys.V)
                {
                    if (m_Bot != null)
                        m_Bot.BackTrace = !m_Bot.BackTrace;
                    e.Handled = true;
                }
            }
        }

        protected virtual void AddLegendEntries(ref List<LegendEntry> legend)
        {
            legend.Add(new LegendEntry("F1: Reload waypoints", false));
            legend.Add(new LegendEntry("F2: Reload nav data", false));
            legend.Add(new LegendEntry("F3: Dump nav data", false));
            legend.Add(new LegendEntry("F4: Clear nav data", false));
            legend.Add(new LegendEntry("F5: Serialize nav data", false));
            legend.Add(new LegendEntry("F6: Deserialize nav data", false));
            legend.Add(new LegendEntry("F10: Activate some test", false));
            legend.Add(new LegendEntry("1: Toggle render grid cells", true, m_RenderGrids));
            legend.Add(new LegendEntry("2: Toggle render cells", true, m_RenderCells));
            legend.Add(new LegendEntry("3: Toggle render explore cells", true, m_RenderExploreCells));
            legend.Add(new LegendEntry("4: Toggle render connections", true, m_RenderConnections));
            legend.Add(new LegendEntry("5: Toggle render IDs", true, m_RenderIds));
            legend.Add(new LegendEntry("6: Toggle render axis", true, m_RenderAxis));
            legend.Add(new LegendEntry("7: Toggle render regions", true, m_RenderRegions));
            legend.Add(new LegendEntry("8: Toggle render original path", true, m_RenderOriginalPath));
            legend.Add(new LegendEntry("9: Toggle render ray cast", true, m_RenderRayCast));
            legend.Add(new LegendEntry("0: Toggle render back track path", true, m_RenderBacktrackPath));
            legend.Add(new LegendEntry("S: Set current pos", false));
            legend.Add(new LegendEntry("E: Set destination pos", false));
            legend.Add(new LegendEntry("B: Run bot", false));            
            legend.Add(new LegendEntry("F7: Reload debug.ini", false));
            legend.Add(new LegendEntry("Ctrl+1: Toggle render path", true, m_RenderPath));
            legend.Add(new LegendEntry("Ctrl+2: Toggle regions", true, m_Navmesh.RegionsEnabled));
            legend.Add(new LegendEntry("Ctrl+4: Toggle render positions history", true, m_RenderPositionsHistory));
        }

        private void Render(object sender, PaintEventArgs e)
        {
            Matrix m = new Matrix();
            m.Scale(m_RenderScale, m_RenderScale);
            m.Translate((Width - 16) / (2 * m_RenderScale), (Height - 30) / (2 * m_RenderScale));

            ModifyRenderMatrix(ref m);

            e.Graphics.Transform = m;
            e.Graphics.CompositingQuality = CompositingQuality.GammaCorrected;
            e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            OnRenderData(e);

            e.Graphics.ResetTransform();

            OnRenderUI(e);
        }

        private void LoadWaypoints(string filename)
        {
            m_LastWaypointsFile = filename;
            m_WaypointsPaths.Clear();

            if (!File.Exists(filename))
                return;

            using (var reader = File.OpenText(filename))
            {
                string line;
                Vec3 last_wp = Vec3.Empty;

                while ((line = reader.ReadLine()) != null)
                {
                    String[] coords = line.Split(';');

                    if (coords.Length >= 3)
                    {
                        Vec3 wp = new Vec3(float.Parse(coords[0], CultureInfo.InvariantCulture),
                                           float.Parse(coords[1], CultureInfo.InvariantCulture),
                                           float.Parse(coords[2], CultureInfo.InvariantCulture));
                        
                        if (!last_wp.IsEmpty)
                        {
                            List<Vec3> path = new List<Vec3>();
                            m_Navigator.FindPath(last_wp, wp, MovementFlag.Walk, ref path, -1, true, true);
                            m_WaypointsPaths.Add(path);
                        }

                        last_wp = wp;
                    }
                }
            }
        }

        private void LoadData(string filename, bool clear = true)
        {
            m_LastDataFile = filename;

            if (m_Navmesh.Load(filename, clear, true))
            {
                Vec3 initial_pos = m_Navigator.CurrentPos;
                if (initial_pos.IsEmpty)
                    initial_pos = m_Navmesh.GetCenter();

                m_RenderCenter.X = initial_pos.X;
                m_RenderCenter.Y = initial_pos.Y;
            }
        }

        private void dbg_ContiniousSerialize()
        {
            while (true)
            {
                m_Navmesh.Serialize("test.dat");
                Thread.Sleep(500);
            }
        }

        private void dbg_MoveRegions()
        {
            Random rng = new Random();
            HashSet<region_data> regions = new HashSet<region_data>();

            for (int i = 0; i < 80; ++i)
            {
                Vec3 pos = m_Navmesh.GetRandomPos();
                float size = 20 + (float)rng.NextDouble() * 10;
                regions.Add(new region_data(new AABB(pos - new Vec3(size * 0.5f, size * 0.5f, 0), pos + new Vec3(size * 0.5f, size * 0.5f, 0)), 2));
            }

            const int dt = 100;

            while (true)
            {

                foreach (var region in regions)
                {
                    Vec3 dir = new Vec3((float)rng.NextDouble() * 2 - 1, (float)rng.NextDouble() * 2 - 1, 0);

                    region.area.Translate(dir * 30 * ((float)dt / 1000));
                }

                m_Navmesh.Regions = regions;

                Thread.Sleep(dt);
            }
        }

        private void refresh_timer_Tick(object sender, EventArgs e)
        {
            OnRefresh(refresh_timer.Interval);

            Refresh();
        }

        private void NavMeshViewer_MouseMove(object sender, MouseEventArgs e)
        {
            if (!Control.MouseButtons.HasFlag(MouseButtons.Left))
                return;

            if (!m_LastDragMousePos.IsEmpty)
            {
                m_RenderCenter.X += m_LastDragMousePos.X - e.X;
                m_RenderCenter.Y += m_LastDragMousePos.Y - e.Y;
            }

            m_LastDragMousePos = new PointF(e.X, e.Y);
        }

        private void NavMeshViewer_MouseUp(object sender, MouseEventArgs e)
        {
            m_LastDragMousePos = PointF.Empty;
        }

        private void NavMeshViewer_MouseWheel(object sender, MouseEventArgs e)
        {
            m_RenderScale += e.Delta * 0.002f;

            m_RenderScale = Math.Max(0.01f, Math.Min(100.0f, m_RenderScale));
        }

        private void NavMeshViewer_KeyPress(object sender, KeyEventArgs e)
        {
            OnKey(e);            
        }

        protected Params m_Params;
        protected Nav.Navmesh m_Navmesh = null;
        protected Nav.NavigationEngine m_Navigator = null;
        protected Nav.ExplorationEngine m_Explorer = null;
        protected PointF m_RenderCenter = new PointF(200, 350);
        protected float m_RenderScale = 1.5f;//0.75f;
        private bool m_RenderIds = false;
        protected bool m_RenderAxis = true;
        private bool m_RenderConnections = false;
        private bool m_RenderOriginalPath = false;
        private bool m_RenderBacktrackPath = false;
        private bool m_RenderPositionsHistory = false;
        private bool m_RenderRayCast = false;
        private bool m_RenderExploreCells = false;
        private bool m_RenderRegions = false;
        private bool m_RenderExploreArea = false;
        private bool m_RenderCells = true;
        private bool m_RenderLegend = true;
        private bool m_RenderGrids = false;
        private bool m_RenderPath = true;
        private bool m_CenterOnBot = true;
        private PointF m_LastDragMousePos = PointF.Empty;
        private TestBot m_Bot = null;
        private string m_LastWaypointsFile;
        private string m_LastDataFile;
        private List<List<Vec3>> m_WaypointsPaths = new List<List<Vec3>>();
        private List<Vec3> m_LastPath = new List<Vec3>();
        private List<Vec3> m_LastBacktrackPath = new List<Vec3>();
        private List<Vec3> m_LastPositionsHistory = new List<Vec3>();
        private List<Vec3> m_LastExplorePath = new List<Vec3>();
        private float m_LastMaxMoveCostMult = 1;

        public static readonly Font LEGEND_FONT = new Font("Arial", 8, FontStyle.Bold);
        public static readonly Font STATS_FONT = new Font("Arial", 8);
    }

    class RenderHelper
    {
        public static float GetProportional(float value, float min, float max, float new_min, float new_max)
        {
            if (min == max)
                return new_max;

            float value_progress = (value - min) / (max - min);
            return new_min + (new_max - new_min) * value_progress;
        }

        public static void Render(Nav.Cell cell, PointF trans, PaintEventArgs e, bool draw_connections, bool draw_id, float max_move_cost_mult)
        {
            if (cell.Disabled)
                return;

            DrawRectangle(e.Graphics, cell.Replacement ? REPLACEMENT_CELL_BORDER_PEN : CELL_BORDER_PEN, trans, cell.Min, cell.Max);

            Color cell_color = Color.White;
            int move_cost_level = 255;

            if (cell.MovementCostMult > 1)
            {
                move_cost_level = 255 - (int)Math.Min(GetProportional(cell.MovementCostMult, 1, 100, 20, 255), 255);
                cell_color = Color.FromArgb(255, 255, move_cost_level, move_cost_level);
            }
            else if (cell.MovementCostMult < 1)
            {
                move_cost_level = 255 - (int)Math.Min(GetProportional(cell.MovementCostMult, 0, 1, 20, 255), 255);
                cell_color = Color.FromArgb(255, move_cost_level, 255, move_cost_level);
            }

            FillRectangle(e.Graphics, cell.Flags == MovementFlag.Fly ? Brushes.Gray : new SolidBrush(cell_color), trans, cell.Min, cell.Max);
            
            if (draw_connections)
            {
                foreach (Nav.Cell.Neighbour neighbour in cell.Neighbours)
                    DrawLine(e.Graphics, CELL_CONNECTION_PEN, trans, cell.Center, neighbour.border_point);
            }

            if (draw_id)
                DrawString(e.Graphics, Brushes.Black, trans, cell.Min, cell.Id.ToString(), 2);
        }

        public static void Render(Nav.GridCell cell, PointF trans, PaintEventArgs e, bool draw_connections, bool draw_id)
        {
            DrawRectangle(e.Graphics, Pens.Black, trans, cell.Min, cell.Max);

            if (draw_connections)
                foreach (Nav.Cell.Neighbour neighbour in cell.Neighbours)
                    DrawLine(e.Graphics, GRID_CELL_CONNECTION_PEN, trans, cell.Center, neighbour.cell.Center);

            if (draw_id)
                DrawString(e.Graphics, Brushes.Black, trans, cell.Min, cell.GlobalId.ToString() + " (" + cell.Id.ToString() + ")", 15);
        }

        public static void Render(Nav.ExploreCell cell, float radius, PointF trans, PaintEventArgs e, bool draw_connections, bool draw_id)
        {
            DrawRectangle(e.Graphics, Pens.Magenta, trans, cell.Min, cell.Max);

            //DrawString(e.Graphics, Brushes.Black, trans, cell.Position, Math.Round(cell.CellsArea()).ToString(), 14);
            
            if (cell.Explored)
            {
                //DrawLine(e.Graphics, explored_pen, trans, cell.Min, cell.Max);
                //DrawLine(e.Graphics, explored_pen, trans, new Vec3(cell.Min.X, cell.Max.Y), new Vec3(cell.Max.X, cell.Min.Y));
                FillRectangle(e.Graphics, explored_brush, trans, cell.Min, cell.Max);
            }
            else
            {
                //DrawCircle(e.Graphics, Pens.Red, trans, cell.Position, radius);
                //DrawString(e.Graphics, Brushes.Black, trans, cell.Position, cell.UserData.ToString(), 10);

                if (draw_connections)
                    foreach (Nav.Cell.Neighbour neighbour in cell.Neighbours)
                    {
                        ExploreCell neighbour_cell = (ExploreCell)neighbour.cell;

                        DrawLine(e.Graphics, EXPLORE_CELL_CONNECTION_PEN, trans, cell.Position, neighbour_cell.Position);
                    }

                if (draw_id)
                    DrawString(e.Graphics, Brushes.Black, trans, cell.Position, cell.Id.ToString(), 10);
            }
        }

        public static void Render(Nav.Navmesh navmesh, Nav.ExploreCell cell, List<Nav.ExploreCell> all_cells, PointF trans, PaintEventArgs e, bool draw_id)
        {
            foreach (ExploreCell other_cell in all_cells)
            {
                if (cell.Id == other_cell.Id)
                    continue;

                DrawLine(e.Graphics, Pens.Gray, trans, cell.Position, other_cell.Position);
                //DrawString(e.Graphics, Brushes.Black, trans, (cell.Position + other_cell.Position) * 0.5f, Math.Round(navmesh.Explorator.ExploreDistance(cell, other_cell)).ToString(), 8);
            }
        }

        public static void DrawString(Graphics g, Brush b, PointF trans, Vec3 pos, string text, int font_size)
        {
            g.DrawString(text, new Font("Arial", font_size), b, pos.X - trans.X, pos.Y - trans.Y);
        }

        public static void DrawRectangle(Graphics g, Pen p, PointF trans, Vec3 min, Vec3 max)
        {
            g.DrawRectangle(p, min.X - trans.X, min.Y - trans.Y, max.X - min.X, max.Y - min.Y);
        }

        public static void FillRectangle(Graphics g, Brush b, PointF trans, Vec3 min, Vec3 max)
        {
            g.FillRectangle(b, min.X - trans.X, min.Y - trans.Y, max.X - min.X, max.Y - min.Y);
        }

        public static void DrawLine(Graphics g, Pen p, PointF trans, Vec3 start, Vec3 end)
        {
            g.DrawLine(p, start.X - trans.X, start.Y - trans.Y, end.X - trans.X, end.Y - trans.Y);
        }

        public static void DrawPoint(Graphics g, Pen p, PointF trans, Vec3 pos)
        {
            DrawCircle(g, p, trans, pos, 1);
        }

        public static void DrawCircle(Graphics g, Pen p, PointF trans, Vec3 pos, float radius)
        {
            g.DrawEllipse(p, pos.X - trans.X - radius, pos.Y - trans.Y - radius, 2 * radius, 2 * radius);
        }

        public static void DrawLines(Graphics g, Pen p, PointF trans, List<Vec3> points, float point_radius)
        {
            if (points.Count < 2)
                return;

            if (point_radius > 0)
                DrawCircle(g, Pens.Black, trans, points[0], point_radius);

            for (int i = 1; i < points.Count; ++i)
            {
                DrawLine(g, p, trans, points[i - 1], points[i]);

                if (point_radius > 0)
                    DrawCircle(g, Pens.Black, trans, points[i], point_radius);
            }
        }

        private static Pen EXPLORE_CELL_CONNECTION_PEN = new Pen(Color.FromArgb(255, 50, 50, 50), 3f);
        private static Pen GRID_CELL_CONNECTION_PEN = new Pen(Color.FromArgb(255, 50, 50, 50), 4);
        private static Pen CELL_CONNECTION_PEN = new Pen(Color.Black, 0.3f);
        private static Pen CELL_BORDER_PEN = new Pen(Color.Blue, 0.3f);
        private static Pen REPLACEMENT_CELL_BORDER_PEN = new Pen(Color.LightGray, 0.3f);
        public static readonly Pen AXIS_PEN = new Pen(Color.SaddleBrown, 0.3f);
        public static readonly Pen EXPLORE_PATH_PEN = new Pen(Color.Black, 5);
        public static readonly Pen PATH_PEN = new Pen(Color.Black, 1.5f);
        private static Brush explored_brush = new SolidBrush(Color.FromArgb(128, 50, 50, 50));
    }
}
