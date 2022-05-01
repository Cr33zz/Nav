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

namespace Nav
{
    // Accepted parameters:
    // -load <text-data-file> example: -load nav_new_tristram.txt
    // -deserialize <name-of-serialized-state> example: -deserialize nav_save_20151012
    // -start X Y Z example: -start 5 23 3.5
    // -end X Y Z example: -end 5 23 3.5
    public partial class NavMeshViewer : Form
    {
        public NavMeshViewer(string[] args, Navmesh navmesh = null, NavigationEngine navigator = null, ExplorationEngine explorer = null)
        {
            Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

            BackColor = Color.LightGray;

            m_Params = new Params(args);

            m_Navmesh = navmesh;
            m_Navigator = navigator;
            m_Explorer = explorer;

            CreateNavigation();
            LoadDebugConfig();

            if (m_Params.HasParam("load"))
            {
                m_Params.GetParam("load", out string file);
                LoadData(file);
            }

            if (m_Params.HasParam("load_waypoints"))
            {
                m_Params.GetParam("load_waypoints", out string file);
                LoadWaypoints(file);
            }

            if (m_Params.HasParam("deserialize"))
            {
                m_Params.GetParam("deserialize", out string file);
                var filename = Path.GetFileNameWithoutExtension(file);
                file = Path.Combine(Path.GetDirectoryName(file), filename);
                m_Navmesh.Deserialize(file);
                m_Navigator.Deserialize(file);
                m_Explorer.Deserialize(file);

                OnDeserialize(file);

                Vec3 initial_pos = m_Navigator.CurrentPos;
                if (initial_pos.IsZero())
                    initial_pos = m_Navmesh.GetCenter();
                m_RenderCenter.X = initial_pos.X;
                m_RenderCenter.Y = initial_pos.Y;
            }

            if (m_Params.HasParam("start"))
            {
                m_Params.GetParam("start", out string str);
                m_Navigator.CurrentPos = new Vec3(str);
            }

            if (m_Params.HasParam("end"))
            {
                m_Params.GetParam("end", out string str);
                m_Navigator.Destination = new destination(new Vec3(str));
            }

            InitializeComponents();
        }

        protected virtual void OnDeserialize(string file)
        {

        }

        protected virtual void CreateNavigation()
        {
            m_Navmesh = new Navmesh();
            m_Navigator = new NavigationEngine(m_Navmesh, new SimplePathFollow());
            m_Explorer = new ExploreEngine.Nearest(m_Navmesh, m_Navigator);
            m_Explorer.Enabled = false;
        }

        protected virtual void LoadDebugConfig()
        {
            if (!File.Exists(DEBUG_CONFIG_FILE))
                return;

            Ini.IniFile debug_ini = new Ini.IniFile(DEBUG_CONFIG_FILE);

            m_Navigator.UpdatePathInterval = int.Parse(debug_ini.IniReadValue("Navigator", "update_path_interval"));
            m_Navigator.MovementFlags = (MovementFlag)Enum.Parse(typeof(MovementFlag), debug_ini.IniReadValue("Navigator", "movement_flags"));
            m_Navigator.PathNodesShiftDist = float.Parse(debug_ini.IniReadValue("Navigator", "path_nodes_shift_dist"));
            m_Navigator.DefaultPrecision = float.Parse(debug_ini.IniReadValue("Navigator", "default_precision"));
            m_Navigator.GridDestPrecision = float.Parse(debug_ini.IniReadValue("Navigator", "grid_dest_precision"));
            m_Navigator.PathSmoothingPrecision = float.Parse(debug_ini.IniReadValue("Navigator", "path_smoothing_precision"));
            m_Navigator.KeepFromEdgePrecision = float.Parse(debug_ini.IniReadValue("Navigator", "keep_from_edge_precision"));
            m_Navigator.CurrentPosDiffRecalcThreshold = float.Parse(debug_ini.IniReadValue("Navigator", "current_pos_diff_recalc_threshold"));

            //m_Explorer.ChangeExploreCellSize(int.Parse(debug_ini.IniReadValue("Explorer", "explore_cell_size")));
            m_Explorer.Enabled = bool.Parse(debug_ini.IniReadValue("Explorer", "enabled"));
            m_Explorer.ExploreDestPrecision= float.Parse(debug_ini.IniReadValue("Explorer", "explore_dest_precision"));

            m_BotSpeed = float.Parse(debug_ini.IniReadValue("Bot", "speed"));
        }

        private void OnLoad(object sender, EventArgs e)
        {
            DoubleBuffered = true;
            Paint += new PaintEventHandler(Render);
        }

        private void OnClosing(object sender, FormClosingEventArgs e)
        {
            m_Navmesh.Dispose();
            m_Navigator.Dispose();
            m_Explorer.Dispose();
        }

        protected class LegendEntry
        {
            public LegendEntry(string text, bool toggleable, bool toggled = false)
            {
                Text = text;
                Toggleable = toggleable;
                Toggled = toggled;
            }

            public string Text;
            public bool Toggleable;
            public bool Toggled;
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
                        var grid_cells = m_Navmesh.dbg_GetGridCells();

                        if (m_RenderGrids)
                        {
                            foreach (Nav.GridCell grid_cell in grid_cells)
                                RenderHelper.Render(grid_cell, m_RenderCenter, e, m_RenderConnections, m_RenderIds);

                            grid_cells_count = grid_cells.Count;
                        }

                        if (m_RenderCells)
                        {
                            foreach (Nav.GridCell grid_cell in grid_cells)
                            {
                                foreach (Nav.Cell cell in grid_cell.GetCells())
                                {
                                    if (m_RenderRegionsMode == RegionsRenderMode.BlockerReplacement && !cell.BlockerReplacement)
                                        continue;

                                    RenderHelper.Render(cell, m_RenderCenter, e, m_RenderConnections, m_RenderIds, m_RenderRegionsMode == RegionsRenderMode.MoveCostMult, m_RenderRegionsMode == RegionsRenderMode.Threat);
                                }

                                cells_count += grid_cell.GetCellsCount();
                            }
                        }
                    }
                }

                if (m_RenderExploreCells || m_RenderExploreArea)
                {
                    using (m_Explorer.AquireReadDataLock())
                    {
                        var explore_cells = m_Explorer.dbg_GetExploreCells();

                        var contraints = m_Explorer.ExploreConstraints;

                        if (contraints != null)
                        {
                            var constraintPen = new Pen(Color.Magenta, 8);

                            foreach (var contraint in contraints)
                                RenderHelper.DrawRectangle(e.Graphics, constraintPen, m_RenderCenter, contraint.Min, contraint.Max);
                        }

                        foreach (Nav.ExploreCell explore_cell in explore_cells)
                        {
                            RenderHelper.Render(explore_cell, m_Explorer.ExploreDestPrecision, m_RenderCenter, e, m_RenderConnections, m_RenderIds);

                            if (m_RenderExploreArea)
                                RenderHelper.DrawString(e.Graphics, explore_cell.Small ? Brushes.DarkRed : Brushes.Black, m_RenderCenter, explore_cell.Position, explore_cell.CellsArea.ToString(), 12);
                        }
                    }
                }

                if (m_RenderPatches)
                {
                    using (new ReadLock(m_Navmesh.PatchesDataLock))
                    {
                        int id = 0;
                        foreach (var patch in m_Navmesh.m_CellsPatches)
                        {
                            var rng = new Random(id++);
                            Color c = Color.FromArgb(rng.Next(255), rng.Next(255), rng.Next(255));

                            foreach (var cell in patch.Cells)
                            {
                                RenderHelper.Render(cell, m_RenderCenter, e, draw_connections: false, draw_id: false, render_move_cost_mult: false, render_threat: false, render_outline: false, render_disabled: true, force_color: c);
                            }
                        }
                    }
                }

                if (m_RenderRegionsMode == RegionsRenderMode.Outline)
                {
                    var regions = m_Navmesh.Regions;

                    foreach (var region in regions)
                        RenderHelper.DrawRectangle(e.Graphics, Pens.Black, m_RenderCenter, region.Area.Min, region.Area.Max);

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
                    if (m_Navigator.TryGetPath(ref m_LastPath, out var last_path_dest))
                        m_LastPath.Insert(0, m_Navigator.CurrentPos);
                    RenderHelper.DrawLines(e.Graphics, RenderHelper.PATH_PEN, m_RenderCenter, m_LastPath, 1, true);

                    if (!m_Navigator.m_Path.path_recalc_trigger_pos.IsZero())
                        RenderHelper.DrawCircle(e.Graphics, Pens.DarkRed, m_RenderCenter, m_Navigator.m_Path.path_recalc_trigger_pos, m_Navigator.m_Path.path_recalc_trigger_precision);
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
                    RenderHelper.DrawLines(e.Graphics, Pens.Green, m_RenderCenter, m_LastPositionsHistory, 1, true);
                }

                Vec3 curr = m_Navigator.CurrentPos;
                Vec3 dest = m_Navigator.Destination.pos;
                
                if (!curr.IsZero())
                    RenderHelper.DrawPoint(e.Graphics, Pens.Blue, m_RenderCenter, curr);

                if (!dest.IsZero())
                    RenderHelper.DrawPoint(e.Graphics, Pens.LightBlue, m_RenderCenter, dest);

                if (!curr.IsZero() && !dest.IsZero())
                {
                    if (m_RenderOriginalPath)
                    {
                        List<Vec3> path = new List<Vec3>();
                        m_Navigator.FindPath(curr, dest, ref path, out var timed_out, false, false, 0, 0, smoothen_distance: 0);
                        path.Insert(0, curr);
                        RenderHelper.DrawLines(e.Graphics, Pens.Black, m_RenderCenter, path, 1);
                    }

                    if (m_RenderRoughPath)
                    {
                        List<Vec3> rought_path = new List<Vec3>();

                        if (m_Navigator.m_RoughtPathEstimator != null)
                            m_Navigator.m_RoughtPathEstimator.FindRoughPath(curr, dest, ref rought_path, out var unused);

                        rought_path.Insert(0, curr);
                        RenderHelper.DrawLines(e.Graphics, RenderHelper.EXPLORE_PATH_PEN, m_RenderCenter, rought_path, 1, true);
                    }

                    if (m_RenderRayCast)
                    {
                        var result = m_Navmesh.RayCast2D(curr, dest, MovementFlag.Walk);
                        RenderHelper.DrawLine(e.Graphics, result ? Pens.Green : Pens.Red, m_RenderCenter, curr, result.End);
                    }

                    if (m_RenderConnected)
                    {
                        bool connected = m_Navmesh.AreConnected(curr, dest, MovementFlag.Walk, 0, 0, out var curr_on_nav, out var dest_on_nav);
                        RenderHelper.DrawLine(e.Graphics, connected ? Pens.Green : Pens.Red, m_RenderCenter, curr, dest);
                        RenderHelper.DrawLine(e.Graphics, Pens.Lavender, m_RenderCenter, curr, curr_on_nav);
                        RenderHelper.DrawLine(e.Graphics, Pens.Lavender, m_RenderCenter, dest, dest_on_nav);
                        RenderHelper.DrawString(e.Graphics, connected ? Brushes.Green : Brushes.Red, m_RenderCenter, dest, connected ? "connected" : "not connected", 4);
                    }
                }

                if (!curr.IsZero() && dest.IsZero())
                {
                    if (m_RenderAvoidancePath)
                    {
                        List<Vec3> path = new List<Vec3>();
                        m_Navigator.FindAvoidancePath(curr, 0, MovementFlag.Walk, ref path, Vec3.ZERO, false, 0, float.MaxValue);
                        path.Insert(0, curr);
                        RenderHelper.DrawLines(e.Graphics, Pens.Black, m_RenderCenter, path, 1);
                    }
                }

                var waypoints = m_Navigator.Waypoints;
                if (waypoints.Count > 0)
                {
                    RenderHelper.DrawLines(e.Graphics, Pens.Red, m_RenderCenter, waypoints, 1);
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
                    if (legend[i].Toggleable)
                        TextRenderer.DrawText(e.Graphics, legend[i].Text, LEGEND_FONT, new Point(10, Y), legend[i].Toggled ? Color.White : Color.Black, legend[i].Toggled ? Color.Black : Color.Transparent);
                    else
                        e.Graphics.DrawString(legend[i].Text, LEGEND_FONT, Brushes.Black, 10, Y);
                }
            }

            e.Graphics.DrawString($"Explored: {m_Explorer.GetExploredPercent()}%", STATS_FONT, Brushes.Black, 10, Height - 100);
            e.Graphics.DrawString($"Grid cells: {m_Navmesh.GridCellsCount}", STATS_FONT, Brushes.Black, 10, Height - 70);
            e.Graphics.DrawString($"Cells: {m_Navmesh.CellsCount}", STATS_FONT, Brushes.Black, 10, Height - 85);

            e.Graphics.DrawString(string.Join("\n", LocksState.GetActiveLocks()), STATS_FONT, Brushes.Black, Width / 2 - 120, 50);
        }

        protected virtual void OnRenderPos(PaintEventArgs e)
        {
            e.Graphics.DrawString(m_Navigator.CurrentPos.ToString(), STATS_FONT, Brushes.Black, 10, Height - 55);
            e.Graphics.DrawString("[" + m_RenderCenter.X + ", " + m_RenderCenter.Y + "]", STATS_FONT, Brushes.Black, Width - 120, 5);
        }

        protected virtual void OnRefresh(int interval)
        {
            m_Bot?.Update(interval * 0.001f);
        }

        protected virtual void OnKey(KeyEventArgs e)
        {
            if (e.Control)
            {
                if (e.KeyCode == Keys.D0)
                {
                    m_RenderAxis = !m_RenderAxis;
                    e.Handled = true;
                }
                if (e.KeyCode == Keys.D1)
                {
                    m_RenderPath = !m_RenderPath;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D2)
                {
                    m_Navmesh.RegionsEnabled = !m_Navmesh.RegionsEnabled;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D3)
                {
                    m_RenderAvoidancePath = !m_RenderAvoidancePath;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D4)
                {
                    m_RenderPositionsHistory = !m_RenderPositionsHistory;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D5)
                {
                    m_RenderRoughPath = !m_RenderRoughPath;
                    e.Handled = true;
                }
            }
            else
            {
                if (e.KeyCode == Keys.S)
                {
                    Vec3 result = default(Vec3);
                    if (!m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                                            new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                                            MovementFlag.Walk,
                                            ref result))
                    {
                        result = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    }

                    m_Navigator.CurrentPos = result;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.E)
                {
                    Vec3 result = default(Vec3);
                    if (!m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                                            new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                                            MovementFlag.Walk,
                                            ref result))
                    {
                        result = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    }

                    m_Navigator.SetCustomDestination(result);
                    e.Handled = true;
                }
                //else if (e.KeyCode == Keys.R)
                //{
                //    Vec3 result = default(Vec3);
                //    if (!m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                //                            new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                //                            MovementFlag.Walk,
                //                            ref result))
                //    {
                //        result = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                //    }

                //    m_Navigator.Destination = new destination(result, DestType.Custom, 400, 700);
                //    e.Handled = true;
                //}
                else if (e.KeyCode == Keys.L)
                {
                    m_RenderLegend = !m_RenderLegend;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D1)
                {
                    m_RenderGrids = !m_RenderGrids;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D2)
                {
                    m_RenderCells = !m_RenderCells;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D3)
                {
                    m_RenderExploreCells = !m_RenderExploreCells;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D4)
                {
                    m_RenderConnections = !m_RenderConnections;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D5)
                {
                    m_RenderIds = !m_RenderIds;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D6)
                {
                    m_RenderConnected = !m_RenderConnected;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D7)
                {
                    m_RenderRegionsMode = (RegionsRenderMode)(((int)m_RenderRegionsMode + 1) % (int)RegionsRenderMode.Count);
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D8)
                {
                    m_RenderOriginalPath = !m_RenderOriginalPath;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D9)
                {
                    m_RenderRayCast = !m_RenderRayCast;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D0)
                {
                    m_RenderBacktrackPath = !m_RenderBacktrackPath;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F1)
                {
                    LoadWaypoints(m_LastWaypointsFile);
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F2)
                {
                    LoadData(m_LastDataFile ?? "nav_dump.txt");
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F3)
                {
                    m_Navmesh.Dump("nav_dump.txt");
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F4)
                {
                    m_Navmesh.Clear();
                    m_Navigator.Clear();
                    m_Explorer.Clear();
                    LoadDebugConfig();
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F5)
                {
                    m_Navmesh.Serialize("nav_save");
                    m_Navigator.Serialize("nav_save");
                    m_Explorer.Serialize("nav_save");
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F6)
                {
                    m_Navmesh.Deserialize("nav_save");
                    m_Navigator.Deserialize("nav_save");
                    m_Explorer.Deserialize("nav_save");

                    Vec3 initial_pos = m_Navigator.CurrentPos;
                    if (initial_pos.IsZero())
                        initial_pos = m_Navmesh.GetCenter();
                    m_RenderCenter.X = initial_pos.X;
                    m_RenderCenter.Y = initial_pos.Y;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F7)
                {
                    LoadDebugConfig();
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F10)
                {
                    //Thread t = new Thread(dbg_ContiniousSerialize);
                    //t.Start();

                    Thread t = new Thread(() => dbg_GenerateGrids());
                    t.Start();

                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.F11)
                {
                    //m_Navmesh.dbg_GenerateRandomAvoidAreas(100, -1, 300, 2);
                    //m_Navmesh.dbg_GenerateBlockOfAvoidAreas(50, 30);

                    //Thread t = new Thread(() => dbg_MovingRegions(200));
                    //Thread t = new Thread(() => dbg_RandomRegions(200));
                    //Thread t = new Thread(() => dbg_RandomExploreConstraints(2000));
                    //t.Start();

                    var area = new AABB(m_Navigator.CurrentPos, 1000);
                    var result = m_Explorer.GetConstraintsFloodFill(m_Navigator.CurrentPos, (x) => x.CellsAABB.Overlaps2D(area));

                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.B)
                {
                    Vec3 result = default(Vec3);

                    if (m_Navigator.CurrentPos.IsZero())
                    {
                        m_Navmesh.RayTrace(new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 1000),
                                           new Vec3(m_RenderCenter.X, m_RenderCenter.Y, -1000),
                                           MovementFlag.Fly,
                                           ref result);
                    }
                    else
                        result = m_Navigator.CurrentPos;

                    m_Bot = new TestBot(m_Navmesh, m_Navigator, m_Explorer, result, m_BotSpeed, m_BotAngularSpeed);
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.C)
                {
                    m_CenterOnBot = !m_CenterOnBot;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.D)
                {
                    if (m_Bot != null)
                        m_Bot.Destination = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.H)
                {
                    if (m_Explorer != null)
                        m_Explorer.HintPos = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.X)
                {
                    m_Navigator.CurrentPos = new Vec3(m_RenderCenter.X, m_RenderCenter.Y, 0);
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.Space)
                {
                    if (m_Bot != null)
                        m_Bot.Paused = !m_Bot.Paused;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.V)
                {
                    if (m_Bot != null)
                        m_Bot.BackTrace = !m_Bot.BackTrace;
                    e.Handled = true;
                }
                else if (e.KeyCode == Keys.P)
                {
                    m_RenderPatches = !m_RenderPatches;
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
            legend.Add(new LegendEntry("6: Toggle render connected", true, m_RenderConnected));
            legend.Add(new LegendEntry($"7: Toggle render regions mode ({m_RenderRegionsMode})", true, m_RenderRegionsMode != RegionsRenderMode.None));
            legend.Add(new LegendEntry("8: Toggle render original path", true, m_RenderOriginalPath));
            legend.Add(new LegendEntry("9: Toggle render ray cast", true, m_RenderRayCast));
            legend.Add(new LegendEntry("0: Toggle render back track path", true, m_RenderBacktrackPath));
            legend.Add(new LegendEntry("P: Toggle render patches", true, m_RenderPatches));
            legend.Add(new LegendEntry("S: Set current pos", false));
            legend.Add(new LegendEntry("E: Set destination pos", false));
            legend.Add(new LegendEntry("B: Run bot", false));
            legend.Add(new LegendEntry("F7: Reload debug.ini", false));
            legend.Add(new LegendEntry("Ctrl+0: Toggle render axis", true, m_RenderAxis));
            legend.Add(new LegendEntry("Ctrl+1: Toggle render path", true, m_RenderPath));
            legend.Add(new LegendEntry("Ctrl+2: Toggle regions update", true, m_Navmesh.RegionsEnabled));
            legend.Add(new LegendEntry("Ctrl+3: Toggle render avoidance path", true, m_RenderAvoidancePath));
            legend.Add(new LegendEntry("Ctrl+4: Toggle render positions history", true, m_RenderPositionsHistory));
            legend.Add(new LegendEntry("Ctrl+5: Toggle render rough path", true, m_RenderRoughPath));
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
            OnRenderPos(e);
        }

        private void LoadWaypoints(string filename)
        {
            m_LastWaypointsFile = filename;
            m_Waypoints.Clear();

            if (!File.Exists(filename))
                return;

            using (var reader = File.OpenText(filename))
            {
                string line;

                while ((line = reader.ReadLine()) != null)
                {
                    String[] coords = line.Split(';');

                    if (coords.Length >= 3)
                        m_Waypoints.Add(new Vec3(float.Parse(coords[0]), float.Parse(coords[1]), float.Parse(coords[2])));
                }
            }

            m_Navigator.Waypoints = m_Waypoints;

        }

        private void LoadData(string filename, bool clear = true)
        {
            m_LastDataFile = filename;

            if (m_Navmesh.Load(filename, clear, true))
            {
                Vec3 initial_pos = m_Navigator.CurrentPos;
                if (initial_pos.IsZero())
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

        private void dbg_MovingRegions(int approx_size)
        {
            Random rng = new Random();
            var regions = new List<Region>();

            for (int i = 0; i < 20; ++i)
            {
                Vec3 pos = m_Navmesh.GetRandomPos(rng);
                float size = approx_size + (float)rng.NextDouble() * (approx_size * 0.5f);
                regions.Add(new Region(new AABB(pos - new Vec3(size * 0.5f, size * 0.5f, 0), pos + new Vec3(size * 0.5f, size * 0.5f, 0)), 2, -5));
            }

            m_Navmesh.Regions = regions;

            const int dt = 50;

            while (true)
            {
                for (int i = 0; i < regions.Count; ++i)
                {
                    var region = regions[i];

                    Vec3 dir = new Vec3((float)rng.NextDouble() * 2 - 1, (float)rng.NextDouble() * 2 - 1, 0);

                    regions[i] = new Region(region.Area.Translated(dir * approx_size * ((float)dt / 1000)), region.MoveCostMult, region.Threat);
                }

                Thread.Sleep(dt);
            }
        }

        private void dbg_RandomRegions(int approx_size)
        {
            Random rng = new Random();

            const int dt = 50;

            //while (true)
            {
                var regions = new List<Region>();

                for (int i = 0; i < 20; ++i)
                {
                    Vec3 pos = m_Navmesh.GetRandomPos(rng);
                    float size = approx_size + (float)rng.NextDouble() * (approx_size * 0.5f);
                    regions.Add(new Region(new AABB(pos - new Vec3(size * 0.5f, size * 0.5f, 0), pos + new Vec3(size * 0.5f, size * 0.5f, 0)), -1, 0));
                }

                m_Navmesh.Regions = regions;

                Thread.Sleep(dt);
            }
        }

        private void dbg_RandomExploreConstraints(int approx_size)
        {
            Random rng = new Random();

            const int dt = 50;

            while (true)
            {
                var constraints = new List<AABB>();

                for (int i = 0; i < 3; ++i)
                {
                    Vec3 pos = m_Navmesh.GetRandomPos(rng);
                    float size = approx_size + (float)rng.NextDouble() * (approx_size * 0.5f);
                    constraints.Add(new AABB(pos - new Vec3(size * 0.5f, size * 0.5f, 0), pos + new Vec3(size * 0.5f, size * 0.5f, 0)));
                }

                m_Explorer.ExploreConstraints = constraints;

                Thread.Sleep(dt);
            }
        }

        private void dbg_GenerateGrids()
        {
            Random rng = new Random();

            const float connectionLength = 200;
            const float connectionWidth = 45;

            for (int i = 0; i < 10; ++i)
            {
                var connectorPos = m_Navmesh.GetRandomPos(rng);

                connectorPos.Z = 0;

                GridCell gCell = new GridCell(connectorPos - new Vec3(1, 1, 0) * connectionLength * 0.5f, connectorPos + new Vec3(1, 1, 0) * connectionLength * 0.5f);

                // add cross connector
                gCell.Add(new Nav.Cell(connectorPos - new Vec3(connectionLength * 0.5f, connectionWidth * 0.5f, 0), connectorPos + new Vec3(connectionLength * 0.5f, connectionWidth * 0.5f, 0), MovementFlag.All, -1));
                gCell.Add(new Nav.Cell(connectorPos - new Vec3(connectionWidth * 0.5f, connectionLength * 0.5f, 0), connectorPos + new Vec3(connectionWidth * 0.5f, connectionLength * 0.5f, 0), MovementFlag.All, -1));

                m_Navmesh.Add(gCell, true);
            }
        }

        private void refresh_timer_Tick(object sender, EventArgs e)
        {
            OnRefresh(RefreshTimer.Interval);

            Refresh();
        }

        private void NavMeshViewer_MouseMove(object sender, MouseEventArgs e)
        {
            if (!Control.MouseButtons.HasFlag(MouseButtons.Left))
                return;

            if (!m_LastDragMousePos.IsEmpty)
            {
                m_RenderCenter.X += (m_LastDragMousePos.X - e.X) / m_RenderScale;
                m_RenderCenter.Y += (m_LastDragMousePos.Y - e.Y) / m_RenderScale;
            }

            m_LastDragMousePos = new PointF(e.X, e.Y);
        }

        private void NavMeshViewer_MouseUp(object sender, MouseEventArgs e)
        {
            m_LastDragMousePos = PointF.Empty;
        }

        private void NavMeshViewer_MouseWheel(object sender, MouseEventArgs e)
        {
            var change = e.Delta * m_ZoomFactor;
            m_Zoom = Math.Max(m_MinZoom, Math.Min(m_MaxZoom, m_Zoom + change));

            m_RenderScale = m_Zoom / 1000;
        }

        private void NavMeshViewer_KeyPress(object sender, KeyEventArgs e)
        {
            OnKey(e);
        }

        private System.ComponentModel.IContainer m_Components = null;
        private System.Windows.Forms.Timer RefreshTimer;
        private System.Windows.Forms.Timer UpdateTimer;

        protected override void Dispose(bool disposing)
        {
            if (disposing && (m_Components != null))
            {
                m_Components.Dispose();
            }
            base.Dispose(disposing);
        }

        private void InitializeComponents()
        {
            m_Components = new System.ComponentModel.Container();
            RefreshTimer = new System.Windows.Forms.Timer(m_Components);
            UpdateTimer = new System.Windows.Forms.Timer(m_Components);

            SuspendLayout();

            RefreshTimer.Enabled = true;
            RefreshTimer.Interval = 50;
            RefreshTimer.Tick += new EventHandler(refresh_timer_Tick);
            AutoScaleDimensions = new SizeF(6F, 13F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(848, 571);
            if (File.Exists(ICON_FILE)) Icon = new Icon(ICON_FILE);
            Name = "NavMeshViewer";
            Text = "NavMeshViewer";
            FormClosing += new FormClosingEventHandler(OnClosing);
            Load += new System.EventHandler(OnLoad);
            KeyDown += new KeyEventHandler(NavMeshViewer_KeyPress);
            MouseMove += new MouseEventHandler(NavMeshViewer_MouseMove);
            MouseUp += new MouseEventHandler(NavMeshViewer_MouseUp);
            MouseWheel += new MouseEventHandler(NavMeshViewer_MouseWheel);

            ResumeLayout(false);
        }

        private enum RegionsRenderMode
        {
            None,
            MoveCostMult,
            BlockerReplacement, // display only blocker replacement cells
            Threat,
            Outline,
            Count
        }

        protected Params m_Params;
        protected Nav.Navmesh m_Navmesh = null;
        protected Nav.NavigationEngine m_Navigator = null;
        protected Nav.ExplorationEngine m_Explorer = null;
        protected float m_BotSpeed = 1500;
        protected float m_BotAngularSpeed = -1;
        protected PointF m_RenderCenter = new PointF(200, 350);
        protected float m_RenderScale = 1.0f;
        protected float m_Zoom = 1000.0f;
        protected float m_ZoomFactor = 0.5f;
        protected float m_MinZoom = 10.0f;
        protected float m_MaxZoom = 10000.0f;
        private bool m_RenderIds = false;
        protected bool m_RenderAxis = true;
        private bool m_RenderConnections = false;
        private bool m_RenderPath = true;
        private bool m_RenderRoughPath = false;
        private bool m_RenderOriginalPath = false;
        private bool m_RenderAvoidancePath = false;
        private bool m_RenderBacktrackPath = false;
        private bool m_RenderPositionsHistory = false;
        private bool m_RenderConnected = false;
        private bool m_RenderRayCast = false;
        private bool m_RenderExploreCells = false;
        private RegionsRenderMode m_RenderRegionsMode = RegionsRenderMode.MoveCostMult;
        private bool m_RenderExploreArea = false;
        private bool m_RenderCells = true;
        private bool m_RenderLegend = true;
        private bool m_RenderGrids = false;
        private bool m_RenderPatches = false;
        private bool m_CenterOnBot = true;
        private PointF m_LastDragMousePos = PointF.Empty;
        private TestBot m_Bot = null;
        private string m_LastWaypointsFile;
        private string m_LastDataFile;
        private List<Vec3> m_Waypoints = new List<Vec3>();
        private List<Vec3> m_LastPath = new List<Vec3>();
        private List<Vec3> m_LastBacktrackPath = new List<Vec3>();
        private List<Vec3> m_LastPositionsHistory = new List<Vec3>();
        private List<Vec3> m_LastExplorePath = new List<Vec3>();

        private readonly string DEBUG_CONFIG_FILE = "./debug.ini";
        private readonly string ICON_FILE = "navmeshviewer_icon.ico";

        public static readonly Font LEGEND_FONT = new Font("Arial", 8, FontStyle.Bold);
        public static readonly Font STATS_FONT = new Font("Arial", 8);
    }

    public class RenderHelper
    {
        public static float GetProportional(float value, float min, float max, float new_min, float new_max)
        {
            if (min == max)
                return new_max;

            float value_progress = (value - min) / (max - min);
            return new_min + (new_max - new_min) * value_progress;
        }

        public static void Render(Nav.Cell cell, PointF trans, PaintEventArgs e, bool draw_connections, bool draw_id, bool render_move_cost_mult, bool render_threat, bool render_outline = true, bool render_disabled = false, Color force_color = default(Color))
        {
            if (!render_disabled && cell.Disabled)
                return;

            if (render_outline)
                DrawRectangle(e.Graphics, cell.Replacement ? REPLACEMENT_CELL_BORDER_PEN : CELL_BORDER_PEN, trans, cell.Min, cell.Max);

            Color cell_color = Color.White;

            if (cell.MovementCostMult < 1)
            {
                int move_cost_level = 255 - (int)Math.Min(GetProportional(cell.MovementCostMult, 0, 1, 20, 255), 255);
                cell_color = Color.FromArgb(255, move_cost_level, 255, move_cost_level);
            }
            else if (cell.MovementCostMult > 1 && render_move_cost_mult)
            {
                int move_cost_level = 255 - (int) Math.Min(GetProportional(cell.MovementCostMult, 1, 100, 20, 255), 255);
                cell_color = Color.FromArgb(255, move_cost_level, move_cost_level, move_cost_level);
            }
            else if (cell.Threat != 0 && render_threat)
            {
                if (cell.Threat > 0)
                {
                    int threat_level = 255 - (int)Math.Min(GetProportional(cell.MovementCostMult, 1, 100, 20, 255), 255);
                    cell_color = Color.FromArgb(255, 255, threat_level, threat_level);
                }
                else
                    cell_color = Color.Aquamarine;
            }

            if (cell.Flags == MovementFlag.Fly)
                cell_color = Color.Gray;

            if (force_color != Color.Empty)
                cell_color = force_color;

            FillRectangle(e.Graphics, new SolidBrush(cell_color), trans, cell.Min, cell.Max);

            if (draw_connections)
            {
                foreach (Nav.Cell.Neighbour neighbour in cell.Neighbours)
                    DrawLine(e.Graphics, CELL_CONNECTION_PEN, trans, cell.Center, neighbour.border_point);
            }

            if (draw_id)
                DrawString(e.Graphics, Brushes.Black, trans, cell.Min, cell.Id < 0 ? cell.GlobalId + "(G)" : cell.Id.ToString(), 2);
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
            //DrawRectangle(e.Graphics, Pens.Magenta, trans, cell.Min, cell.Max);
            DrawRectangle(e.Graphics, Pens.Purple, trans, cell.CellsAABB.Min, cell.CellsAABB.Max);

            //DrawString(e.Graphics, Brushes.Black, trans, cell.Position, Math.Round(cell.CellsArea()).ToString(), 14);

            if (cell.Explored)
            {
                //DrawLine(e.Graphics, explored_pen, trans, cell.Min, cell.Max);
                //DrawLine(e.Graphics, explored_pen, trans, new Vec3(cell.Min.X, cell.Max.Y), new Vec3(cell.Max.X, cell.Min.Y));
                FillRectangle(e.Graphics, explored_brush, trans, cell.CellsAABB.Min, cell.CellsAABB.Max);
            }
            else if (cell.Small)
            {
                FillRectangle(e.Graphics, small_brush, trans, cell.CellsAABB.Min, cell.CellsAABB.Max);
            }

            //DrawCircle(e.Graphics, Pens.Red, trans, cell.Position, radius);
            //DrawString(e.Graphics, Brushes.Black, trans, cell.Position, cell.UserData.ToString(), 10);

            if (draw_connections)
            {
                foreach (Nav.Cell.Neighbour neighbour in cell.Neighbours)
                {
                    ExploreCell neighbour_cell = (ExploreCell)neighbour.cell;

                    DrawLine(e.Graphics, EXPLORE_CELL_CONNECTION_PEN, trans, cell.Center, neighbour.cell.Center);
                }
            }

            if (draw_id)
                DrawString(e.Graphics, Brushes.Black, trans, cell.Position, cell.GlobalId.ToString() + " (" + cell.Id.ToString() + ")", 35);
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

        public static void DrawLines(Graphics g, Pen p, PointF trans, List<Vec3> points, float point_radius, bool draw_ids = false)
        {
            if (points.Count < 2)
                return;

            if (point_radius > 0)
                DrawCircle(g, Pens.Black, trans, points[0], point_radius);

            if (draw_ids)
                DrawString(g, Brushes.Black, trans, points[0] + new Vec3(point_radius, 0,0), "0", 3);

            for (int i = 1; i < points.Count; ++i)
            {
                DrawLine(g, p, trans, points[i - 1], points[i]);

                if (point_radius > 0)
                    DrawCircle(g, Pens.Black, trans, points[i], point_radius);

                if (draw_ids)
                    DrawString(g, Brushes.Black, trans, points[i] + new Vec3(point_radius, 0, 0), i.ToString(), 3);
            }
        }

        private static Pen EXPLORE_CELL_CONNECTION_PEN = new Pen(Color.FromArgb(255, 50, 50, 50), 3f);
        private static Pen GRID_CELL_CONNECTION_PEN = new Pen(Color.FromArgb(255, 50, 50, 50), 4);
        private static Pen CELL_CONNECTION_PEN = new Pen(Color.Black, 0.3f);
        private static Pen CELL_BORDER_PEN = new Pen(Color.Blue, 0.3f);
        private static Pen REPLACEMENT_CELL_BORDER_PEN = new Pen(Color.Black, 0.3f);
        public static readonly Pen AXIS_PEN = new Pen(Color.SaddleBrown, 0.3f);
        public static readonly Pen EXPLORE_PATH_PEN = new Pen(Color.Black, 5);
        public static readonly Pen PATH_PEN = new Pen(Color.Black, 1.5f);
        private static Brush explored_brush = new SolidBrush(Color.FromArgb(128, 50, 50, 50));
        private static Brush small_brush = new SolidBrush(Color.FromArgb(128, 255, 192, 203));
    }
}
