using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace Nav
{
    internal class TestBot : IDisposable, INavigationObserver
    {
        public TestBot(Navmesh navmesh, NavigationEngine navigator, ExplorationEngine explorer, Vec3 pos, Vec3 dest, bool explore = false, bool simulate_stuck = false, int dest_grid_id = -1, List<Vec3> waypoints = null)
        {
            m_Navmesh = navmesh;
            //m_Navigator = new NavigationEngine(navmesh);
            m_Navigator = navigator;
            m_Navigator.AddObserver(this);
            m_Navigator.CurrentPos = pos;
            m_Navigator.Precision = 2;
            m_Navigator.EnableAntiStuck = false;
            m_Navigator.DestinationGridsId = dest_grid_id != -1 ? new List<int>(new int[] { dest_grid_id }) : null;
            if (waypoints != null)
                m_Navigator.Waypoints = waypoints;  

            //m_Explorer = new Nav.ExploreEngine.Nearest(m_Navmesh, m_Navigator);
            m_Explorer = explorer;
            m_Explorer.Enabled = explore;

            Destination = dest;
            SimulateStuck = simulate_stuck;
            Paused = false;
            m_GotoPosUpdateTimer.Start();
        }

        public void Dispose()
        {
            m_Navigator.Dispose();
            m_Explorer.Dispose();
        }

        public static float SPEED = 100;//25; //approximated movement speed with 25% move speed bonus

        public void OnDestinationReached(DestType type, Vec3 dest)
        {
            if (dest.Equals(m_Destination))
                m_Destination = Vec3.ZERO;
        }

        public void OnDestinationReachFailed(DestType type, Vec3 dest)
        {
        }

        public void OnHugeCurrentPosChange()
        {
        }
        
        public void Update(float dt)
        {
            if (m_GotoPosUpdateTimer.ElapsedMilliseconds > GOTO_POS_UPDATE_INTERVAL)
            {
                m_LastGotoPos = m_Navigator.GoToPosition;
                m_GotoPosUpdateTimer.Restart();
            }

            if (!m_Destination.IsZero())
                m_Navigator.Destination = m_Destination;

            if (m_Explorer.IsExplored() || m_LastGotoPos.IsZero())
                return;

            Vec3 dir = Vec3.ZERO;
            float dist = 0;

            if (!Paused && !SimulateStuck && !m_LastGotoPos.Equals(m_Navigator.CurrentPos))
            {
                dir = m_LastGotoPos - m_Navigator.CurrentPos;
                dist = dir.Length();
                dir.Normalize();
            }

            m_Navigator.IsStandingOnPurpose = false;
            m_Navigator.CurrentPos = m_Navigator.CurrentPos + dir * Math.Min(SPEED * dt, dist);
        }

        public void Render(Graphics g, PointF trans)
        {
            RenderHelper.DrawCircle(g, Pens.Blue, trans, m_Navigator.CurrentPos, 4);
        }

        public bool Paused { get; set; }
        
        public bool SimulateStuck { get; set; }
        
        public bool BackTrace
        {
            get { return m_Navigator.BackTrackEnabled; }
            set { m_Navigator.BackTrackEnabled = value; }
        }

        public Vec3 Position
        {
            get { return m_Navigator.CurrentPos; }
        }

        public Vec3 Destination
        {
            set { m_Destination = value; m_Navigator.Destination = value; }
        }

        private Nav.Navmesh m_Navmesh = null;
        private Nav.NavigationEngine m_Navigator = null;
        private Nav.ExplorationEngine m_Explorer = null;
        private Vec3 m_LastGotoPos = Vec3.ZERO;
        private Vec3 m_Destination = Vec3.ZERO;
        private Stopwatch m_GotoPosUpdateTimer = new Stopwatch();
        private const int GOTO_POS_UPDATE_INTERVAL = 25;
    }
}
