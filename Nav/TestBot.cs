using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace Nav
{
    internal class TestBot : INavigationObserver
    {
        public TestBot(Navmesh navmesh, NavigationEngine navigator, ExplorationEngine explorer, Vec3 pos, float speed, bool enableExplorer = true)
        {
            m_Navmesh = navmesh;
            m_Navmesh.Verbose = true;
            //m_Navigator = new NavigationEngine(navmesh);
            m_Navigator = navigator;
            m_Navigator.AddObserver(this);
            m_Navigator.CurrentPos = pos;
            //m_Navigator.EnableThreatAvoidance = true;
            //m_Navigator.DefaultPrecision = 20;
            //m_Navigator.Precision = 20;
            //m_Navigator.PathNodesShiftDist = 20;
            //m_Navigator.EnableAntiStuck = false;
            //m_Navigator.DestinationGridsId = dest_grid_id != -1 ? new List<int>(new int[] { dest_grid_id }) : null;
            //if (waypoints != null)
            //    m_Navigator.Waypoints = waypoints;  

            //m_Explorer = new Nav.ExploreEngine.Nearest(m_Navmesh, m_Navigator);
            m_Explorer = explorer;
            if (enableExplorer)
                m_Explorer.Enabled = true;

            m_Speed = speed;

            SimulateStuck = false;
            Paused = false;
            m_GotoPosUpdateTimer.Start();
        }

        public void OnDestinationReached(DestType type, Vec3 dest, Object userData)
        {
            if (dest.Equals(m_Destination))
                m_Destination = Vec3.ZERO;
        }

        public void OnDestinationReachFailed(DestType type, Vec3 dest, Object userData)
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

            m_Navigator.IsStandingOnPurpose = m_Navigator.IsThreatAhead;

            if (!m_Navigator.IsThreatAhead)
                m_Navigator.CurrentPos = m_Navigator.CurrentPos + dir * Math.Min(m_Speed * dt, dist);
        }

        public void Render(Graphics g, PointF trans)
        {
            RenderHelper.DrawCircle(g, m_Navigator.IsInThreat ? Pens.DarkRed : Pens.Blue, trans, m_Navigator.CurrentPos, 4);
            if (m_Navigator.IsThreatAhead)
                RenderHelper.DrawCircle(g, Pens.IndianRed, trans, m_Navigator.CurrentPos, 3);
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

        private Navmesh m_Navmesh = null;
        private NavigationEngine m_Navigator = null;
        private ExplorationEngine m_Explorer = null;
        private Vec3 m_LastGotoPos = Vec3.ZERO;
        private Vec3 m_Destination = Vec3.ZERO;
        private Stopwatch m_GotoPosUpdateTimer = new Stopwatch();
        private float m_Speed = 10;
        private const int GOTO_POS_UPDATE_INTERVAL = 25;
    }
}
