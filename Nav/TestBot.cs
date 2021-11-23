using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace Nav
{
    internal class TestBot : INavigationObserver
    {
        public TestBot(Navmesh navmesh, NavigationEngine navigator, ExplorationEngine explorer, Vec3 pos, float speed, float angular_speed, bool enableExplorer = true)
        {
            m_Navmesh = navmesh;
            m_Navmesh.Verbose = true;
            //m_Navigator = new NavigationEngine(navmesh);
            m_Navigator = navigator;
            m_Navigator.EnableThreatAvoidance = true;
            m_Navigator.UpdatePathInterval = -1;
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
            m_AngularSpeed = angular_speed;

            SimulateStuck = false;
            Paused = false;
            m_GotoPosUpdateTimer.Start();
        }

        public void OnDestinationReached(destination dest)
        {
            if (dest.pos.Equals(m_Destination))
                m_Destination = Vec3.ZERO;
        }

        public void OnDestinationReachFailed(destination dest)
        {
        }

        public void OnHugeCurrentPosChange()
        {
        }
        
        public void Update(float dt)
        {
            if (!m_Destination.IsZero())
                m_Navigator.Destination = new destination(m_Destination);

            m_LastGotoPos = m_Navigator.GoToPosition.pos;

            var newPos = m_Navigator.CurrentPos;

            if (!m_LastGotoPos.IsZero())
            {
                Vec3 wanted_dir = Vec3.ZERO;
                float dist = 0;

                wanted_dir = m_LastGotoPos - m_Navigator.CurrentPos;
                dist = wanted_dir.Length();
                wanted_dir.Normalize();

                //turn
                if (m_AngularSpeed > 0)
                    m_Direction = Vec3.RotateTowards(m_Direction, wanted_dir, m_AngularSpeed * dt);
                else
                    m_Direction = wanted_dir;

                //move
                if (m_Navigator.ThreatAhead.Item2 == 0 && !Paused && !SimulateStuck)
                    newPos += m_Direction * Math.Min(m_Speed * dt, dist);
            }

            m_Navigator.CurrentPos = newPos;

            m_Navigator.IsStandingOnPurpose = m_Navigator.ThreatAhead.Item2 > 0;
        }

        public void Render(Graphics g, PointF trans)
        {
            RenderHelper.DrawCircle(g, m_Navigator.IsInThreat ? Pens.DarkRed : Pens.Blue, trans, m_Navigator.CurrentPos, 6);
            RenderHelper.DrawLine(g, Pens.DarkMagenta, trans, m_Navigator.CurrentPos, m_Navigator.CurrentPos + m_Direction * 12);
            if (m_Navigator.ThreatAhead.Item2 > 0)
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
            set { m_Destination = value; m_Navigator.Destination = new destination(value); }
        }

        private Navmesh m_Navmesh = null;
        private NavigationEngine m_Navigator = null;
        private ExplorationEngine m_Explorer = null;
        private Vec3 m_Direction = new Vec3(1, 0, 0);
        private Vec3 m_LastGotoPos = Vec3.ZERO;
        private Vec3 m_Destination = Vec3.ZERO;
        private Stopwatch m_GotoPosUpdateTimer = new Stopwatch();
        private float m_Speed = 10;
        private float m_AngularSpeed = 30; // degrees/second
        private const int GOTO_POS_UPDATE_INTERVAL = 25;
    }
}
