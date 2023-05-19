Nav is easy to use 3D navigation library. It operates on axis-aligned cuboids (cells). Navigation data is built on-the-fly as soon as cells are fed to the system. Separate navigation module is responsible for pathing. Exploration module can be supplied for automatic navmesh traversal. Nav can be used in any project with AABB navmesh data.
For documentation check out Wiki.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/zTLo_vWpHo0/0.jpg)](https://www.youtube.com/watch?v=zTLo_vWpHo0)

Features
* dynamically builds navmesh from soup of axis-aligned cuboids
* simple navigation (set Destination, update CurrentPos, then move toward GoToPosition)
* generates exploration data allowing dynamic navmesh traversal
* easy exploration (create instance of exploration engine, update CurrentPos, then move toward GoToPosition, Destination will be automatically set by exploration engine)
* all updated and calculations are performed on separate threads, without blocking user's code execution
* automatic anti-stuck system (altering pathing when actor is potentially stuck on collision with geometry), no configuration required
* supports ray casts and ray traces
* supports both walking and flying
* NavMeshViewer allows easy viewing/debugging

Special thanks to [JetBrains](https://jb.gg/OpenSourceSupport) for supporting development of this project.<br>
<img src="https://resources.jetbrains.com/storage/products/company/brand/logos/jb_beam.png" alt="JetBrains Logo (Main) logo." width=15% height=15%>
