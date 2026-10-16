# Maps

| File | Use |
| --- | --- |
| `example_oval.csv` | Waypoint loop for `kinematic.launch.py` (no real map needed) |
| `<track>.csv` | Waypoints recorded with `ros2 run planning_sim waypoint_recorder` |
| `<track>.posegraph` / `.data` | Saved slam_toolbox maps (Git LFS) |

Waypoint CSV format: one `x, y` per line in the map frame, `#` for comments,
roughly 0.1-0.2 m apart, in driving order.
