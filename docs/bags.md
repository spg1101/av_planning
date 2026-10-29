# Bags

Bags are never committed. Store them on the shared drive and list them here.

## Recording

Record from a CARLA run (raw sensors + ground truth; the launch file picks the topics):

```bash
ros2 launch planning_bringup carla.launch.py scenario:=town03_route_traffic \
  localization:=slam_mapping monitor:=false record:=t03_traffic_01
```

Lidar bags grow at roughly 5 MB per second; keep runs to a few minutes.

Don't record `/tf`, `/odometry/filtered` or `/map`: replay regenerates them.

## Index

| Name | Platform | Date | Length | Contents | Notes |
| --- | --- | --- | --- | --- | --- |
| t03_traffic_01 | CARLA Town03 | | | lidar, scan, imu, gnss, odom, ground truth | example row |
