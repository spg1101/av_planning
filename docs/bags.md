# Bags

Bags are never committed. Store them on the shared drive and list them here.

## Recording

Record raw sensors and ground truth only, so localization can be re-run on replay:

```bash
ros2 bag record -s mcap -o gym_levine_01 \
  /sensors/scan /sensors/imu /vehicle/odom /sensors/gnss \
  /perception/detections /ground_truth/odom /tf_static /clock
```

Don't record `/tf`, `/odometry/filtered` or `/map`: replay regenerates them.

## Index

| Name | Platform | Date | Length | Contents | Notes |
| --- | --- | --- | --- | --- | --- |
| gym_levine_01 | f1tenth_gym | | | scan, odom, ground truth | teleop lap |
