# guandao

`guandao` is a standalone no-lidar recorded-path controller. It has its own
Pure Pursuit implementation and does not launch, link against, or depend on
the `lio_pure_pursuit` package.

Runtime data flow:

```text
/odom_combined (wheel + IMU EKF) -> Pure Pursuit -> /cmd_vel
/win_detection (forward camera HAT detections) -> visual avoidance
```

It does not start `planar_lio_node`, does not subscribe to `/scan`, and does
not use laser points for collision checking. The HAT selection, trigger gates,
side decision, and pixel target construction share the policy used by
`origincar_control`; this is source-level behavior, not a runtime dependency.

The controller supports forward and reverse direction columns in the path file,
approaches direction changes with a controlled stop, and publishes its path,
lookahead point, and state for RViz/debugging. Visual HAT detections reduce
speed and steer around the target; there is no geometric laser obstacle planner.

Start the base/EKF and the forward-camera YOLO node first, then run:

```bash
ros2 launch guandao guandao.launch.py \
  path_file:=/path/to/path.txt
```

Override `odom_topic`, `vision_topic`, or the two speed limits from the launch
command when the target vehicle uses different names.
