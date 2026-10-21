# lio_hybird

当前阶段的中文开发、真车调试和交接基线见
[`STAGE_HANDOFF_20260806.md`](STAGE_HANDOFF_20260806.md)。当前结论是扫码后优先使用
`post_qr_route_mode:=parking`；launch 的代码默认值仍为 `right_turn`，测试时必须显式传参。

This package is an isolated hybrid copy of `lio_new`. It keeps the same NDT-EKF LIO, visual QR
sprint option and ring pure-pursuit controller. `lio_new` is not modified by this package.

## Control ownership

`origincar_control` and `lio_hybird` are launched in separate terminals. Both use `/cmd_vel`, but
the QR mission state guarantees that only one publishes motion commands in each phase.

1. Before QR: standalone `origincar_control` publishes directly to `/cmd_vel`; LIO keeps localizing and
   publishing diagnostics without emitting commands.
2. QR received: `origincar_control` stops its image subscription, publishes zero once and
   acknowledges `/origincar_control/control_released`.
3. Once chassis `/odom` reports a speed below `0.04 m/s`, ownership switches permanently to
   `lio_hybird_controller`, which also publishes directly to `/cmd_vel`.
   The HSV bridge is idle before this release and begins ring-image processing afterward.
4. LIO switches to the selected fixed route whose head is
   `(4.20, 1.50, 30.2 deg)` and whose point 72 is the center-corridor/ring entry. `parking` performs
   one forward/reverse/forward parking manoeuvre; `right_turn` uses an all-forward right U-turn.
   There is no command replay, reliable return-point search or runtime connector generation.
5. Built-in pure pursuit and its lidar/vision local avoidance own every fixed route segment,
   including the QR-to-ring corridor and all reverse/forward direction switches. There is no Nav2
   controller handoff or external local planner in this package.

Build on the vehicle:

```bash
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
source install/setup.bash
colcon build --packages-select qr_code_detection image_bridge_hsv origincar_control lio_hybird qr_screen_display --symlink-install
source install/setup.bash
```

Terminal 1 starts LIO monitoring, QR detection and the post-QR controller:

```bash
ros2 launch lio_hybird lio_hybird.launch.py \
  pre_qr_control_mode:=external \
  post_qr_route_mode:=parking
```

Terminal 2 starts the original visual controller. It waits for one keyboard character before moving:

```bash
ros2 run origincar_control origincar_control
```

Do not start Nav2 `controller_server` for this package. It would create a competing command path and
is not consumed by `lio_hybird_controller`.

Useful diagnostics:

- `/lio_hybird/tracking_path`: selected post-QR route and ring path;
- `/lio_hybird/state`: controller ownership and recovery state;
- `/lio_hybird/local_plan`: built-in candidate local trajectories;
- `/lio_hybird/obstacles`: filtered lidar obstacle cloud.
- `/origincar_control/started`: latched confirmation that the keypress armed the first stage;
- `/origincar_control/control_released`: one-shot confirmation that visual control stopped.

## Original lio_new notes retained below

This package is an independent experimental copy of `lio_pure_pursuit`. The controller and
competition-specific avoidance are retained, while localization is replaced by a keyframe NDT
submap front end inspired by ESKF/IESKF LIO. It does not use Nav2's global occupancy-grid planner.

## Data flow

```text
/imu/data --------> planar error-state prediction ----------------+
/odom wheel speed -> longitudinal-velocity observation -----------+--> /lio_hybird/odom
/scan -> deskew -> keyframe NDT submap -> iterative pose update ---+
                         +-> bootstrap fallback: scan-to-scan ICP

/lio_hybird/odom + recorded CSV path -> curvature-limited pure pursuit --+
/win_detection HAT -----------> visual avoidance policy -----------+--> /cmd_vel
/scan ------------------------> directional safety corridor --------+
/image_yellow_binary ---------> upper-ring visual track guard ------+
```

The reference configuration uses 3D point clouds, incremental NDT and ESKF/IESKF. The installed
LSN10 publishes a 2D `sensor_msgs/LaserScan`, so this package implements the SE(2) equivalent. A
wheel/IMU prediction deskews the scan and supplies the optimization prior. Scans are inserted as
keyframes after 0.20 m or 30 degrees, and the current scan is iteratively matched against the recent
keyframe voxel submap. The old point-to-line ICP remains only for startup or rejected NDT updates.

This is not a full 3D IESKF: there is no z/roll/pitch state and accelerometer position integration is
intentionally omitted. The vehicle's low-cost 20 Hz IMU and wheel speed are better suited to the
five-state planar model. The architectural experiment is the important part: persistent submap
matching replaces accumulation against only the immediately previous scan.

The EKF state is `[x, y, yaw, longitudinal_velocity, gyro_z_bias]`. The raw wheel odometry pose is not
used; only its measured longitudinal velocity is used. This avoids feeding the same wheel-integrated
pose into the estimator twice.

The node subscribes to `/imu/data`, but does not fuse its orientation quaternion by default. The
vehicle's Madgwick filter has `use_mag=false`, so that yaw is another integration of the same gyro;
treating it as an independent absolute observation would double-count one sensor and make the yaw
covariance falsely collapse. Angular velocity and stationary gyro-bias estimation provide prediction,
while NDT is the independent yaw correction. `use_imu_orientation` remains available only for a
future IMU orientation source with a genuinely independent heading reference.

The current live transform reports `base_footprint -> laser = (-0.03, 0.0, 0.23)` with zero yaw, so
the planar configuration uses `laser_x=-0.03`, `laser_y=0.0`, and `laser_yaw=0.0`.

`publish_tf` is false by default because `base_orin`'s EKF already publishes
`odom -> base_footprint`. Publishing `lio_hybird_odom -> base_footprint` at the same time would give one
child frame two competing parents. The controller consumes `/lio_hybird/odom` directly, so this does
not affect control. The NDT submap, estimated path, controller markers and local plans remain visible
in RViz; the raw `/scan` display requires a valid TF chain and may be unavailable in this isolated
A/B configuration.

## Safety and ownership

Do not run Nav2 `controller_server`, `dzfollowpath`, or another velocity controller. The standalone
visual controller owns `/cmd_vel` before QR and permanently stops after releasing control; the LIO
controller is silent before QR and owns the same topic afterward. The removed timer-based handoff
does not inject timeouts, resample commands or add a 30 Hz forwarding delay.

The visual HAT trigger and target-pixel policy are extracted from `origincar_control.cpp`:

- normal trigger at bounding-box height ratio `0.22` with the same horizontal gate;
- forced trigger at ratio `0.50`;
- avoidance target `hat_center + 1.60 * side * hat_width`;
- the existing PID defaults are retained.

The competition profile is vision-led. Lidar may trigger earlier at race speed, but vision takes
ownership of the side decision as soon as a valid HAT target appears. Lidar then rejects only
trajectories that collide; it does not replace the camera steering policy.

During avoidance a short-horizon sampler evaluates executable curvature-profiled trajectories against
the deskewed laser points using the full rectangular footprint. Each candidate is a spatial S-curve:
it turns toward the selected free side, holds the lateral offset, then applies opposite curvature to
return parallel to the recorded path. The horizon is measured in metres. The current YAML limits
forward tracking to `0.60 m/s`; the original visual-controller target was `0.90 m/s`, but a 2026-07-28
bag measured roughly `0.75 s` from yaw command to IMU response.
`obstacle_avoid_speed` is also capped by `max_forward_speed`, so raising the avoidance parameter alone
cannot exceed the current `0.60 m/s` ceiling. The local sampler does not generate a
slower duplicate of the same geometric rollout. A stale odometry or laser stream commands zero
velocity.

If every forward avoidance candidate is blocked, or normal tracking cannot find a legal white-area
trajectory, the controller enters one shared recovery state machine: `ESCAPE_BRAKING`,
`ESCAPE_TRAJECTORY_SELECTED`, `ESCAPE_REVERSING`, a settling brake state, then `ESCAPE_REPLAN`.
After the vehicle has physically stopped, an 11-sample constant-curvature reverse rollout searches
for space using the rectangular footprint, confirmed lidar obstacles, the 5 m field boundary, path
corridor and semantic mask. The selected curvature is locked
for that recovery so scan jitter cannot alternate steering. A measured actuator reaction delay is
included in its stopping distance. Two
bounded attempts are permitted; the budget resets only after `0.30 m` of subsequent path progress.
On an intentionally reverse path segment, the same mechanism moves forward because recovery always
backs away from the currently blocked direction of travel.

Because the LSN10 has a blind sector, the controller keeps a voxelized one-second local memory of
deskewed obstacle points in the LIO frame. The competition walls and cones are static during a run, so
an object does not become free space immediately after it moves into the sensor blind sector.

Sparse reflection or thermal returns are filtered before that memory. A point must belong to a
three-point cluster on neighbouring laser beams, then receive a second observation within `0.30 s`
and `0.10 m` in the LIO frame. Unconfirmed candidates expire after `0.35 s` and never participate in
collision checking. This prevents a transient speckle from being promoted into a persistent static
obstacle while adding only one lidar frame of confirmation latency. Raw speckles remain visible on
`/scan`; `/lio_hybird/obstacles` contains confirmed points only.

The competition wall is also rejected by shape before local collision checking. Consecutive beams
are grouped and a PCA test removes clusters that are at least `0.45 m` long, at least eight points,
thin, and strongly linear. Compact cone clusters remain. This filter does not alter `/scan` or the
scan points used by LIO, because the long paper walls are useful localization features.

The 5 m x 5 m outer paper wall is a known field boundary, not a random obstacle. Returns inside a
0.20 m perimeter band are filtered from the cone cloud; the metric field extent and semantic mask
still prevent the vehicle footprint from crossing the physical boundary. This deliberately prevents
the observed skewed wall scan from creating phantom local-planner blockages.

The supplied 1024 x 1024 field drawing is converted to a true binary PGM and interpreted as a 5 m x
5 m semantic rule mask at `0.0048828125 m/pixel`. It is not used for localization or global planning.
The assumed convention is image bottom-left `(0,0)`, right `+x`, up `+y`, yaw `0`. During normal
tracking a short mask guard adjusts curvature before the footprint cuts into black. During active
obstacle avoidance, black becomes a high-cost soft constraint, so the controller can cross it briefly
when that gives the fast collision-free pass required by the competition rules.

If the semantic mask is disabled, the controller falls back to two path corridors. Candidates inside
`preferred_path_half_width` are searched first; `emergency_path_half_width` is opened only when every
preferred-corridor trajectory collides.

Path reacquisition is restricted to a forward arc-length window. This matters in the upper ring where
two valid path segments are spatially close: the controller must not jump to the opposite side of the
ring merely because it is the nearest Euclidean point.

## Upper-ring drift recovery

The normal steering chain remains `LIO pose -> recorded path -> pure pursuit -> /cmd_vel` everywhere,
including the ring. `origincar_track` is only a source-code reference: it is not modified, linked or
launched, and its pixel steering command is never blended into pure pursuit.

The ideal path reaches the ring through points 72-77. Points 78-86 are the first straight lower edge
near `y=3.20 m`, heading approximately `+x`; this is a known semantic landmark. In that short window,
the controller combines `/image_yellow_binary`, `/aurora/depth/image_raw` and
`/aurora/rgb/camera_info`. Yellow boundary pixels are back-projected to 3D, the observed ground plane
removes the camera pitch and roll, and lane-center lateral position is fitted against forward distance.
The fit intercept is translation and the slope is heading, so a centered vehicle with a rotated nose
is not mistaken for a translated vehicle. If only one boundary is visible, the known `0.50 m` lane
width supplies the missing half. Five consistent metric observations produce one
`/lio_hybird/ring_y_correction` message.

RGB and depth must be aligned 640x400 frames with header timestamps within the configured tolerance.
If depth, camera calibration or synchronization is unavailable, the correction is skipped; the
controller does not inject a monocular pixel estimate into LIO. `submap_lio` applies an accepted y
registration to both its filter state and scan-to-scan ICP anchor, preventing the next lidar update
from restoring the old offset.

The continuously available yellow mask is only a safety cue for the bounded ring recovery sampler.
It may confirm that a mask conflict is localization drift, but it does not choose the normal tracking
curvature. Recovery still enforces the vehicle footprint, lidar collision checks, field boundary,
path corridor and yaw-rate limits.

## Build on the vehicle

```bash
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select qr_code_detection image_bridge_hsv origincar_control lio_hybird --symlink-install
source install/setup.bash
```

This SSHFS-mounted PC workspace is not the target build environment, so the package must be compiled
on the vehicle.

## Run

Start Aurora in synchronized RGBD mode. The default driver launch has `rgbd_enable=false`, which
produces independent RGB/depth timestamps and is intentionally rejected by the localization gate:

```bash
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py \
  rgbd_enable:=true align_mode:=true depth_correction:=true point_cloud_enable:=false
```

Start the base, lidar and YOLO launch files first. Do not start Nav2 or `origincar_track`.

Terminal 1 starts the LIO monitor, QR detector and single command arbiter:

```bash
ros2 launch lio_hybird lio_hybird.launch.py \
  pre_qr_control_mode:=external \
  post_qr_route_mode:=parking
```

Terminal 2 starts the original visual line following and HAT avoidance logic. Standard input works
because this is a direct `ros2 run`, not a launch child process:

```bash
ros2 run origincar_control origincar_control
```

For an LIO-controlled first stage without the standalone visual process, use
`pre_qr_control_mode:=lio`. With `enable_qr_mission:=false`, the LIO controller follows `path_file`
directly.

The default launch now runs the two-stage QR mission and starts `qr_code_detection`. The first
path sprints to `(4.8 m, 1.8 m, 45 deg)`. A valid `/sign_switch` message commands zero velocity
immediately, then selects clockwise (`sign_data=3`) or counterclockwise (`sign_data=4`). After the
vehicle stops, it switches to the selected fixed post-QR path. The default
`post_qr_route_mode:=reverse_ring` immediately reverses from `(4.20, 1.50, 30.2 deg)`, backs
through the center channel, switches to forward on the lower ring and follows the QR-selected
direction. No route argument is needed for this normal reverse-entry mode.

Use `post_qr_route_mode:=reverse_ring_inner` for the A/B variant: its channel entry is shifted
left, its bends start earlier, and its upper ring segment is moved inward to about `y=4.30 m`.
Use `post_qr_route_mode:=reverse_full_ring` to keep reversing for the complete QR-selected ring
lap. That route reverses past the channel center after the lap, stops at a parking-style cusp,
then drives forward through the channel exit. The legacy names `reverse_ring_ccw` and
`reverse_ring_ccw_inner` remain accepted as aliases for the normal and inner route families.
Use `post_qr_route_mode:=reverse_full_real_ring` for the added measured full-reverse pair:
`sign_data=3` selects `path_post_qr_reverse_full_real_ring_clockwise.txt`, while
`sign_data=4` selects `path_post_qr_reverse_full_real_ring_counterclockwise.txt`.
Use `post_qr_route_mode:=symmetry` to select `symmetry_path_clockwise.txt` for `sign_data=3`
and `symmetry_path_counterclockwise.txt` for `sign_data=4`.
The older `parking` and `right_turn` route families also remain available.
Set `auto_select_post_qr_route:=true` to select among the reverse-entry route files after the QR
stop. The controller scores only the QR-to-channel segment using the current confirmed lidar
obstacles, prioritizing fewer body-envelope collisions and then larger clearance. If the candidate
list is unavailable or invalid, it falls back to the fixed route selected by `post_qr_route_mode`.
The built-in lidar/vision local avoidance remains active after the route switch. If QR detection is already running
separately, pass `qr_detection:=false`. To run the legacy single-path controller, pass
`enable_qr_mission:=false qr_detection:=false`.

The post-QR transition speed is independently limited until the selected route first reaches the ring-channel
entrance. Defaults are `post_qr_transition_forward_speed:=0.28`,
`post_qr_transition_reverse_speed:=0.28`, and `post_qr_transition_end_y:=1.90`. The controller resolves the
corresponding path point separately for every route, so routes with different point densities use the same map
boundary. After that point, the normal global forward/reverse speed limits apply.

`enable_speed_weighted_obstacle_memory:=true` makes persistent obstacle positioning trust slow scans more.
At or above `obstacle_low_speed_threshold:=0.35`, a new scan contributes the configured high-speed weight
(`0.20` by default); the contribution increases continuously to `0.75` when stopped. Disable the switch to
restore the fixed historical fusion weight of `0.35`.

The controller publishes one `/take_photo/trigger` before the third ring corner. Normal forward
ring routes publish `front`; `reverse_full_ring` routes publish `aurora`. The one-based trigger
points are independent because the two cameras have different fields of view:

```bash
ros2 launch lio_hybird lio_hybird.launch.py \
  ring_photo_forward_path_point:=125 \
  ring_photo_reverse_path_point:=120
```

It waits for measured yaw rate to fall below `ring_photo_max_abs_yaw_rate` so the shot occurs on a
straight, then forces the trigger after `ring_photo_force_after_points` to avoid missing it. Start
`qr_screen_display.launch.py` before motion; its background client takes the latest camera frame,
rotates the rear-mounted Aurora image 180 degrees, exchanges it with the PC server, and publishes
the returned Chinese text without blocking vehicle control.

The launch starts the Aurora HSV and binary pipeline by default from
`/aurora/rgb/image_raw`. Override that with `ring_camera_topic:=...`, or use
`ring_vision:=false` when validating LIO without a camera. `ring_vision_debug:=true` publishes
`/image_new_binary_rgb` for checking the segmentation. The ring feature automatically disables
itself when the selected path has fewer than the configured 160 points.

The ideal competition path generated from the field sketch is installed at
`share/lio_hybird/paths/path_competition_ideal.txt`. Its optional eighth CSV column is an
explicit travel direction (`1` forward, `-1` reverse). Legacy seven-column recorded paths remain
supported and have their direction inferred from displacement and quaternion heading. Direction
changes are hard cusps: pursuit and progress searches cannot cross them, and the controller stops
before changing the velocity sign. The stop is latched from the speed-dependent braking distance;
overshooting the positional tolerance cannot accidentally re-enable the old direction.

For a direction-only test in a physically clear area, append
`enable_obstacle_avoidance:=false`. This bypasses lidar/vision avoidance and the fresh-scan gate, but
does not bypass odometry, path, curvature, or keepout checks. Never use it for a competition run.

The competition frame is anchored at the field bottom-left with `+x` right and `+y` up. The configured
vehicle start is `(0.54 m, 0.30 m, yaw 0)`, with the nose pointing right. Recorded paths and the keepout
mask therefore stay in one persistent metric frame and `align_path_to_start` is disabled. Start the
LIO at this physical departure pose before any task motion.

Useful topics:

```bash
ros2 topic echo /lio_hybird/state
ros2 topic echo /lio_hybird/odom
ros2 topic echo /cmd_vel
ros2 topic hz /lio_hybird/odom
ros2 topic hz /scan
```

`/lio_hybird/path` is spatially downsampled and published at no more than 5 Hz. It is an RViz diagnostic,
not a control input; publishing the complete growing path at the 20 Hz IMU rate previously consumed
unnecessary serialization, DDS and rosbag bandwidth. The localization node also prints cumulative
NDT acceptance, ICP fallback, rejection and scan-matching timing statistics every five seconds while
the vehicle is moving.

## Visualization and tuning

The package installs an RViz2 layout with fixed frame `lio_hybird_odom`. On a machine with a display, append
`rviz:=true` to the main launch command. From the networked PC, open the source-tree configuration
directly so RViz runs locally while the nodes remain on the vehicle:

```bash
source /opt/ros/humble/setup.bash
rviz2 -d /home/saioog/remote_dev_ws/src/origincar/lio_hybird/rviz/lio_hybird.rviz
```

The display uses this legend:

- grey/black map: white driving area and black keepout area from `/lio_hybird/keepout_map`;
- white NDT map cells: recent keyframe submap from `/lio_hybird/submap`;
- cyan points: raw `/scan`, including the outer paper wall;
- magenta points: `/lio_hybird/obstacles` after the known outer-wall band is removed;
- green line: recorded `/lio_hybird/tracking_path`;
- red line: estimated `/lio_hybird/path` actually driven by the vehicle;
- yellow line: currently selected `/lio_hybird/local_plan` during avoidance;
- yellow pose arrow: `/lio_hybird/lookahead` target;
- vehicle box, cyan command arrow and status text: `/lio_hybird/debug_markers`;
- red sphere: the exact lidar point that triggered the current avoidance cycle.

In the upper ring, the vehicle status text additionally shows `conf`, `steer`, `lateral`, and the
current nine-sample footprint `black` fraction. Normal control remains `TRACK_FORWARD` or
`TRACK_REVERSE`; `YLOC src=RGBD` reports a valid metric localization observation.
`RING_VISION_RECOVERY_BLACK` means vision only authorized a bounded mask-conflict recovery, not that
it supplied steering. `RING_GEOMETRIC_RECOVERY` is the corresponding path-and-mask-only recovery.
`TRACK_NO_WHITE_TRAJECTORY` means neither the strict path nor a bounded ring recovery could be proven
safe.

`/lio_hybird/ring_vision` is available before motion for calibration: `vector.x` is the
ROS-signed visual steering error in radians, `vector.y` is normalized near-field lateral offset
(image right positive), and `vector.z` is confidence from zero to one.

During avoidance, `local_plan` is the exact trajectory selected by the sampler; the state suffix
`_WHITE` or `_BLACK` reports whether it remains white or temporarily enters black.

The bounded reverse escape implementation is enabled in the competition profile. Keep the confirmed
lidar-point filter enabled with it; otherwise a phantom point can command an unnecessary reverse
manoeuvre.

## First validation order

1. Run only `submap_lio_node`, push and rotate the vehicle slowly, and compare `/lio_hybird/path` with the
   wheel path. Do not enable autonomous commands until NDT keyframes appear and rejection is rare.
2. Put the vehicle on blocks or cap speeds at `0.20 m/s`; verify forward and reverse command signs.
3. Follow a path without obstacles and measure lateral error.
4. Test one cone at low speed using lidar only, then vision only, then both.
5. Increase `max_forward_speed` and `max_reverse_speed` only after logging tracking error, ICP RMSE,
   command saturation and stopping distance.

`max_forward_speed` and `max_reverse_speed` are chassis limits, not unconditional commands. At each
control step the controller chooses the highest feasible speed allowed by curvature, lateral
acceleration, yaw-rate and remaining braking distance.
