# Scout Mini ROSIntegration

Every `ScoutMiniPawn` contains a `ScoutMiniROSComponent` by default. The component
uses ROSIntegration/rosbridge and provides:

- subscription: `/cmd_vel` (`geometry_msgs/Twist`)
- subscription: `/pos_cmd` (`nav_msgs/Path`), rendered as a green debug line
- subscription: `/trajs_visual_ue` (`std_msgs/Float32MultiArray`), rendered as score-coloured debug points
- publisher: `/odom` (`nav_msgs/Odometry`)
- publisher: `/tf` (`tf2_msgs/TFMessage`), transform `odom -> base_link`

The project uses `ROSIntegrationGameInstance` and defaults to connection ID 0
(`127.0.0.1:9090`). Connection parameters can be changed with
`ROSBridgeParamOverride` in the level.

## Coordinate convention

The ROS interface follows REP-103: X forward, Y left, Z up and positive yaw to
the left. Unreal positions and rotations are converted before publishing.

Enable `bUseAbsoluteWorldOdometry` on `ScoutMiniROSComponent` to convert the pawn's
full absolute UE world transform directly to ROS metres and REP-103 axes. When it is
disabled, the full pose is relative to the pawn transform at BeginPlay. Position,
roll, pitch and yaw are never flattened.

`/odom` and `/tf` are generated from the same pose and timestamp. Twist follows the
YOPO UE consumer convention: measured linear and angular velocities are published in
the ROS world basis. This makes `sqrt(linear.x^2 + linear.y^2)` the world-horizontal
speed consumed by `test_yopo_ros_UE5.py`; `angular.z` has ROS's positive-left sign.

## Safety

ROS communication and vehicle control authority are independent. By default the
ROS component preserves the movement component's configured control mode. In
Manual mode, W/S/A/D controls the vehicle while odometry, TF and visualization
remain active; incoming `/cmd_vel` messages are ignored.

Enable `bTakeControlOnBeginPlay` to switch the movement component to Programmatic
control automatically after a successful ROS connection. Alternatively, call
`SetControlMode` at runtime to switch between Manual and Programmatic control.

In Programmatic mode, valid commands are limited by the movement component's speed
limits. Non-finite commands are rejected, and the robot stops when `/cmd_vel` has
not been received for `CommandTimeoutSeconds` (0.5 s by default). The command
watchdog does not stop or modify Manual input.

## ROS 1 smoke test

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

rostopic echo /odom
rosrun tf tf_echo odom base_link
```

Publishing a positive `angular.z` must turn the robot left.

Planned-path positions use the same coordinates as the published odometry. With
`bUseAbsoluteWorldOdometry` enabled they are converted directly from absolute ROS world
coordinates to UE world coordinates; otherwise they are transformed through the pawn's
BeginPlay odometry origin. The message's existing `camera_init` frame name is not used.

Candidate trajectory arrays contain repeated `[x, y, z, intensity]` float tuples.
The original `/trajs_visual` PointCloud2 remains available for RViz. UE points are
evenly sampled when the array exceeds `MaxCandidatePoints`; stale points are cleared after
`CandidateTimeoutSeconds`. Candidate positions use the same absolute-or-relative
conversion as `/pos_cmd`.
