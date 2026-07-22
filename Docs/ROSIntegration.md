# Scout Mini ROSIntegration

Every `ScoutMiniPawn` contains a `ScoutMiniROSComponent` by default. The component
uses ROSIntegration/rosbridge and provides:

- subscription: `/cmd_vel` (`geometry_msgs/Twist`)
- publisher: `/odom` (`nav_msgs/Odometry`)
- publisher: `/tf` (`tf2_msgs/TFMessage`), transform `odom -> base_link`

The project uses `ROSIntegrationGameInstance` and defaults to connection ID 0
(`127.0.0.1:9090`). Connection parameters can be changed with
`ROSBridgeParamOverride` in the level.

## Coordinate convention

The ROS interface follows REP-103: X forward, Y left, Z up and positive yaw to
the left. Unreal positions and rotations are converted before publishing. The
odometry origin is the pawn pose at BeginPlay; `/odom` and `/tf` are generated
from the same pose and timestamp.

## Safety

The movement component is switched to Programmatic control at BeginPlay. A valid
command is limited by the movement component's speed limits. Non-finite commands
are rejected, and the robot stops when `/cmd_vel` has not been received for
`CommandTimeoutSeconds` (0.5 s by default).

## ROS 1 smoke test

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

rostopic echo /odom
rosrun tf tf_echo odom base_link
```

Publishing a positive `angular.z` must turn the robot left.
