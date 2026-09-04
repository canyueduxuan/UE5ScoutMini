# Scout Mini Path Following

`ScoutMiniPathFollowerComponent` follows UE world-space path points with a
regulated pure-pursuit controller for the skid-steer base. It outputs linear
velocity in m/s and UE yaw rate in rad/s; it never teleports the pawn.

## Basic use

1. Query a complete path with the pawn's `Navigation` component.
2. Call `Start Following Current Path` on `PathFollower`.
3. Observe `On Goal Reached`, `On Following Failed`, or `On Status Changed`.

Set `Auto Follow New Navigation Path` to start automatically whenever a usable
navigation path arrives. It is disabled by default so a path query cannot make
the vehicle move unexpectedly.

The follower normally switches the movement component to Programmatic while it
runs and restores the previous mode when it stops. `Force Command Authority` is
disabled by default; consequently an active ROS controller prevents path
following from starting instead of having its commands silently overwritten.

## Controller behavior

- A speed-dependent lookahead selects a point by path arc length.
- Pure pursuit converts that point to linear and angular velocity.
- Large heading errors cause an in-place skid-steer alignment.
- Sharp NavMesh corners shorten lookahead and reduce speed to avoid corner cuts.
- Curvature and remaining goal distance regulate forward speed.
- Path progress is monotonic and nearest-segment search is bounded to reduce
  jumps at path self-intersections.
- Excessive cross-track error or a sustained commanded-but-stationary condition
  stops the vehicle with `OffPath` or `Stuck` status.

All path geometry remains in UE centimetres. Public distances and velocity
commands use SI units.

## Debug display

- Magenta sphere: current lookahead point.
- White point: closest point on the followed path.
- White line: current horizontal cross-track error.

The cyan line and endpoint spheres belong to `ScoutMiniNavigationComponent`.

## Expert-data integration

The following getters expose the controller labels and tracking state required
by a later trajectory recorder:

- `Get Command Linear Velocity`
- `Get Command Angular Velocity`
- `Get Cross Track Error Metres`
- `Get Distance To Goal Metres`
- `Get Path Progress Metres`
- `Get Current Lookahead Point`

With movement dynamics disabled these are kinematic expert commands. With
dynamics enabled, record them together with measured movement state to retain
the physical tracking response.
