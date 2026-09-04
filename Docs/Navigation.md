# Scout Mini UE Navigation Query

Every `ScoutMiniPawn` owns a `ScoutMiniNavigationComponent`. The component only
queries and visualizes NavMesh paths; it never changes the movement control mode
or sends velocity commands.

## Level setup

1. Add a `NavMeshBoundsVolume` that covers the driveable area.
2. Make sure the terrain collision contributes to navigation.
3. Press `P` in the editor viewport to inspect the generated NavMesh.
4. Configure a supported navigation agent close to the component defaults
   (43 cm radius and 30 cm height), or let the component fall back to the
   level's default navigation data.

Use dynamic NavMesh generation only when obstacles or driveable surfaces change
at runtime. Static generation is preferable for fixed dataset environments.

## Blueprint use

Call `Find Path To Location` on the pawn's `Navigation` component. A successful
call stores world-space UE path points in centimetres and broadcasts
`On Path Ready`. `Get Path Status`, `Get Path Points`, `Get Path Length Metres`,
and `Is Partial Path` expose the result.

Partial paths are disabled by default because they do not reach the requested
goal. Enable `Allow Partial Path` only when the caller handles that condition.

`Find Path Between Locations` is provided for offline dataset generation where
the query start does not need to be the pawn's current position.

## C++ use

```cpp
if (ScoutPawn->Navigation->FindPathToLocation(GoalWorldLocation))
{
    const TArray<FVector> Points = ScoutPawn->Navigation->GetPathPoints();
}
```

The stored points remain on the NavMesh surface. `Debug Path Height Offset` is
applied only while drawing and never changes the query result.

## Scope

The returned path is a geometric polyline, not a dynamically feasible vehicle
trajectory. Path smoothing, time parameterization, skid-steer tracking, and
expert state/action recording belong in separate components built on top of this
query result.
