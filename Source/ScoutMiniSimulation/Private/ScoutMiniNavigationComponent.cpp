#include "ScoutMiniNavigationComponent.h"

#include "AI/Navigation/NavigationTypes.h"
#include "DrawDebugHelpers.h"
#include "NavFilters/NavigationQueryFilter.h"
#include "NavigationData.h"
#include "NavigationSystem.h"

UScoutMiniNavigationComponent::UScoutMiniNavigationComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UScoutMiniNavigationComponent::TickComponent(const float DeltaTime, const ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    DrawPathDebug();
}

bool UScoutMiniNavigationComponent::FindPathToLocation(const FVector& GoalWorldLocation)
{
    const AActor* Owner = GetOwner();
    if (!Owner)
    {
        return FinishQuery(EScoutMiniPathStatus::InvalidInput, TEXT("navigation component has no owner"));
    }

    return FindPathBetweenLocations(Owner->GetActorLocation(), GoalWorldLocation);
}

bool UScoutMiniNavigationComponent::FindPathBetweenLocations(
    const FVector& StartWorldLocation, const FVector& GoalWorldLocation)
{
    PathPoints.Reset();
    PathLengthMetres = 0.0f;
    bIsPartialPath = false;
    ProjectedStart = FVector::ZeroVector;
    ProjectedGoal = FVector::ZeroVector;

    if (StartWorldLocation.ContainsNaN() || GoalWorldLocation.ContainsNaN())
    {
        return FinishQuery(EScoutMiniPathStatus::InvalidInput, TEXT("start or goal contains a non-finite value"));
    }

    UWorld* World = GetWorld();
    UNavigationSystemV1* NavSystem = World
        ? FNavigationSystem::GetCurrent<UNavigationSystemV1>(World)
        : nullptr;
    if (!NavSystem)
    {
        return FinishQuery(EScoutMiniPathStatus::NavigationUnavailable,
            TEXT("world has no NavigationSystemV1"));
    }

    const FNavAgentProperties AgentProperties(
        FMath::Max(AgentRadius, 1.0f), FMath::Max(AgentHeight, 1.0f));
    const ANavigationData* NavData = NavSystem->GetNavDataForProps(
        AgentProperties, StartWorldLocation, ProjectionExtent);
    if (!NavData)
    {
        NavData = NavSystem->GetDefaultNavDataInstance(FNavigationSystem::DontCreate);
        if (NavData && !bLoggedDefaultNavDataFallback)
        {
            UE_LOG(LogTemp, Warning,
                TEXT("ScoutMiniNavigation: no NavData matched agent radius %.1f cm / height %.1f cm; using default NavData %s."),
                AgentRadius, AgentHeight, *NavData->GetName());
            bLoggedDefaultNavDataFallback = true;
        }
    }
    if (!NavData)
    {
        return FinishQuery(EScoutMiniPathStatus::NavigationUnavailable,
            TEXT("no navigation data is available; add a NavMeshBoundsVolume"));
    }

    const FSharedConstNavQueryFilter QueryFilter = UNavigationQueryFilter::GetQueryFilter(
        *NavData, GetOwner(), QueryFilterClass);
    FNavLocation StartOnNavMesh;
    if (!NavSystem->ProjectPointToNavigation(
        StartWorldLocation, StartOnNavMesh, ProjectionExtent, NavData, QueryFilter))
    {
        return FinishQuery(EScoutMiniPathStatus::StartProjectionFailed,
            TEXT("start could not be projected to the NavMesh"));
    }

    FNavLocation GoalOnNavMesh;
    if (!NavSystem->ProjectPointToNavigation(
        GoalWorldLocation, GoalOnNavMesh, ProjectionExtent, NavData, QueryFilter))
    {
        ProjectedStart = StartOnNavMesh.Location;
        return FinishQuery(EScoutMiniPathStatus::GoalProjectionFailed,
            TEXT("goal could not be projected to the NavMesh"));
    }

    ProjectedStart = StartOnNavMesh.Location;
    ProjectedGoal = GoalOnNavMesh.Location;

    FPathFindingQuery Query(GetOwner(), *NavData, ProjectedStart, ProjectedGoal, QueryFilter);
    Query.SetAllowPartialPaths(bAllowPartialPath);
    Query.SetNavAgentProperties(AgentProperties);
    const FPathFindingResult Result = NavSystem->FindPathSync(
        AgentProperties, MoveTemp(Query), EPathFindingMode::Regular);
    if (!Result.IsSuccessful() || !Result.Path.IsValid())
    {
        return FinishQuery(EScoutMiniPathStatus::PathNotFound, TEXT("NavMesh path query failed"));
    }

    bIsPartialPath = Result.Path->IsPartial();
    if (bIsPartialPath && !bAllowPartialPath)
    {
        return FinishQuery(EScoutMiniPathStatus::PathNotFound,
            TEXT("query produced a partial path while partial paths are disabled"));
    }

    const TArray<FNavPathPoint>& NavPathPoints = Result.Path->GetPathPoints();
    PathPoints.Reserve(NavPathPoints.Num());
    for (const FNavPathPoint& Point : NavPathPoints)
    {
        if (!Point.Location.ContainsNaN())
        {
            PathPoints.Add(Point.Location);
        }
    }
    if (PathPoints.Num() < 2)
    {
        PathPoints.Reset();
        return FinishQuery(EScoutMiniPathStatus::PathNotFound,
            TEXT("path contained fewer than two finite points"));
    }

    double LengthCm = 0.0;
    for (int32 Index = 1; Index < PathPoints.Num(); ++Index)
    {
        LengthCm += FVector::Distance(PathPoints[Index - 1], PathPoints[Index]);
    }
    PathLengthMetres = static_cast<float>(LengthCm / 100.0);

    return FinishQuery(bIsPartialPath ? EScoutMiniPathStatus::Partial : EScoutMiniPathStatus::Success);
}

void UScoutMiniNavigationComponent::ClearPath()
{
    PathPoints.Reset();
    PathStatus = EScoutMiniPathStatus::None;
    ProjectedStart = FVector::ZeroVector;
    ProjectedGoal = FVector::ZeroVector;
    PathLengthMetres = 0.0f;
    bIsPartialPath = false;
    ++PathRevision;
}

bool UScoutMiniNavigationComponent::FinishQuery(
    const EScoutMiniPathStatus NewStatus, const TCHAR* Message)
{
    PathStatus = NewStatus;
    ++PathRevision;

    const bool bSucceeded = NewStatus == EScoutMiniPathStatus::Success
        || NewStatus == EScoutMiniPathStatus::Partial;
    if (!bSucceeded)
    {
        PathPoints.Reset();
        PathLengthMetres = 0.0f;
        bIsPartialPath = false;
        if (Message)
        {
            UE_LOG(LogTemp, Warning, TEXT("ScoutMiniNavigation: %s (%s)."),
                Message, GetOwner() ? *GetOwner()->GetName() : TEXT("no owner"));
        }
    }

    OnPathReady.Broadcast(PathStatus, PathPoints);
    return bSucceeded;
}

void UScoutMiniNavigationComponent::DrawPathDebug() const
{
    UWorld* World = GetWorld();
    if (!bDrawPath || !World || PathPoints.Num() < 2)
    {
        return;
    }

    const FVector Offset(0.0f, 0.0f, DebugPathHeightOffset);
    for (int32 Index = 1; Index < PathPoints.Num(); ++Index)
    {
        DrawDebugLine(World, PathPoints[Index - 1] + Offset, PathPoints[Index] + Offset,
            DebugPathColor, false, 0.0f, 0, DebugPathThickness);
    }

    DrawDebugSphere(World, PathPoints[0] + Offset, 8.0f, 12, FColor::Green, false, 0.0f);
    DrawDebugSphere(World, PathPoints.Last() + Offset, 8.0f, 12,
        bIsPartialPath ? FColor::Yellow : FColor::Red, false, 0.0f);
}
