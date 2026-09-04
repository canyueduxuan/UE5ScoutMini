#include "ScoutMiniPathFollowerComponent.h"

#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"

namespace
{
    constexpr float MinimumSegmentLengthCm = 1.0f;

    FVector Flattened(const FVector& Value)
    {
        return FVector(Value.X, Value.Y, 0.0f);
    }
}

UScoutMiniPathFollowerComponent::UScoutMiniPathFollowerComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UScoutMiniPathFollowerComponent::BeginPlay()
{
    Super::BeginPlay();
    AActor* Owner = GetOwner();
    Movement = Owner ? Owner->FindComponentByClass<UScoutMiniMovementComponent>() : nullptr;
    Navigation = Owner ? Owner->FindComponentByClass<UScoutMiniNavigationComponent>() : nullptr;

    if (Movement)
    {
        // The follower must publish this frame's target before movement consumes it.
        Movement->AddTickPrerequisiteComponent(this);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("ScoutMiniPathFollower: %s has no movement component."),
            Owner ? *Owner->GetName() : TEXT("no owner"));
    }

    if (Navigation)
    {
        Navigation->OnPathReady.AddDynamic(this, &UScoutMiniPathFollowerComponent::HandleNavigationPathReady);
    }
}

void UScoutMiniPathFollowerComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (Navigation)
    {
        Navigation->OnPathReady.RemoveDynamic(this, &UScoutMiniPathFollowerComponent::HandleNavigationPathReady);
    }
    if (Movement && Movement->HasCommandAuthority(this))
    {
        Movement->StopFrom(this);
        Movement->ReleaseCommandAuthority(this);
    }
    Super::EndPlay(EndPlayReason);
}

void UScoutMiniPathFollowerComponent::TickComponent(const float DeltaTime, const ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if ((IsActivelyFollowing() || FollowStatus == EScoutMiniPathFollowStatus::Paused)
        && bFollowingNavigationPath && Navigation
        && Navigation->GetPathRevision() != FollowedPathRevision)
    {
        if (bAutoAcceptPathUpdates && IsNavigationPathUsable())
        {
            if (!LoadPath(Navigation->GetPathPoints()))
            {
                FinishFollowing(EScoutMiniPathFollowStatus::PathInvalid, true);
            }
            else
            {
                FollowedPathRevision = Navigation->GetPathRevision();
            }
        }
        else
        {
            FinishFollowing(EScoutMiniPathFollowStatus::PathInvalid, true);
        }
    }

    if (IsActivelyFollowing())
    {
        UpdateFollowing(DeltaTime);
    }
    DrawFollowingDebug();
}

bool UScoutMiniPathFollowerComponent::StartFollowingCurrentPath()
{
    if (!Navigation || !IsNavigationPathUsable())
    {
        if (IsActivelyFollowing() || FollowStatus == EScoutMiniPathFollowStatus::Paused)
        {
            FinishFollowing(EScoutMiniPathFollowStatus::PathInvalid, true);
        }
        else
        {
            SetStatus(EScoutMiniPathFollowStatus::PathInvalid);
            OnFollowingFailed.Broadcast(FollowStatus);
        }
        return false;
    }

    return BeginFollowingPath(
        Navigation->GetPathPoints(), true, Navigation->GetPathRevision());
}

bool UScoutMiniPathFollowerComponent::StartFollowingPath(const TArray<FVector>& WorldPathPoints)
{
    return BeginFollowingPath(WorldPathPoints, false, -1);
}

bool UScoutMiniPathFollowerComponent::BeginFollowingPath(
    const TArray<FVector>& WorldPathPoints, const bool bFromNavigation, const int64 NavigationRevision)
{
    if (!Movement || !GetOwner())
    {
        SetStatus(EScoutMiniPathFollowStatus::ControlUnavailable);
        OnFollowingFailed.Broadcast(FollowStatus);
        return false;
    }

    TArray<FVector> ValidatedPath;
    ValidatedPath.Reserve(WorldPathPoints.Num());
    for (const FVector& Point : WorldPathPoints)
    {
        if (Point.ContainsNaN()) continue;
        if (ValidatedPath.Num() == 0
            || FVector::DistSquared2D(ValidatedPath.Last(), Point)
                >= FMath::Square(MinimumSegmentLengthCm))
        {
            ValidatedPath.Add(Point);
        }
    }
    if (ValidatedPath.Num() < 2)
    {
        if (IsActivelyFollowing() || FollowStatus == EScoutMiniPathFollowStatus::Paused)
        {
            FinishFollowing(EScoutMiniPathFollowStatus::PathInvalid, true);
        }
        else
        {
            SetStatus(EScoutMiniPathFollowStatus::PathInvalid);
            OnFollowingFailed.Broadcast(FollowStatus);
        }
        return false;
    }

    const bool bAlreadyOwnsControl = Movement->HasCommandAuthority(this);
    if (!bAlreadyOwnsControl)
    {
        if (!bTakeProgrammaticControlOnStart
            && Movement->ControlMode != EScoutMiniControlMode::Programmatic)
        {
            SetStatus(EScoutMiniPathFollowStatus::ControlUnavailable);
            OnFollowingFailed.Broadcast(FollowStatus);
            return false;
        }
        if (!Movement->AcquireCommandAuthority(this, bForceCommandAuthority))
        {
            SetStatus(EScoutMiniPathFollowStatus::ControlUnavailable);
            OnFollowingFailed.Broadcast(FollowStatus);
            return false;
        }

        PreviousControlMode = Movement->ControlMode;
        bSavedControlMode = true;
        if (bTakeProgrammaticControlOnStart
            && Movement->ControlMode != EScoutMiniControlMode::Programmatic)
        {
            Movement->SetControlMode(EScoutMiniControlMode::Programmatic);
        }
    }

    if (!LoadPath(ValidatedPath))
    {
        if (!bAlreadyOwnsControl)
        {
            Movement->ReleaseCommandAuthority(this);
        }
        SetStatus(EScoutMiniPathFollowStatus::PathInvalid);
        OnFollowingFailed.Broadcast(FollowStatus);
        return false;
    }

    bFollowingNavigationPath = bFromNavigation;
    FollowedPathRevision = NavigationRevision;
    StuckElapsedSeconds = 0.0f;
    CommandLinearVelocity = 0.0f;
    CommandAngularVelocity = 0.0f;
    SetStatus(EScoutMiniPathFollowStatus::Following);
    OnFollowingStarted.Broadcast();
    return true;
}

bool UScoutMiniPathFollowerComponent::LoadPath(const TArray<FVector>& WorldPathPoints)
{
    FollowPath.Reset(WorldPathPoints.Num());
    for (const FVector& Point : WorldPathPoints)
    {
        if (Point.ContainsNaN()) continue;
        if (FollowPath.Num() == 0
            || FVector::DistSquared2D(FollowPath.Last(), Point)
                >= FMath::Square(MinimumSegmentLengthCm))
        {
            FollowPath.Add(Point);
        }
    }
    if (FollowPath.Num() < 2)
    {
        FollowPath.Reset();
        CumulativeDistanceCm.Reset();
        return false;
    }

    CumulativeDistanceCm.SetNumZeroed(FollowPath.Num());
    for (int32 Index = 1; Index < FollowPath.Num(); ++Index)
    {
        CumulativeDistanceCm[Index] = CumulativeDistanceCm[Index - 1]
            + FVector::Dist2D(FollowPath[Index - 1], FollowPath[Index]);
    }
    CurrentSegmentIndex = 0;
    PathProgressMetres = 0.0f;
    CrossTrackErrorMetres = 0.0f;
    DistanceToGoalMetres = FVector::Dist2D(GetOwner()->GetActorLocation(), FollowPath.Last()) / 100.0f;
    ClosestPathPoint = FollowPath[0];
    CurrentLookaheadPoint = FollowPath[0];
    return CumulativeDistanceCm.Last() >= MinimumSegmentLengthCm;
}

void UScoutMiniPathFollowerComponent::StopFollowing()
{
    FinishFollowing(EScoutMiniPathFollowStatus::Idle, true);
}

void UScoutMiniPathFollowerComponent::PauseFollowing()
{
    if (!IsActivelyFollowing()) return;
    SendCommand(0.0f, 0.0f);
    SetStatus(EScoutMiniPathFollowStatus::Paused);
}

bool UScoutMiniPathFollowerComponent::ResumeFollowing()
{
    if (FollowStatus != EScoutMiniPathFollowStatus::Paused || !Movement
        || !Movement->HasCommandAuthority(this) || FollowPath.Num() < 2)
    {
        return false;
    }
    StuckElapsedSeconds = 0.0f;
    SetStatus(EScoutMiniPathFollowStatus::Following);
    return true;
}

void UScoutMiniPathFollowerComponent::HandleNavigationPathReady(
    const EScoutMiniPathStatus Status, const TArray<FVector>& PathPoints)
{
    const bool bUsable = Status == EScoutMiniPathStatus::Success
        || (Status == EScoutMiniPathStatus::Partial && bAcceptPartialNavigationPaths);

    if ((IsActivelyFollowing() || FollowStatus == EScoutMiniPathFollowStatus::Paused)
        && bFollowingNavigationPath)
    {
        if (bAutoAcceptPathUpdates && bUsable && LoadPath(PathPoints))
        {
            FollowedPathRevision = Navigation ? Navigation->GetPathRevision() : -1;
            if (FollowStatus != EScoutMiniPathFollowStatus::Paused)
            {
                SetStatus(EScoutMiniPathFollowStatus::Following);
            }
        }
        else
        {
            FinishFollowing(EScoutMiniPathFollowStatus::PathInvalid, true);
        }
        return;
    }

    if (bAutoFollowNewNavigationPath && bUsable)
    {
        BeginFollowingPath(PathPoints, true, Navigation ? Navigation->GetPathRevision() : -1);
    }
}

bool UScoutMiniPathFollowerComponent::UpdateClosestPathPosition(const FVector& VehicleLocation)
{
    if (FollowPath.Num() < 2 || CumulativeDistanceCm.Num() != FollowPath.Num()) return false;

    const int32 FirstSegment = FMath::Clamp(CurrentSegmentIndex - 1, 0, FollowPath.Num() - 2);
    float BestDistanceSquared = TNumericLimits<float>::Max();
    float BestProgressCm = 0.0f;
    int32 BestSegment = INDEX_NONE;
    const FVector Vehicle2D = Flattened(VehicleLocation);

    const int32 LastSegmentExclusive = FMath::Min(
        FollowPath.Num() - 1, CurrentSegmentIndex + FMath::Max(MaximumSegmentSearchAhead, 1) + 1);
    for (int32 Index = FirstSegment; Index < LastSegmentExclusive; ++Index)
    {
        const FVector Start = Flattened(FollowPath[Index]);
        const FVector End = Flattened(FollowPath[Index + 1]);
        const FVector Segment = End - Start;
        const float LengthSquared = Segment.SizeSquared();
        if (LengthSquared < FMath::Square(MinimumSegmentLengthCm)) continue;

        const float Alpha = FMath::Clamp(
            FVector::DotProduct(Vehicle2D - Start, Segment) / LengthSquared, 0.0f, 1.0f);
        const FVector Candidate2D = Start + Segment * Alpha;
        const float DistanceSquared = FVector::DistSquared(Vehicle2D, Candidate2D);
        const float CandidateProgressCm = CumulativeDistanceCm[Index]
            + FMath::Sqrt(LengthSquared) * Alpha;

        // At self-intersections, prefer the forward candidate when distances tie.
        if (DistanceSquared < BestDistanceSquared - KINDA_SMALL_NUMBER
            || (FMath::IsNearlyEqual(DistanceSquared, BestDistanceSquared)
                && CandidateProgressCm > BestProgressCm))
        {
            BestDistanceSquared = DistanceSquared;
            BestProgressCm = CandidateProgressCm;
            BestSegment = Index;
        }
    }

    if (BestSegment == INDEX_NONE) return false;
    CurrentSegmentIndex = FMath::Max(CurrentSegmentIndex, BestSegment);
    BestProgressCm = FMath::Max(BestProgressCm, PathProgressMetres * 100.0f);
    ClosestPathPoint = FindPointAtProgress(BestProgressCm);
    PathProgressMetres = BestProgressCm / 100.0f;
    CrossTrackErrorMetres = FVector::Dist2D(VehicleLocation, ClosestPathPoint) / 100.0f;
    return true;
}

FVector UScoutMiniPathFollowerComponent::FindPointAtProgress(const float ProgressCm) const
{
    if (FollowPath.Num() == 0 || CumulativeDistanceCm.Num() != FollowPath.Num())
    {
        return FVector::ZeroVector;
    }
    if (ProgressCm <= 0.0f) return FollowPath[0];
    if (ProgressCm >= CumulativeDistanceCm.Last()) return FollowPath.Last();

    for (int32 Index = FMath::Clamp(CurrentSegmentIndex, 0, FollowPath.Num() - 2);
        Index < FollowPath.Num() - 1; ++Index)
    {
        if (ProgressCm <= CumulativeDistanceCm[Index + 1])
        {
            const float SegmentLength = CumulativeDistanceCm[Index + 1] - CumulativeDistanceCm[Index];
            const float Alpha = SegmentLength > KINDA_SMALL_NUMBER
                ? (ProgressCm - CumulativeDistanceCm[Index]) / SegmentLength : 0.0f;
            return FMath::Lerp(FollowPath[Index], FollowPath[Index + 1], Alpha);
        }
    }
    return FollowPath.Last();
}

int32 UScoutMiniPathFollowerComponent::FindUpcomingCorner(float& OutDistanceMetres) const
{
    OutDistanceMetres = TNumericLimits<float>::Max();
    if (FollowPath.Num() < 3) return INDEX_NONE;

    const float ProgressCm = PathProgressMetres * 100.0f;
    const float SearchDistanceCm = FMath::Max(CornerSlowdownDistance, MaximumLookahead) * 100.0f;
    for (int32 Vertex = FMath::Max(CurrentSegmentIndex + 1, 1);
        Vertex < FollowPath.Num() - 1; ++Vertex)
    {
        const float DistanceCm = CumulativeDistanceCm[Vertex] - ProgressCm;
        if (DistanceCm < -KINDA_SMALL_NUMBER) continue;
        if (DistanceCm > SearchDistanceCm) break;

        const FVector Incoming = Flattened(FollowPath[Vertex] - FollowPath[Vertex - 1]).GetSafeNormal();
        const FVector Outgoing = Flattened(FollowPath[Vertex + 1] - FollowPath[Vertex]).GetSafeNormal();
        if (Incoming.IsNearlyZero() || Outgoing.IsNearlyZero()) continue;
        const float AngleDegrees = FMath::RadiansToDegrees(FMath::Acos(
            FMath::Clamp(FVector::DotProduct(Incoming, Outgoing), -1.0f, 1.0f)));
        if (AngleDegrees >= CornerAngleThresholdDegrees)
        {
            OutDistanceMetres = FMath::Max(0.0f, DistanceCm / 100.0f);
            return Vertex;
        }
    }
    return INDEX_NONE;
}

void UScoutMiniPathFollowerComponent::UpdateFollowing(const float DeltaTime)
{
    AActor* Owner = GetOwner();
    if (!Owner || !Movement || !Movement->HasCommandAuthority(this)
        || Movement->ControlMode != EScoutMiniControlMode::Programmatic)
    {
        FinishFollowing(EScoutMiniPathFollowStatus::ControlUnavailable, true);
        return;
    }

    const FVector VehicleLocation = Owner->GetActorLocation();
    if (!UpdateClosestPathPosition(VehicleLocation))
    {
        FinishFollowing(EScoutMiniPathFollowStatus::PathInvalid, true);
        return;
    }
    if (CrossTrackErrorMetres > FMath::Max(MaximumPathDeviation, 0.05f))
    {
        FinishFollowing(EScoutMiniPathFollowStatus::OffPath, true);
        return;
    }

    DistanceToGoalMetres = FVector::Dist2D(VehicleLocation, FollowPath.Last()) / 100.0f;
    if (DistanceToGoalMetres <= FMath::Max(GoalTolerance, 0.01f))
    {
        FinishFollowing(EScoutMiniPathFollowStatus::ReachedGoal, true);
        return;
    }

    const float ActualSpeed = FMath::Abs(Movement->GetLinearVelocity());
    const float LookaheadMetres = FMath::Clamp(
        FMath::Max(MinimumLookahead, 0.05f) + FMath::Max(LookaheadTime, 0.0f) * ActualSpeed,
        FMath::Max(MinimumLookahead, 0.05f),
        FMath::Max(MaximumLookahead, MinimumLookahead));
    float TargetProgressCm = PathProgressMetres * 100.0f + LookaheadMetres * 100.0f;

    float CornerDistanceMetres = TNumericLimits<float>::Max();
    const int32 CornerIndex = FindUpcomingCorner(CornerDistanceMetres);
    if (CornerIndex != INDEX_NONE && CornerDistanceMetres > FMath::Max(CornerTolerance, 0.01f))
    {
        TargetProgressCm = FMath::Min(TargetProgressCm, CumulativeDistanceCm[CornerIndex]);
    }
    CurrentLookaheadPoint = FindPointAtProgress(TargetProgressCm);

    FVector Forward = Flattened(Owner->GetActorForwardVector()).GetSafeNormal();
    FVector Right = Flattened(Owner->GetActorRightVector()).GetSafeNormal();
    if (Forward.IsNearlyZero() || Right.IsNearlyZero())
    {
        FinishFollowing(EScoutMiniPathFollowStatus::PathInvalid, true);
        return;
    }

    const FVector TargetDelta = Flattened(CurrentLookaheadPoint - VehicleLocation);
    const float LocalXMetres = FVector::DotProduct(TargetDelta, Forward) / 100.0f;
    const float LocalYMetres = FVector::DotProduct(TargetDelta, Right) / 100.0f;
    const float TargetDistanceSquaredMetres = FMath::Square(LocalXMetres) + FMath::Square(LocalYMetres);
    if (TargetDistanceSquaredMetres <= KINDA_SMALL_NUMBER)
    {
        FinishFollowing(EScoutMiniPathFollowStatus::PathInvalid, true);
        return;
    }

    const float HeadingError = FMath::Atan2(LocalYMetres, LocalXMetres);
    const float AbsHeadingDegrees = FMath::Abs(FMath::RadiansToDegrees(HeadingError));
    const bool bShouldAlign = FollowStatus == EScoutMiniPathFollowStatus::Aligning
        ? AbsHeadingDegrees > FMath::Min(AlignExitAngleDegrees, AlignEnterAngleDegrees)
        : AbsHeadingDegrees > FMath::Max(AlignEnterAngleDegrees, AlignExitAngleDegrees);

    if (bShouldAlign)
    {
        SetStatus(EScoutMiniPathFollowStatus::Aligning);
        float AngularCommand = FMath::Clamp(
            FMath::Max(HeadingGain, 0.0f) * HeadingError,
            -FMath::Min(MaximumAngularSpeed, Movement->MaxAngularSpeed),
            FMath::Min(MaximumAngularSpeed, Movement->MaxAngularSpeed));
        if (FMath::Abs(AngularCommand) < MinimumAlignAngularSpeed && !FMath::IsNearlyZero(HeadingError))
        {
            AngularCommand = FMath::Sign(HeadingError) * FMath::Min(
                MinimumAlignAngularSpeed, FMath::Min(MaximumAngularSpeed, Movement->MaxAngularSpeed));
        }
        SendCommand(0.0f, AngularCommand);
        StuckElapsedSeconds = 0.0f;
        return;
    }

    SetStatus(EScoutMiniPathFollowStatus::Following);
    const float Curvature = 2.0f * LocalYMetres / TargetDistanceSquaredMetres;
    const float MaxLinear = FMath::Max(0.0f, FMath::Min(MaximumLinearSpeed, Movement->MaxLinearSpeed));
    const float MaxAngular = FMath::Max(0.0f, FMath::Min(MaximumAngularSpeed, Movement->MaxAngularSpeed));
    const float CurvatureLimited = MaxLinear
        / (1.0f + FMath::Max(CurvatureSpeedGain, 0.0f) * FMath::Abs(Curvature));
    const float GoalRatio = FMath::Clamp(
        DistanceToGoalMetres / FMath::Max(GoalSlowdownDistance, GoalTolerance), 0.0f, 1.0f);
    float RegulatedSpeed = FMath::Min(MaxLinear, CurvatureLimited);
    RegulatedSpeed = FMath::Min(RegulatedSpeed, MaxLinear * GoalRatio);
    if (CornerIndex != INDEX_NONE)
    {
        const float CornerRatio = FMath::Clamp(
            CornerDistanceMetres / FMath::Max(CornerSlowdownDistance, CornerTolerance), 0.0f, 1.0f);
        RegulatedSpeed = FMath::Min(RegulatedSpeed, MaxLinear * CornerRatio);
    }
    if (RegulatedSpeed > 0.0f)
    {
        RegulatedSpeed = FMath::Max(
            FMath::Min(MinimumApproachSpeed, MaxLinear), RegulatedSpeed);
    }
    RegulatedSpeed *= FMath::Clamp(FMath::Cos(HeadingError), 0.0f, 1.0f);

    const float AngularCommand = FMath::Clamp(RegulatedSpeed * Curvature, -MaxAngular, MaxAngular);
    SendCommand(RegulatedSpeed, AngularCommand);

    if (FMath::Abs(CommandLinearVelocity) > FMath::Max(StuckCommandSpeed, 0.0f)
        && ActualSpeed < FMath::Max(StuckActualSpeed, 0.0f))
    {
        StuckElapsedSeconds += FMath::Max(DeltaTime, 0.0f);
        if (StuckTimeout > 0.0f && StuckElapsedSeconds >= StuckTimeout)
        {
            FinishFollowing(EScoutMiniPathFollowStatus::Stuck, true);
        }
    }
    else
    {
        StuckElapsedSeconds = 0.0f;
    }
}

void UScoutMiniPathFollowerComponent::SendCommand(
    const float LinearVelocity, const float AngularVelocity)
{
    CommandLinearVelocity = LinearVelocity;
    CommandAngularVelocity = AngularVelocity;
    if (Movement && !Movement->SetVelocityCommandFrom(this, LinearVelocity, AngularVelocity))
    {
        CommandLinearVelocity = 0.0f;
        CommandAngularVelocity = 0.0f;
    }
}

void UScoutMiniPathFollowerComponent::SetStatus(const EScoutMiniPathFollowStatus NewStatus)
{
    if (FollowStatus == NewStatus) return;
    FollowStatus = NewStatus;
    OnStatusChanged.Broadcast(FollowStatus);
}

void UScoutMiniPathFollowerComponent::FinishFollowing(
    const EScoutMiniPathFollowStatus TerminalStatus, const bool bRestoreControlMode)
{
    if (Movement && Movement->HasCommandAuthority(this))
    {
        Movement->StopFrom(this);
        Movement->ReleaseCommandAuthority(this);
    }
    CommandLinearVelocity = 0.0f;
    CommandAngularVelocity = 0.0f;
    StuckElapsedSeconds = 0.0f;
    bFollowingNavigationPath = false;
    FollowedPathRevision = -1;

    if (bRestoreControlMode && bRestorePreviousControlModeOnStop && bSavedControlMode
        && Movement && Movement->ControlMode != PreviousControlMode)
    {
        Movement->SetControlMode(PreviousControlMode);
    }
    bSavedControlMode = false;
    SetStatus(TerminalStatus);

    if (TerminalStatus == EScoutMiniPathFollowStatus::ReachedGoal)
    {
        OnGoalReached.Broadcast(TerminalStatus);
    }
    else if (TerminalStatus != EScoutMiniPathFollowStatus::Idle
        && TerminalStatus != EScoutMiniPathFollowStatus::Paused)
    {
        OnFollowingFailed.Broadcast(TerminalStatus);
    }
}

bool UScoutMiniPathFollowerComponent::IsActivelyFollowing() const
{
    return FollowStatus == EScoutMiniPathFollowStatus::Following
        || FollowStatus == EScoutMiniPathFollowStatus::Aligning;
}

bool UScoutMiniPathFollowerComponent::IsNavigationPathUsable() const
{
    if (!Navigation) return false;
    const EScoutMiniPathStatus Status = Navigation->GetPathStatus();
    return Status == EScoutMiniPathStatus::Success
        || (Status == EScoutMiniPathStatus::Partial && bAcceptPartialNavigationPaths);
}

void UScoutMiniPathFollowerComponent::DrawFollowingDebug() const
{
    UWorld* World = GetWorld();
    if (!bDrawDebug || !World || FollowPath.Num() < 2) return;

    const FVector Offset(0.0f, 0.0f, DebugHeightOffset);
    DrawDebugSphere(World, CurrentLookaheadPoint + Offset, 10.0f, 12,
        FColor::Magenta, false, 0.0f);
    DrawDebugPoint(World, ClosestPathPoint + Offset, 10.0f, FColor::White, false, 0.0f);
    if (GetOwner())
    {
        DrawDebugLine(World, GetOwner()->GetActorLocation(), ClosestPathPoint + Offset,
            FColor::White, false, 0.0f, 0, 1.0f);
    }
}
