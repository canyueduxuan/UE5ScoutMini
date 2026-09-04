#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ScoutMiniMovementComponent.h"
#include "ScoutMiniNavigationComponent.h"
#include "ScoutMiniPathFollowerComponent.generated.h"

UENUM(BlueprintType)
enum class EScoutMiniPathFollowStatus : uint8
{
    Idle,
    Aligning,
    Following,
    ReachedGoal,
    Paused,
    PathInvalid,
    OffPath,
    Stuck,
    ControlUnavailable
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FScoutMiniFollowingStartedSignature);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(
    FScoutMiniFollowStatusSignature, EScoutMiniPathFollowStatus, Status);

/** Regulated pure-pursuit controller for the Scout Mini skid-steer base. */
UCLASS(ClassGroup=(Navigation), meta=(BlueprintSpawnableComponent))
class SCOUTMINISIMULATION_API UScoutMiniPathFollowerComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UScoutMiniPathFollowerComponent();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Control")
    bool bTakeProgrammaticControlOnStart = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Control")
    bool bRestorePreviousControlModeOnStop = true;

    /** Explicitly allows StartFollowing to replace another programmatic command owner. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Control")
    bool bForceCommandAuthority = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Path")
    bool bAutoFollowNewNavigationPath = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Path")
    bool bAutoAcceptPathUpdates = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Path")
    bool bAcceptPartialNavigationPaths = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Speed", meta=(ClampMin="0.0", Units="m/s"))
    float MaximumLinearSpeed = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Speed", meta=(ClampMin="0.0"))
    float MaximumAngularSpeed = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Speed", meta=(ClampMin="0.0", Units="m/s"))
    float MinimumApproachSpeed = 0.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Lookahead", meta=(ClampMin="0.05", Units="m"))
    float MinimumLookahead = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Lookahead", meta=(ClampMin="0.05", Units="m"))
    float MaximumLookahead = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Lookahead", meta=(ClampMin="0.0", Units="s"))
    float LookaheadTime = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Regulation", meta=(ClampMin="0.0"))
    float CurvatureSpeedGain = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Regulation", meta=(ClampMin="0.05", Units="m"))
    float GoalSlowdownDistance = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Regulation", meta=(ClampMin="0.01", Units="m"))
    float GoalTolerance = 0.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Alignment", meta=(ClampMin="0.0"))
    float HeadingGain = 2.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Alignment", meta=(ClampMin="0.0", ClampMax="180.0", Units="deg"))
    float AlignEnterAngleDegrees = 45.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Alignment", meta=(ClampMin="0.0", ClampMax="180.0", Units="deg"))
    float AlignExitAngleDegrees = 12.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Alignment", meta=(ClampMin="0.0"))
    float MinimumAlignAngularSpeed = 0.25f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Corners", meta=(ClampMin="0.0", ClampMax="180.0", Units="deg"))
    float CornerAngleThresholdDegrees = 35.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Corners", meta=(ClampMin="0.05", Units="m"))
    float CornerSlowdownDistance = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Corners", meta=(ClampMin="0.01", Units="m"))
    float CornerTolerance = 0.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Safety", meta=(ClampMin="0.05", Units="m"))
    float MaximumPathDeviation = 1.0f;

    /** Limits forward nearest-segment search so nearby self-intersections do not skip large path sections. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Safety", meta=(ClampMin="1"))
    int32 MaximumSegmentSearchAhead = 20;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Safety", meta=(ClampMin="0.0", Units="m/s"))
    float StuckCommandSpeed = 0.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Safety", meta=(ClampMin="0.0", Units="m/s"))
    float StuckActualSpeed = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Safety", meta=(ClampMin="0.0", Units="s"))
    float StuckTimeout = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Debug")
    bool bDrawDebug = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Path Following|Debug", meta=(ClampMin="0.0", Units="cm"))
    float DebugHeightOffset = 10.0f;

    UPROPERTY(BlueprintAssignable, Category="Path Following")
    FScoutMiniFollowingStartedSignature OnFollowingStarted;

    UPROPERTY(BlueprintAssignable, Category="Path Following")
    FScoutMiniFollowStatusSignature OnStatusChanged;

    UPROPERTY(BlueprintAssignable, Category="Path Following")
    FScoutMiniFollowStatusSignature OnGoalReached;

    UPROPERTY(BlueprintAssignable, Category="Path Following")
    FScoutMiniFollowStatusSignature OnFollowingFailed;

    UFUNCTION(BlueprintCallable, Category="Scout Mini|Path Following")
    bool StartFollowingCurrentPath();

    UFUNCTION(BlueprintCallable, Category="Scout Mini|Path Following")
    bool StartFollowingPath(const TArray<FVector>& WorldPathPoints);

    UFUNCTION(BlueprintCallable, Category="Scout Mini|Path Following")
    void StopFollowing();

    UFUNCTION(BlueprintCallable, Category="Scout Mini|Path Following")
    void PauseFollowing();

    UFUNCTION(BlueprintCallable, Category="Scout Mini|Path Following")
    bool ResumeFollowing();

    UFUNCTION(BlueprintPure, Category="Scout Mini|Path Following")
    EScoutMiniPathFollowStatus GetFollowStatus() const { return FollowStatus; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Path Following")
    float GetDistanceToGoalMetres() const { return DistanceToGoalMetres; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Path Following")
    float GetCrossTrackErrorMetres() const { return CrossTrackErrorMetres; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Path Following")
    float GetPathProgressMetres() const { return PathProgressMetres; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Path Following")
    FVector GetCurrentLookaheadPoint() const { return CurrentLookaheadPoint; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Path Following")
    float GetCommandLinearVelocity() const { return CommandLinearVelocity; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Path Following")
    float GetCommandAngularVelocity() const { return CommandAngularVelocity; }

private:
    UPROPERTY(Transient)
    UScoutMiniMovementComponent* Movement = nullptr;

    UPROPERTY(Transient)
    UScoutMiniNavigationComponent* Navigation = nullptr;

    UPROPERTY(Transient)
    TArray<FVector> FollowPath;

    TArray<float> CumulativeDistanceCm;
    EScoutMiniPathFollowStatus FollowStatus = EScoutMiniPathFollowStatus::Idle;
    EScoutMiniControlMode PreviousControlMode = EScoutMiniControlMode::Manual;
    int32 CurrentSegmentIndex = 0;
    int64 FollowedPathRevision = -1;
    float DistanceToGoalMetres = 0.0f;
    float CrossTrackErrorMetres = 0.0f;
    float PathProgressMetres = 0.0f;
    float CommandLinearVelocity = 0.0f;
    float CommandAngularVelocity = 0.0f;
    float StuckElapsedSeconds = 0.0f;
    FVector ClosestPathPoint = FVector::ZeroVector;
    FVector CurrentLookaheadPoint = FVector::ZeroVector;
    bool bFollowingNavigationPath = false;
    bool bSavedControlMode = false;

    UFUNCTION()
    void HandleNavigationPathReady(EScoutMiniPathStatus Status, const TArray<FVector>& PathPoints);

    bool BeginFollowingPath(const TArray<FVector>& WorldPathPoints,
        bool bFromNavigation, int64 NavigationRevision);
    bool LoadPath(const TArray<FVector>& WorldPathPoints);
    bool UpdateClosestPathPosition(const FVector& VehicleLocation);
    FVector FindPointAtProgress(float ProgressCm) const;
    int32 FindUpcomingCorner(float& OutDistanceMetres) const;
    void UpdateFollowing(float DeltaTime);
    void SendCommand(float LinearVelocity, float AngularVelocity);
    void SetStatus(EScoutMiniPathFollowStatus NewStatus);
    void FinishFollowing(EScoutMiniPathFollowStatus TerminalStatus, bool bRestoreControlMode);
    bool IsActivelyFollowing() const;
    bool IsNavigationPathUsable() const;
    void DrawFollowingDebug() const;
};
