#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ScoutMiniNavigationComponent.generated.h"

class UNavigationQueryFilter;

UENUM(BlueprintType)
enum class EScoutMiniPathStatus : uint8
{
    None,
    Success,
    Partial,
    InvalidInput,
    StartProjectionFailed,
    GoalProjectionFailed,
    NavigationUnavailable,
    PathNotFound
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
    FScoutMiniPathReadySignature,
    EScoutMiniPathStatus, Status,
    const TArray<FVector>&, PathPoints);

/** Queries UE navigation paths without taking control of the vehicle. */
UCLASS(ClassGroup=(Navigation), meta=(BlueprintSpawnableComponent))
class SCOUTMINISIMULATION_API UScoutMiniNavigationComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UScoutMiniNavigationComponent();
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;

    /** Half extent, in UE centimetres, used to find the nearest NavMesh point. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Query")
    FVector ProjectionExtent = FVector(100.0f, 100.0f, 300.0f);

    /** Conservative cylindrical footprint radius used to select navigation data. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Agent", meta=(ClampMin="1.0", Units="cm"))
    float AgentRadius = 43.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Agent", meta=(ClampMin="1.0", Units="cm"))
    float AgentHeight = 30.0f;

    /** A partial path reaches the closest navigable point, not the requested goal. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Query")
    bool bAllowPartialPath = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Query")
    TSubclassOf<UNavigationQueryFilter> QueryFilterClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Debug")
    bool bDrawPath = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Debug")
    FColor DebugPathColor = FColor::Cyan;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Debug", meta=(ClampMin="0.0", Units="cm"))
    float DebugPathHeightOffset = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Navigation|Debug", meta=(ClampMin="0.0"))
    float DebugPathThickness = 3.0f;

    UPROPERTY(BlueprintAssignable, Category="Navigation")
    FScoutMiniPathReadySignature OnPathReady;

    UFUNCTION(BlueprintCallable, Category="Scout Mini|Navigation")
    bool FindPathToLocation(const FVector& GoalWorldLocation);

    UFUNCTION(BlueprintCallable, Category="Scout Mini|Navigation")
    bool FindPathBetweenLocations(const FVector& StartWorldLocation, const FVector& GoalWorldLocation);

    UFUNCTION(BlueprintCallable, Category="Scout Mini|Navigation")
    void ClearPath();

    UFUNCTION(BlueprintPure, Category="Scout Mini|Navigation")
    TArray<FVector> GetPathPoints() const { return PathPoints; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Navigation")
    EScoutMiniPathStatus GetPathStatus() const { return PathStatus; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Navigation")
    float GetPathLengthMetres() const { return PathLengthMetres; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Navigation")
    bool IsPartialPath() const { return bIsPartialPath; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Navigation")
    int64 GetPathRevision() const { return PathRevision; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Navigation")
    FVector GetProjectedStart() const { return ProjectedStart; }

    UFUNCTION(BlueprintPure, Category="Scout Mini|Navigation")
    FVector GetProjectedGoal() const { return ProjectedGoal; }

private:
    UPROPERTY(Transient)
    TArray<FVector> PathPoints;

    UPROPERTY(Transient)
    EScoutMiniPathStatus PathStatus = EScoutMiniPathStatus::None;

    UPROPERTY(Transient)
    FVector ProjectedStart = FVector::ZeroVector;

    UPROPERTY(Transient)
    FVector ProjectedGoal = FVector::ZeroVector;

    UPROPERTY(Transient)
    float PathLengthMetres = 0.0f;

    UPROPERTY(Transient)
    bool bIsPartialPath = false;

    UPROPERTY(Transient)
    int64 PathRevision = 0;

    bool bLoggedDefaultNavDataFallback = false;

    bool FinishQuery(EScoutMiniPathStatus NewStatus, const TCHAR* Message = nullptr);
    void DrawPathDebug() const;
};
