#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ScoutMiniMovementComponent.generated.h"

UENUM(BlueprintType)
enum class EScoutMiniControlMode : uint8
{
    Manual UMETA(DisplayName = "Manual (W/S/A/D)"),
    Programmatic UMETA(DisplayName = "Programmatic")
};

UCLASS(ClassGroup=(Simulation), meta=(BlueprintSpawnableComponent))
class SCOUTMINISIMULATION_API UScoutMiniMovementComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UScoutMiniMovementComponent();
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle")
    EScoutMiniControlMode ControlMode = EScoutMiniControlMode::Manual;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Limits", meta=(ClampMin="0.0"))
    float MaxLinearSpeed = 4.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Limits", meta=(ClampMin="0.0"))
    float MaxAngularSpeed = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Limits", meta=(ClampMin="0.0"))
    float LinearAcceleration = 4.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Limits", meta=(ClampMin="0.0"))
    float AngularAcceleration = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Geometry", meta=(ClampMin="0.01"))
    float WheelBase = 0.463951f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Geometry", meta=(ClampMin="0.01"))
    float TrackWidth = 0.416503f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Geometry", meta=(ClampMin="0.01"))
    float WheelRadius = 0.16f;

    /** Wheel-centre height relative to base_link, from scout_mini.urdf.xacro. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Geometry")
    float WheelVerticalOffset = -0.100998f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Collision")
    bool bSweepCollision = true;

    /** Use four contact patches, suspension and tyre forces instead of teleport-style kinematics. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics")
    bool bUseDynamics = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="1.0"))
    float VehicleMassKg = 72.0f;

    /** base_link yaw inertia (izz) from scout_mini.urdf.xacro, kg m^2. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.01"))
    float YawInertiaKgM2 = 3.431465f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float SuspensionRestLength = 0.03f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float SuspensionStiffness = 12000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float SuspensionDamping = 800.0f;

    /** Keeps the chassis aligned with the supporting surface without locking pitch or roll. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float GroundAlignmentStiffness = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float GroundAlignmentDamping = 60.0f;

    /** Prevent a single contact from injecting an unstable force spike. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="1.0"))
    float MaxSuspensionForcePerWheel = 400.0f;

    /** Proportional response used to track commanded yaw rate, in 1/s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float YawRateGain = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float LongitudinalTyreStiffness = 1800.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float CorneringStiffness = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float TyreFrictionCoefficient = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float AerodynamicDrag = 0.45f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0"))
    float RollingResistance = 35.0f;

    /** Distance below the centre of mass where drive/brake force is applied. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Dynamics", meta=(ClampMin="0.0", ClampMax="0.2"))
    float DriveForceApplicationDepth = 0.08f;

    UFUNCTION(BlueprintCallable, Category="Vehicle|Control")
    void SetControlMode(EScoutMiniControlMode NewMode);

    UFUNCTION(BlueprintCallable, Category="Vehicle|Control", meta=(DisplayName="Set Velocity Command (m/s, rad/s)"))
    void SetVelocityCommand(float LinearVelocityMps, float AngularVelocityRadps);

    /** Claim exclusive programmatic command ownership. Existing ownership is kept unless bForce is true. */
    UFUNCTION(BlueprintCallable, Category="Vehicle|Control")
    bool AcquireCommandAuthority(UObject* Requester, bool bForce = false);

    UFUNCTION(BlueprintCallable, Category="Vehicle|Control")
    void ReleaseCommandAuthority(UObject* Requester);

    /** Send a velocity command only when Requester owns programmatic control. */
    UFUNCTION(BlueprintCallable, Category="Vehicle|Control")
    bool SetVelocityCommandFrom(UObject* Requester, float LinearVelocityMps, float AngularVelocityRadps);

    /** Stop only when Requester owns programmatic control. */
    UFUNCTION(BlueprintCallable, Category="Vehicle|Control")
    bool StopFrom(UObject* Requester);

    UFUNCTION(BlueprintPure, Category="Vehicle|Control")
    bool HasCommandAuthority(const UObject* Requester) const;

    UFUNCTION(BlueprintPure, Category="Vehicle|Control")
    bool HasAnyCommandAuthority() const { return CommandAuthority.IsValid(); }

    UFUNCTION(BlueprintCallable, Category="Vehicle|Control")
    void Stop();

    UFUNCTION(BlueprintCallable, Category="Vehicle|Control")
    void SetManualInput(float Throttle, float Steering);

    UFUNCTION(BlueprintPure, Category="Vehicle|State")
    float GetLinearVelocity() const { return CurrentLinearVelocity; }

    UFUNCTION(BlueprintPure, Category="Vehicle|State")
    float GetAngularVelocity() const { return CurrentAngularVelocity; }

    /** Actual world-space linear velocity in m/s. */
    UFUNCTION(BlueprintPure, Category="Vehicle|State")
    FVector GetWorldLinearVelocityMps() const;

    /** Actual world-space angular velocity in rad/s using UE axes. */
    UFUNCTION(BlueprintPure, Category="Vehicle|State")
    FVector GetWorldAngularVelocityRadps() const;

    UFUNCTION(BlueprintPure, Category="Vehicle|State")
    void GetDifferentialWheelVelocities(float& LeftMps, float& RightMps) const;

private:
    UPROPERTY(Transient)
    TWeakObjectPtr<UObject> CommandAuthority;
    bool bHadCommandAuthority = false;

    float CommandLinearVelocity = 0.0f;
    float CommandAngularVelocity = 0.0f;
    float ManualThrottle = 0.0f;
    float ManualSteering = 0.0f;
    float CurrentLinearVelocity = 0.0f;
    float CurrentAngularVelocity = 0.0f;
    void ApplyVelocityCommand(float LinearVelocityMps, float AngularVelocityRadps);
    void ResolveTarget(float& OutLinear, float& OutAngular) const;
    void TickDynamics(float DeltaTime, float TargetLinear, float TargetAngular);
    void TickKinematic(float DeltaTime);
};
