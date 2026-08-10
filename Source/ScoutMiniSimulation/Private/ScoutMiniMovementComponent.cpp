#include "ScoutMiniMovementComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

UScoutMiniMovementComponent::UScoutMiniMovementComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UScoutMiniMovementComponent::BeginPlay()
{
    Super::BeginPlay();
    if (bUseDynamics)
    {
        if (UPrimitiveComponent* Body = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent()))
        {
            Body->SetMassOverrideInKg(NAME_None, VehicleMassKg, true);
            Body->SetSimulatePhysics(true);
            Body->SetEnableGravity(true);
            Body->SetLinearDamping(0.05f);
            Body->SetAngularDamping(0.2f);
        }
    }
}

void UScoutMiniMovementComponent::SetControlMode(const EScoutMiniControlMode NewMode)
{
    ControlMode = NewMode;
    Stop();
}

void UScoutMiniMovementComponent::SetVelocityCommand(const float LinearVelocityMps, const float AngularVelocityRadps)
{
    CommandLinearVelocity = FMath::Clamp(LinearVelocityMps, -MaxLinearSpeed, MaxLinearSpeed);
    CommandAngularVelocity = FMath::Clamp(AngularVelocityRadps, -MaxAngularSpeed, MaxAngularSpeed);
}

void UScoutMiniMovementComponent::Stop()
{
    CommandLinearVelocity = 0.0f;
    CommandAngularVelocity = 0.0f;
    ManualThrottle = 0.0f;
    ManualSteering = 0.0f;
}

void UScoutMiniMovementComponent::SetManualInput(const float Throttle, const float Steering)
{
    ManualThrottle = FMath::Clamp(Throttle, -1.0f, 1.0f);
    ManualSteering = FMath::Clamp(Steering, -1.0f, 1.0f);
}

void UScoutMiniMovementComponent::ResolveTarget(float& OutLinear, float& OutAngular) const
{
    OutLinear = ControlMode == EScoutMiniControlMode::Manual
        ? ManualThrottle * MaxLinearSpeed : CommandLinearVelocity;
    OutAngular = ControlMode == EScoutMiniControlMode::Manual
        ? ManualSteering * MaxAngularSpeed : CommandAngularVelocity;
}

void UScoutMiniMovementComponent::TickComponent(const float DeltaTime, const ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    AActor* Owner = GetOwner();
    if (!Owner || DeltaTime <= 0.0f)
    {
        return;
    }

    float TargetLinear = 0.0f;
    float TargetAngular = 0.0f;
    ResolveTarget(TargetLinear, TargetAngular);

    if (bUseDynamics)
    {
        TickDynamics(DeltaTime, TargetLinear, TargetAngular);
        return;
    }

    CurrentLinearVelocity = FMath::FInterpConstantTo(CurrentLinearVelocity, TargetLinear, DeltaTime, LinearAcceleration);
    CurrentAngularVelocity = FMath::FInterpConstantTo(CurrentAngularVelocity, TargetAngular, DeltaTime, AngularAcceleration);
    TickKinematic(DeltaTime);
}

void UScoutMiniMovementComponent::TickKinematic(const float DeltaTime)
{
    AActor* Owner = GetOwner();

    // UE uses centimetres; the public API intentionally uses SI units.
    const FVector DeltaLocation = Owner->GetActorForwardVector() * CurrentLinearVelocity * 100.0f * DeltaTime;
    const FQuat DeltaRotation(FVector::UpVector, CurrentAngularVelocity * DeltaTime);
    FHitResult Hit;
    Owner->SetActorLocationAndRotation(
        Owner->GetActorLocation() + DeltaLocation,
        DeltaRotation * Owner->GetActorQuat(),
        bSweepCollision,
        bSweepCollision ? &Hit : nullptr,
        ETeleportType::None);

    if (Hit.IsValidBlockingHit())
    {
        CurrentLinearVelocity = 0.0f;
    }
}

void UScoutMiniMovementComponent::TickDynamics(const float DeltaTime, const float TargetLinear, const float TargetAngular)
{
    AActor* Owner = GetOwner();
    UPrimitiveComponent* Body = Owner ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr;
    UWorld* World = GetWorld();
    if (!Body || !Body->IsSimulatingPhysics() || !World) return;

    const FTransform Transform = Body->GetComponentTransform();
    const FVector Up = Transform.GetUnitAxis(EAxis::Z);
    const FVector BodyForward = Transform.GetUnitAxis(EAxis::X);
    const FVector BodyRight = Transform.GetUnitAxis(EAxis::Y);
    const float HalfWheelBaseCm = WheelBase * 50.0f;
    const float HalfTrackCm = TrackWidth * 50.0f;
    const float WheelVerticalOffsetCm = WheelVerticalOffset * 100.0f;
    const float TraceLengthCm = (SuspensionRestLength + WheelRadius) * 100.0f;
    const float BodyYawRate = FVector::DotProduct(Body->GetPhysicsAngularVelocityInRadians(), Up);
    const float MaxLinearForceN = VehicleMassKg * LinearAcceleration;
    const float MaxYawTorqueNm = YawInertiaKgM2 * AngularAcceleration;
    const float RequestedYawTorqueNm = FMath::Clamp(
        (TargetAngular - BodyYawRate) * YawInertiaKgM2 * YawRateGain,
        -MaxYawTorqueNm,
        MaxYawTorqueNm);

    const FVector LocalWheels[4] = {
        FVector(HalfWheelBaseCm, -HalfTrackCm, WheelVerticalOffsetCm),
        FVector(HalfWheelBaseCm, HalfTrackCm, WheelVerticalOffsetCm),
        FVector(-HalfWheelBaseCm, -HalfTrackCm, WheelVerticalOffsetCm),
        FVector(-HalfWheelBaseCm, HalfTrackCm, WheelVerticalOffsetCm) };

    FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(ScoutMiniSuspension), false, Owner);
    int32 ContactCount = 0;
    float TotalNormalForceN = 0.0f;
    FVector GroundNormalSum = FVector::ZeroVector;
    for (int32 Index = 0; Index < 4; ++Index)
    {
        const FVector Mount = Transform.TransformPosition(LocalWheels[Index]);
        FHitResult Hit;
        if (!World->LineTraceSingleByChannel(Hit, Mount, Mount - Up * TraceLengthCm, ECC_Visibility, QueryParams)) continue;
        ++ContactCount;

        const float DistanceM = Hit.Distance / 100.0f;
        const float CompressionM = FMath::Clamp(SuspensionRestLength + WheelRadius - DistanceM, 0.0f, SuspensionRestLength);
        const FVector PointVelocityCm = Body->GetPhysicsLinearVelocityAtPoint(Hit.ImpactPoint);
        const float SuspensionVelocityMps = FVector::DotProduct(PointVelocityCm, Up) / 100.0f;
        const float NormalForceN = FMath::Clamp(
            CompressionM * SuspensionStiffness - SuspensionVelocityMps * SuspensionDamping,
            0.0f,
            MaxSuspensionForcePerWheel);
        TotalNormalForceN += NormalForceN;
        GroundNormalSum += Hit.ImpactNormal;

        // The stable model keeps only vertical suspension at the contact patch.
        // Drive and lateral stabilization are applied at the centre of mass
        // below, avoiding pitch/roll impulses from discrete ray contacts.
        const FVector ForceUE = Up * NormalForceN * 100.0f;
        Body->AddForce(ForceUE);
    }

    const FVector VelocityCm = Body->GetPhysicsLinearVelocity();
    CurrentLinearVelocity = FVector::DotProduct(VelocityCm, BodyForward) / 100.0f;
    CurrentAngularVelocity = FVector::DotProduct(Body->GetPhysicsAngularVelocityInRadians(), Up);

    if (ContactCount > 0)
    {
        const FVector GroundNormal = GroundNormalSum.GetSafeNormal(KINDA_SMALL_NUMBER, Up);
        const FVector AngularVelocity = Body->GetPhysicsAngularVelocityInRadians();
        const FVector TiltAngularVelocity = FVector::VectorPlaneProject(AngularVelocity, GroundNormal);
        const FVector AlignmentTorqueNm =
            FVector::CrossProduct(Up, GroundNormal) * GroundAlignmentStiffness
            - TiltAngularVelocity * GroundAlignmentDamping;
        Body->AddTorqueInRadians(AlignmentTorqueNm * 10000.0f);

        // Drive tangent to the supporting surface. Using raw BodyForward on a
        // slope can inject a vertical force that pitches the short-wheelbase
        // vehicle over its front axle.
        const FVector DriveForward = FVector::VectorPlaneProject(BodyForward, GroundNormal).GetSafeNormal(KINDA_SMALL_NUMBER, BodyForward);
        const FVector DriveRight = FVector::CrossProduct(GroundNormal, DriveForward).GetSafeNormal(KINDA_SMALL_NUMBER, BodyRight);
        const float ForwardSpeedMps = FVector::DotProduct(VelocityCm, DriveForward) / 100.0f;
        const float LateralSpeedMps = FVector::DotProduct(VelocityCm, DriveRight) / 100.0f;
        float DriveForceN = FMath::Clamp(
            (TargetLinear - ForwardSpeedMps) * LongitudinalTyreStiffness,
            -MaxLinearForceN,
            MaxLinearForceN);
        float LateralForceN = -LateralSpeedMps * CorneringStiffness;
        const float FrictionLimitN = TotalNormalForceN * TyreFrictionCoefficient;
        const float PlanarForceN = FMath::Sqrt(FMath::Square(DriveForceN) + FMath::Square(LateralForceN));
        if (PlanarForceN > FrictionLimitN && PlanarForceN > KINDA_SMALL_NUMBER)
        {
            const float Scale = FrictionLimitN / PlanarForceN;
            DriveForceN *= Scale;
            LateralForceN *= Scale;
        }
        Body->AddForce((DriveForward * DriveForceN + DriveRight * LateralForceN) * 100.0f);
        Body->AddTorqueInRadians(GroundNormal * RequestedYawTorqueNm * 10000.0f);

        const float SpeedMps = VelocityCm.Size() / 100.0f;
        if (SpeedMps > KINDA_SMALL_NUMBER)
        {
            const FVector Direction = VelocityCm.GetSafeNormal();
            const float ResistanceN = RollingResistance + AerodynamicDrag * SpeedMps * SpeedMps;
            Body->AddForce(-Direction * ResistanceN * 100.0f);
        }
    }
}

void UScoutMiniMovementComponent::GetDifferentialWheelVelocities(float& LeftMps, float& RightMps) const
{
    LeftMps = CurrentLinearVelocity + CurrentAngularVelocity * TrackWidth * 0.5f;
    RightMps = CurrentLinearVelocity - CurrentAngularVelocity * TrackWidth * 0.5f;
}
