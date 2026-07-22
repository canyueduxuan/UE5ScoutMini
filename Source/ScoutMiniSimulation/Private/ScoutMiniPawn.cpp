#include "ScoutMiniPawn.h"
#include "ScoutMiniMovementComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/BoxComponent.h"
#include "Components/InputComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputCoreTypes.h"
#include "UObject/ConstructorHelpers.h"

AScoutMiniPawn::AScoutMiniPawn()
{
    PrimaryActorTick.bCanEverTick = true;
    AutoPossessPlayer = EAutoReceiveInput::Player0;

    Collision = CreateDefaultSubobject<UBoxComponent>(TEXT("Collision"));
    SetRootComponent(Collision);
    Collision->SetBoxExtent(FVector(31.0f, 29.25f, 11.75f));
    Collision->SetCollisionProfileName(UCollisionProfile::Pawn_ProfileName);
    Collision->SetSimulatePhysics(true);
    Collision->SetEnableGravity(true);
    // Match scout_mini.urdf.xacro inertia instead of using the much smaller
    // pitch inertia generated automatically from the simple collision box.
    // Ratios compare the URDF tensor (2.288641, 5.103976, 3.431465 kg m^2)
    // with this 62 x 58.5 x 23.5 cm, 72 kg box approximation.
    Collision->BodyInstance.InertiaTensorScale = FVector(0.96f, 1.935f, 0.787f);

    Chassis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Chassis"));
    Chassis->SetupAttachment(Collision);
    Chassis->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Chassis->SetRelativeLocation(FVector::ZeroVector);
    Chassis->SetRelativeScale3D(FVector::OneVector);

    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
    if (Cube.Succeeded()) Chassis->SetStaticMesh(Cube.Object);

    FrontLeftPivot = CreateDefaultSubobject<USceneComponent>(TEXT("FrontLeftPivot"));
    FrontLeftPivot->SetupAttachment(Collision);
    FrontLeftPivot->SetRelativeLocation(FVector(23.19755f, -20.82515f, -10.0998f));
    FrontRightPivot = CreateDefaultSubobject<USceneComponent>(TEXT("FrontRightPivot"));
    FrontRightPivot->SetupAttachment(Collision);
    FrontRightPivot->SetRelativeLocation(FVector(23.19755f, 20.82515f, -10.0998f));
    FrontLeftWheel = CreateWheel(TEXT("FrontLeftWheel"), FrontLeftPivot);
    FrontRightWheel = CreateWheel(TEXT("FrontRightWheel"), FrontRightPivot);
    RearLeftPivot = CreateDefaultSubobject<USceneComponent>(TEXT("RearLeftPivot"));
    RearLeftPivot->SetupAttachment(Collision);
    RearLeftPivot->SetRelativeLocation(FVector(-23.19755f, -20.82515f, -10.0998f));
    RearRightPivot = CreateDefaultSubobject<USceneComponent>(TEXT("RearRightPivot"));
    RearRightPivot->SetupAttachment(Collision);
    RearRightPivot->SetRelativeLocation(FVector(-23.19755f, 20.82515f, -10.0998f));
    RearLeftWheel = CreateWheel(TEXT("RearLeftWheel"), RearLeftPivot);
    RearRightWheel = CreateWheel(TEXT("RearRightWheel"), RearRightPivot);

    // The imported Scout wheel meshes use the base_link origin rather than a
    // wheel-centre origin. Counter-offset each mesh below its axle pivot so all
    // five imported meshes overlap correctly at rest while the pivot still
    // provides the proper centre of wheel rotation.
    FrontLeftWheel->SetRelativeLocation(-FrontLeftPivot->GetRelativeLocation());
    FrontRightWheel->SetRelativeLocation(-FrontRightPivot->GetRelativeLocation());
    RearLeftWheel->SetRelativeLocation(-RearLeftPivot->GetRelativeLocation());
    RearRightWheel->SetRelativeLocation(-RearRightPivot->GetRelativeLocation());

    USpringArmComponent* SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(Collision);
    SpringArm->TargetArmLength = 300.0f;
    SpringArm->SetRelativeRotation(FRotator(-20.0f, 0.0f, 0.0f));
    SpringArm->bEnableCameraLag = true;
    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm);

    VehicleMovement = CreateDefaultSubobject<UScoutMiniMovementComponent>(TEXT("VehicleMovement"));
}

UStaticMeshComponent* AScoutMiniPawn::CreateWheel(const FName Name, USceneComponent* Parent)
{
    UStaticMeshComponent* Wheel = CreateDefaultSubobject<UStaticMeshComponent>(Name);
    Wheel->SetupAttachment(Parent);
    Wheel->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Wheel->SetRelativeRotation(FRotator::ZeroRotator);
    Wheel->SetRelativeScale3D(FVector::OneVector);
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    if (Cylinder.Succeeded()) Wheel->SetStaticMesh(Cylinder.Object);
    return Wheel;
}

void AScoutMiniPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);
    PlayerInputComponent->BindKey(EKeys::W, IE_Pressed, this, &AScoutMiniPawn::ForwardPressed);
    PlayerInputComponent->BindKey(EKeys::W, IE_Released, this, &AScoutMiniPawn::ForwardReleased);
    PlayerInputComponent->BindKey(EKeys::S, IE_Pressed, this, &AScoutMiniPawn::BackwardPressed);
    PlayerInputComponent->BindKey(EKeys::S, IE_Released, this, &AScoutMiniPawn::BackwardReleased);
    PlayerInputComponent->BindKey(EKeys::A, IE_Pressed, this, &AScoutMiniPawn::LeftPressed);
    PlayerInputComponent->BindKey(EKeys::A, IE_Released, this, &AScoutMiniPawn::LeftReleased);
    PlayerInputComponent->BindKey(EKeys::D, IE_Pressed, this, &AScoutMiniPawn::RightPressed);
    PlayerInputComponent->BindKey(EKeys::D, IE_Released, this, &AScoutMiniPawn::RightReleased);
}

void AScoutMiniPawn::Tick(const float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (!VehicleMovement) return;
    UpdateManualInput();

    const float Radius = FMath::Max(VehicleMovement->WheelRadius, 0.01f);
    float LeftMps = 0.0f;
    float RightMps = 0.0f;
    VehicleMovement->GetDifferentialWheelVelocities(LeftMps, RightMps);
    LeftWheelRollDegrees = FMath::Fmod(LeftWheelRollDegrees
        + FMath::RadiansToDegrees(LeftMps * DeltaSeconds / Radius), 360.0f);
    RightWheelRollDegrees = FMath::Fmod(RightWheelRollDegrees
        + FMath::RadiansToDegrees(RightMps * DeltaSeconds / Radius), 360.0f);
    // Imported wheel vertices are expressed in base_link coordinates. Rotate
    // their axle pivots around vehicle Y; the counter-offset mesh then rotates
    // around its wheel centre instead of orbiting around the chassis origin.
    const FQuat LeftSpin(FVector::RightVector, FMath::DegreesToRadians(LeftWheelRollDegrees));
    const FQuat RightSpin(FVector::RightVector, FMath::DegreesToRadians(RightWheelRollDegrees));
    FrontLeftPivot->SetRelativeRotation(LeftSpin);
    RearLeftPivot->SetRelativeRotation(LeftSpin);
    FrontRightPivot->SetRelativeRotation(RightSpin);
    RearRightPivot->SetRelativeRotation(RightSpin);
}

void AScoutMiniPawn::UpdateManualInput()
{
    if (VehicleMovement->ControlMode != EScoutMiniControlMode::Manual) return;
    VehicleMovement->SetManualInput((bForward ? 1.0f : 0.0f) - (bBackward ? 1.0f : 0.0f),
        (bRight ? 1.0f : 0.0f) - (bLeft ? 1.0f : 0.0f));
}

void AScoutMiniPawn::ForwardPressed() { bForward = true; }
void AScoutMiniPawn::ForwardReleased() { bForward = false; }
void AScoutMiniPawn::BackwardPressed() { bBackward = true; }
void AScoutMiniPawn::BackwardReleased() { bBackward = false; }
void AScoutMiniPawn::LeftPressed() { bLeft = true; }
void AScoutMiniPawn::LeftReleased() { bLeft = false; }
void AScoutMiniPawn::RightPressed() { bRight = true; }
void AScoutMiniPawn::RightReleased() { bRight = false; }
