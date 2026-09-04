#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "ScoutMiniPawn.generated.h"

class UBoxComponent;
class UCameraComponent;
class USceneComponent;
class UStaticMeshComponent;
class UScoutMiniMovementComponent;
class UScoutMiniNavigationComponent;
class UScoutMiniPathFollowerComponent;
class UScoutMiniROSComponent;

UCLASS(Blueprintable)
class SCOUTMINISIMULATION_API AScoutMiniPawn : public APawn
{
    GENERATED_BODY()

public:
    AScoutMiniPawn();
    virtual void Tick(float DeltaSeconds) override;
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    UBoxComponent* Collision;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    UStaticMeshComponent* Chassis;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    UScoutMiniMovementComponent* VehicleMovement;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation")
    UScoutMiniNavigationComponent* Navigation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation")
    UScoutMiniPathFollowerComponent* PathFollower;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="ROS")
    UScoutMiniROSComponent* ROSController;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    UCameraComponent* Camera;

protected:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    USceneComponent* FrontLeftPivot;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    USceneComponent* FrontRightPivot;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    USceneComponent* RearLeftPivot;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    USceneComponent* RearRightPivot;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    UStaticMeshComponent* FrontLeftWheel;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    UStaticMeshComponent* FrontRightWheel;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    UStaticMeshComponent* RearLeftWheel;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle")
    UStaticMeshComponent* RearRightWheel;

private:

    bool bForward = false;
    bool bBackward = false;
    bool bLeft = false;
    bool bRight = false;
    float LeftWheelRollDegrees = 0.0f;
    float RightWheelRollDegrees = 0.0f;

    void ForwardPressed();
    void ForwardReleased();
    void BackwardPressed();
    void BackwardReleased();
    void LeftPressed();
    void LeftReleased();
    void RightPressed();
    void RightReleased();
    void UpdateManualInput();
    UStaticMeshComponent* CreateWheel(const FName Name, USceneComponent* Parent);
};
