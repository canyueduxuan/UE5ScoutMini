#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ScoutMiniROSComponent.generated.h"

class UTopic;
class UScoutMiniMovementComponent;
class FROSBaseMsg;

/** ROSIntegration adapter for cmd_vel control, odometry and odom->base_link TF. */
UCLASS(ClassGroup=(ROS), meta=(BlueprintSpawnableComponent))
class SCOUTMINISIMULATION_API UScoutMiniROSComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UScoutMiniROSComponent();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Connection")
    bool bEnabled = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Connection", meta=(ClampMin="0"))
    int32 ROSConnectionId = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Topics")
    FString CmdVelTopicName = TEXT("/cmd_vel");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Topics")
    FString OdometryTopicName = TEXT("/odom");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Frames")
    FString OdometryFrameId = TEXT("odom");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Frames")
    FString BaseFrameId = TEXT("base_link");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Publishing", meta=(ClampMin="1.0", ClampMax="200.0"))
    float PublishRateHz = 30.0f;

    /** Stop when no valid cmd_vel has arrived for this many seconds. Zero disables the watchdog. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Safety", meta=(ClampMin="0.0"))
    float CommandTimeoutSeconds = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Control")
    float LinearScale = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Control")
    float AngularScale = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Odometry")
    bool bPublishTF = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="ROS|Odometry")
    bool bPlanarOdometry = true;

private:
    UPROPERTY(Transient)
    UTopic* CmdVelTopic = nullptr;
    UPROPERTY(Transient)
    UTopic* OdometryTopic = nullptr;
    UPROPERTY(Transient)
    UTopic* TFTopic = nullptr;
    UPROPERTY(Transient)
    UScoutMiniMovementComponent* Movement = nullptr;

    FTransform OdometryOrigin;
    double LastCommandTime = 0.0;
    float PublishAccumulator = 0.0f;
    uint32 Sequence = 0;
    bool bHasReceivedCommand = false;
    bool bWatchdogStopped = false;

    void HandleVelocityMessage(TSharedPtr<FROSBaseMsg> Message);
    void ApplyVelocityCommand(double LinearX, double AngularZ);
    void PublishOdometryAndTF();
    void ShutdownROS();
};
