#include "ScoutMiniROSComponent.h"

#include "ScoutMiniMovementComponent.h"
#include "Async/Async.h"
#include "GameFramework/Actor.h"
#include "RI/Topic.h"
#include "ROSIntegrationCore.h"
#include "ROSIntegrationGameInstance.h"
#include "ROSTime.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf2_msgs/TFMessage.h"

UScoutMiniROSComponent::UScoutMiniROSComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UScoutMiniROSComponent::BeginPlay()
{
    Super::BeginPlay();
    if (!bEnabled || !GetOwner()) return;

    Movement = GetOwner()->FindComponentByClass<UScoutMiniMovementComponent>();
    if (!Movement)
    {
        UE_LOG(LogTemp, Error, TEXT("ScoutMiniROS: %s has no ScoutMiniMovementComponent"), *GetOwner()->GetName());
        return;
    }

    UROSIntegrationGameInstance* ROSInstance = Cast<UROSIntegrationGameInstance>(GetWorld()->GetGameInstance());
    UROSIntegrationCore* Core = ROSInstance ? ROSInstance->GetROSConnectionFromID(ROSConnectionId) : nullptr;
    if (!Core)
    {
        UE_LOG(LogTemp, Error, TEXT("ScoutMiniROS: ROS connection %d is unavailable; check ROSIntegrationGameInstance"), ROSConnectionId);
        return;
    }

    OdometryOrigin = GetOwner()->GetActorTransform();
    Movement->SetControlMode(EScoutMiniControlMode::Programmatic);
    Movement->Stop();

    CmdVelTopic = NewObject<UTopic>(this);
    CmdVelTopic->Init(Core, CmdVelTopicName, TEXT("geometry_msgs/Twist"), 1, false);
    TWeakObjectPtr<UScoutMiniROSComponent> WeakThis(this);
    CmdVelTopic->Subscribe([WeakThis](TSharedPtr<FROSBaseMsg> Message)
    {
        if (!WeakThis.IsValid()) return;
        TSharedPtr<ROSMessages::geometry_msgs::Twist> Twist = StaticCastSharedPtr<ROSMessages::geometry_msgs::Twist>(Message);
        if (!Twist.IsValid()) return;
        const double LinearX = Twist->linear.x;
        const double AngularZ = Twist->angular.z;
        AsyncTask(ENamedThreads::GameThread, [WeakThis, LinearX, AngularZ]()
        {
            if (WeakThis.IsValid()) WeakThis->ApplyVelocityCommand(LinearX, AngularZ);
        });
    });

    OdometryTopic = NewObject<UTopic>(this);
    OdometryTopic->Init(Core, OdometryTopicName, TEXT("nav_msgs/Odometry"), 10, false);
    OdometryTopic->Advertise();

    if (bPublishTF)
    {
        TFTopic = NewObject<UTopic>(this);
        TFTopic->Init(Core, TEXT("/tf"), TEXT("tf2_msgs/TFMessage"), 10, false);
        TFTopic->Advertise();
    }

    UE_LOG(LogTemp, Display, TEXT("ScoutMiniROS: subscribed %s; publishing %s and %s -> %s TF"),
        *CmdVelTopicName, *OdometryTopicName, *OdometryFrameId, *BaseFrameId);
}

void UScoutMiniROSComponent::ApplyVelocityCommand(const double LinearX, const double AngularZ)
{
    if (!Movement || !FMath::IsFinite(LinearX) || !FMath::IsFinite(AngularZ))
    {
        if (Movement) Movement->Stop();
        return;
    }

    // ROS: X forward, Y left, positive Z yaw left. UE uses Y right and positive yaw right.
    Movement->SetVelocityCommand(static_cast<float>(LinearX) * LinearScale,
        static_cast<float>(-AngularZ) * AngularScale);
    LastCommandTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
    bHasReceivedCommand = true;
    bWatchdogStopped = false;
}

void UScoutMiniROSComponent::TickComponent(const float DeltaTime, const ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!bEnabled || !Movement || !GetWorld()) return;

    if (CommandTimeoutSeconds > 0.0f && bHasReceivedCommand && !bWatchdogStopped
        && GetWorld()->GetTimeSeconds() - LastCommandTime > CommandTimeoutSeconds)
    {
        Movement->Stop();
        bWatchdogStopped = true;
        UE_LOG(LogTemp, Warning, TEXT("ScoutMiniROS: cmd_vel timeout, stopping %s"), *GetOwner()->GetName());
    }

    PublishAccumulator += DeltaTime;
    const float Period = 1.0f / FMath::Max(PublishRateHz, 1.0f);
    if (PublishAccumulator >= Period)
    {
        PublishAccumulator = FMath::Fmod(PublishAccumulator, Period);
        PublishOdometryAndTF();
    }
}

void UScoutMiniROSComponent::PublishOdometryAndTF()
{
    if (!OdometryTopic || !GetOwner() || !Movement) return;

    const FTransform Relative = GetOwner()->GetActorTransform().GetRelativeTransform(OdometryOrigin);
    FVector Location = Relative.GetLocation() / 100.0f;
    FQuat Rotation = Relative.GetRotation().GetNormalized();
    if (bPlanarOdometry)
    {
        Location.Z = 0.0f;
        const float YawRadians = FMath::DegreesToRadians(Relative.Rotator().Yaw);
        Rotation = FQuat(FVector::UpVector, YawRadians);
    }

    const double RosX = Location.X;
    const double RosY = -Location.Y;
    const double RosZ = Location.Z;
    const double RosQX = -Rotation.X;
    const double RosQY = Rotation.Y;
    const double RosQZ = -Rotation.Z;
    const double RosQW = Rotation.W;
    const FROSTime Stamp = FROSTime::Now();

    TSharedPtr<ROSMessages::nav_msgs::Odometry> Odom = MakeShared<ROSMessages::nav_msgs::Odometry>();
    Odom->header.seq = Sequence++;
    Odom->header.time = Stamp;
    Odom->header.frame_id = OdometryFrameId;
    Odom->child_frame_id = BaseFrameId;
    Odom->pose.pose.position.x = RosX;
    Odom->pose.pose.position.y = RosY;
    Odom->pose.pose.position.z = RosZ;
    Odom->pose.pose.orientation.x = RosQX;
    Odom->pose.pose.orientation.y = RosQY;
    Odom->pose.pose.orientation.z = RosQZ;
    Odom->pose.pose.orientation.w = RosQW;
    Odom->pose.covariance.Init(0.0, 36);
    Odom->twist.twist.linear.x = Movement->GetLinearVelocity();
    Odom->twist.twist.angular.z = -Movement->GetAngularVelocity();
    Odom->twist.covariance.Init(0.0, 36);
    // Non-zero diagonal entries prevent consumers from interpreting this as perfect ground truth.
    Odom->pose.covariance[0] = Odom->pose.covariance[7] = 0.01;
    Odom->pose.covariance[35] = 0.02;
    Odom->twist.covariance[0] = Odom->twist.covariance[7] = 0.02;
    Odom->twist.covariance[35] = 0.04;
    OdometryTopic->Publish(Odom);

    if (TFTopic)
    {
        TSharedPtr<ROSMessages::tf2_msgs::TFMessage> TF = MakeShared<ROSMessages::tf2_msgs::TFMessage>();
        ROSMessages::geometry_msgs::TransformStamped Transform;
        Transform.header.seq = Odom->header.seq;
        Transform.header.time = Stamp;
        Transform.header.frame_id = OdometryFrameId;
        Transform.child_frame_id = BaseFrameId;
        Transform.transform.translation.x = RosX;
        Transform.transform.translation.y = RosY;
        Transform.transform.translation.z = RosZ;
        Transform.transform.rotation.x = RosQX;
        Transform.transform.rotation.y = RosQY;
        Transform.transform.rotation.z = RosQZ;
        Transform.transform.rotation.w = RosQW;
        TF->transforms.Add(Transform);
        TFTopic->Publish(TF);
    }
}

void UScoutMiniROSComponent::ShutdownROS()
{
    if (Movement) Movement->Stop();
    if (CmdVelTopic) CmdVelTopic->Unsubscribe();
    if (OdometryTopic) OdometryTopic->Unadvertise();
    if (TFTopic) TFTopic->Unadvertise();
    CmdVelTopic = nullptr;
    OdometryTopic = nullptr;
    TFTopic = nullptr;
}

void UScoutMiniROSComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    ShutdownROS();
    Super::EndPlay(EndPlayReason);
}
