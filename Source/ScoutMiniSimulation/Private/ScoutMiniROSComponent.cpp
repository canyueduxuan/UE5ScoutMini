#include "ScoutMiniROSComponent.h"

#include "ScoutMiniMovementComponent.h"
#include "Async/Async.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "RI/Topic.h"
#include "ROSIntegrationCore.h"
#include "ROSIntegrationGameInstance.h"
#include "ROSTime.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32MultiArray.h"
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

    PathTopic = NewObject<UTopic>(this);
    PathTopic->Init(Core, PathTopicName, TEXT("nav_msgs/Path"), 1, false);
    PathTopic->Subscribe([WeakThis](TSharedPtr<FROSBaseMsg> Message)
    {
        if (!WeakThis.IsValid()) return;
        const TSharedPtr<ROSMessages::nav_msgs::Path> Path =
            StaticCastSharedPtr<ROSMessages::nav_msgs::Path>(Message);
        if (!Path.IsValid()) return;

        TArray<FVector> RosPoints;
        RosPoints.Reserve(Path->poses.Num());
        for (const ROSMessages::geometry_msgs::PoseStamped& Pose : Path->poses)
        {
            const auto& Position = Pose.pose.position;
            if (FMath::IsFinite(Position.x) && FMath::IsFinite(Position.y) && FMath::IsFinite(Position.z))
            {
                RosPoints.Emplace(Position.x, Position.y, Position.z);
            }
        }

        AsyncTask(ENamedThreads::GameThread, [WeakThis, RosPoints = MoveTemp(RosPoints)]() mutable
        {
            if (WeakThis.IsValid()) WeakThis->ApplyPlannedPath(MoveTemp(RosPoints));
        });
    });

    // Migrate existing Blueprint/component instances that serialized the old
    // PointCloud2 topic before the UE-specific lightweight transport existed.
    if (CandidateTrajectoriesTopicName == TEXT("/trajs_visual"))
    {
        CandidateTrajectoriesTopicName = TEXT("/trajs_visual_ue");
    }

    CandidateTrajectoriesTopic = NewObject<UTopic>(this);
    CandidateTrajectoriesTopic->Init(Core, CandidateTrajectoriesTopicName, TEXT("std_msgs/Float32MultiArray"), 1, false);
    const int32 CandidatePointLimit = FMath::Max(1, MaxCandidatePoints);
    CandidateTrajectoriesTopic->Subscribe([WeakThis, CandidatePointLimit](TSharedPtr<FROSBaseMsg> Message)
    {
        if (!WeakThis.IsValid()) return;
        const TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> Array =
            StaticCastSharedPtr<ROSMessages::std_msgs::Float32MultiArray>(Message);
        if (!Array.IsValid() || Array->data.Num() < 4 || Array->data.Num() % 4 != 0)
        {
            UE_LOG(LogTemp, Warning, TEXT("ScoutMiniROS: rejected %s: expected non-empty [x,y,z,intensity] tuples"),
                *WeakThis->CandidateTrajectoriesTopicName);
            return;
        }

        const int32 TotalPoints = Array->data.Num() / 4;
        const int32 OutputCount = FMath::Min(TotalPoints, CandidatePointLimit);

        TArray<FCandidatePoint> RosPoints;
        RosPoints.Reserve(OutputCount);
        for (int32 OutputIndex = 0; OutputIndex < OutputCount; ++OutputIndex)
        {
            // Even sampling retains the full trajectory set instead of truncating its tail.
            const int32 SourceIndex = OutputCount == TotalPoints
                ? OutputIndex
                : static_cast<int32>((static_cast<int64>(OutputIndex) * TotalPoints) / OutputCount);
            const int32 DataIndex = SourceIndex * 4;

            FCandidatePoint Point;
            Point.Position = FVector(
                Array->data[DataIndex],
                Array->data[DataIndex + 1],
                Array->data[DataIndex + 2]);
            Point.Intensity = Array->data[DataIndex + 3];
            if (Point.Position.ContainsNaN() || !FMath::IsFinite(Point.Intensity)) continue;
            RosPoints.Add(Point);
        }

        AsyncTask(ENamedThreads::GameThread, [WeakThis, RosPoints = MoveTemp(RosPoints)]() mutable
        {
            if (WeakThis.IsValid()) WeakThis->ApplyCandidateTrajectories(MoveTemp(RosPoints));
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

    UE_LOG(LogTemp, Display, TEXT("ScoutMiniROS: subscribed %s, %s and %s; publishing %s and %s -> %s TF"),
        *CmdVelTopicName, *PathTopicName, *CandidateTrajectoriesTopicName,
        *OdometryTopicName, *OdometryFrameId, *BaseFrameId);
    UE_LOG(LogTemp, Display, TEXT("ScoutMiniROS: candidate display=%s point_size=%.1f timeout=%.2fs max_points=%d"),
        bShowCandidateTrajectories ? TEXT("true") : TEXT("false"),
        CandidatePointSize, CandidateTimeoutSeconds, MaxCandidatePoints);
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

    DrawPlannedPath();
    DrawCandidateTrajectories();
    if (CandidateTimeoutSeconds > 0.0f && CandidateTrajectoryPoints.Num() > 0
        && GetWorld()->GetTimeSeconds() - LastCandidateTrajectoryTime > CandidateTimeoutSeconds)
    {
        CandidateTrajectoryPoints.Reset();
    }

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

void UScoutMiniROSComponent::ApplyPlannedPath(TArray<FVector>&& RosPoints)
{
    PlannedPathWorldPoints.Reset(RosPoints.Num());
    for (const FVector& RosPoint : RosPoints)
    {
        // ROS uses metres with X forward/Y left/Z up. UE uses centimetres with Y right.
        const FVector LocalUE(RosPoint.X * 100.0, -RosPoint.Y * 100.0, RosPoint.Z * 100.0);
        FVector WorldPoint = OdometryOrigin.TransformPosition(LocalUE);
        WorldPoint.Z += PlannedPathHeightOffset;
        PlannedPathWorldPoints.Add(WorldPoint);
    }
}

void UScoutMiniROSComponent::DrawPlannedPath() const
{
    const AActor* Owner = GetOwner();
    if (!bShowPlannedPath || !GetWorld() || !Owner || PlannedPathWorldPoints.Num() < 2) return;

    const float VehicleHeightDelta = Owner->GetActorLocation().Z - OdometryOrigin.GetLocation().Z;

    for (int32 Index = 1; Index < PlannedPathWorldPoints.Num(); ++Index)
    {
        FVector Start = PlannedPathWorldPoints[Index - 1];
        FVector End = PlannedPathWorldPoints[Index];
        Start.Z += VehicleHeightDelta;
        End.Z += VehicleHeightDelta;
        DrawDebugLine(GetWorld(), Start, End,
            PlannedPathColor, false, 0.0f, 0, PlannedPathThickness);
    }
}

void UScoutMiniROSComponent::ApplyCandidateTrajectories(TArray<FCandidatePoint>&& RosPoints)
{
    CandidateTrajectoryPoints = MoveTemp(RosPoints);
    if (CandidateTrajectoryPoints.Num() == 0) return;

    if (!bLoggedFirstCandidateCloud)
    {
        UE_LOG(LogTemp, Display, TEXT("ScoutMiniROS: received first candidate array with %d drawable points"),
            CandidateTrajectoryPoints.Num());
        bLoggedFirstCandidateCloud = true;
    }

    float MinIntensity = TNumericLimits<float>::Max();
    float MaxIntensity = TNumericLimits<float>::Lowest();
    for (FCandidatePoint& Point : CandidateTrajectoryPoints)
    {
        MinIntensity = FMath::Min(MinIntensity, Point.Intensity);
        MaxIntensity = FMath::Max(MaxIntensity, Point.Intensity);

        const FVector LocalUE(Point.Position.X * 100.0, -Point.Position.Y * 100.0, Point.Position.Z * 100.0);
        Point.Position = OdometryOrigin.TransformPosition(LocalUE);
        Point.Position.Z += CandidateHeightOffset;
    }

    const float Range = MaxIntensity - MinIntensity;
    for (FCandidatePoint& Point : CandidateTrajectoryPoints)
    {
        float Alpha = Range > SMALL_NUMBER ? (Point.Intensity - MinIntensity) / Range : 0.0f;
        if (bInvertIntensityColors) Alpha = 1.0f - Alpha;
        Point.Color = FLinearColor::LerpUsingHSV(
            FLinearColor(LowScoreColor), FLinearColor(HighScoreColor), Alpha).ToFColor(true);
    }
    LastCandidateTrajectoryTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
}

void UScoutMiniROSComponent::DrawCandidateTrajectories() const
{
    const AActor* Owner = GetOwner();
    if (!bShowCandidateTrajectories || !GetWorld() || !Owner) return;

    const float VehicleHeightDelta = Owner->GetActorLocation().Z - OdometryOrigin.GetLocation().Z;
    for (const FCandidatePoint& Point : CandidateTrajectoryPoints)
    {
        FVector DrawPosition = Point.Position;
        DrawPosition.Z += VehicleHeightDelta;
        DrawDebugPoint(GetWorld(), DrawPosition, CandidatePointSize, Point.Color, false, 0.0f, 0);
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
    if (PathTopic) PathTopic->Unsubscribe();
    if (CandidateTrajectoriesTopic) CandidateTrajectoriesTopic->Unsubscribe();
    if (OdometryTopic) OdometryTopic->Unadvertise();
    if (TFTopic) TFTopic->Unadvertise();
    CmdVelTopic = nullptr;
    PathTopic = nullptr;
    CandidateTrajectoriesTopic = nullptr;
    OdometryTopic = nullptr;
    TFTopic = nullptr;
    PlannedPathWorldPoints.Reset();
    CandidateTrajectoryPoints.Reset();
}

void UScoutMiniROSComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    ShutdownROS();
    Super::EndPlay(EndPlayReason);
}
