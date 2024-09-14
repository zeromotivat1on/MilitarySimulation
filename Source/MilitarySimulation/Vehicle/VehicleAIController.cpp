// Fill out your copyright notice in the Description page of Project Settings.

#include "MilitarySimulation/Vehicle/VehicleAIController.h"
#include "MilitarySimulation/Vehicle/VehiclePath.h"
#include "MilitarySimulation/Vehicle/VehicleInterface.h"
#include "Components/SplineComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"

float FVehicleMovementBehavior::GetDesiredSpeedPathOffset(const float VehicleSpeed) const
{
	return UKismetMathLibrary::MapRangeClamped(VehicleSpeed, MinSpeed, MaxSpeed, MinSpeedPathOffset, MaxSpeedPathOffset);
}

float FVehicleMovementBehavior::GetDesiredSpeedPathOffset(const float VehicleSpeed, const float MinSpeedOverride, const float MaxSpeedOverride) const
{
	return UKismetMathLibrary::MapRangeClamped(VehicleSpeed, MinSpeedOverride, MaxSpeedOverride, MinSpeedPathOffset, MaxSpeedPathOffset);
}

float FVehicleMovementBehavior::GetDesiredPathSpeed(const float Angle) const
{
	return UKismetMathLibrary::MapRangeClamped(FMath::Abs(Angle), 0.0f, TopSpeedAngle, MaxSpeed, MinSpeed);
}

float FVehicleMovementBehavior::GetDesiredPathSpeed(const float Angle, const float MinSpeedOverride, const float MaxSpeedOverride) const
{
	return UKismetMathLibrary::MapRangeClamped(FMath::Abs(Angle), 0.0f, TopSpeedAngle, MaxSpeedOverride, MinSpeedOverride);
}

float FVehicleMovementBehavior::GetDesiredSteeringPathOffset(const float VehicleSpeed) const
{
	return UKismetMathLibrary::MapRangeClamped(VehicleSpeed, MinSpeed, MaxSpeed, MinSteeringPathOffset, MaxSteeringPathOffset);
}

float FVehicleMovementBehavior::GetDesiredSteeringPathOffset(const float VehicleSpeed, const float MinSpeedOverride, const float MaxSpeedOverride) const
{
	return UKismetMathLibrary::MapRangeClamped(VehicleSpeed, MinSpeedOverride, MaxSpeedOverride, MinSteeringPathOffset, MaxSteeringPathOffset);
}

float FVehicleMovementBehavior::GetDesiredPathSteering(const float Angle) const
{
	return UKismetMathLibrary::MapRangeClamped(Angle, -SteeringAngle, SteeringAngle, -1.0f, 1.0f);
}

void FVehicleObstacleAvoidance::DetectStartData(const FVector& Location)
{
	if (GridLocations.IsEmpty())
	{
		return;
	}

	int32 NearestIndex = 0;
	float DistanceSquared = FVector::DistSquared(GridLocations[0], Location);
	
	for (int32 Idx = 1; Idx < GridLocations.Num(); ++Idx)
	{
		const float GridDistanceSquared = FVector::DistSquared(GridLocations[Idx], Location);
		if (GridDistanceSquared < DistanceSquared)
		{
			DistanceSquared = GridDistanceSquared;
			NearestIndex = Idx;
		}
	}

	StartIndex = NearestIndex;
	StartLocation = GridLocations[NearestIndex];
}

void FVehicleObstacleAvoidance::DetectTargetData(const AVehiclePath* Path)
{
	if (!Path || GridLocations.IsEmpty() || GridCollisions.IsEmpty())
	{
		return;
	}

	int32 BestIndex = 0;
	float BaseScore = 0.0f;

	for (int32 Idx = 0; Idx < GridCollisions.Num(); ++Idx)
	{
		if (GridCollisions[Idx] != 0)
		{
			continue;
		}

		const FVector PathLocation = Path->GetClosestPathLocation(GridLocations[Idx]);
		const float TargetToPathDistance = FVector::DistSquared(GridLocations[Idx], PathLocation);
		const float StartToTargetDistance = FVector::DistSquared(GridLocations[Idx], StartLocation);

		const float Score = StartToTargetDistance / (TargetToPathDistance + KINDA_SMALL_NUMBER);
		if (Score > BaseScore)
		{
			BaseScore = Score;
			BestIndex = Idx;
		}
	}

	TargetIndex = BestIndex;
	TargetLocation = GridLocations[BestIndex];
}

void FVehicleObstacleAvoidance::GeneratePath()
{
	const FGridPoint StartPoint = GridIndexToPoint(StartIndex);
	const FGridPoint TargetPoint = GridIndexToPoint(TargetIndex);
	PathPoints = UPathfindingLibrary::AStar(StartPoint, TargetPoint, GridCollisions, GridWidth, GridHeight);
}

FGridPoint FVehicleObstacleAvoidance::GridIndexToPoint(int32 Index) const
{
	return FGridPoint(Index / GridWidth, Index % GridWidth);
}

FVector FVehicleObstacleAvoidance::GridPointToLocation(const FGridPoint& Point, const FVector& forward, const FVector& right) const
{
	return GridStartLocation + forward * ((Point.X + 1) * GridGap) + right * ((Point.Y + 1) * GridGap);
}

void AVehicleAIController::BeginPlay()
{
	Super::BeginPlay();

	MinSpeed = GetDesiredMovementData().MinSpeed;
	MaxSpeed = GetDesiredMovementData().MaxSpeed;
}

void AVehicleAIController::OnPossess(APawn* InPawn)
{
	Super::OnPossess(InPawn);
	check(InPawn->Implements<UVehicleInterface>());
}

void AVehicleAIController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	
	UpdateObstacleDetection();
	UpdateObstacleAvoidance();

	UpdateSpeedExtremes();
	UpdateVehicleSpeed();
	
	UpdateVehicleSteering();
}


void AVehicleAIController::UpdateSpeedExtremes()
{
	// Default update based on desired movement data.
	MinSpeed = GetDesiredMovementData().MinSpeed;
	MaxSpeed = GetDesiredMovementData().MaxSpeed;

	// In case of obstacle avoidance, remain desired movement data speed extremes.
	if (IsAvoidingObstacle())
	{
		return;
	}

	if (FrontAIController && FrontAIController->GetPawn() && ShouldSlowdownFront(FrontAIController->GetPawn()))
	{
		MinSpeed = FrontAIController->MinSpeed;
		MaxSpeed = IVehicleInterface::Execute_GetForwardSpeed(FrontAIController->GetPawn());

		if (ObstacleAvoidanceData.bShowDebug)
		{
			const FString Message = FString::Printf(
				TEXT("%ls is slowed down due to %ls is too close front."),
				*GetName(), *FrontAIController->GetPawn()->GetName());
			GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::White, Message);
		}
	}
	else if (BackAIController && BackAIController->GetPawn() && ShouldSlowdownBack(BackAIController->GetPawn()))
	{
		const float DistanceToBack = FMath::Sqrt(GetDistanceSquaredBetween(BackAIController->GetPawn(), GetPawn()));
		const float DesiredDistanceToBack = ObstacleAvoidanceData.DesiredDistanceToBackVehicle;
		
		MaxSpeed = UKismetMathLibrary::MapRangeClamped(
			DistanceToBack,
			DesiredDistanceToBack * 0.5f, DesiredDistanceToBack * 1.5f,
			MaxSpeed, MinSpeed);

		if (ObstacleAvoidanceData.bShowDebug)
		{
			const FString Message = FString::Printf(
				TEXT("%ls is slowed down due to %ls is too far back."),
				*GetName(), *BackAIController->GetPawn()->GetName());
			GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::White, Message);
		}
	}
}

void AVehicleAIController::UpdateObstacleDetection()
{
	const FHitResult HitResultLeft  = LineTraceFrontObstacle(ESide::Left);
	const FHitResult HitResultRight = LineTraceFrontObstacle(ESide::Right);

	const bool bHit = HitResultLeft.bBlockingHit | HitResultRight.bBlockingHit;

	if (!FollowPath || !bHit || IsAvoidingObstacle())
	{
		return;
	}

	State = EVehicleControlState::CheckObstacle;

	const bool bAnyHitLocationZero = HitResultLeft.Location.IsZero() || HitResultRight.Location.IsZero();
	const FVector HitLocationSum = HitResultLeft.Location + HitResultRight.Location;
	const FVector HitLocation = HitLocationSum * (bAnyHitLocationZero ? 1.0f : 0.5f);
	const FVector StartLocation = FollowPath->GetClosestPathLocation(HitLocation);

	TraceObstacleAvoidGrid(StartLocation);

	ObstacleAvoidanceData.DetectStartData(GetVehicleLocation());
	ObstacleAvoidanceData.DetectTargetData(FollowPath);

	if (ObstacleAvoidanceData.bShowDebug)
	{
		DrawDebugSphere(GetWorld(), ObstacleAvoidanceData.StartLocation, ObstacleAvoidanceData.DebugSphereRadius, 6, FColor::Cyan, false, 20.0f);
		DrawDebugSphere(GetWorld(), ObstacleAvoidanceData.TargetLocation, ObstacleAvoidanceData.DebugSphereRadius, 6, FColor::Cyan, false, 20.0f);
	}
	
	ObstacleAvoidanceData.GeneratePath();

	SpawnObstacleAvoidPath();
}

void AVehicleAIController::UpdateObstacleAvoidance()
{
	if (!IsAvoidingObstacle())
	{
		return;
	}

	const float DistanceToTargetSquared = FVector::DistSquared(GetVehicleLocation(), ObstacleAvoidanceData.TargetLocation);
	const float AcceptableFinishDistanceSquared = ObstacleAvoidanceData.GridGap * ObstacleAvoidanceData.GridGap * 2.0f;
	const bool bWithinAcceptableDistance = DistanceToTargetSquared < AcceptableFinishDistanceSquared; 

	const float TargetPathDistance = DefaultFollowPath->Spline->GetDistanceAlongSplineAtLocation(ObstacleAvoidanceData.TargetLocation, ESplineCoordinateSpace::World);
	const float VehiclePathDistance = DefaultFollowPath->Spline->GetDistanceAlongSplineAtLocation(GetVehicleLocation(), ESplineCoordinateSpace::World);
	const bool bVehicleFurtherTarget = VehiclePathDistance > TargetPathDistance;
	
	if (bWithinAcceptableDistance || bVehicleFurtherTarget)
	{
		ResetFollowPath();
	}
}

void AVehicleAIController::UpdateVehicleSpeed()
{
	const float TopSpeed = DetermineTopSpeed();
	const float CurrentSpeed = IVehicleInterface::Execute_GetForwardSpeed(GetPawn());

	if (CurrentSpeed > TopSpeed)
	{
		IVehicleInterface::Execute_SetThrottle(GetPawn(), 0.0f);

		const float SmoothBrake = UKismetMathLibrary::MapRangeClamped(CurrentSpeed, TopSpeed + 1.0f, TopSpeed + 5.0f, 0.0f, 1.0f);
		IVehicleInterface::Execute_SetBrake(GetPawn(), SmoothBrake);
	}
	else
	{
		IVehicleInterface::Execute_SetThrottle(GetPawn(), 1.0f);
		IVehicleInterface::Execute_SetBrake(GetPawn(), 0.0f);
	}

	if (GetDesiredMovementData().bShowDebug)
	{
		{
			const FString Message = FString::Printf(
				TEXT("%ls Speed (min = %.2f, current = %.2f, top = %.2f, max = %.2f)"),
				*GetName(), MinSpeed, CurrentSpeed, TopSpeed, MaxSpeed);
			GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Magenta, Message);	
		}

		{
			const FString Message = FString::Printf(
				TEXT("%ls Input (throttle = %.2f, brake = %.2f)"),
				*GetName(), IVehicleInterface::Execute_GetThrottle(GetPawn()), IVehicleInterface::Execute_GetBrake(GetPawn()));
			GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Magenta, Message);
		}
	}
}

float AVehicleAIController::DetermineTopSpeed()
{
	if (!FollowPath)
	{
		return GetDesiredMovementData().MaxSpeed;
	}

	const float CurrentSpeed = IVehicleInterface::Execute_GetForwardSpeed(GetPawn());
	const FVector VehicleFrontLocation = IVehicleInterface::Execute_GetFrontLocation(GetPawn());
	const FVector VehiclePathLocation = FollowPath->GetClosestPathLocation(VehicleFrontLocation, GetDesiredMovementData().GetDesiredSpeedPathOffset(CurrentSpeed, MinSpeed, MaxSpeed));
	const FRotator LookAtPath = UKismetMathLibrary::FindLookAtRotation(VehicleFrontLocation, VehiclePathLocation);
	const FRotator NormalizedLookAtPath = UKismetMathLibrary::NormalizedDeltaRotator(LookAtPath, GetVehicleRotation());

	if (GetDesiredMovementData().bShowDebug)
	{
		const FString Message = FString::Printf(TEXT("%ls Top Speed Angle = %.2f"), *GetName(), NormalizedLookAtPath.Yaw);
		GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Magenta, Message);
		
		DrawDebugLine(GetWorld(), GetVehicleLocation(), VehiclePathLocation, FColor::Magenta, false, 0.0f, 0, 4.0f);
	}

	return GetDesiredMovementData().GetDesiredPathSpeed(NormalizedLookAtPath.Yaw, MinSpeed, MaxSpeed);
}

void AVehicleAIController::UpdateVehicleSteering()
{
	const float Steering = DetermineSteering();
	IVehicleInterface::Execute_SetSteering(GetPawn(), Steering);

	if (GetDesiredMovementData().bShowDebug)
	{
		const FString Message = FString::Printf(TEXT("%ls Input (steering = %.2f)"), *GetName(), Steering);
		GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Blue, Message);
	}
}

float AVehicleAIController::DetermineSteering()
{
	if (!FollowPath)
	{
		return 0.0f;
	}
	
	const float CurrentSpeed = IVehicleInterface::Execute_GetForwardSpeed(GetPawn());
	const FVector VehicleFrontLocation = IVehicleInterface::Execute_GetFrontLocation(GetPawn());
	const FVector VehiclePathLocation = FollowPath->GetClosestPathLocation(VehicleFrontLocation, GetDesiredMovementData().GetDesiredSteeringPathOffset(CurrentSpeed));
	const FRotator LookAtPath = UKismetMathLibrary::FindLookAtRotation(VehicleFrontLocation, VehiclePathLocation);
	const FRotator NormalizedLookAtPath = UKismetMathLibrary::NormalizedDeltaRotator(LookAtPath, GetVehicleRotation());

	if (GetDesiredMovementData().bShowDebug)
	{
		const FString Message = FString::Printf(TEXT("%ls Steering Angle = %.2f"), *GetName(), NormalizedLookAtPath.Yaw);
		GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Blue, Message);
		
		DrawDebugLine(GetWorld(), GetVehicleLocation(), VehiclePathLocation, FColor::Blue, false, 0.0f, 0, 4.0f);
	}
	
	return GetDesiredMovementData().GetDesiredPathSteering(NormalizedLookAtPath.Yaw);
}

FHitResult AVehicleAIController::LineTraceFrontObstacle(ESide Side) const
{
	const float SideScale = Side == ESide::Right ? 1.0f : -1.0f;
	const float SideOffset = IVehicleInterface::Execute_GetChassisWidth(GetPawn()) * 0.5f;
	
	const FVector Start = IVehicleInterface::Execute_GetFrontLocation(GetPawn()) + (GetVehicleRightVector() * SideOffset * SideScale);
	const FVector End = Start + (GetVehicleForwardVector() * IVehicleInterface::Execute_GetForwardSpeed(GetPawn())) + (GetVehicleForwardVector() * ObstacleAvoidanceData.ExtraTraceLength);

	FHitResult HitResult;
	const ETraceTypeQuery TraceChannel = UEngineTypes::ConvertToTraceType(ObstacleAvoidanceData.DetectCollisionChannel);
	const TArray<AActor*> IgnoreActors = { GetPawn() }; // ignore self
	const EDrawDebugTrace::Type DebugDrawType = ObstacleAvoidanceData.bShowDebug ? EDrawDebugTrace::ForOneFrame : EDrawDebugTrace::None;
	UKismetSystemLibrary::LineTraceSingle(this, Start, End, TraceChannel, false, IgnoreActors, DebugDrawType, HitResult, true);

	return HitResult;
}

void AVehicleAIController::ResetFollowPath()
{
	if (FollowPath == DefaultFollowPath)
	{
		return;
	}

	if (FollowPath)
	{
		FollowPath->Destroy();
	}
	
	FollowPath = DefaultFollowPath;
	
	State = EVehicleControlState::FollowPath;
}

void AVehicleAIController::TraceObstacleAvoidGrid(const FVector& StartLocation)
{
	const FVector StartLocationDeltaForward = GetVehicleForwardVector() * ((ObstacleAvoidanceData.GridHeight + 1) * ObstacleAvoidanceData.GridGap * 0.5f);
	const FVector StartLocationDeltaRight = GetVehicleRightVector() * ((ObstacleAvoidanceData.GridWidth + 1) * ObstacleAvoidanceData.GridGap * 0.5f);

	ObstacleAvoidanceData.GridStartLocation = StartLocation - StartLocationDeltaForward - StartLocationDeltaRight;
	
	ObstacleAvoidanceData.GridLocations.SetNumZeroed(ObstacleAvoidanceData.GridHeight * ObstacleAvoidanceData.GridWidth);
	ObstacleAvoidanceData.GridCollisions.SetNumZeroed(ObstacleAvoidanceData.GridHeight * ObstacleAvoidanceData.GridWidth);
	
	FVector ColumnLocation = ObstacleAvoidanceData.GridStartLocation;
	for (int32 HeightIdx = 0; HeightIdx < ObstacleAvoidanceData.GridHeight; ++HeightIdx)
	{
		ColumnLocation += GetVehicleForwardVector() * ObstacleAvoidanceData.GridGap;

		FVector RowLocation = ColumnLocation;
		for (int32 WidthIdx = 0; WidthIdx < ObstacleAvoidanceData.GridWidth; ++WidthIdx)
		{
			RowLocation += GetVehicleRightVector() * ObstacleAvoidanceData.GridGap;

			const int32 GridIdx = HeightIdx * ObstacleAvoidanceData.GridWidth + WidthIdx;  
			ObstacleAvoidanceData.GridLocations[GridIdx] = RowLocation;

			FHitResult HitResult;
			const float Radius = ObstacleAvoidanceData.GridGap;
			const ETraceTypeQuery TraceChannel = UEngineTypes::ConvertToTraceType(ObstacleAvoidanceData.GridCollisionChannel);
			const TArray<AActor*> IgnoreActors = { GetPawn() }; // ignore self
			const EDrawDebugTrace::Type DebugDrawType = EDrawDebugTrace::None;
			const bool bHit = UKismetSystemLibrary::SphereTraceSingle(this, RowLocation, RowLocation, Radius, TraceChannel, false, IgnoreActors, DebugDrawType, HitResult, true);
			ObstacleAvoidanceData.GridCollisions[GridIdx] = static_cast<int32>(bHit);

			if (ObstacleAvoidanceData.bShowDebug)
			{
				const FColor Color = (HeightIdx == 0 && WidthIdx == 0) ? FColor::Silver : (bHit ? FColor::Red : FColor::Green);
				DrawDebugSphere(GetWorld(), RowLocation, ObstacleAvoidanceData.DebugSphereRadius, 6, Color, false, 10.0f);
			}
		}
	}
}

void AVehicleAIController::SpawnObstacleAvoidPath()
{
	if (AVehiclePath* VehiclePath = GetWorld()->SpawnActor<AVehiclePath>())
	{
		VehiclePath->Spline->SetDrawDebug(true);
		VehiclePath->Spline->ClearSplinePoints();

		for (const FGridPoint PathPoint : ObstacleAvoidanceData.PathPoints)
		{
			const FVector PathPointLocation = ObstacleAvoidanceData.GridPointToLocation(PathPoint, GetVehicleForwardVector(), GetVehicleRightVector());
			const FVector SplinePointLocation = FVector(PathPointLocation.X, PathPointLocation.Y, GetVehicleLocation().Z);
			VehiclePath->Spline->AddSplinePoint(SplinePointLocation, ESplineCoordinateSpace::World);

			if (ObstacleAvoidanceData.bShowDebug)
			{
				DrawDebugSphere(GetWorld(), SplinePointLocation, ObstacleAvoidanceData.DebugSphereRadius, 6, FColor::Purple, false, 10.0f);
			}
		}
		
		FollowPath = VehiclePath;
		State = EVehicleControlState::AvoidObstacle;
	}
}

float AVehicleAIController::GetDistanceSquaredBetween(APawn* BackVehicle, APawn* FrontVehicle)
{
	return FVector::DistSquared(IVehicleInterface::Execute_GetFrontLocation(BackVehicle), IVehicleInterface::Execute_GetBackLocation(FrontVehicle));
}

float AVehicleAIController::GetPathDistanceBetween(APawn* BackVehicle, APawn* FrontVehicle)
{
	return FollowPath->GetDistanceBetween(IVehicleInterface::Execute_GetFrontLocation(BackVehicle), IVehicleInterface::Execute_GetBackLocation(FrontVehicle));
}

void AVehicleAIController::SetShowDebug(bool bShow)
{
	ObstacleAvoidanceData.bShowDebug = bShow;
	FollowMovementBehaviorData.bShowDebug = bShow;
	AvoidanceMovementBehaviorData.bShowDebug = bShow;
}

const FVehicleMovementBehavior& AVehicleAIController::GetDesiredMovementData() const
{
	switch(State)
	{
	case EVehicleControlState::CheckObstacle:
	case EVehicleControlState::AvoidObstacle:
		return AvoidanceMovementBehaviorData;
	case EVehicleControlState::FollowPath:
	default:
		return FollowMovementBehaviorData;	
	}
}

bool AVehicleAIController::IsFollowingPath()
{
	return State == EVehicleControlState::FollowPath;
}

bool AVehicleAIController::IsCheckingObstacle()
{
	return State == EVehicleControlState::CheckObstacle;
}

bool AVehicleAIController::IsAvoidingObstacle()
{
	return State == EVehicleControlState::AvoidObstacle;
}

FVector AVehicleAIController::GetVehicleLocation() const
{
	return GetPawn()->GetActorLocation();
}

FRotator AVehicleAIController::GetVehicleRotation() const
{
	return GetPawn()->GetActorRotation();
}

FVector AVehicleAIController::GetVehicleForwardVector() const
{
	return GetPawn()->GetActorForwardVector();
}

FVector AVehicleAIController::GetVehicleRightVector() const
{
	return GetPawn()->GetActorRightVector();
}

bool AVehicleAIController::ShouldSlowdownFront(APawn* InFrontVehicle)
{
	if (!InFrontVehicle)
	{
		return false;
	}
	
	return GetDistanceSquaredBetween(GetPawn(), InFrontVehicle) < FMath::Square(ObstacleAvoidanceData.DesiredDistanceToFrontVehicle)
		|| GetPathDistanceBetween	(GetPawn(), InFrontVehicle) < ObstacleAvoidanceData.DesiredPathDistanceToFrontVehicle;
}

bool AVehicleAIController::ShouldSlowdownBack(APawn* InBackVehicle)
{
	if (!InBackVehicle)
	{
		return false;
	}
	
	return GetDistanceSquaredBetween(InBackVehicle, GetPawn()) > FMath::Square(ObstacleAvoidanceData.DesiredDistanceToBackVehicle)
		|| GetPathDistanceBetween	(InBackVehicle, GetPawn()) > ObstacleAvoidanceData.DesiredPathDistanceToBackVehicle;
}
