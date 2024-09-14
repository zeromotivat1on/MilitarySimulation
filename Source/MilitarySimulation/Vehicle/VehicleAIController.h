// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "MilitarySimulation/Pathfinding.h"
#include "MilitarySimulation/MilitarySimulation.h"
#include "VehicleAIController.generated.h"

class AVehiclePath;

// State of vehicle AI control.
UENUM(BlueprintType)
enum class EVehicleControlState : uint8
{
	FollowPath,		// following the default vehicle path
	CheckObstacle,	// checking the best way to avoid the obstacle
	AvoidObstacle,	// actually avoiding the obstacle
};

// Data to setup vehicle movement (speed, steering etc).
USTRUCT(BlueprintType)
struct FVehicleMovementBehavior
{
	GENERATED_BODY()

	// Draw vehicle movement related debug data.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowDebug = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	float DebugSphereRadius = 32.0f;
	
	// Angle between vehicle and follow path within which the vehicle can start steering its wheels.
	// Lower values make it more accurate and precise.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering", meta = (UIMin = "0", ClampMin = "0", UIMax = "90", ClampMax = "90", Units = "Degrees"))
	float SteeringAngle = 10.0f;

	// Offset to use when determining steering at min vehicle speed.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float MinSteeringPathOffset = 500.0f;

	// Offset to use when determining steering at max vehicle speed.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float MaxSteeringPathOffset = 1000.0f;

	// Vehicle min speed in km/h.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Speed", meta = (UIMin = "0", ClampMin = "0"))
	float MinSpeed = 40.0f;

	// Vehicle max speed in km/h.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Speed", meta = (UIMin = "0", ClampMin = "0"))
	float MaxSpeed = 60.0f;
	
	// Angle between vehicle and follow spline within which we override vehicle top speed.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Speed", meta = (UIMin = "0", ClampMin = "0", UIMax = "90", ClampMax = "90", Units = "Degrees"))
	float TopSpeedAngle = 20.0f;

	// Offset to use when determining spline min top speed.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Speed", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float MinSpeedPathOffset = 1000.0f;

	// Offset to use when determining spline max top speed.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Speed", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float MaxSpeedPathOffset = 3000.0f;

	// Get desired offset on vehicle path for smooth speed.
	float GetDesiredSpeedPathOffset(const float VehicleSpeed) const;

	// Overload that takes custom min and max speed.
	float GetDesiredSpeedPathOffset(const float VehicleSpeed, const float MinSpeedOverride, const float MaxSpeedOverride) const;
	
	// Get desired vehicle speed based on angle between vehicle and its follow path.
	float GetDesiredPathSpeed(const float Angle) const;

	// Overload that takes custom min and max speed.
	float GetDesiredPathSpeed(const float Angle, const float MinSpeedOverride, const float MaxSpeedOverride) const;
	
	// Get desired offset on vehicle path for smooth wheel steering.
	float GetDesiredSteeringPathOffset(const float VehicleSpeed) const;

	// Overload that takes custom min and max speed.
	float GetDesiredSteeringPathOffset(const float VehicleSpeed, const float MinSpeedOverride, const float MaxSpeedOverride) const;
	
	// Get desired vehicle steering based on angle between vehicle and its follow path.
	float GetDesiredPathSteering(const float Angle) const;
};

// Data for default vehicle follow path.
USTRUCT(BlueprintType)
struct FVehicleFollowPath
{
	GENERATED_BODY()

	// Default path to follow.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	AVehiclePath* Path = nullptr;
};

// Data to setup vehicle obstacle avoidance.
USTRUCT(BlueprintType)
struct FVehicleObstacleAvoidance
{
	GENERATED_BODY()

	// Draw obstacle avoidance related debug data.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bShowDebug = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	float DebugSphereRadius = 24.0f;
	
	// Collision channel to detect vehicle obstacles.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance")
	TEnumAsByte<ECollisionChannel> DetectCollisionChannel = ECC_Visibility;
	
	// Obstacle avoidance trace length depends on vehicle, but you can set additional trace length.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float ExtraTraceLength = 800.0f;

	// Desired direct distance between this and front vehicle to maintain.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float DesiredDistanceToFrontVehicle = 800.0f;

	// Desired path distance between this and front vehicle to maintain.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float DesiredPathDistanceToFrontVehicle = 500.0f;

	// Desired direct distance between this and front vehicle to maintain.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float DesiredDistanceToBackVehicle = 1600.0f;

	// Desired path distance between this and front vehicle to maintain.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	float DesiredPathDistanceToBackVehicle = 1000.0f;
	
	// Collision channel to detect vehicle obstacles during grid check.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid")
	TEnumAsByte<ECollisionChannel> GridCollisionChannel = ECC_Visibility;
	
	// Amount of grid cells in height.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid", meta = (UIMin = "0", ClampMin = "0"))
	int32 GridHeight = 16;

	// Amount of grid cells in width.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid", meta = (UIMin = "0", ClampMin = "0"))
	int32 GridWidth = 16;

	// Distance between grid cells.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grid", meta = (UIMin = "0", ClampMin = "0", Units = "Centimeters"))
	int32 GridGap = 256;
	
	// Start obstacle avoidance location.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Avoidance")
	FVector StartLocation = FVector::ZeroVector;
	
	// Target obstacle avoidance location.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Avoidance")
	FVector TargetLocation = FVector::ZeroVector;
	
	// Initial grid location.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Grid")
	FVector GridStartLocation = FVector::ZeroVector;

	// Start obstacle avoidance grid index.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Grid")
	int32 StartIndex = 0;

	// Target obstacle avoidance grid index.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Grid")
	int32 TargetIndex = 0;
	
	// Array of grid cell collision status.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Grid")
	TArray<int32> GridCollisions = {};

	// Array of grid cell world locations.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Grid")
	TArray<FVector> GridLocations = {};

	// Generated optimal path of points to follow. 
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Grid")
	TArray<FGridPoint> PathPoints = {};
	
	// Detect start obstacle avoidance location and index based on a given location.
	void DetectStartData(const FVector& Location);

	// Detect target obstacle avoidance location and index based on a given vehicle path.
	// Note: start data should be detected before calling this.
	void DetectTargetData(const AVehiclePath* Path);

	// Generate optimal point path for vehicle to follow.
	void GeneratePath();

	// Convert grid index to path point.
	FGridPoint GridIndexToPoint(int32 Index) const;

	// Convert path point to world location.
	// Additional forward and right vehicle vectors can be given to correct path world position in case of rotated grid.  
	FVector GridPointToLocation(const FGridPoint& Point, const FVector& forward = FVector::ForwardVector, const FVector& right = FVector::RightVector) const;
};

/**
 * AI Controller designed for any abstract vehicle.
 * Supports spline follow, runtime obstacle detection and avoidance, with fine tune settings.
 */
UCLASS()
class MILITARYSIMULATION_API AVehicleAIController : public AAIController
{
	GENERATED_BODY()

protected:
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	EVehicleControlState State = EVehicleControlState::FollowPath;

	// Movement data for default path follow.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVehicleMovementBehavior FollowMovementBehaviorData;

	// Movement data for obstacle avoidance.
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVehicleMovementBehavior AvoidanceMovementBehaviorData;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVehicleObstacleAvoidance ObstacleAvoidanceData;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	AVehiclePath* DefaultFollowPath = nullptr;
	
	// Current vehicle path to follow.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	AVehiclePath* FollowPath = DefaultFollowPath;
	
	// Current vehicle min speed.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, meta = (ForceUnits = "KilometersPerHour"))
	float MinSpeed = 0.0f;

	// Current vehicle max speed.
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, meta = (ForceUnits = "KilometersPerHour"))
	float MaxSpeed = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	AVehicleAIController* FrontAIController = nullptr;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	AVehicleAIController* BackAIController = nullptr;

protected:
	virtual void BeginPlay() override;
	
public:
	virtual void OnPossess(APawn* InPawn) override;
	virtual void Tick(float DeltaSeconds) override;

	UFUNCTION(BlueprintCallable)
	void SetFrontAIController(AVehicleAIController* InFrontVehicle) { FrontAIController = InFrontVehicle; }

	UFUNCTION(BlueprintCallable)
	void SetBackAIController(AVehicleAIController* InBackVehicle) { BackAIController = InBackVehicle; }

	UFUNCTION(BlueprintCallable)
	void SetShowDebug(bool bShow);
	
	// Get movement data based on control state.
	UFUNCTION(BlueprintGetter)
	const FVehicleMovementBehavior& GetDesiredMovementData() const;

	UFUNCTION(BlueprintGetter)
	bool IsFollowingPath();

	UFUNCTION(BlueprintGetter)
	bool IsCheckingObstacle();
	
	UFUNCTION(BlueprintGetter)
	bool IsAvoidingObstacle();

	UFUNCTION(BlueprintGetter)
	FVector GetVehicleLocation() const;

	UFUNCTION(BlueprintGetter)
	FRotator GetVehicleRotation() const;
	
	UFUNCTION(BlueprintGetter)
	FVector GetVehicleForwardVector() const;

	UFUNCTION(BlueprintGetter)
	FVector GetVehicleRightVector() const;

	// Check if a given front vehicle is too close, so this one should be slowed down.
	UFUNCTION(BlueprintCallable)
	bool ShouldSlowdownFront(APawn* InFrontVehicle);

	// Check if a given back vehicle is too far, so this one should be slowed down.
	UFUNCTION(BlueprintCallable)
	bool ShouldSlowdownBack(APawn* InBackVehicle);

protected:
	UFUNCTION(BlueprintCallable)
	void UpdateSpeedExtremes();

	// Detect new obstacles.
	UFUNCTION(BlueprintCallable)
	void UpdateObstacleDetection();

	// Check obstacle avoidance progress.
	UFUNCTION(BlueprintCallable)
	void UpdateObstacleAvoidance();
	
	// Update vehicle speed (throttle and brake).
	UFUNCTION(BlueprintCallable)
	void UpdateVehicleSpeed();
	
	// Calculate best desired top speed.
	UFUNCTION(BlueprintCallable)
	float DetermineTopSpeed();

	// Update vehicle wheels steering.
	UFUNCTION(BlueprintCallable)
	void UpdateVehicleSteering();

	// Calculate best desired steering.
	UFUNCTION(BlueprintCallable)
	float DetermineSteering();
	
	/**
	 * Detect obstacles at front of vehicle by firing a line trace.
	 * Trace length varies depending on vehicle speed, higher speed - longer trace.
	 * @param Side From which vehicle side we should start line trace.
	 * @return Hit result.
	 */
	UFUNCTION(BlueprintCallable)
	FHitResult LineTraceFrontObstacle(ESide Side) const;

	// Reset path to follow to default state.
	UFUNCTION(BlueprintCallable)
	void ResetFollowPath();

	// Perform trace grid to detect obstacle avoidance path at given center location.
	UFUNCTION(BlueprintCallable)
	void TraceObstacleAvoidGrid(const FVector& StartLocation);

	// Spawn vehicle path to avoid an obstacle.
	UFUNCTION(BlueprintCallable)
	void SpawnObstacleAvoidPath();
	
	UFUNCTION(BlueprintCallable)
	float GetDistanceSquaredBetween(APawn* BackVehicle, APawn* FrontVehicle);

	UFUNCTION(BlueprintCallable)
	float GetPathDistanceBetween(APawn* BackVehicle, APawn* FrontVehicle);
};
