// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "VehicleInterface.generated.h"

// This class does not need to be modified.
UINTERFACE(MinimalAPI)
class UVehicleInterface : public UInterface
{
	GENERATED_BODY()
};

// Interface to describe abstract vehicle.
class MILITARYSIMULATION_API IVehicleInterface
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	void SetThrottle(float Throttle);

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	void SetBrake(float Brake);

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	void SetHandbrake(float Brake);

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	void SetSteering(float Steering);

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	float GetThrottle() const;

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	float GetBrake() const;

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	float GetHandbrake() const;

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	float GetSteering() const;

	// Get current forward vehicle speed in kilometers per hour.
	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	float GetForwardSpeed() const;

	// Get most front centered location of the vehicle.
	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	FVector GetFrontLocation() const;

	// Get most back centered location of the vehicle.
	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	FVector GetBackLocation() const;

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	float GetChassisWidth() const;

	// Returns whether this vehicle can be overtaken.
	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	bool CanOvertake() const;

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	void SetFollowPath(class AVehiclePath* FollowPath);
};

// This class does not need to be modified.
UINTERFACE(MinimalAPI)
class UConvoyVehicleInterface : public UVehicleInterface
{
	GENERATED_BODY()
};

// Vehicle that can be added to vehicle convoy.
class MILITARYSIMULATION_API IConvoyVehicleInterface : public IVehicleInterface
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	void SetConvoyIndex(int32 Index);

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	int32 GetConvoyIndex() const;
	
	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	void OnAddedToConvoy();

	UFUNCTION(BlueprintCallable, BlueprintImplementableEvent)
	void OnRemovedFromConvoy();
};
