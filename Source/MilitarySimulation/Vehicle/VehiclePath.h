// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VehiclePath.generated.h"

class USplineComponent;

UCLASS()
class MILITARYSIMULATION_API AVehiclePath : public AActor
{
	GENERATED_BODY()
	
public:	
	AVehiclePath();

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	USplineComponent* Spline = nullptr;

public:
	UFUNCTION(BlueprintCallable)
	FVector GetClosestPathLocation(const FVector& WorldLocation, const float PathOffset = 0.0f) const;

	// Get path distance between two given world locations.
	UFUNCTION(BlueprintCallable)
	float GetDistanceBetween(const FVector& BackLocation, const FVector& FrontLocation);
};
