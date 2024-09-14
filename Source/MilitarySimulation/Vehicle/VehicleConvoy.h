// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VehicleConvoy.generated.h"

class AVehicleAIController;
class AVehiclePath;
class APawn;

UCLASS()
class MILITARYSIMULATION_API AVehicleConvoy : public AActor
{
	GENERATED_BODY()
	
public:	
	AVehicleConvoy();

protected:
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	TArray<APawn*> Convoy = {};
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Convoy Settings")
	AVehiclePath* FollowPath = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Settings")
	bool bSpawnOnBeginPlay = false;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Settings")
	int32 SpawnCount = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Settings")
	TSubclassOf<APawn> SpawnClass = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Settings")
	TSubclassOf<AVehicleAIController> SpawnAIControllerClass = nullptr;

	// Initial spawn spacing between vehicles.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spawn Settings")
	float SpawnSpacing = 1500.0f;
	
protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

	// Spawn convoy based on settings.
	UFUNCTION(BlueprintCallable)
	void Spawn();

	UFUNCTION(BlueprintCallable)
	bool Remove(APawn* Vehicle);

	UFUNCTION(BlueprintCallable)
	bool RemoveBy(int32 ConvoyIndex);
	
	UFUNCTION(BlueprintCallable)
	void Clear();

	// Destroy convoy vehicles and clear convoy.
	UFUNCTION(BlueprintCallable)
	void DestroyConvoy();
	
protected:
	void UpdateVehicleNeighbours();
};
