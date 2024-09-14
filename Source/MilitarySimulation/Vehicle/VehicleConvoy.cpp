// Fill out your copyright notice in the Description page of Project Settings.

#include "MilitarySimulation/Vehicle/VehicleConvoy.h"
#include "MilitarySimulation/Vehicle/VehiclePath.h"
#include "MilitarySimulation/Vehicle/VehicleInterface.h"
#include "MilitarySimulation/Vehicle/VehicleAIController.h"
#include "Components/SplineComponent.h"

AVehicleConvoy::AVehicleConvoy()
{
	PrimaryActorTick.bCanEverTick = true;
}

void AVehicleConvoy::BeginPlay()
{
	Super::BeginPlay();
	
	if (bSpawnOnBeginPlay)
	{
		Spawn();
	}
}

void AVehicleConvoy::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	UpdateVehicleNeighbours();
}

void AVehicleConvoy::UpdateVehicleNeighbours()
{
	for (int32 i = 0; i < Convoy.Num() - 1; ++i)
	{
		APawn* BackVehicle	= Convoy[i + 0];
		APawn* FrontVehicle	= Convoy[i + 1];

		if (!BackVehicle || !FrontVehicle)
		{
			continue;
		}
		
		auto* BackController	= Cast<AVehicleAIController>(BackVehicle->GetController());
		auto* FrontController	= Cast<AVehicleAIController>(FrontVehicle->GetController());

		if (!BackController || !FrontController)
		{
			continue;
		}

		BackController->SetFrontAIController(FrontController);
		FrontController->SetBackAIController(BackController);
	}
}

void AVehicleConvoy::Spawn()
{
	if (!Convoy.IsEmpty())
	{
		return;
	}
	
	for (int32 i = 0; i < SpawnCount; ++i)
	{
		const float SpawnDistance = i * SpawnSpacing;

		const FVector SpawnLocation = FollowPath->Spline->GetLocationAtDistanceAlongSpline(SpawnDistance, ESplineCoordinateSpace::World);
		const FRotator SpawnRotation = FollowPath->Spline->GetRotationAtDistanceAlongSpline(SpawnDistance, ESplineCoordinateSpace::World);
		
		FActorSpawnParameters SpawnParams;
		SpawnParams.Owner = Owner;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		if (APawn* SpawnedVehicle = GetWorld()->SpawnActor<APawn>(SpawnClass, SpawnLocation, SpawnRotation, SpawnParams))
		{
			check(SpawnedVehicle->Implements<UConvoyVehicleInterface>());
			
			Convoy.Add(SpawnedVehicle);
			
			SpawnedVehicle->AIControllerClass = SpawnAIControllerClass;
			SpawnedVehicle->SpawnDefaultController();

			IConvoyVehicleInterface::Execute_SetConvoyIndex(SpawnedVehicle, i);
			IConvoyVehicleInterface::Execute_SetFollowPath(SpawnedVehicle, FollowPath);
			IConvoyVehicleInterface::Execute_OnAddedToConvoy(SpawnedVehicle);
		}
	}
}

bool AVehicleConvoy::Remove(APawn* Vehicle)
{
	if (!Vehicle)
	{
		return false;
	}
	
	return RemoveBy(IConvoyVehicleInterface::Execute_GetConvoyIndex(Vehicle));
}

bool AVehicleConvoy::RemoveBy(int32 ConvoyIndex)
{
	for (APawn* Vehicle : Convoy)
	{
		if (ConvoyIndex == IConvoyVehicleInterface::Execute_GetConvoyIndex(Vehicle) && Convoy.RemoveSingle(Vehicle))
		{
			IConvoyVehicleInterface::Execute_OnRemovedFromConvoy(Vehicle);
			return true;
		}
	}
	
	return false;
}

void AVehicleConvoy::Clear()
{
	Convoy.Empty(SpawnCount);
}

void AVehicleConvoy::DestroyConvoy()
{
	for (APawn* Vehicle : Convoy)
	{
		if (Vehicle)
		{
			Vehicle->Destroy();
		}
	}

	Clear();
}
