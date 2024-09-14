// Fill out your copyright notice in the Description page of Project Settings.

#include "MilitarySimulation/Vehicle/VehiclePath.h"
#include "Components/SplineComponent.h"

AVehiclePath::AVehiclePath()
{
	PrimaryActorTick.bCanEverTick = false;

	Spline = CreateDefaultSubobject<USplineComponent>("Spline");
}

FVector AVehiclePath::GetClosestPathLocation(const FVector& WorldLocation, const float PathOffset) const
{
	const float PathDistance = Spline->GetDistanceAlongSplineAtLocation(WorldLocation, ESplineCoordinateSpace::World);
	return Spline->GetLocationAtDistanceAlongSpline(PathDistance + PathOffset, ESplineCoordinateSpace::World);
}

float AVehiclePath::GetDistanceBetween(const FVector& BackLocation, const FVector& FrontLocation)
{
	const float BackPathDistance = Spline->GetDistanceAlongSplineAtLocation(BackLocation, ESplineCoordinateSpace::World);
	const float FrontPathDistance = Spline->GetDistanceAlongSplineAtLocation(FrontLocation, ESplineCoordinateSpace::World);
	return FrontPathDistance - BackPathDistance;
}
