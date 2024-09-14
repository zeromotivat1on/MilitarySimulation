// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "Pathfinding.generated.h"

USTRUCT(BlueprintType)
struct FGridPoint
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	int32 X;

	UPROPERTY(BlueprintReadWrite)
	int32 Y;

	FGridPoint(): X(0), Y(0) {}
	FGridPoint(int32 InX, int32 InY) : X(InX), Y(InY) {}

	bool operator==(const FGridPoint& Other) const
	{
		return X == Other.X && Y == Other.Y;
	}

	bool operator!=(const FGridPoint& Other) const
	{
		return !(*this == Other);
	}
};

FORCEINLINE uint32 GetTypeHash(const FGridPoint& Point)
{
	return HashCombine(::GetTypeHash(Point.X), ::GetTypeHash(Point.Y));
}

UCLASS()
class UPathfindingLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// Perform A* pathfinding algorithm.
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	static TArray<FGridPoint> AStar(const FGridPoint& Start, const FGridPoint& Goal, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight, bool bUseOctileHeuristic = true);

private:
	struct FNode
	{
		FGridPoint Point;
		double G;
		double H;
		TSharedPtr<FNode> Parent;

		FNode(const FGridPoint& InPoint, double InG, double InH, TSharedPtr<FNode> InParent = nullptr)
			: Point(InPoint), G(InG), H(InH), Parent(InParent) {}

		double F() const { return G + H; }
	};

	struct FCompareNode
	{
		bool operator()(const TSharedPtr<FNode>& Lhs, const TSharedPtr<FNode>& Rhs) const
		{
			return Lhs->F() > Rhs->F();
		}
	};

	static double Heuristic(const FGridPoint& A, const FGridPoint& B, bool bUseOctileHeuristic);
	static bool IsValid(const FGridPoint& P, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight);
	static TArray<FGridPoint> GetNeighbors(const FGridPoint& P, bool bUseOctileHeuristic);
	static TArray<FGridPoint> ReconstructPath(TSharedPtr<FNode> Node);
};
