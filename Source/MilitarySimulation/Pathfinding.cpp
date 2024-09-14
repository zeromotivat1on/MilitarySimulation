// Fill out your copyright notice in the Description page of Project Settings.

#include "MilitarySimulation/Pathfinding.h"
#include "Containers/Queue.h"
#include "Containers/UnrealString.h"
#include "Algo/Reverse.h"

double UPathfindingLibrary::Heuristic(const FGridPoint& A, const FGridPoint& B, bool bUseOctileHeuristic)
{
	if (bUseOctileHeuristic)
	{
		// Octile heuristic (diagonal movement allowed)
		double dx = FMath::Abs(A.X - B.X);
		double dy = FMath::Abs(A.Y - B.Y);
		double D = 1.0;
		double D2 = 1.41421356237;
		return D * (dx + dy) + (D2 - 2 * D) * FMath::Min(dx, dy);
	}

	return FMath::Abs(A.X - B.X) + FMath::Abs(A.Y - B.Y); // Manhattan distance
}

bool UPathfindingLibrary::IsValid(const FGridPoint& P, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight)
{
	return P.Y >= 0 && P.Y < GridWidth && P.X >= 0 && P.X < GridHeight && Grid[P.X * GridWidth + P.Y] == 0;
}

TArray<FGridPoint> UPathfindingLibrary::GetNeighbors(const FGridPoint& P, bool bUseOctileHeuristic)
{
	TArray<FGridPoint> Neighbors = {
		FGridPoint(P.X + 1, P.Y), FGridPoint(P.X - 1, P.Y),
		FGridPoint(P.X, P.Y + 1), FGridPoint(P.X, P.Y - 1)
	};

	if (bUseOctileHeuristic)
	{
		Neighbors.Append({
			FGridPoint(P.X + 1, P.Y + 1), FGridPoint(P.X + 1, P.Y - 1),
			FGridPoint(P.X - 1, P.Y + 1), FGridPoint(P.X - 1, P.Y - 1)
		});
	}

	return Neighbors;
}

TArray<FGridPoint> UPathfindingLibrary::ReconstructPath(TSharedPtr<FNode> Node)
{
	TArray<FGridPoint> Path;
	while (Node.IsValid())
	{
		Path.Add(Node->Point);
		Node = Node->Parent;
	}
	Algo::Reverse(Path);
	return Path;
}

TArray<FGridPoint> UPathfindingLibrary::AStar(const FGridPoint& Start, const FGridPoint& Goal, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight, bool bUseOctileHeuristic)
{
	TQueue<TSharedPtr<FNode>> OpenList;
	TMap<FGridPoint, double> GScore;
	TMap<FGridPoint, TSharedPtr<FNode>> AllNodes;

	auto StartNode = MakeShared<FNode>(Start, 0.0, Heuristic(Start, Goal, bUseOctileHeuristic));
	OpenList.Enqueue(StartNode);
	GScore.Add(Start, 0.0);
	AllNodes.Add(Start, StartNode);

	while (!OpenList.IsEmpty())
	{
		TSharedPtr<FNode> Current;
		OpenList.Dequeue(Current);

		if (Current->Point == Goal)
		{
			return ReconstructPath(Current);
		}

		for (const FGridPoint& Neighbor : GetNeighbors(Current->Point, bUseOctileHeuristic))
		{
			if (!IsValid(Neighbor, Grid, GridWidth, GridHeight)) continue;
			double TentativeGScore = GScore[Current->Point] + 1.0;
			if (!GScore.Contains(Neighbor) || TentativeGScore < GScore[Neighbor])
			{
				GScore.Add(Neighbor, TentativeGScore);
				auto NeighborNode = MakeShared<FNode>(Neighbor, TentativeGScore, Heuristic(Neighbor, Goal, bUseOctileHeuristic), Current);
				OpenList.Enqueue(NeighborNode);
				AllNodes.Add(Neighbor, NeighborNode);
			}
		}
	}

	return TArray<FGridPoint>(); // No path found
}

