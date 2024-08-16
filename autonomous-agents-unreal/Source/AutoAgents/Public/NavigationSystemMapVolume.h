#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavigationValidLeisurePlaceVolume.h"

#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <variant>
#include <optional>
#include <random>

#include "NavigationSystemMapVolume.generated.h"

enum class TileType {
	EMPTY = 0,
	WALL = 1,
	AIRWALL = 2,
	BENCH = 3,
	VENDING_MACHINE = 4,
	TICKET_OFFICE = 5,
};

enum QuadNodeType {
	VALID = 0,
	INVALID = 1,
};

UCLASS()
class AUTOAGENTS_API ANavigationSystemMapVolume : public AVolume
{
	GENERATED_BODY()

public:
	ANavigationSystemMapVolume();

	struct QuadNode {
		int Level;
		std::tuple<int, int> Pos;
		std::unordered_set<int> Neighbor;

		QuadNode(int _Level, std::tuple<int, int> _Pos) : Level(_Level), Pos(_Pos) {
		}
	};
	using Path = std::optional<std::vector<int>>;

	// Environmental-map-related constants
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") float StationaryCellSize;
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") float MobileCellSize;
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") int DebugHUDPixelScale;
	// Every target is expanded on a quadtree map until it touches any node at a certain level Lt
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") int PathwayFindingExpansionMinimalLevel;
	// Display the bounding box of obstacles (used for debugging)
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayOverlapBoudingBox;
	// Display HUD
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayHUD;

	virtual void Tick(float DeltaTime) override;

	int GetQuadtreeNode(FVector2D Pos) const;
	bool IsEmptyPosition(FVector2D Pos, bool AllowAirWall = false) const;
	std::optional<FVector> GetNearestValidPosition(FVector Pos) const;
	FVector2D ActualPosition(int NodeId) const;
	double DefaultZ() const;

	// a path planning algorithm whose main phases are as follows : 
	// 1) insert a target into the quadtree map and expand the target if necessary; 
	// 2) from a given start node, try to find any node of the expanded target 
	// using one of several available search schemes; 
	// 3) if the search reaches an expanded target node, then backtrack 
	// through the visited nodes to construct an initial path back to the start node; 
	// 4) post-process this initial path to compute the final path
	Path PathFinding(FVector2D StartPos, FVector2D EndPos, bool bEnableExpansion = true) const;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditMove(bool bFinished) override;
#endif

protected:
	virtual void BeginPlay() override;

private:
	friend class ANavigationDebugHUD;
	friend class AAutoAgentsPlayerController;
	friend class AAgentAIController;
	friend class AAgentsGroupController;

	void InitGridmap();
	void VisualizeGridmap();
	void InitQuadtreeMap();
	void SearchQuadtreeNeighbor(int NodeI, int CurLevel, std::tuple<int, int> CurPos, std::tuple<int, int> DXY);
	void UpdateMobileMap(const TArray<AAgentAIController*> &Agents);
	void VisualizeQuadtreeMap(int NodeId = -1);
	void VisualizePathOnQuadtree(const Path &NavPath);

	using EnvObjQueryData = std::optional<std::tuple<int, FVector, FVector2D>>;

	void RegisterLeisurePlaces();
	FVector GetValidRandomLeisurePosition(std::mt19937& Rnd) const;

	const float BenchFacingOffset = 25.f;
	EnvObjQueryData QueryANearestValidBench(FVector Location);
	EnvObjQueryData QueryARandomValidBench(std::mt19937& Rnd);
	void ReleaseAnOccupiedBench(int BenchId);

	const float VendingMachineTangentOffset = -55.f;
	EnvObjQueryData QueryARandomVendingMachine(std::mt19937& Rnd);
	EnvObjQueryData QueryANearestVendingMachine(FVector Location);
	
	const float TicketOfficeEntranceDist = 500.f;
	const float TicketOfficeTangentOffset = 30.f;
	const float TicketOfficeFacingOffset = 70.f;
	EnvObjQueryData QueryAMostVacantTicketOffice(FVector Location);
	EnvObjQueryData QueryANearestTicketOffice(FVector Location);
	void QuitATicketOffice(int TicketOfficeId);

	FVector Corner;
	FVector Extent;

	// GRID MAP
	// Grid maps, which are useful in visual sensing, are also very useful for path planning.
	std::tuple<int, int> GridmapSize;
	std::vector<TileType> Gridmap;

	// SPECIALIZED ENVIRONMENT OBJECTS
	// Many of the environment objects are specialized to support quick perceptual queries.
	struct _EnvObj {
		TArray<AActor*> Benches;
		std::vector<int> BenchOccupiedNum;

		TArray<AActor*> VendingMachines;

		TArray<AActor*> TicketOffices;
		std::vector<int> TicketOfficeQueueNum;

		TArray<ANavigationValidLeisurePlaceVolume*> LeisurePlaces;
	} EnvObj;

	// QUADTREE MAP
	// The quadtree map supports fast online path planning. 
	int QuadtreeLevel;
	std::vector<std::tuple<int, int>> QuadtreeMapSize;
	std::vector<std::vector<QuadNodeType>> QuadtreeMap;
	std::vector<std::vector<int>> QuadtreeID;
	std::vector<QuadNode> QuadtreeNode;

	// MOBILE OBJECTS
	// A 2D grid map is used for sensing mobile objects (typically other pedestrians).
	std::tuple<int, int> MobileMapSize;
	TArray<TArray<AAgentAIController*>> MobileMap;

	// PATH MAPS
	// A-star path-way planning algorithm tool-kits
	Path SortedQExpansion(FVector2D EndPos, int &MidpointId) const;
	Path PMultiQBase(FVector2D StartPos, std::variant<FVector2D, int> EndPos) const;
};
