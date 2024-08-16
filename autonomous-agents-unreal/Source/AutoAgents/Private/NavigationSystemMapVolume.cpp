#include "NavigationSystemMapVolume.h"
#include "NavigationDebugHUD.h"
#include "AgentAIController.h"
#include "Kismet/GameplayStatics.h"
#include "Components/BrushComponent.h"

ANavigationSystemMapVolume::ANavigationSystemMapVolume()
{
	PrimaryActorTick.bCanEverTick = true;
	BrushColor = FColor(200, 200, 200, 255);
	bColored = true;
}

void ANavigationSystemMapVolume::InitGridmap()
{
	float CollisionTestHeight = 100.f;
	FCollisionShape CCell = FCollisionShape::MakeBox(FVector(StationaryCellSize / 2.f, StationaryCellSize / 2.f, CollisionTestHeight));
	FVector Origin;

	GetActorBounds(false, Origin, Extent);
	Corner = FVector(Origin.X - Extent.X, Origin.Y - Extent.Y, Origin.Z);
	Extent.X *= 2.f;
	Extent.Y *= 2.f;
	std::get<0>(GridmapSize) = int(std::ceil(Extent.X / StationaryCellSize));
	std::get<1>(GridmapSize) = int(std::ceil(Extent.Y / StationaryCellSize));
	GEngine->AddOnScreenDebugMessage(-1, INFINITY, FColor::Green, FString::Printf(TEXT("GridmapSize = (%d, %d)"), std::get<0>(GridmapSize), std::get<1>(GridmapSize)));
	Gridmap.resize(std::get<0>(GridmapSize) * std::get<1>(GridmapSize));

	for (int i = 0; i < std::get<0>(GridmapSize); i++) {
		for (int j = 0; j < std::get<1>(GridmapSize); j++) {
			FCollisionQueryParams QueryParams;
			TArray<FOverlapResult> OverlapResults;
			FVector CurrentPos = Corner + FVector((i + 0.5) * StationaryCellSize, (j + 0.5) * StationaryCellSize, CollisionTestHeight);
			bool bOverlap = GetWorld()->OverlapMultiByChannel(OverlapResults, CurrentPos, FQuat::Identity, ECC_Visibility, CCell, QueryParams);

			TileType CurTile = TileType::EMPTY;
			if (bOverlap) {
				if (DisplayOverlapBoudingBox) {
					DrawDebugBox(GetWorld(), CurrentPos, CCell.GetBox(), FQuat::Identity, FColor::Green, false, INFINITY, 0, 1);
					DrawDebugPoint(GetWorld(), Corner + FVector((i + 0.5) * StationaryCellSize, (j + 0.5) * StationaryCellSize, CollisionTestHeight),
						5.0,
						FColor::Red, false, INFINITY, 0);
				}

				CurTile = TileType::WALL;

				for (const auto Result : OverlapResults) {
					AActor* CollidedActor = Result.GetActor();
					if (CollidedActor && CollidedActor->Tags.Num() > 0) {
						FString ObjectTag = CollidedActor->Tags[0].ToString();
						if (ObjectTag == "Bench") {
							CurTile = TileType::BENCH;
							if (!EnvObj.Benches.Contains(CollidedActor)) {
								EnvObj.Benches.Add(CollidedActor);
								EnvObj.BenchOccupiedNum.push_back(0);
							}
							break;
						}
						if (ObjectTag == "VendingMachine") {
							CurTile = TileType::VENDING_MACHINE;
							if (!EnvObj.VendingMachines.Contains(CollidedActor)) {
								EnvObj.VendingMachines.Add(CollidedActor);
							}
							break;
						}
						if (ObjectTag == "TicketOffice") {
							CurTile = TileType::TICKET_OFFICE;
							if (!EnvObj.TicketOffices.Contains(CollidedActor)) {
								EnvObj.TicketOffices.Add(CollidedActor);
								EnvObj.TicketOfficeQueueNum.push_back(0);
							}
							break;
						}
					}
				}
			}

			Gridmap[i * std::get<1>(GridmapSize) + j] = CurTile;
		}
	}
	GEngine->AddOnScreenDebugMessage(-1, INFINITY, FColor::Green, 
		FString::Printf(TEXT("Benches.Num() = %d; VendingMachines.Num() = %d, TicketOffices.Num() = %d, LeisurePlaces.Num() = %d"), 
			EnvObj.Benches.Num(), 
			EnvObj.VendingMachines.Num(),
			EnvObj.TicketOffices.Num(),
			EnvObj.LeisurePlaces.Num()));

	for (int i = 0; i < std::get<0>(GridmapSize); i++)
		for (int j = 0; j < std::get<1>(GridmapSize); j++) {
			if (Gridmap[i * std::get<1>(GridmapSize) + j] != TileType::EMPTY && Gridmap[i * std::get<1>(GridmapSize) + j] != TileType::AIRWALL /* Airwall cannot expand */) {
				for (int dx = -1; dx <= 1; dx++) {
					for (int dy = -1; dy <= 1; dy++) {
						if (i + dx >= 0 && i + dx < std::get<0>(GridmapSize) &&
							j + dy >= 0 && j + dy < std::get<1>(GridmapSize) &&
							Gridmap[(i + dx) * std::get<1>(GridmapSize) + (j + dy)] == TileType::EMPTY)
							Gridmap[(i + dx) * std::get<1>(GridmapSize) + (j + dy)] = TileType::AIRWALL;
					}
				}
			}
		}
}

void ANavigationSystemMapVolume::VisualizeGridmap()
{
	UTexture2D* GridmapTexture = UTexture2D::CreateTransient(std::get<1>(GridmapSize) * DebugHUDPixelScale, std::get<0>(GridmapSize) * DebugHUDPixelScale, PF_B8G8R8A8);
	FTexture2DMipMap& Mip = GridmapTexture->GetPlatformData()->Mips[0];
	void* Data = Mip.BulkData.Lock(LOCK_READ_WRITE);
	FColor* TextureData = static_cast<FColor*>(Data);
	for (int i = 0; i < std::get<0>(GridmapSize); i++) {
		for (int j = 0; j < std::get<1>(GridmapSize); j++) {
			if (Gridmap[i * std::get<1>(GridmapSize) + j] == TileType::EMPTY) {
				for (int dx = 0; dx < DebugHUDPixelScale; dx++)
					for (int dy = 0; dy < DebugHUDPixelScale; dy++) {
						TextureData[(i * DebugHUDPixelScale + dx) * std::get<1>(GridmapSize) * DebugHUDPixelScale + (j * DebugHUDPixelScale + dy)] = FColor::Blue;
					}
			}
			else {
				for (int dx = 0; dx < DebugHUDPixelScale; dx++)
					for (int dy = 0; dy < DebugHUDPixelScale; dy++) {
						TextureData[(i * DebugHUDPixelScale + dx) * std::get<1>(GridmapSize) * DebugHUDPixelScale + (j * DebugHUDPixelScale + dy)] = FColor::Black;
					}
			}
		}
	}
	Mip.BulkData.Unlock();
	ANavigationDebugHUD* HUD = Cast<ANavigationDebugHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
	if (HUD) {
		HUD->SetTexture(GridmapTexture);
	}
}

void ANavigationSystemMapVolume::InitQuadtreeMap()
{
	int level = 1;
	QuadtreeMapSize.push_back(GridmapSize);
	QuadtreeMap.push_back(std::vector<QuadNodeType>(Gridmap.size()));
	for (int i = 0; i < Gridmap.size(); i++) {
		QuadtreeMap[0][i] = (Gridmap[i] == TileType::EMPTY ? QuadNodeType::VALID : QuadNodeType::INVALID);
	}

	while (true) {
		bool bExistValid = false;
		QuadtreeMapSize.push_back(std::make_tuple(std::get<0>(QuadtreeMapSize[level - 1]) / 2,
			std::get<1>(QuadtreeMapSize[level - 1]) / 2));
		QuadtreeMap.push_back(std::vector<QuadNodeType>(std::get<0>(QuadtreeMapSize[level]) * std::get<1>(QuadtreeMapSize[level])));

		for (int i = 0; i < std::get<0>(QuadtreeMapSize[level]); i++) {
			for (int j = 0; j < std::get<1>(QuadtreeMapSize[level]); j++) {
				int value = 0;
				value += QuadtreeMap[level - 1][(i * 2 + 0) * std::get<1>(QuadtreeMapSize[level - 1]) + (j * 2 + 0)];
				value += QuadtreeMap[level - 1][(i * 2 + 1) * std::get<1>(QuadtreeMapSize[level - 1]) + (j * 2 + 0)];
				value += QuadtreeMap[level - 1][(i * 2 + 0) * std::get<1>(QuadtreeMapSize[level - 1]) + (j * 2 + 1)];
				value += QuadtreeMap[level - 1][(i * 2 + 1) * std::get<1>(QuadtreeMapSize[level - 1]) + (j * 2 + 1)];
				if (value == 0) {
					QuadtreeMap[level][i * std::get<1>(QuadtreeMapSize[level]) + j] = QuadNodeType::VALID;
					QuadtreeMap[level - 1][(i * 2 + 0) * std::get<1>(QuadtreeMapSize[level - 1]) + (j * 2 + 0)] = QuadNodeType::INVALID;
					QuadtreeMap[level - 1][(i * 2 + 1) * std::get<1>(QuadtreeMapSize[level - 1]) + (j * 2 + 0)] = QuadNodeType::INVALID;
					QuadtreeMap[level - 1][(i * 2 + 0) * std::get<1>(QuadtreeMapSize[level - 1]) + (j * 2 + 1)] = QuadNodeType::INVALID;
					QuadtreeMap[level - 1][(i * 2 + 1) * std::get<1>(QuadtreeMapSize[level - 1]) + (j * 2 + 1)] = QuadNodeType::INVALID;
					bExistValid = true;
				}
				else {
					QuadtreeMap[level][i * std::get<1>(QuadtreeMapSize[level]) + j] = QuadNodeType::INVALID;
				}
			}
		}

		if (!bExistValid)
			break;
		level += 1;
	}

	QuadtreeLevel = level;
	GEngine->AddOnScreenDebugMessage(-1, INFINITY, FColor::Green, FString::Printf(TEXT("QuadtreeLevel = %d"), QuadtreeLevel));

	for (level = 0; level < QuadtreeLevel; level++) {
		QuadtreeID.push_back(std::vector<int>(QuadtreeMap[level].size()));
		for (int i = 0; i < std::get<0>(QuadtreeMapSize[level]); i++) {
			for (int j = 0; j < std::get<1>(QuadtreeMapSize[level]); j++) {
				if (QuadtreeMap[level][i * std::get<1>(QuadtreeMapSize[level]) + j] == QuadNodeType::VALID) {
					QuadtreeID[level][i * std::get<1>(QuadtreeMapSize[level]) + j] = QuadtreeNode.size();
					QuadtreeNode.push_back(QuadNode(level, std::make_tuple(i, j)));
				}
			}
		}
	}
	GEngine->AddOnScreenDebugMessage(-1, INFINITY, FColor::Green, FString::Printf(TEXT("QuadtreeNode.size() = %d"), QuadtreeNode.size()));

	std::vector<std::tuple<int, int>> DXY = {
		std::make_tuple(-1, -1),
		std::make_tuple(-1, +0),
		std::make_tuple(-1, +1),
		std::make_tuple(+0, -1),
		std::make_tuple(+0, +1),
		std::make_tuple(+1, -1),
		std::make_tuple(+1, +0),
		std::make_tuple(+1, +1)
	};

	for (int i = 0; i < QuadtreeNode.size(); i++) {
		const QuadNode& q = QuadtreeNode[i];
		for (const auto& dxy : DXY) {
			SearchQuadtreeNeighbor(i, q.Level, q.Pos, dxy);
		}
	}
}

void ANavigationSystemMapVolume::SearchQuadtreeNeighbor(int NodeI, int CurLevel, std::tuple<int, int> CurPos, std::tuple<int, int> DXY)
{
	int TarPosX = std::get<0>(CurPos) + std::get<0>(DXY);
	int TarPosY = std::get<1>(CurPos) + std::get<1>(DXY);
	if (TarPosX >= 0 && TarPosX < std::get<0>(QuadtreeMapSize[CurLevel]) &&
		TarPosY >= 0 && TarPosY < std::get<1>(QuadtreeMapSize[CurLevel]) &&
		QuadtreeMap[CurLevel][TarPosX * std::get<1>(QuadtreeMapSize[CurLevel]) + TarPosY] == QuadNodeType::VALID) {
		int NodeJ = QuadtreeID[CurLevel][TarPosX * std::get<1>(QuadtreeMapSize[CurLevel]) + TarPosY];
		QuadtreeNode[NodeI].Neighbor.insert(NodeJ);
		QuadtreeNode[NodeJ].Neighbor.insert(NodeI);
		return;
	}
	else if (CurLevel == 0) {
		return;
	}
	else {
		std::tuple<int, int> CurPos1 = std::make_tuple(std::get<0>(CurPos) * 2 + (std::get<0>(DXY) == 1 ? 1 : 0),
			std::get<1>(CurPos) * 2 + (std::get<1>(DXY) == 1 ? 1 : 0));
		if (std::get<1>(DXY) == 0) {
			std::tuple<int, int> CurPos2 = std::make_tuple(std::get<0>(CurPos1), std::get<1>(CurPos1) + 1);
			SearchQuadtreeNeighbor(NodeI, CurLevel - 1, CurPos1, DXY);
			SearchQuadtreeNeighbor(NodeI, CurLevel - 1, CurPos2, DXY);
		}
		else if (std::get<0>(DXY) == 0) {
			std::tuple<int, int> CurPos2 = std::make_tuple(std::get<0>(CurPos1) + 1, std::get<1>(CurPos1));
			SearchQuadtreeNeighbor(NodeI, CurLevel - 1, CurPos1, DXY);
			SearchQuadtreeNeighbor(NodeI, CurLevel - 1, CurPos2, DXY);
		}
		else {
			SearchQuadtreeNeighbor(NodeI, CurLevel - 1, CurPos1, DXY);
		}
	}
}

void ANavigationSystemMapVolume::UpdateMobileMap(const TArray<AAgentAIController*>& Agents)
{
	MobileMapSize = std::make_tuple(FMath::CeilToInt(Extent.X / MobileCellSize),
		FMath::CeilToInt(Extent.Y / MobileCellSize));
	
	MobileMap.Empty();
	MobileMap.Init(TArray<AAgentAIController*>(), std::get<0>(MobileMapSize) * std::get<1>(MobileMapSize));

	for (int i = 0; i < Agents.Num(); i++) {
		int CellX = int((Agents[i]->GetPawn()->GetActorLocation().X - Corner.X) / MobileCellSize);
		int CellY = int((Agents[i]->GetPawn()->GetActorLocation().Y - Corner.Y) / MobileCellSize);
		MobileMap[CellX * std::get<1>(MobileMapSize) + CellY].Add(Agents[i]);
	}
}

void ANavigationSystemMapVolume::VisualizeQuadtreeMap(int NodeId)
{
	UTexture2D* GridmapTexture = UTexture2D::CreateTransient(std::get<1>(GridmapSize) * DebugHUDPixelScale, std::get<0>(GridmapSize) * DebugHUDPixelScale, PF_B8G8R8A8);
	FTexture2DMipMap& Mip = GridmapTexture->GetPlatformData()->Mips[0];
	void* Data = Mip.BulkData.Lock(LOCK_READ_WRITE);
	FColor* TextureData = static_cast<FColor*>(Data);
	memset(TextureData, 0x0, sizeof(FColor) * (std::get<0>(GridmapSize) * DebugHUDPixelScale * std::get<1>(GridmapSize) * DebugHUDPixelScale));
	for (int level = 0; level < QuadtreeLevel; level++) {
		int scale = DebugHUDPixelScale * (1U << level);
		for (int i = 0; i < std::get<0>(GridmapSize) * DebugHUDPixelScale; i++) {
			for (int j = 0; j < std::get<1>(GridmapSize) * DebugHUDPixelScale; j++) {
				int MapIndex = (i / scale) * std::get<1>(QuadtreeMapSize[level]) + (j / scale);
				int PixelIndex = i * std::get<1>(GridmapSize) * DebugHUDPixelScale + j;
				if (i / scale < std::get<0>(QuadtreeMapSize[level]) &&
					j / scale < std::get<1>(QuadtreeMapSize[level]) &&
					QuadtreeMap[level][MapIndex] == QuadNodeType::VALID) {
					if (QuadtreeID[level][MapIndex] == NodeId) {
						TextureData[PixelIndex] = FColor::Red;
					}
					else if (NodeId >= 0 && NodeId < QuadtreeNode.size() && QuadtreeNode[NodeId].Neighbor.find(MapIndex) != QuadtreeNode[NodeId].Neighbor.end()) {
						TextureData[PixelIndex] = FColor::Orange;
					}
					else {
						TextureData[PixelIndex] = FColor::Blue;
					}
					if (i % scale == 0 || j % scale == 0 || i % scale == (scale - 1) || j % scale == (scale - 1))
						TextureData[PixelIndex] = FColor::White;
				}
			}
		}
	}
	Mip.BulkData.Unlock();
	ANavigationDebugHUD* HUD = Cast<ANavigationDebugHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
	if (HUD) {
		HUD->SetTexture(GridmapTexture);
	}
}

int ANavigationSystemMapVolume::GetQuadtreeNode(FVector2D Pos) const {
	int CellX = int((Pos.X - Corner.X) / StationaryCellSize);
	int CellY = int((Pos.Y - Corner.Y) / StationaryCellSize);
	int Level = 0;
	while (Level < QuadtreeLevel) {
		if (QuadtreeMap[Level][CellX * std::get<1>(QuadtreeMapSize[Level]) + CellY] == QuadNodeType::VALID) {
			return QuadtreeID[Level][CellX * std::get<1>(QuadtreeMapSize[Level]) + CellY];
		}
		Level += 1;
		CellX /= 2;
		CellY /= 2;
	}
	return -1;
}

FVector2D ANavigationSystemMapVolume::ActualPosition(int NodeId) const {
	const QuadNode& q = QuadtreeNode[NodeId];
	return FVector2D(
		Corner.X + (std::get<0>(q.Pos) + 0.5) * float(1 << q.Level) * StationaryCellSize,
		Corner.Y + (std::get<1>(q.Pos) + 0.5) * float(1 << q.Level) * StationaryCellSize
	);
}

bool ANavigationSystemMapVolume::IsEmptyPosition(FVector2D Pos, bool AllowAirWall) const {
	int CellX = int((Pos.X - Corner.X) / StationaryCellSize);
	int CellY = int((Pos.Y - Corner.Y) / StationaryCellSize);
	if (0 <= CellX && CellX < std::get<0>(GridmapSize) && 0 <= CellY && CellY < std::get<1>(GridmapSize)) {
		if (AllowAirWall) {
			return Gridmap[CellX * std::get<1>(GridmapSize) + CellY] == TileType::EMPTY || 
				   Gridmap[CellX * std::get<1>(GridmapSize) + CellY] == TileType::AIRWALL;
		}
		return Gridmap[CellX * std::get<1>(GridmapSize) + CellY] == TileType::EMPTY;
	}
	else {
		return true;
	}
}

std::optional<FVector> ANavigationSystemMapVolume::GetNearestValidPosition(FVector Pos) const {
	FVector2D Pos2D = FVector2D(Pos.X, Pos.Y);
	if (IsEmptyPosition(Pos2D)) {
		return Pos;
	}

	int CellX = int((Pos.X - Corner.X) / StationaryCellSize);
	int CellY = int((Pos.Y - Corner.Y) / StationaryCellSize);
	if (0 <= CellX && CellX < std::get<0>(GridmapSize) && 0 <= CellY && CellY < std::get<1>(GridmapSize)) {
		if (IsEmptyPosition(Pos2D, true)) {
			// is a ariwall
		}
		else {
			// return std::nullopt;
		}
	}
	else {
		return std::nullopt;
	}

	struct BFSData {
		int X, Y, lastDXY;
	};
	
	std::queue<BFSData> Q;
	std::unordered_set<int> IsVis;

	std::vector<std::pair<int, int>> DXY = {
		std::make_pair(-1, +0),
		std::make_pair(+0, -1),
		std::make_pair(+0, +1),
		std::make_pair(+1, +0),
	};

	Q.push(BFSData{ CellX, CellY, 0 });
	Q.push(BFSData{ CellX, CellY, 1 });
	Q.push(BFSData{ CellX, CellY, 2 });
	Q.push(BFSData{ CellX, CellY, 3 });
	while (!Q.empty()) {
		const auto& u = Q.front();
		int UIndex = u.X * std::get<1>(GridmapSize) + u.Y;
		Q.pop();

		for (int i = 0; i < 4; i++) {
			const auto& dxy = DXY[(i + u.lastDXY) % 4];
			const auto& v = BFSData{ u.X + dxy.first, u.Y + dxy.second, i };
			if (0 <= v.X && v.X < std::get<0>(GridmapSize) && 
				0 <= v.Y && v.Y < std::get<1>(GridmapSize)) {
				int VIndex = v.X * std::get<1>(GridmapSize) + v.Y;
				if (Gridmap[VIndex] == TileType::AIRWALL && IsVis.find(VIndex) == IsVis.end()) {
					Q.push(v);
					IsVis.insert(VIndex);
				}
				else if (Gridmap[VIndex] == TileType::EMPTY) {
					return FVector((v.X + 0.5) * StationaryCellSize + Corner.X, (v.Y + 0.5) * StationaryCellSize + Corner.Y, DefaultZ());
				}
			}
		}
	}

	return std::nullopt;
}

double ANavigationSystemMapVolume::DefaultZ() const {
	return Corner.Z;
}

ANavigationSystemMapVolume::Path ANavigationSystemMapVolume::SortedQExpansion(FVector2D EndPos, int &MidpointId) const {
	// "sortedQ" Keep heuristic part zero, thus it first visits the frontier node 
	// (frontier nodes are nodes in the boundary between visited and unvisited regions) 
	// which is closest to the start node
	using StoreType = double;
	int EndId = GetQuadtreeNode(EndPos);
	if (EndId == -1) {
		return std::nullopt;
	}

	std::unordered_set<int> CloseSet;
	std::unordered_map<int, int> CameFrom;
	std::unordered_map<int, StoreType> FScore;
	CameFrom.insert(std::make_pair(EndId, -1));
	FScore.insert(std::make_pair(EndId, FVector2D::Distance(ActualPosition(EndId), EndPos)));

	std::priority_queue<std::tuple<StoreType, int>,
		std::deque<std::tuple<StoreType, int>>,
		std::greater<std::tuple<StoreType, int>>
	> OpenSet;
	std::unordered_set<int> OpenSetExist;

	const int OpenSetSizeLimit = 10000;

	OpenSet.push(std::make_tuple(FScore[EndId], EndId));
	int CurrentId;
	while (true) {
		if (OpenSet.size() > OpenSetSizeLimit) return std::nullopt;
		CurrentId = std::get<1>(OpenSet.top());
		OpenSet.pop();
		OpenSetExist.erase(CurrentId);

		if (QuadtreeNode[CurrentId].Level >= PathwayFindingExpansionMinimalLevel) {
			MidpointId = CurrentId;
			break;
		}

		CloseSet.insert(CurrentId);
		for (const auto& Neighbor : QuadtreeNode[CurrentId].Neighbor) {
			StoreType TentativeScore = FVector2D::Distance(ActualPosition(Neighbor), EndPos);

			if (CloseSet.find(Neighbor) != CloseSet.end() && TentativeScore >= FScore[Neighbor]) {
				continue;
			}

			if (TentativeScore < FScore[Neighbor] || OpenSetExist.find(Neighbor) == OpenSetExist.end()) {
				CameFrom[Neighbor] = CurrentId;
				FScore[Neighbor] = TentativeScore;
				OpenSet.push(std::make_pair(FScore[Neighbor], Neighbor));
				OpenSetExist.insert(Neighbor);
			}
		}
	}

	Path TempPath = std::vector<int>();
	while (CameFrom.find(CurrentId) != CameFrom.end()) {
		TempPath.value().push_back(CurrentId);
		CurrentId = CameFrom[CurrentId];
	}

	return TempPath;
}

ANavigationSystemMapVolume::Path ANavigationSystemMapVolume::PMultiQBase(FVector2D StartPos, std::variant<FVector2D, int> EndPos) const {
	using StoreType = double;
	int StartId = GetQuadtreeNode(StartPos);
	int EndId = EndPos.index() ? std::get<int>(EndPos) : GetQuadtreeNode(std::get<FVector2D>(EndPos));
	if (StartId == -1 || EndId == -1) {
		return std::nullopt;
	}

	std::unordered_set<int> CloseSet;
	std::unordered_map<int, int> CameFrom;
	std::unordered_map<int, StoreType> GScore;
	std::unordered_map<int, StoreType> FScore;
	CameFrom.insert(std::make_pair(StartId, -1));
	GScore.insert(std::make_pair(StartId, 0));
	FScore.insert(std::make_pair(StartId, FVector2D::Distance(ActualPosition(StartId), EndPos.index() ? ActualPosition(std::get<int>(EndPos)) : std::get<FVector2D>(EndPos))));

	std::vector <
		std::priority_queue<std::tuple<StoreType, int>,
		std::deque<std::tuple<StoreType, int>>,
		std::greater<std::tuple<StoreType, int>>
		>
	> OpenSet(QuadtreeLevel);
	std::vector <
		std::unordered_set<int>
	> OpenSetExist(QuadtreeLevel);

	OpenSet[QuadtreeNode[StartId].Level].push(std::make_tuple(FScore[StartId], StartId));
	OpenSetExist[QuadtreeNode[StartId].Level].insert(StartId);

	int CurrentId;
	while (true) {
		CurrentId = -1;
		for (int Level = QuadtreeLevel - 1; Level >= 0; Level--) {
			if (!OpenSet[Level].empty()) {
				CurrentId = std::get<1>(OpenSet[Level].top());
				OpenSet[Level].pop();
				OpenSetExist[Level].erase(CurrentId);
				break;
			}
		}
		if (CurrentId == -1) {
			return std::nullopt;
		}
		if (CurrentId == EndId) {
			break;
		}
		
		CloseSet.insert(CurrentId);
		for (const auto& Neighbor : QuadtreeNode[CurrentId].Neighbor) {
			int NeighborLevel = QuadtreeNode[Neighbor].Level;
			StoreType TentativeGScore = GScore[CurrentId] +
				FVector2D::Distance(ActualPosition(Neighbor),
					ActualPosition(CurrentId));
			
			if (CloseSet.find(Neighbor) != CloseSet.end() && TentativeGScore >= GScore[Neighbor]) {
				continue;
			}

			if (TentativeGScore < GScore[Neighbor] ||
				OpenSetExist[NeighborLevel].find(Neighbor) == OpenSetExist[NeighborLevel].end()) {
				CameFrom[Neighbor] = CurrentId;
				GScore[Neighbor] = TentativeGScore;
				FScore[Neighbor] = TentativeGScore + FVector2D::Distance(ActualPosition(Neighbor), EndPos.index() ? ActualPosition(std::get<int>(EndPos)) : std::get<FVector2D>(EndPos));
				OpenSet[NeighborLevel].push(std::make_pair(FScore[Neighbor], Neighbor));
				OpenSetExist[NeighborLevel].insert(Neighbor);
			}
		}
	}

	Path TempPath = std::vector<int>();
	while (CameFrom.find(CurrentId) != CameFrom.end()) {
		TempPath.value().push_back(CurrentId);
		CurrentId = CameFrom[CurrentId];
	}
	std::reverse(TempPath.value().begin(), TempPath.value().end());

	return TempPath;
}

ANavigationSystemMapVolume::Path ANavigationSystemMapVolume::PathFinding(FVector2D StartPos, FVector2D EndPos, bool bEnableExpansion) const
{
	// "PmultiQ", is a prioritized MultiQ scheme, 
	// which tries to visit the largest frontier node first
	// and therefore exhibits the fastest growth speed among the four methods.
	Path NavigationPath;
	if (bEnableExpansion) {
		int MidpointId;
		// To make the search even faster, every target is
		// expanded on a quadtree map until it touches any node at a certain level Lt.
		// "SortedQ" is used in order for the expansion to be nearly isotropic.
		Path ExpansionPath = SortedQExpansion(EndPos, MidpointId);
		if (!ExpansionPath.has_value()) {
			return std::nullopt;
		}
		NavigationPath = PMultiQBase(StartPos, MidpointId);
		if (!NavigationPath.has_value()) {
			return std::nullopt;
		}
		NavigationPath.value().pop_back();
		NavigationPath.value().insert(NavigationPath.value().end(), ExpansionPath.value().begin(), ExpansionPath.value().end());
	}
	else {
		NavigationPath = PMultiQBase(StartPos, EndPos);
		if (!NavigationPath.has_value()) {
			return std::nullopt;
		}
	}

	return NavigationPath;
}

void ANavigationSystemMapVolume::VisualizePathOnQuadtree(const ANavigationSystemMapVolume::Path& NavPath) {
	if (!NavPath.has_value()) return;
	std::unordered_set<int> ExistPath(NavPath.value().begin(), NavPath.value().end());
	UTexture2D* GridmapTexture = UTexture2D::CreateTransient(std::get<1>(GridmapSize) * DebugHUDPixelScale, std::get<0>(GridmapSize) * DebugHUDPixelScale, PF_B8G8R8A8);
	FTexture2DMipMap& Mip = GridmapTexture->GetPlatformData()->Mips[0];
	void* Data = Mip.BulkData.Lock(LOCK_READ_WRITE);
	FColor* TextureData = static_cast<FColor*>(Data);
	memset(TextureData, 0x0, sizeof(FColor) * (std::get<0>(GridmapSize) * DebugHUDPixelScale * std::get<1>(GridmapSize) * DebugHUDPixelScale));
	for (int level = 0; level < QuadtreeLevel; level++) {
		int scale = DebugHUDPixelScale * (1U << level);
		for (int i = 0; i < std::get<0>(GridmapSize) * DebugHUDPixelScale; i++) {
			for (int j = 0; j < std::get<1>(GridmapSize) * DebugHUDPixelScale; j++) {
				int MapIndex = (i / scale) * std::get<1>(QuadtreeMapSize[level]) + (j / scale);
				int PixelIndex = i * std::get<1>(GridmapSize) * DebugHUDPixelScale + j;
				if (i / scale < std::get<0>(QuadtreeMapSize[level]) &&
					j / scale < std::get<1>(QuadtreeMapSize[level]) &&
					QuadtreeMap[level][MapIndex] == QuadNodeType::VALID) {
					if (QuadtreeID[level][MapIndex] == NavPath.value()[0] || QuadtreeID[level][MapIndex] == NavPath.value().back()) {
						TextureData[PixelIndex] = FColor::Red;
					}
					else if (ExistPath.find(QuadtreeID[level][MapIndex]) != ExistPath.end()) {
						TextureData[PixelIndex] = FColor::Orange;
					}
					else {
						TextureData[PixelIndex] = FColor::Blue;
					}
					if (i % scale == 0 || j % scale == 0 || i % scale == (scale - 1) || j % scale == (scale - 1))
						TextureData[PixelIndex] = FColor::White;
				}
			}
		}
	}
	Mip.BulkData.Unlock();
	ANavigationDebugHUD* HUD = Cast<ANavigationDebugHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
	if (HUD) {
		HUD->SetTexture(GridmapTexture);
	}
}

void ANavigationSystemMapVolume::RegisterLeisurePlaces() {
	TArray<AActor*> QueryData;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANavigationValidLeisurePlaceVolume::StaticClass(), QueryData);
	for (int i = 0; i < QueryData.Num(); i++) {
		EnvObj.LeisurePlaces.Add(Cast<ANavigationValidLeisurePlaceVolume>(QueryData[i]));
	}
}

FVector ANavigationSystemMapVolume::GetValidRandomLeisurePosition(std::mt19937& Rnd) const {
	double X, Y;
	int CellX, CellY;
	std::uniform_int_distribution<int> LP(0, EnvObj.LeisurePlaces.Num() - 1);
	std::uniform_real_distribution<float> D(0.f, 1.f);
	int LPId = LP(Rnd);
	FVector LPOrigin, LPExtent, LPCorner;
	EnvObj.LeisurePlaces[LPId]->GetActorBounds(false, LPOrigin, LPExtent);
	LPCorner = FVector(LPOrigin.X - LPExtent.X, LPOrigin.Y - LPExtent.Y, LPOrigin.Z);
	LPExtent.X *= 2.f;
	LPExtent.Y *= 2.f;
	do {
		X = D(Rnd) * LPExtent.X + LPCorner.X;
		Y = D(Rnd) * LPExtent.Y + LPCorner.Y;
		CellX = int((X - Corner.X) / StationaryCellSize);
		CellY = int((Y - Corner.Y) / StationaryCellSize);
	} while (Gridmap[CellX * std::get<1>(GridmapSize) + CellY] != TileType::EMPTY);
	UE_LOG(LogTemp, Warning, TEXT("Random Leisure LPCorner = (%lf, %lf), LPExtent = (%lf, %lf), Pos = (%lf, %lf)"), 
		LPOrigin.X, LPOrigin.Y,
		LPExtent.X, LPExtent.Y,
		X, Y);
	return FVector(X, Y, DefaultZ());
}

ANavigationSystemMapVolume::EnvObjQueryData ANavigationSystemMapVolume::QueryANearestValidBench(FVector Location)
{
	const float INF = 1e7;
	float MinDist = INF;
	int BenchId = -1;
	for (int i = 0; i < EnvObj.Benches.Num(); i++) {
		float Dist = FVector::Distance(Location, EnvObj.Benches[i]->GetActorLocation());
		if (EnvObj.BenchOccupiedNum[i] == 0 && Dist < MinDist) {
			BenchId = i;
			MinDist = Dist;
		}
	}

	if (BenchId == -1) {
		return std::nullopt;
	}
	EnvObj.BenchOccupiedNum[BenchId] = 1;

	FVector RotVec = EnvObj.Benches[BenchId]->GetActorRotation().Vector();
	FVector2D Facing = FVector2D(-RotVec.X, -RotVec.Y).GetSafeNormal();
	UE_LOG(LogTemp, Warning, TEXT("Bench (%d) Facing = (%lf, %lf, %lf)"), BenchId, RotVec.X, RotVec.Y, RotVec.Z);
	return std::make_tuple(BenchId, EnvObj.Benches[BenchId]->GetActorLocation() + FVector(Facing.X * BenchFacingOffset, 
																						  Facing.Y * BenchFacingOffset, 0.f), Facing);
}

ANavigationSystemMapVolume::EnvObjQueryData ANavigationSystemMapVolume::QueryARandomValidBench(std::mt19937& Rnd)
{
	std::vector<int> BenchIdList;
	for (int i = 0; i < EnvObj.Benches.Num(); i++) {
		if (EnvObj.BenchOccupiedNum[i] == 0) {
			BenchIdList.push_back(i);
		}
	}

	if (BenchIdList.size() == 0) {
		return std::nullopt;
	}
	std::uniform_int_distribution<int> D(0, BenchIdList.size()-1);
	int BenchId = BenchIdList[D(Rnd)];

	EnvObj.BenchOccupiedNum[BenchId] = 1;

	FVector RotVec = EnvObj.Benches[BenchId]->GetActorRotation().Vector();
	FVector2D Facing = FVector2D(-RotVec.X, -RotVec.Y).GetSafeNormal();
	UE_LOG(LogTemp, Warning, TEXT("Bench (%d) Facing = (%lf, %lf, %lf)"), BenchId, RotVec.X, RotVec.Y, RotVec.Z);
	return std::make_tuple(BenchId, EnvObj.Benches[BenchId]->GetActorLocation() + FVector(Facing.X * BenchFacingOffset,
																				  Facing.Y * BenchFacingOffset, 0.f), Facing);
}

void ANavigationSystemMapVolume::ReleaseAnOccupiedBench(int BenchId)
{
	check(BenchId < EnvObj.Benches.Num() && EnvObj.BenchOccupiedNum[BenchId] == 1);
	EnvObj.BenchOccupiedNum[BenchId] = 0;
}

ANavigationSystemMapVolume::EnvObjQueryData ANavigationSystemMapVolume::QueryARandomVendingMachine(std::mt19937& Rnd)
{
	if (EnvObj.VendingMachines.Num() == 0) {
		return std::nullopt;
	}
	std::uniform_int_distribution<int> D(0, EnvObj.VendingMachines.Num() - 1);
	int VMId = D(Rnd);

	FVector RotVec = EnvObj.VendingMachines[VMId]->GetActorRotation().Vector();
	FVector2D Facing = FVector2D(-RotVec.X, -RotVec.Y).GetSafeNormal();
	FVector2D Tangent = Facing.GetRotated(90);
	UE_LOG(LogTemp, Warning, TEXT("VendingMachine (%d) Facing = (%lf, %lf, %lf)"), VMId, RotVec.X, RotVec.Y, RotVec.Z);
	return std::make_tuple(VMId, EnvObj.VendingMachines[VMId]->GetActorLocation() + FVector(Tangent.X * VendingMachineTangentOffset, 
																							Tangent.Y * VendingMachineTangentOffset, 0.f), Tangent);
}

ANavigationSystemMapVolume::EnvObjQueryData ANavigationSystemMapVolume::QueryANearestVendingMachine(FVector Location)
{
	const float INF = 1e7;
	float MinDist = INF;
	int VMId = -1;
	for (int i = 0; i < EnvObj.VendingMachines.Num(); i++) {
		float Dist = FVector::Distance(Location, EnvObj.VendingMachines[i]->GetActorLocation());
		if (Dist < MinDist) {
			VMId = i;
			MinDist = Dist;
		}
	}

	if (VMId == -1) {
		return std::nullopt;
	}

	FVector RotVec = EnvObj.VendingMachines[VMId]->GetActorRotation().Vector();
	FVector2D Facing = FVector2D(-RotVec.X, -RotVec.Y).GetSafeNormal();
	FVector2D Tangent = Facing.GetRotated(90);
	UE_LOG(LogTemp, Warning, TEXT("VendingMachine (%d) Facing = (%lf, %lf, %lf)"), VMId, RotVec.X, RotVec.Y, RotVec.Z);
	return std::make_tuple(VMId, EnvObj.VendingMachines[VMId]->GetActorLocation() + FVector(Tangent.X * VendingMachineTangentOffset,
																							Tangent.Y * VendingMachineTangentOffset, 0.f), Tangent);
}

ANavigationSystemMapVolume::EnvObjQueryData ANavigationSystemMapVolume::QueryAMostVacantTicketOffice(FVector Location)
{
	const int INF = 100000;
	int MinPeopleNum = INF;
	int TicketOfficeId = -1;
	for (int i = 0; i < EnvObj.TicketOffices.Num(); i++) {
		if (EnvObj.TicketOfficeQueueNum[i] < MinPeopleNum) {
			MinPeopleNum = EnvObj.TicketOfficeQueueNum[i];
			TicketOfficeId = i;
		}
	}

	if (TicketOfficeId == -1) {
		return std::nullopt;
	}
	EnvObj.TicketOfficeQueueNum[TicketOfficeId]++;

	FVector RotVec = EnvObj.TicketOffices[TicketOfficeId]->GetActorRotation().Vector();
	FVector2D Facing = FVector2D(RotVec.X, RotVec.Y).GetSafeNormal();
	FVector2D Tangent = Facing.GetRotated(90);
	UE_LOG(LogTemp, Warning, TEXT("TicketOffice (%d) Facing = (%lf, %lf, %lf)"), TicketOfficeId, RotVec.X, RotVec.Y, RotVec.Z);

	return std::make_tuple(TicketOfficeId, EnvObj.TicketOffices[TicketOfficeId]->GetActorLocation() +
		FVector(Tangent.X * TicketOfficeTangentOffset - Facing.X * TicketOfficeFacingOffset,
			Tangent.Y * TicketOfficeTangentOffset - Facing.Y * TicketOfficeFacingOffset,
			30.f), Facing);
}

ANavigationSystemMapVolume::EnvObjQueryData ANavigationSystemMapVolume::QueryANearestTicketOffice(FVector Location)
{
	const float INF = 1e7;
	float MinDist = INF;
	int TicketOfficeId = -1;
	for (int i = 0; i < EnvObj.TicketOffices.Num(); i++) {
		float Dist = FVector::Distance(Location, EnvObj.TicketOffices[i]->GetActorLocation());
		if (Dist < MinDist) {
			TicketOfficeId = i;
			MinDist = Dist;
		}
	}

	if (TicketOfficeId == -1) {
		return std::nullopt;
	}
	EnvObj.TicketOfficeQueueNum[TicketOfficeId]++;

	FVector RotVec = EnvObj.TicketOffices[TicketOfficeId]->GetActorRotation().Vector();
	FVector2D Facing = FVector2D(RotVec.X, RotVec.Y).GetSafeNormal();
	FVector2D Tangent = Facing.GetRotated(90);
	UE_LOG(LogTemp, Warning, TEXT("TicketOffice (%d) Facing = (%lf, %lf, %lf)"), TicketOfficeId, RotVec.X, RotVec.Y, RotVec.Z);

	return std::make_tuple(TicketOfficeId, EnvObj.TicketOffices[TicketOfficeId]->GetActorLocation() + 
							FVector(Tangent.X * TicketOfficeTangentOffset - Facing.X * TicketOfficeFacingOffset, 
									Tangent.Y * TicketOfficeTangentOffset - Facing.Y * TicketOfficeFacingOffset, 
									30.f), Facing);
}

void ANavigationSystemMapVolume::QuitATicketOffice(int TicketOfficeId)
{
	check(TicketOfficeId < EnvObj.TicketOffices.Num() && EnvObj.TicketOfficeQueueNum[TicketOfficeId] > 0);
	--EnvObj.TicketOfficeQueueNum[TicketOfficeId];
}

void ANavigationSystemMapVolume::BeginPlay()
{
	Super::BeginPlay();
	RegisterLeisurePlaces();
	InitGridmap();
	// NOTE(changyu): visualization debugging tool
	// VisualizeGridmap();
	InitQuadtreeMap();
	VisualizeQuadtreeMap();
}

void ANavigationSystemMapVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	ANavigationDebugHUD* HUD = Cast<ANavigationDebugHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
	if (HUD) {
		HUD->bShowHUD = DisplayHUD;
	}
}

#if WITH_EDITOR
void ANavigationSystemMapVolume::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) {
	Super::PostEditChangeProperty(PropertyChangedEvent);
}

void ANavigationSystemMapVolume::PostEditMove(bool bFinished) {
	Super::PostEditMove(bFinished);
}
#endif

