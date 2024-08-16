#include "AgentAIController.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "AgentCharacter.h"
#include "AgentsGroupController.h"
#include "GameFramework/Pawn.h"
#include "Blueprint/AIBlueprintHelperLibrary.h"
#include "GameFramework/CharacterMovementComponent.h"

const int AAgentAIController::MaximumAttentionAgents = 16;

const float AAgentAIController::AgentWalkVelocity = 175.f;

const float AAgentAIController::TargetEps = 3.f;
const float AAgentAIController::AgentPerceptualRange = 500.f;

const float AAgentAIController::CosSensingFanAngle = 0.5f;
const float AAgentAIController::VisibilitySegmentSamplingDensity = 5.f;

const float AAgentAIController::PedestrianBoudingBoxSize = 25.f;
const float AAgentAIController::PedestrianBoudingBoxScale = 2.5f;
const float AAgentAIController::MinimumDistanceAllowed =
AAgentAIController::PedestrianBoudingBoxScale * AAgentAIController::PedestrianBoudingBoxSize;

const float AAgentAIController::RoutineASlowDownFactorRatio = 0.4f;
const float AAgentAIController::AngleDensity = 10.f;

const float AAgentAIController::RoutineESlowDownFactorRatio = 0.3f;

const float AAgentAIController::Repulsiveness = -0.025f;

const float AAgentAIController::CosSimilarity = 0.967f; // cos(20-deg.)
const float AAgentAIController::HeadonCosSimilarity = 0.84f; // cos(33-deg.)
const float AAgentAIController::Inertia = 50.f;

const float AAgentAIController::CrowdingFactor = 2.5f;
const float AAgentAIController::WaitingInLineCrowdingFactor = 5.f;

const float AAgentAIController::IncreaseSpeedRatio = 1.2f;
const float AAgentAIController::DecreaseSpeedRatio = 0.8f;

const float AAgentAIController::TurningFactorRatio = 5.f;
const float AAgentAIController::MaxHeadonCollisionAvoidanceDegree = 35.f;
const float AAgentAIController::MaxCrossCollisionAvoidanceDegree = 35.f;

const float AAgentAIController::RoutineBSlowDownFactorRatio = 0.99f;
const float AAgentAIController::RoutineBUrgentSlowDownFactorRatio = 0.8f;
const float AAgentAIController::RoutineFSlowDownFactorRatio = 0.1f;
const float AAgentAIController::RoutineBTurningFactorThreshold = 5.f;

const float AAgentAIController::AvoidOnComingAgentsDuration = 0.5f;
const float AAgentAIController::MaximumLineLength = 750.f;

AAgentAIController::AAgentAIController()
{
	// The tick of each Agent is managed by AgentsGroupController
	PrimaryActorTick.bCanEverTick = false;
}

void AAgentAIController::BeginPlay()
{
	Super::BeginPlay();

	TArray<AActor*> NavMap;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANavigationSystemMapVolume::StaticClass(), NavMap);
	NavigationMapPtr = Cast<ANavigationSystemMapVolume>(NavMap[0]);

	TArray<AActor*> GroupCtr;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAgentsGroupController::StaticClass(), GroupCtr);
	GroupControllerPtr = Cast<AAgentsGroupController>(GroupCtr[0]);

	ID = GroupControllerPtr->AddAgent(this);
	Rnd = std::mt19937(ID + GroupControllerPtr->GlobalRandomSeed);

	InitMentalState();
}

void AAgentAIController::InitMentalState()
{
	MentalStVar.AgentType = PedestrianType::Commuter;
	if (GetPawn()->Tags.Num() > 0) {
		FString AgentTag = GetPawn()->Tags[0].ToString();
		if (AgentTag == "Tourist") MentalStVar.AgentType = PedestrianType::Tourist;
		else if (AgentTag == "Policeman") MentalStVar.AgentType = PedestrianType::Policeman;
		else if (AgentTag == "Performer") MentalStVar.AgentType = PedestrianType::Performer;
	}

	// TODO(changyu): Use random to make each agent different.
	std::uniform_real_distribution<float> D(0.f, 3000.f);
	std::uniform_real_distribution<float> U(0.f, 1.f);
	MentalStVar.Tiredness = D(Rnd);
	MentalStVar.Thirst = D(Rnd);
	MentalStVar.Curiosity = U(Rnd);
	MentalStVar.bNeedATicket = (MentalStVar.AgentType == PedestrianType::Commuter || MentalStVar.AgentType == PedestrianType::Tourist) && (U(Rnd) > 0.4f);
}

void AAgentAIController::UpdateMentalState(float DeltaTime)
{
	switch (MentalStVar.AgentType) {
	case PedestrianType::Commuter: UpdateCommuterMentalState(DeltaTime); break;
	case PedestrianType::Tourist: UpdateTouristMentalState(DeltaTime); break;
	case PedestrianType::Policeman: UpdatePolicemanMentalState(DeltaTime); break;
	case PedestrianType::Performer: UpdatePerformerMentalState(DeltaTime); break;
	}
}

void AAgentAIController::UpdateCommuterMentalState(float DeltaTime)
{
	std::uniform_real_distribution<float> RateD(3.f, 10.f);
	const float DebugScalingFactor = 1.f;
	const float TirednessRate = RateD(Rnd) * DebugScalingFactor;
	const float TirednessThreshold = 3000.f;
	const float TakeARestDuration = 30.f;
	const float ThirstRate = RateD(Rnd) * DebugScalingFactor;
	const float ThirstThreshold = 3000.f;
	const float BuyADrinkDuration = 7.f;
	const float GoToRandomPositionDuration = 5.f;
	std::uniform_real_distribution<float> BTD(10.f, 20.f);
	const float BuyATicketDuration = BTD(Rnd);

	if (IsArrivedTarget()) {
		MentalStVar.DurationTime += DeltaTime;

		switch (MentalStVar.GoalStack.back().first) {
		case AgentGoalType::TakeARest: SelectAction(EnumAgentState::VE_Sit, EnumEnvironmentalState::VE_None); break;
		case AgentGoalType::BuyADrink: SelectAction(EnumAgentState::VE_Normal, EnumEnvironmentalState::VE_Drink); break;
		case AgentGoalType::GoToRandomPosition: SelectAction(EnumAgentState::VE_Normal, EnumEnvironmentalState::VE_CallPhone); break;
		case AgentGoalType::BuyATicket: SelectAction(EnumAgentState::VE_Normal, EnumEnvironmentalState::VE_Chat); break;
		}

		if (MentalStVar.DurationTime > MentalStVar.DurationThreshold.back()) {

			if (MentalStVar.GoalStack.back().first == AgentGoalType::TakeARest) {
				MentalStVar.Tiredness = 0.f;
				NavigationMapPtr->ReleaseAnOccupiedBench(std::get<int>(MentalStVar.GoalStack.back().second));
			}
			else if (MentalStVar.GoalStack.back().first == AgentGoalType::BuyADrink) {
				MentalStVar.Thirst = 0.f;
			}
			else if (MentalStVar.GoalStack.back().first == AgentGoalType::GoToTicketOfficeEntrance) {
			}
			else if (MentalStVar.GoalStack.back().first == AgentGoalType::BuyATicket) {
				MentalStVar.bNeedATicket = false;
				NavigationMapPtr->QuitATicketOffice(std::get<int>(MentalStVar.GoalStack.back().second));
			}

			// Popback current state & retarget to last state.
			PopbackCurrentState();
		}
	}
	else {
		MentalStVar.DurationTime = 0.f;
		MentalStVar.Tiredness += TirednessRate * DeltaTime;
		MentalStVar.Thirst += ThirstRate * DeltaTime;

		if (GroupControllerPtr->DisplayMentalState) {
			DrawDebugState(FColor(int(FMath::Min(MentalStVar.Tiredness / TirednessThreshold * 255, 255.f)), 0, 0), 100.f);
			DrawDebugState(FColor(0, int(FMath::Min(MentalStVar.Thirst / ThirstThreshold * 255, 255.f)), 0), 120.f);
			DrawDebugState(MentalStVar.bNeedATicket ? FColor::Blue : FColor::Black, 140.f);
		}

		if (MentalStVar.Tiredness >= TirednessThreshold && IsHigherPriority(AgentGoalType::TakeARest)) {
			auto targetBench = NavigationMapPtr->QueryANearestValidBench(To3D(CurrentLocation(), NavigationMapPtr->DefaultZ()));
			if (targetBench.has_value()) {
				auto [BenchId, TargetPos, TargetFacing] = targetBench.value();
				MentalStVar.DurationThreshold.push_back(TakeARestDuration);
				MentalStVar.GoalStack.push_back(AgentGoal(AgentGoalType::TakeARest, AgentGoalDetail(BenchId)));
				SetTargetPositionAndFacing(TargetPos, TargetFacing);
			}
		}
		if (MentalStVar.Thirst >= ThirstThreshold && IsHigherPriority(AgentGoalType::BuyADrink)) {
			// auto targetVM = NavigationMapPtr->QueryANearestVendingMachine(To3D(CurrentLocation(), NavigationMapPtr->DefaultZ()));
			auto targetVM = NavigationMapPtr->QueryARandomVendingMachine(Rnd);
			if (targetVM.has_value()) {
				auto [VMId, TargetPos, TargetFacing] = targetVM.value();
				MentalStVar.DurationThreshold.push_back(BuyADrinkDuration);
				MentalStVar.GoalStack.push_back(AgentGoal(AgentGoalType::BuyADrink, AgentGoalDetail(VMId)));
				SetTargetPositionAndFacing(TargetPos, TargetFacing);
			}
		}
		if (MentalStVar.bNeedATicket && IsHigherPriority(AgentGoalType::BuyATicket)) {
			auto targetTO = NavigationMapPtr->QueryAMostVacantTicketOffice(To3D(CurrentLocation(), NavigationMapPtr->DefaultZ()));
			if (targetTO.has_value()) {
				auto [TOId, TargetPos, TargetFacing] = targetTO.value();
				MentalStVar.DurationThreshold.push_back(BuyATicketDuration);
				MentalStVar.GoalStack.push_back(AgentGoal(AgentGoalType::BuyATicket, AgentGoalDetail(TOId)));
				SetTargetPositionAndFacing(TargetPos, TargetFacing);

				// Sub-task
				MentalStVar.DurationThreshold.push_back(0.f);
				MentalStVar.GoalStack.push_back(AgentGoal(AgentGoalType::GoToTicketOfficeEntrance, AgentGoalDetail(TOId)));
				SetTargetPositionAndFacing(TargetPos - To3D(TargetFacing * NavigationMapPtr->TicketOfficeEntranceDist, 0.f), TargetFacing);
			}
		}
	}

	if (MentalStVar.GoalStack.empty()) {
		std::uniform_real_distribution<float> U(0.f, 1.f);
		bool HaveRested = false;
		if (U(Rnd) < MentalStVar.Curiosity) {
			// No curiosity, just want to rest
			auto targetBench = NavigationMapPtr->QueryARandomValidBench(Rnd);
			if (targetBench.has_value()) {
				auto [BenchId, TargetPos, TargetFacing] = targetBench.value();
				MentalStVar.DurationThreshold.push_back(TakeARestDuration);
				MentalStVar.GoalStack.push_back(AgentGoal(AgentGoalType::TakeARest, AgentGoalDetail(BenchId)));
				HaveRested = SetTargetPositionAndFacing(TargetPos, TargetFacing);
			}
		}
		if (!HaveRested) {
			// If having curiosity, or no seat to rest
			MentalStVar.DurationThreshold.push_back(GoToRandomPositionDuration);
			MentalStVar.GoalStack.push_back(AgentGoal(AgentGoalType::GoToRandomPosition, AgentGoalDetail(0)));
			std::uniform_real_distribution<float> D(0.f, 360.f);
			float Theta = D(Rnd);
			// This could fail in some cases. (target is not reachable / empty space)
			while (!SetTargetPositionAndFacing(NavigationMapPtr->GetValidRandomLeisurePosition(Rnd), FVector2D(FMath::Cos(Theta), FMath::Sin(Theta))));
		}
	}
}
void AAgentAIController::UpdateTouristMentalState(float DeltaTime)
{
	check(false);
}

void AAgentAIController::UpdatePolicemanMentalState(float DeltaTime)
{
	check(false);
}
void AAgentAIController::UpdatePerformerMentalState(float DeltaTime)
{
	check(false);
}

bool AAgentAIController::IsHigherPriority(AgentGoalType NewGoal) const
{
	return (MentalStVar.GoalStack.size() == 0 || NewGoal > MentalStVar.GoalStack.back().first);
}

void AAgentAIController::SelectAction(EnumAgentState AgentState, EnumEnvironmentalState EnvironmentalState)
{
	AAgentCharacter* ControlledCharacter = Cast<AAgentCharacter>(GetPawn());
	UAnimInstance* AnimInstance = ControlledCharacter->GetMesh()->GetAnimInstance();
	FNumericProperty* PlayerPR = FindField<FNumericProperty>(AnimInstance->GetClass(), TEXT("PlayerStateEnum"));
	PlayerPR->SetValue_InContainer(AnimInstance, &AgentState);
	FNumericProperty* EnvPR = FindField<FNumericProperty>(AnimInstance->GetClass(), TEXT("EnvironmentalState"));
	EnvPR->SetValue_InContainer(AnimInstance, &EnvironmentalState);
}

void AAgentAIController::PopbackCurrentState()
{
	MentalStVar.CompletedGoalStack.push_back(MentalStVar.GoalStack.back());
	MentalStVar.GoalStack.pop_back();
	MentalStVar.DurationThreshold.pop_back();
	NaviVar.TargetPosition.pop_back();
	NaviVar.TargetFacing.pop_back();
	if (NaviVar.TargetPosition.size() > 0) {
		ResetCurrentTargetPositionAndFacing();
	}
	MentalStVar.DurationTime = 0.f;
	SelectAction(EnumAgentState::VE_Normal);
}

FVector2D AAgentAIController::CurrentLocation() const
{
	FVector ActorLocation = GetPawn()->GetActorLocation();
	return FVector2D(ActorLocation.X, ActorLocation.Y);
}

bool AAgentAIController::IsVisible(FVector2D Position, bool AllowAirWall) const
{
	FVector StartPos = To3D(CurrentLocation(), NavigationMapPtr->DefaultZ());
	FVector EndPos = To3D(Position, NavigationMapPtr->DefaultZ());
	if (FVector::Distance(StartPos, EndPos) > AgentPerceptualRange) {
		return false;
	}
	float DY = EndPos.Y - StartPos.Y;
	float DX = EndPos.X - StartPos.X;
	if (FMath::Abs(DY) > FMath::Abs(DX)) { // |K| > 1
		for (int d = 0; d < int(FMath::Floor(FMath::Abs(DY) * VisibilitySegmentSamplingDensity)); d++) {
			float delta = d / VisibilitySegmentSamplingDensity;
			FVector2D RasterizedPos(StartPos.X + (DX / DY) * delta * FMath::Sign(DY), 
									StartPos.Y + delta * FMath::Sign(DY));
			if (!NavigationMapPtr->IsEmptyPosition(RasterizedPos, AllowAirWall)) {
				return false;
			}
		}
	}
	else { // |K| < 1
		for (int d = 0; d < int(FMath::Floor(FMath::Abs(DX) * VisibilitySegmentSamplingDensity)); d++) {
			float delta = d / VisibilitySegmentSamplingDensity;
			FVector2D RasterizedPos(StartPos.X + delta * FMath::Sign(DX),
									StartPos.Y + (DY / DX) * delta * FMath::Sign(DX));
			if (!NavigationMapPtr->IsEmptyPosition(RasterizedPos, AllowAirWall)) {
				return false;
			}
		}
	}

	return true;
}

bool AAgentAIController::HasAgentInFrontSafeArea(float DeltaTime)
{
	ReactRtVar.OnComingBlock = false;
	ReactRtVar.OnComingAgent = nullptr;
	// Used in Routine A, B, E.
	// W and D depend on H's bounding box size 
	// and D is determined by H's current speed s.
	float W = MinimumDistanceAllowed;
	float D = MinimumDistanceAllowed + ReactRtVar.ModifiedVelocity.Length() * DeltaTime;
	bool Ret = false;
	for (int i = 0; i < Neighbors.Num(); i++) {
		FVector2D Dir = Neighbors[i]->CurrentLocation() - CurrentLocation();
		FVector2D NorDir, TanDir;
		DecomposeVector(Dir, ReactRtVar.ModifiedFacing, NorDir, TanDir);
		if (NorDir.Length() < D && TanDir.Length() < W) {
			Ret = true;
			bool IsOnComing = (Neighbors[i]->ReactRtVar.ModifiedFacing.Dot(ReactRtVar.ModifiedFacing) < 0) ||
							  (Neighbors[i]->ReactRtVar.ModifiedFacing.Dot(ReactRtVar.ModifiedFacing) >= 0 &&
										 Neighbors[i]->ReactRtVar.ModifiedVelocity.IsNearlyZero());
			if (IsOnComing) {
				ReactRtVar.OnComingBlock = true;
				ReactRtVar.OnComingAgent = Neighbors[i];
				return true;
			}
		}
	}

	if (GroupControllerPtr->DisplayOnComingBlock && ReactRtVar.OnComingBlock) {
		DrawDebugState(FColor::Red);
	}
	
	return Ret;
}

bool AAgentAIController::HasAgentInDirection(float DeltaTime, FVector2D MovingDir) const
{
	float W = PedestrianBoudingBoxSize / 4.f;
	float D = PedestrianBoudingBoxSize + Velocity.Length() * DeltaTime;
	for (int i = 0; i < Neighbors.Num(); i++) {
		FVector2D Dir = Neighbors[i]->CurrentLocation() - CurrentLocation();
		FVector2D NorDir, TanDir;
		DecomposeVector(Dir, MovingDir, NorDir, TanDir);
		if (NorDir.Length() < D && TanDir.Length() < W) {
			return true;
		}
	}

	return false;
}

void AAgentAIController::UpdateFarthestVisiblePoint(bool bResetPath)
{
	if (bResetPath) {
		NaviVar.NavigationStop = 0;
	}

	FVector2D CurrentPos = CurrentLocation();
	FVector2D Target2D = To2D(NaviVar.EndMidPosition);
	if (IsVisible(Target2D)) {
		NaviVar.VisiblePosition = Target2D;
		NaviVar.NavigationStop = NaviVar.NavigationPath.value().size() + 1;
	}

	FVector2D TempPos;
	for (int p = NaviVar.NavigationStop; p <= NaviVar.NavigationPath.value().size(); p++) {
		if (p == NaviVar.NavigationPath.value().size()) {
			TempPos = Target2D;
		}
		else {
			TempPos = NavigationMapPtr->ActualPosition(NaviVar.NavigationPath.value()[p]);
		}

		bool bOutRange = false;
		if (FVector2D::Distance(CurrentPos, TempPos) > AgentPerceptualRange) {
			bOutRange = true;
			FVector2D PrevTempPos = (p == 0 ? CurrentPos : NavigationMapPtr->ActualPosition(NaviVar.NavigationPath.value()[p - 1]));
			if (FVector2D::Distance(CurrentPos, TempPos) < FVector2D::Distance(PrevTempPos, TempPos)) {
				PrevTempPos = CurrentPos;
			}
			if (FVector2D::Distance(CurrentPos, PrevTempPos) > AgentPerceptualRange) {
				break;
			}
			double Left = FVector2D::Distance(CurrentPos, PrevTempPos);
			double Right = FVector2D::Distance(CurrentPos, TempPos);
			double alpha = (AgentPerceptualRange - Left) / (Right - Left);
			TempPos = (1.f - alpha) * PrevTempPos + alpha * TempPos;
		}

		if (IsVisible(TempPos)) {
			NaviVar.VisiblePosition = TempPos;
			if (bOutRange) {
				break;
			}
		}
		else {
			break;
		}

		NaviVar.NavigationStop = p + 1;
	}
}

bool AAgentAIController::SetTargetPositionAndFacing(FVector TargetPosition, FVector2D TargetFacing)
{
	NaviVar.TargetPosition.push_back(TargetPosition);
	NaviVar.TargetFacing.push_back(TargetFacing);
	UE_LOG(LogTemp, Warning, TEXT("Location = (%lf, %lf), Destination = (%lf, %lf)"), 
		CurrentLocation().X, CurrentLocation().Y, 
		NaviVar.TargetPosition.back().X, NaviVar.TargetPosition.back().Y);
	if (!ResetCurrentTargetPositionAndFacing()) {
		// fallback original state
		NaviVar.TargetPosition.pop_back();
		NaviVar.TargetFacing.pop_back();
		return false;
	}

	return true;
}

bool AAgentAIController::ResetCurrentTargetPositionAndFacing()
{
	NaviVar.ArriveState = EnumArriveState::None;
	auto SMPTemp = NavigationMapPtr->GetNearestValidPosition(To3D(CurrentLocation(), NavigationMapPtr->DefaultZ()));
	auto EMPTemp = NavigationMapPtr->GetNearestValidPosition(NaviVar.TargetPosition.back());
	if (!SMPTemp.has_value() || !EMPTemp.has_value()) return false;
	NaviVar.StartMidPosition = SMPTemp.value();
	NaviVar.EndMidPosition = EMPTemp.value();

	NaviVar.NavigationPath = NavigationMapPtr->PathFinding(
		To2D(NaviVar.StartMidPosition),
		To2D(NaviVar.EndMidPosition));
	if (!NaviVar.NavigationPath.has_value()) return false;
	UpdateFarthestVisiblePoint(true);
	ReactRtVar.Reset();
	return true;
}

void AAgentAIController::UpdateVelocityFromPathPlanning(float DeltaTime)
{
	if (NaviVar.ArriveState == EnumArriveState::None) {
		if (FVector2D::Distance(CurrentLocation(), To2D(NaviVar.StartMidPosition)) >= TargetEps) {
			FVector2D DPos = To2D(NaviVar.StartMidPosition) - CurrentLocation();
			Facing = DPos.GetSafeNormal();
			Velocity = Facing * FMath::Min(AgentWalkVelocity, DPos.Length() / DeltaTime);
			ReactRtVar.BackupFacing = Facing;
			ReactRtVar.BackupVelocity = Velocity;
		}
		else {
			NaviVar.ArriveState = EnumArriveState::ArrivedStartMid;
		}
	}
	if (NaviVar.ArriveState == EnumArriveState::ArrivedStartMid) {
		if (FVector2D::Distance(CurrentLocation(), To2D(NaviVar.EndMidPosition)) >= TargetEps) {
			UpdateFarthestVisiblePoint(false);

			FVector2D DPos = NaviVar.VisiblePosition - CurrentLocation();
			Facing = DPos.GetSafeNormal();
			if (NaviVar.NavigationStop != NaviVar.NavigationPath.value().size() + 1) {
				Velocity = Facing * AgentWalkVelocity;
			}
			else {
				Velocity = Facing * FMath::Min(AgentWalkVelocity, DPos.Length() / DeltaTime);
			}
			ReactRtVar.BackupFacing = Facing;
			ReactRtVar.BackupVelocity = Velocity;
		}
		else {
			NaviVar.ArriveState = EnumArriveState::ArrivedEndMid;
		}
	}
	if (NaviVar.ArriveState == EnumArriveState::ArrivedEndMid) {
		if (FVector2D::Distance(CurrentLocation(), To2D(NaviVar.TargetPosition.back())) >= TargetEps) {
			FVector2D DPos = To2D(NaviVar.TargetPosition.back()) - CurrentLocation();
			Facing = DPos.GetSafeNormal();
			Velocity = Facing * FMath::Min(AgentWalkVelocity, DPos.Length() / DeltaTime);
			ReactRtVar.BackupFacing = Facing;
			ReactRtVar.BackupVelocity = Velocity;
		}
		else {
			NaviVar.ArriveState = EnumArriveState::ArrivedTarget;
		}
	}
	if (NaviVar.ArriveState == EnumArriveState::ArrivedTarget) {
		Velocity = FVector2D(0.f);
		Facing = NaviVar.TargetFacing.back();
	}

	if (GroupControllerPtr->DisplayArrivedState) {
		FColor StateColor;
		switch (NaviVar.ArriveState) {
		case EnumArriveState::None: StateColor = FColor::Black; break;
		case EnumArriveState::ArrivedStartMid: StateColor = FColor::Yellow; break;
		case EnumArriveState::ArrivedEndMid: StateColor = FColor::Green; break;
		case EnumArriveState::ArrivedTarget: StateColor = FColor::Red; break;
		}
		DrawDebugState(StateColor);
	}
}

void AAgentAIController::UpdateVelocityToAvoidOnComingAgents(float DeltaTime)
{
	if (NaviVar.ArriveState == EnumArriveState::ArrivedTarget) return;

	if (ReactRtVar.OnComingBlock && !IsWaitingInLine(DeltaTime)) {
		float Angle = 0.f;
		while (Angle < 360.f) {
			if (!HasAgentInDirection(DeltaTime, Facing.GetRotated(Angle)) 
				// TODO (changyu): should enable more precise judgement.
				// && IsVisible(CurrentLocation() + PedestrianBoudingBoxSize * Facing.GetRotated(Angle), true)
				) {
				ReactRtVar.AvoidFacing = Facing.GetRotated(Angle);
				ReactRtVar.AvoidRemain = AvoidOnComingAgentsDuration;
				ReactRtVar.OnComingBlock = false;
				break;
			}
			Angle += AngleDensity;
		}
	}

	if (ReactRtVar.AvoidRemain > 0.f) {
		if (GroupControllerPtr->DisplayAvoidRemain) {
			DrawDebugState(FColor::Red);
		}

		ReactRtVar.AvoidRemain -= DeltaTime;
		if (ReactRtVar.AvoidRemain <= 0) {
			ResetCurrentTargetPositionAndFacing();
			UpdateVelocityFromPathPlanning(DeltaTime);
			ReactRtVar.DisableE = false;
		}
		Facing = ReactRtVar.AvoidFacing;
		Velocity = Facing * Velocity.Length();

		ReactRtVar.DisableE = true;
		ReactRtVar.ESlowDownFactor = 1.f;
	}
	else {
		ReactRtVar.DisableE = false;
	}
}

bool AAgentAIController::IsGoalNeedsToWaitInLine() const
{
	return
		(MentalStVar.GoalStack.back().first == AgentGoalType::BuyADrink) ||
		(MentalStVar.GoalStack.back().first == AgentGoalType::BuyATicket) ||
		(MentalStVar.GoalStack.back().first == AgentGoalType::GoToThePlatform);
}

bool AAgentAIController::IsSameGoalWithOnComingAgent() const
{
	if (MentalStVar.GoalStack.size() == 0) return false;
	// TODO(changyu): a WIP condition
	if (HistoryVar.IsStucked && ReactRtVar.OnComingAgent->HistoryVar.IsStucked &&
		Facing.Dot(ReactRtVar.OnComingAgent->Facing) <= 0.f
		&& FVector2D::Distance(CurrentLocation(), NaviVar.VisiblePosition) < 
		FVector2D::Distance(ReactRtVar.OnComingAgent->CurrentLocation(), ReactRtVar.OnComingAgent->NaviVar.VisiblePosition))
		return false;

	const auto& OnComingGoals = ReactRtVar.OnComingAgent->MentalStVar.GoalStack;
	if (OnComingGoals.size() > 0) {
		if (MentalStVar.GoalStack.back() == OnComingGoals.back()) {
			return true;
		}
	}
	const auto& OnComingCompletedGoals = ReactRtVar.OnComingAgent->MentalStVar.CompletedGoalStack;
	if (OnComingCompletedGoals.size() > 0) {
		return MentalStVar.GoalStack.back() == OnComingCompletedGoals.back();
	}

	return false;
}

bool AAgentAIController::IsWaitingInLine(float DeltaTime) const
{
	if (ReactRtVar.OnComingAgent == nullptr) return false;
	bool IsArrvingSoon = (NaviVar.NavigationStop == NaviVar.NavigationPath.value().size() + 1) &&
		(NaviVar.ArriveState == EnumArriveState::ArrivedStartMid) &&
		FVector2D::Distance(CurrentLocation(), To2D(NaviVar.TargetPosition.back())) <= MaximumLineLength;

	bool Ret = IsArrvingSoon && IsGoalNeedsToWaitInLine() && IsSameGoalWithOnComingAgent();

	if (Ret && GroupControllerPtr->DisplayWaitingInLine) {
		DrawDebugState(FColor::Red);
	}
	return Ret;
}

void AAgentAIController::FindNeighboringAgents()
{
	int CellX = int((CurrentLocation().X - NavigationMapPtr->Corner.X) / NavigationMapPtr->MobileCellSize);
	int CellY = int((CurrentLocation().Y - NavigationMapPtr->Corner.Y) / NavigationMapPtr->MobileCellSize);

	using AgentPair = std::pair<float, const AAgentAIController*>;
	std::vector<AgentPair> Candidates;
	for (int dX = -1; dX <= 1; dX++) {
		for (int dY = -1; dY <= 1; dY++) {
			if (CellX + dX >= 0 && CellX + dX < std::get<0>(NavigationMapPtr->MobileMapSize) &&
				CellY + dY >= 0 && CellY + dY < std::get<1>(NavigationMapPtr->MobileMapSize)) {
				int CellIndex = (CellX + dX) * std::get<1>(NavigationMapPtr->MobileMapSize) + (CellY + dY);
				for (int i = 0; i < NavigationMapPtr->MobileMap[CellIndex].Num(); i++) {
					const AAgentAIController* Agent = NavigationMapPtr->MobileMap[CellIndex][i];
					FVector2D AgentPos = Agent->CurrentLocation();
					FVector2D Dir = AgentPos - CurrentLocation();
					float DirNorm = Dir.Length();
					if (DirNorm < AgentPerceptualRange && Dir.Dot(Facing) > DirNorm * CosSensingFanAngle &&
						this->ID != Agent->ID) {
						if (IsVisible(AgentPos, true)) {
							Candidates.push_back(std::make_pair(DirNorm, Agent));
						}
					}
				}
			}
		}
	}

	std::sort(Candidates.begin(), Candidates.end(), [](const AgentPair& A1, const AgentPair& A2) {
		return A1.first < A2.first;
	});
	Neighbors.Empty();
	for (int i = 0; i < Candidates.size(); i++) {
		Neighbors.Add(Candidates[i].second);
		if (Neighbors.Num() == MaximumAttentionAgents) {
			break;
		}
	}
}

void AAgentAIController::UpdateFinalMovement(float DeltaTime)
{
	FVector Velocity3 = To3D(Velocity, 0.f);
	FVector Facing3 = To3D(Facing, 0.f);
	Cast<ACharacter>(GetPawn())->GetCharacterMovement()->Velocity = Velocity3;

	if (!Facing3.IsNearlyZero()) {
		// Record facing in last time-step for Routine B.
		ReactRtVar.PreviousFacing = Facing;
		SetControlRotation(Facing3.Rotation());
		Cast<ACharacter>(GetPawn())->FaceRotation(Facing3.Rotation(), DeltaTime);
		DrawDebugFacing(Facing, FColor::Green);
	}

	if (GroupControllerPtr->DisplayIntermediateTargets) {
		if (NaviVar.ArriveState != EnumArriveState::ArrivedTarget) {
			DrawDebugPoint(GetWorld(), To3D(NaviVar.VisiblePosition, NavigationMapPtr->DefaultZ()), 4.f, FColor::Blue, false, -1.f, 0);
			DrawDebugPoint(GetWorld(), To3D(To2D(NaviVar.StartMidPosition), NavigationMapPtr->DefaultZ()), 4.f, FColor::Yellow, false, -1.f, 0);
			DrawDebugPoint(GetWorld(), To3D(To2D(NaviVar.EndMidPosition), NavigationMapPtr->DefaultZ()), 4.f, FColor::Green, false, -1.f, 0);
			DrawDebugPoint(GetWorld(), To3D(To2D(NaviVar.TargetPosition.back()), NavigationMapPtr->DefaultZ()), 4.f, FColor::Red, false, -1.f, 0);
		}
	}
}

bool AAgentAIController::IsArrivedTarget() const
{
	return NaviVar.ArriveState == EnumArriveState::ArrivedTarget && (NaviVar.TargetFacing.back() - Facing).IsNearlyZero();
}

void AAgentAIController::MergeRoutineUpdate()
{
	Facing = ReactRtVar.ModifiedFacing;
	Velocity = ReactRtVar.ModifiedVelocity;
}

void AAgentAIController::RoutineE(float DeltaTime)
{
	ReactRtVar.ModifiedFacing = Facing;
	ReactRtVar.ModifiedVelocity = Velocity;
	if (ReactRtVar.DisableE || Velocity.IsNearlyZero() || NaviVar.ArriveState == EnumArriveState::ArrivedTarget) {
		return;
	}

	ReactRtVar.ModifiedVelocity *= ReactRtVar.ESlowDownFactor;

	if (HasAgentInFrontSafeArea(DeltaTime)) {
		ReactRtVar.ESlowDownFactor *= RoutineESlowDownFactorRatio;
		return;
	}
	else {
		ReactRtVar.ESlowDownFactor = 1.f;
	}
}

void AAgentAIController::RoutineC(float DeltaTime)
{
	ReactRtVar.ModifiedFacing = Facing;
	ReactRtVar.ModifiedVelocity = Velocity;
	if (Velocity.IsNearlyZero() || NaviVar.ArriveState == EnumArriveState::ArrivedTarget) {
		return;
	}

	float R = MinimumDistanceAllowed;
	FVector2D Force(0.f);
	
	for (int i = 0; i < Neighbors.Num(); i++) {
		FVector2D Dir = Neighbors[i]->CurrentLocation() - CurrentLocation();
		FVector2D NorDir, TanDir;
		DecomposeVector(Dir, Facing, NorDir, TanDir);
		if (NorDir.Length() < R - (4 / R) * (TanDir.SquaredLength()) &&
			Facing.Dot(Neighbors[i]->Facing) > CosSimilarity) {
			Force += Repulsiveness * (Dir / Dir.Length()) / (Dir.Length() - MinimumDistanceAllowed);
		}
	}

	FVector2D Acc = Force / Inertia;
	FVector2D NorAcc, TanAcc;
	DecomposeVector(Acc, Facing, NorAcc, TanAcc);
	TanAcc *= IsWaitingInLine(DeltaTime) ? WaitingInLineCrowdingFactor : CrowdingFactor;
	ReactRtVar.ModifiedVelocity += (NorAcc + TanAcc) * DeltaTime;
	ReactRtVar.ModifiedFacing = ReactRtVar.ModifiedVelocity.GetSafeNormal();
}

void AAgentAIController::RoutineD(float DeltaTime)
{
	ReactRtVar.ModifiedFacing = Facing;
	ReactRtVar.ModifiedVelocity = Velocity;
	if (Velocity.IsNearlyZero() || NaviVar.ArriveState == EnumArriveState::ArrivedTarget) {
		return;
	}

	DrawDebugFacing(ReactRtVar.ModifiedFacing, FColor::Blue);
	// ReactRtVar.ModifiedVelocity = ReactRtVar.ModifiedVelocity.GetRotated(ReactRtVar.DTurningFactor);
	// ReactRtVar.ModifiedFacing = ReactRtVar.ModifiedVelocity.GetSafeNormal();

	const float INF = 1e6;
	float R = MinimumDistanceAllowed;
	// The most imminent potential collision
	float ImmTAgent = INF;
	float ImmTSelf = INF;
	const AAgentAIController* ImmCStar = nullptr;
	COLLSION_TYPE ImmCollisionType = INVALID;

	for (int i = 0; i < Neighbors.Num(); i++) {
		FVector2D Dir = Neighbors[i]->CurrentLocation() - CurrentLocation();
		FVector2D NorDir, TanDir;
		DecomposeVector(Dir, Facing, NorDir, TanDir);

		// Skip one's temporary crowd
		if (NorDir.Length() < R - (4 / R) * (TanDir.SquaredLength()) &&
			Facing.Dot(Neighbors[i]->Facing) > CosSimilarity) {
			continue;
		}

		if (Facing.Dot(Neighbors[i]->Facing) > -HeadonCosSimilarity) {
			// H will estimate who will arrive first at the anticipated intersection point p
			float TSelf = INF;
			float TAgent = INF;
			FMatrix44d TraM;
			TraM.M[0][0] = Facing.X;
			TraM.M[0][1] = 0.f;
			TraM.M[0][2] = -1.f;
			TraM.M[0][3] = 0.f;

			TraM.M[1][0] = Facing.Y;
			TraM.M[1][1] = 0.f;
			TraM.M[1][2] = 0.f;
			TraM.M[1][3] = -1.f;

			TraM.M[2][0] = 0.f;
			TraM.M[2][1] = Neighbors[i]->Facing.X;
			TraM.M[2][2] = -1.f;
			TraM.M[2][3] = 0.f;

			TraM.M[3][0] = 0.f;
			TraM.M[3][1] = Neighbors[i]->Facing.Y;
			TraM.M[3][2] = 0.f;
			TraM.M[3][3] = -1.f;

			if (TraM.Determinant() < 1e-3) { // Singular Matrix
				continue;
			}

			FMatrix44d InvTraM = TraM.Inverse();

			for (int _ = 0; _ < 9; _++) { // Considering both boundary side of the pedesitrans
				FVector2D PosSelf = CurrentLocation() +
					Facing.GetRotated(90) * MinimumDistanceAllowed * (_ / 3 - 1);
				FVector2D PosAgent = Neighbors[i]->CurrentLocation() +
					Neighbors[i]->Facing.GetRotated(90) * MinimumDistanceAllowed * (_ % 3 - 1);
				FVector4d TraB(
					-PosSelf[0],
					-PosSelf[1],
					-PosAgent[0],
					-PosAgent[1]
				);
				FVector4d TraX = InvTraM.TransformFVector4(TraB);
				if (TraX[0] < 0.f || TraX[1] < 0.f) {
					continue;
				}

				if (TraX[0] < TSelf) {
					TSelf = TraX[0];
					TAgent = TraX[1];
				}
			}

			if (TSelf < ImmTSelf) {
				ImmTSelf = TSelf;
				ImmTAgent = TAgent;
				ImmCStar = Neighbors[i];
				ImmCollisionType = CROSS_COLLISION;
			}
		}
		else {
			FVector2D SelfNorVel, SelfTanVel;
			DecomposeVector(Velocity, Facing, SelfNorVel, SelfTanVel);
			FVector2D AgentNorVel, AgentTanVel;
			DecomposeVector(Neighbors[i]->Velocity, Neighbors[i]->Facing, AgentNorVel, AgentTanVel);
			float T = NorDir.Length() / (SelfNorVel.Length() + AgentNorVel.Length());
			float Gap = (TanDir + T * (AgentTanVel - SelfTanVel)).Length();
			if (Gap < MinimumDistanceAllowed && T < ImmTSelf) {
				ImmTSelf = T;
				ImmTAgent = T;
				ImmCStar = Neighbors[i];
				ImmCollisionType = HEADON_COLLISION;
			}
		}
	}

	FVector2D NorDir, TanDir;
	auto TurningVelocity = [&](float MaxDegree, int sign) {
		/*if (sign * FMath::Sign(FVector2D::CrossProduct(NorDir, TanDir)) != FMath::Sign(DTurningFactor)) {
			DTurningFactor = 0.f;
		}*/

		/*
		DTurningFactor += sign * FMath::Sign(FVector2D::CrossProduct(NorDir, TanDir)) * TurningFactorRatio;
		DTurningFactor = FMath::Clamp(DTurningFactor, -MaxDegree, MaxDegree);
		ModifiedVelocity = ModifiedVelocity.GetRotated(DTurningFactor);
		ModifiedFacing = ModifiedVelocity.GetSafeNormal();
		*/
		float NewDTurningFactor = ReactRtVar.DTurningFactor + sign * FMath::Sign(FVector2D::CrossProduct(NorDir, TanDir)) * TurningFactorRatio;
		NewDTurningFactor = FMath::Clamp(NewDTurningFactor, -MaxDegree, MaxDegree);
		ReactRtVar.ModifiedVelocity = ReactRtVar.ModifiedVelocity.GetRotated(NewDTurningFactor - ReactRtVar.DTurningFactor);
		ReactRtVar.ModifiedFacing = ReactRtVar.ModifiedVelocity.GetSafeNormal();
		ReactRtVar.DTurningFactor = NewDTurningFactor;
	};

	if (ImmCollisionType == HEADON_COLLISION) {
		DecomposeVector(ImmCStar->CurrentLocation() - CurrentLocation(), Facing, NorDir, TanDir);
		TurningVelocity(MaxHeadonCollisionAvoidanceDegree, -1.f);
		DrawDebugFacing(ReactRtVar.ModifiedFacing, FColor::Red);
	}
	else if (ImmCollisionType == CROSS_COLLISION) {
		DecomposeVector(ImmCStar->CurrentLocation() - CurrentLocation(), Facing, NorDir, TanDir);

		// If H determines that it will arrive sooner,
		// it will increase its speed and turn slightly away from C*
		if (ImmTSelf < ImmTAgent) {
			ReactRtVar.ModifiedVelocity *= IncreaseSpeedRatio;
			TurningVelocity(MaxCrossCollisionAvoidanceDegree, -1.f);
		}
		// Otherwise, it will decrease its speed and turn slightly towards C∗
		else {
			ReactRtVar.ModifiedVelocity *= DecreaseSpeedRatio;
			TurningVelocity(MaxCrossCollisionAvoidanceDegree, +1.f);
		}
		DrawDebugFacing(ReactRtVar.ModifiedFacing, FColor::Orange);
	}
	else { // No potential collision
		// Reset turning
		ReactRtVar.DTurningFactor -= FMath::Sign(ReactRtVar.DTurningFactor) * 
			FMath::Min(TurningFactorRatio * TurningFactorRatio, FMath::Abs(ReactRtVar.DTurningFactor));
		DrawDebugFacing(ReactRtVar.ModifiedFacing, FColor::Yellow);
	}
}

void AAgentAIController::RoutineA(float DeltaTime)
{
	ReactRtVar.ModifiedFacing = Facing;
	ReactRtVar.ModifiedVelocity = Velocity;
	if (Velocity.IsNearlyZero() || NaviVar.ArriveState != EnumArriveState::ArrivedStartMid) { // Only avaible during StartMid->EndMid
		return;
	}
	else if (FVector2D::Distance(CurrentLocation(), To2D(NaviVar.EndMidPosition)) < 100.f) {
		return;
	}

	// Temporarily disabling the two routines 
	// and letting the pedestrian accurately follow the detailed path, which already avoids obstacles
	if (FVector2D::Distance(CurrentLocation(), To2D(NaviVar.TargetPosition.back())) <=
		PedestrianBoudingBoxScale * NavigationMapPtr->StationaryCellSize)
		return;
	
	float Angle = 0.f;
	while (Angle < 180.f) {
		// If a large angle (currently set to 90-deg) must be swep
		float W = Angle >= 90.f ? RoutineASlowDownFactorRatio : 1.f;
		for (const auto d : {-1.f, 1.f}) {
			float SignedAngle = d * Angle;
			if (IsVisible(CurrentLocation()
				+ PedestrianBoudingBoxSize * Facing.GetRotated(SignedAngle)
				+ PedestrianBoudingBoxScale * W * DeltaTime * Velocity.GetRotated(SignedAngle))) {
				ReactRtVar.ModifiedVelocity = W * Velocity.GetRotated(SignedAngle);
				ReactRtVar.ModifiedFacing = Facing.GetRotated(SignedAngle);
				ReactRtVar.BackupFacing = ReactRtVar.ModifiedFacing;
				ReactRtVar.BackupVelocity = ReactRtVar.ModifiedVelocity;
				return;
			}
		}
		Angle += AngleDensity;
	}
}

void AAgentAIController::RoutineF(float DeltaTime)
{
	ReactRtVar.ModifiedFacing = Facing;
	ReactRtVar.ModifiedVelocity = Velocity;
	if (Velocity.IsNearlyZero() || NaviVar.ArriveState != EnumArriveState::ArrivedStartMid) { // Only avaible during StartMid->EndMid
		return;
	}
	else if (FVector2D::Distance(CurrentLocation(), To2D(NaviVar.EndMidPosition)) < 100.f) {
		return;
	}

	ReactRtVar.ModifiedVelocity *= ReactRtVar.FSlowDownFactor;

	if (!IsVisible(CurrentLocation()
		+ PedestrianBoudingBoxSize * Facing
		+ PedestrianBoudingBoxScale * Velocity * DeltaTime)) {
		ReactRtVar.ModifiedFacing = ReactRtVar.BackupFacing;
		ReactRtVar.ModifiedVelocity = ReactRtVar.BackupVelocity;
		if (HasAgentInFrontSafeArea(DeltaTime)) {
			ReactRtVar.FSlowDownFactor *= RoutineFSlowDownFactorRatio;
			return;
		}
	}

	ReactRtVar.FSlowDownFactor = FMath::Min(1.f, ReactRtVar.FSlowDownFactor / RoutineFSlowDownFactorRatio);
}

void AAgentAIController::RoutineB(float DeltaTime)
{
	ReactRtVar.ModifiedFacing = Facing;
	ReactRtVar.ModifiedVelocity = Velocity;
	
	// NOTE: Routine B is always at work even when agent is arrived or velocity is zero.
	if (ReactRtVar.PreviousFacing.IsNearlyZero()) {
		ReactRtVar.PreviousFacing = Facing;
		return;
	}

	ReactRtVar.ModifiedVelocity *= ReactRtVar.BSlowDownFactor;

	// Fix sharp turns
	// DrawDebugFacing(ReactRtVar.PreviousFacing, FColor::Blue);
	// DrawDebugFacing(Facing, FColor::Red);
	if (FMath::RadiansToDegrees(FMath::Acos(ReactRtVar.PreviousFacing.Dot(Facing))) > RoutineBTurningFactorThreshold) {
		float TurnDeg = RoutineBTurningFactorThreshold * FMath::Sign(FVector2D::CrossProduct(ReactRtVar.PreviousFacing, Facing));
		ReactRtVar.BSlowDownFactor *= RoutineBSlowDownFactorRatio;
		ReactRtVar.ModifiedFacing = ReactRtVar.PreviousFacing.GetRotated(TurnDeg);
		ReactRtVar.ModifiedVelocity = ReactRtVar.ModifiedFacing * ReactRtVar.ModifiedVelocity.Length();
		ReactRtVar.BackupFacing = ReactRtVar.ModifiedFacing;
		ReactRtVar.BackupVelocity = ReactRtVar.ModifiedVelocity;

		if (!IsVisible(CurrentLocation()
			+ PedestrianBoudingBoxSize * Facing
			+ PedestrianBoudingBoxScale * Velocity * DeltaTime)) {
			ReactRtVar.BSlowDownFactor *= RoutineBUrgentSlowDownFactorRatio;
		}

		return;
	}

	ReactRtVar.BSlowDownFactor = FMath::Min(1.f, ReactRtVar.BSlowDownFactor / RoutineBUrgentSlowDownFactorRatio);
}

void AAgentAIController::StuckWarning()
{
	HistoryVar.IsStucked = false;
	if (NaviVar.ArriveState == EnumArriveState::ArrivedTarget) {
		HistoryVar.HistoryPosList.clear();
		HistoryVar.HistoryFacingList.clear();
		return;
	}

	HistoryVar.HistoryPosList.push_back(CurrentLocation());
	HistoryVar.HistoryFacingList.push_back(Facing);
	if (HistoryVar.HistoryPosList.size() > HistoryVar.HistoryListMaxLength) {
		HistoryVar.HistoryPosList.pop_front();
		HistoryVar.HistoryFacingList.pop_front();
	}

	if (HistoryVar.HistoryPosList.size() == HistoryVar.HistoryListMaxLength) {
		float AverageX = 0.f, AverageY = 0.f;
		float AverageA = 0.f, AverageB = 0.f;
		for (const auto& Pos : HistoryVar.HistoryPosList) {
			AverageX += Pos.X;
			AverageY += Pos.Y;
		}
		for (const auto& Fac : HistoryVar.HistoryFacingList) {
			AverageA += Fac.X;
			AverageB += Fac.Y;
		}
		AverageX /= HistoryVar.HistoryListMaxLength;
		AverageY /= HistoryVar.HistoryListMaxLength;
		AverageA /= HistoryVar.HistoryListMaxLength;
		AverageB /= HistoryVar.HistoryListMaxLength;
		float VarianceX = 0.f, VarianceY = 0.f;
		float VarianceA = 0.f, VarianceB = 0.f;
		for (const auto& Pos : HistoryVar.HistoryPosList) {
			VarianceX += FMath::Square(Pos.X - AverageX);
			VarianceY += FMath::Square(Pos.Y - AverageY);
		}
		for (const auto& Fac : HistoryVar.HistoryFacingList) {
			VarianceA += FMath::Square(Fac.X - AverageA);
			VarianceB += FMath::Square(Fac.Y - AverageB);
		}
		VarianceX = FMath::Sqrt(VarianceX / HistoryVar.HistoryListMaxLength);
		VarianceY = FMath::Sqrt(VarianceY / HistoryVar.HistoryListMaxLength);
		VarianceA = FMath::Sqrt(VarianceA / HistoryVar.HistoryListMaxLength);
		VarianceB = FMath::Sqrt(VarianceB / HistoryVar.HistoryListMaxLength);
		if (VarianceX + VarianceY + VarianceA + VarianceB < HistoryVar.AgentStuckThreshold) {
			HistoryVar.IsStucked = true;
			if (GroupControllerPtr->DisplayStuckWarning) {
				DrawDebugState(FColor::Red);
			}
		}
	}
}

void AAgentAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AAgentAIController::DecomposeVector(const FVector2D& Vector,
	const FVector2D& Facing,
	FVector2D& Nor,
	FVector2D& Tan)
{
	Nor = Vector.Dot(Facing) * Facing;
	Tan = Vector - Nor;
}

FVector2D AAgentAIController::To2D(const FVector& Vector)
{
	return FVector2D(Vector.X, Vector.Y);
}

FVector AAgentAIController::To3D(const FVector2D& Vector2D, float Z)
{
	return FVector(Vector2D.X, Vector2D.Y, Z);
}

void AAgentAIController::DrawDebugState(FColor Color, float Height) const
{
	DrawDebugPoint(GetWorld(), To3D(CurrentLocation(), NavigationMapPtr->DefaultZ() + Height), 5.f,
		Color, false, -1.f, 0);
}

void AAgentAIController::DrawDebugFacing(FVector2D F, FColor Color) const
{
	/*
	FVector StartPos(To3D(CurrentLocation(), NavigationMapPtr->DefaultZ() + 10.f));
	FVector EndPos = StartPos + To3D(F, 0) * 50.f;
	DrawDebugLine(GetWorld(), StartPos, EndPos, Color, false, -1, 0, 3);
	*/
}