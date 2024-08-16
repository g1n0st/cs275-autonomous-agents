#include "AgentsGroupController.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

AAgentsGroupController::AAgentsGroupController()
{
	PrimaryActorTick.bCanEverTick = true;
}

int AAgentsGroupController::AddAgent(AAgentAIController *Agent)
{
	int AgentID = Agents.Num();
	Agents.Push(Agent);

	UE_LOG(LogTemp, Warning, TEXT("TotalAgent = %d"), Agents.Num());
	return AgentID;
}

void AAgentsGroupController::BeginPlay()
{
	Super::BeginPlay();

	TArray<AActor*> NavMap;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANavigationSystemMapVolume::StaticClass(), NavMap);
	NavigationMapPtr = Cast<ANavigationSystemMapVolume>(NavMap[0]);
}

void AAgentsGroupController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	NavigationMapPtr->UpdateMobileMap(Agents);

	UpdateCognitiveControl(DeltaTime);
	FindNeighboringAgents();

	UpdatePathPlanning(DeltaTime);
	ReactiveBehaviorRoutines(DeltaTime);
	AllAdvance(DeltaTime);

	StuckWarning();
}

void AAgentsGroupController::UpdateCognitiveControl(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->UpdateMentalState(DeltaTime);
	}
}

void AAgentsGroupController::UpdatePathPlanning(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->UpdateVelocityFromPathPlanning(DeltaTime);
	}
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->UpdateVelocityToAvoidOnComingAgents(DeltaTime);
	}
}

void AAgentsGroupController::FindNeighboringAgents()
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->FindNeighboringAgents();
	}
}

void AAgentsGroupController::ReactiveBehaviorRoutines(float DeltaTime)
{
	ApplyRoutineC(DeltaTime);
	ApplyRoutineA(DeltaTime);
	ApplyRoutineB(DeltaTime);
	ApplyRoutineF(DeltaTime);
	ApplyRoutineE(DeltaTime);
	ApplyRoutineD(DeltaTime);
}

void AAgentsGroupController::ApplyRoutineE(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->RoutineE(DeltaTime);
	}
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->MergeRoutineUpdate();
	}
}

void AAgentsGroupController::ApplyRoutineC(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->RoutineC(DeltaTime);
	}
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->MergeRoutineUpdate();
	}
}

void AAgentsGroupController::ApplyRoutineD(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->RoutineD(DeltaTime);
	}
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->MergeRoutineUpdate();
	}
}

void AAgentsGroupController::ApplyRoutineA(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->RoutineA(DeltaTime);
	}
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->MergeRoutineUpdate();
	}
}

void AAgentsGroupController::ApplyRoutineF(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->RoutineF(DeltaTime);
	}
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->MergeRoutineUpdate();
	}
}

void AAgentsGroupController::ApplyRoutineB(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->RoutineB(DeltaTime);
	}
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->MergeRoutineUpdate();
	}
}

void AAgentsGroupController::AllAdvance(float DeltaTime)
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->UpdateFinalMovement(DeltaTime);
	}
}

void AAgentsGroupController::StuckWarning()
{
	for (int i = 0; i < Agents.Num(); i++) {
		Agents[i]->StuckWarning();
	}
}