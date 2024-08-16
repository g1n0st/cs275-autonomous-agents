#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AgentAIController.h"
#include "AgentsGroupController.generated.h"

UCLASS()
class AUTOAGENTS_API AAgentsGroupController : public AActor
{
	GENERATED_BODY()

public:
	AAgentsGroupController();
	int AddAgent(AAgentAIController* Agent);
	virtual void Tick(float DeltaTime) override;

	// Display Agent's Mental State
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayMentalState;
	// Display Agent's Intermediate Targets
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayIntermediateTargets;
	// Display Agent's Arrived State
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayArrivedState;
	// Display Agent's Stuck Warning
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayStuckWarning;
	// Display Agent's On Coming Block
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayOnComingBlock;
	// Display Agent's Waiting In Line State
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayWaitingInLine;
	// Display Whether Agent is Avoiding
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") bool DisplayAvoidRemain;
	// View Height for Player Agent
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") float ViewHeight;
	// Global Random Seed
	UPROPERTY(EditAnywhere, Category = "Autonomous Agents Config") int GlobalRandomSeed;

protected:
	virtual void BeginPlay() override;

private:
	// Global navigation map for path planning and obstacles avoidance
	ANavigationSystemMapVolume* NavigationMapPtr;

	TArray<AAgentAIController*> Agents;

	void UpdateCognitiveControl(float DeltaTime);
	void UpdatePathPlanning(float DeltaTime);
	void FindNeighboringAgents();

	void ReactiveBehaviorRoutines(float DeltaTime);
	void ApplyRoutineE(float DeltaTime);
	void ApplyRoutineC(float DeltaTime);
	void ApplyRoutineD(float DeltaTime);
	void ApplyRoutineA(float DeltaTime);
	void ApplyRoutineF(float DeltaTime);
	void ApplyRoutineB(float DeltaTime);
	void AllAdvance(float DeltaTime);

	void StuckWarning();
};
