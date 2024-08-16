#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "AgentAIController.h"
#include "AgentCharacter.generated.h"

UCLASS(Blueprintable)
class AUTOAGENTS_API AAgentCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	AAgentCharacter();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

};