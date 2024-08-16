// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Templates/SubclassOf.h"
#include "GameFramework/PlayerController.h"
#include "InputActionValue.h"
#include "NavigationSystemMapVolume.h"
#include "AutoAgentsPlayerController.generated.h"
class UNiagaraSystem;
class ANavigationSystemMapVolume;

UCLASS()
class AAutoAgentsPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	AAutoAgentsPlayerController();

	/** Time Threshold to know if it was a short press */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	float ShortPressThreshold;

	/** FX Class that we will spawn when clicking */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
	UNiagaraSystem* FXCursor;

	/** MappingContext */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Input, meta=(AllowPrivateAccess = "true"))
	class UInputMappingContext* DefaultMappingContext;
	
	/** Jump Input Action */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Input, meta=(AllowPrivateAccess = "true"))
	class UInputAction* SetDestinationClickAction;

	/** Jump Input Action */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Input, meta=(AllowPrivateAccess = "true"))
	class UInputAction* SetDestinationTouchAction;

protected:
	virtual void SetupInputComponent() override;
	
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Input handlers for SetDestination action. */
	void OnInputStarted();
	void OnSetDestinationTriggered();
	void OnSetDestinationReleased();

private:
	// Global navigation map for path planning and obstacles avoidance
	ANavigationSystemMapVolume* NavigationMapPtr;

	bool IsVisible(FVector2D Position);
	void UpdateFarthestVisiblePoint(bool bResetPath = false);
	FVector TargetPosition;
	FVector2D VisiblePosition;
	ANavigationSystemMapVolume::Path NavigationPath;
	int NavigationStop;
	float FollowTime; // For how long it has been pressed

	// Constants
	const float AgentPerceptualRange = 500.f;
};


