// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutoAgentsPlayerController.h"
#include "GameFramework/Pawn.h"
#include "Blueprint/AIBlueprintHelperLibrary.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "AutoAgentsCharacter.h"
#include "Engine/World.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Kismet/GameplayStatics.h"

AAutoAgentsPlayerController::AAutoAgentsPlayerController()
{
	bShowMouseCursor = true;
	DefaultMouseCursor = EMouseCursor::Default;
	TargetPosition = FVector::ZeroVector;
	FollowTime = 0.f;
}

void AAutoAgentsPlayerController::BeginPlay()
{
	Super::BeginPlay();

	//Add Input Mapping Context
	if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(GetLocalPlayer()))
	{
		Subsystem->AddMappingContext(DefaultMappingContext, 0);
	}

	TArray<AActor*> NavMap;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANavigationSystemMapVolume::StaticClass(), NavMap);
	NavigationMapPtr = Cast<ANavigationSystemMapVolume>(NavMap[0]);
}

void AAutoAgentsPlayerController::SetupInputComponent()
{
	// set up gameplay key bindings
	Super::SetupInputComponent();

	// Set up action bindings
	if (UEnhancedInputComponent* EnhancedInputComponent = CastChecked<UEnhancedInputComponent>(InputComponent))
	{
		// Setup mouse input events
		EnhancedInputComponent->BindAction(SetDestinationClickAction, ETriggerEvent::Started, this, &AAutoAgentsPlayerController::OnInputStarted);
		EnhancedInputComponent->BindAction(SetDestinationClickAction, ETriggerEvent::Triggered, this, &AAutoAgentsPlayerController::OnSetDestinationTriggered);
		EnhancedInputComponent->BindAction(SetDestinationClickAction, ETriggerEvent::Completed, this, &AAutoAgentsPlayerController::OnSetDestinationReleased);
		EnhancedInputComponent->BindAction(SetDestinationClickAction, ETriggerEvent::Canceled, this, &AAutoAgentsPlayerController::OnSetDestinationReleased);
	}
}

void AAutoAgentsPlayerController::OnInputStarted()
{
	StopMovement();
}

// Triggered every frame when the input is held down
void AAutoAgentsPlayerController::OnSetDestinationTriggered()
{
	FollowTime += GetWorld()->GetDeltaSeconds();
	
	FHitResult Hit;
	bool bHitSuccessful = GetHitResultUnderCursor(ECollisionChannel::ECC_Visibility, true, Hit);
	// If we hit a surface, cache the location
	if (bHitSuccessful) {
		TargetPosition = Hit.Location;
	}
}

void AAutoAgentsPlayerController::OnSetDestinationReleased()
{
	// If it was a short press
	if (FollowTime <= ShortPressThreshold)
	{
		FVector ActorLocation = GetPawn()->GetActorLocation();
		UE_LOG(LogTemp, Warning, TEXT("Location = (%lf, %lf), Destination = (%lf, %lf)"), ActorLocation.X, ActorLocation.Y, TargetPosition.X, TargetPosition.Y);
		
		if (NavigationMapPtr->GetQuadtreeNode(FVector2D(TargetPosition.X, TargetPosition.Y)) > 0) {
			auto TmpS = NavigationMapPtr->GetNearestValidPosition(FVector(ActorLocation.X, ActorLocation.Y, NavigationMapPtr->DefaultZ()));
			auto TmpE = NavigationMapPtr->GetNearestValidPosition(FVector(TargetPosition.X, TargetPosition.Y, NavigationMapPtr->DefaultZ()));
			if (TmpS.has_value() && TmpE.has_value()) {
				NavigationPath = NavigationMapPtr->PathFinding(FVector2D(TmpS.value().X, TmpS.value().Y), FVector2D(TmpE.value().X, TmpE.value().Y));
				if (NavigationPath.has_value()) {
					NavigationMapPtr->VisualizePathOnQuadtree(NavigationPath);
					UpdateFarthestVisiblePoint(true);
				}
			}
			// In other case, target is not valid. Make sure path-finding algorithm will not fail.
		}

		// We move there and spawn some particles
		// UAIBlueprintHelperLibrary::SimpleMoveToLocation(this, TargetPosition);
		UNiagaraFunctionLibrary::SpawnSystemAtLocation(this, FXCursor, TargetPosition, FRotator::ZeroRotator, FVector(1.f, 1.f, 1.f), true, true, ENCPoolMethod::None, true);
	}

	FollowTime = 0.f;
}

bool AAutoAgentsPlayerController::IsVisible(FVector2D Position)
{
	FVector ActorLocation = GetPawn()->GetActorLocation();
	FVector StartPos = FVector(ActorLocation.X, ActorLocation.Y, NavigationMapPtr->DefaultZ());
	FVector EndPos = FVector(Position.X, Position.Y, NavigationMapPtr->DefaultZ());
	if (FVector::Distance(StartPos, EndPos) > AgentPerceptualRange) {
		return false;
	}

	FCollisionQueryParams CollisionParams;
	CollisionParams.AddIgnoredActor(this);
	FHitResult HitResult;
	bool bHit = GetWorld()->LineTraceSingleByChannel(HitResult, StartPos, EndPos, ECC_Visibility, CollisionParams);
	// NOTE(changyu): debug visualization
	/*
	if (bHit) {
		DrawDebugLine(GetWorld(), StartPos, HitResult.ImpactPoint, FColor::Red, false, 1, 0, 1);
	}
	else {
		DrawDebugLine(GetWorld(), StartPos, EndPos, FColor::Green, false, 1, 0, 1);
	}*/
	return !bHit;
}

void AAutoAgentsPlayerController::UpdateFarthestVisiblePoint(bool bResetPath)
{
	if (!NavigationPath.has_value()) return;
	if (bResetPath) {
		NavigationStop = 0;
	}

	FVector ActorLocation = GetPawn()->GetActorLocation();
	FVector2D CurrentPos = FVector2D(ActorLocation.X, ActorLocation.Y);

	if (IsVisible(FVector2D(TargetPosition.X, TargetPosition.Y))) {
		VisiblePosition = FVector2D(TargetPosition.X, TargetPosition.Y);
		NavigationStop = NavigationPath.value().size() + 1;
	}

	FVector2D TempPos;
	for (int p = NavigationStop; p <= NavigationPath.value().size(); p++) {
		if (p == NavigationPath.value().size()) {
			TempPos = FVector2D(TargetPosition.X, TargetPosition.Y);
		}
		else {
			TempPos = NavigationMapPtr->ActualPosition(NavigationPath.value()[p]);
		}

		bool bOutRange = false;
		if (FVector2D::Distance(CurrentPos, TempPos) > AgentPerceptualRange) {
			bOutRange = true;
			FVector2D PrevTempPos = (p == 0 ? CurrentPos : NavigationMapPtr->ActualPosition(NavigationPath.value()[p - 1]));
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
			VisiblePosition = TempPos;
			if (bOutRange) {
				break;
			}
		}
		else {
			break;
		}

		NavigationStop = p + 1;
	}
}

void AAutoAgentsPlayerController::Tick(float DeltaSeconds) {
	Super::Tick(DeltaSeconds);

	if (NavigationPath.has_value()) {
		if (NavigationPath.value().size() > 0) {
			UpdateFarthestVisiblePoint(false);
			// UAIBlueprintHelperLibrary::SimpleMoveToLocation(this, FVector(TargetPosition.X, TargetPosition.Y, NavigationMapPtr->DefaultZ()));
			UAIBlueprintHelperLibrary::SimpleMoveToLocation(this, FVector(VisiblePosition.X, VisiblePosition.Y, NavigationMapPtr->DefaultZ()));
			// DrawDebugSphere(GetWorld(), FVector(VisiblePosition.X, VisiblePosition.Y, NavigationMapPtr->DefaultZ()), 10.f, 30, FColor::Red, false, 1.f, 0, 1.f);
		}
	}
}