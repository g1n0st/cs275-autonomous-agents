// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutoAgentsGameMode.h"
#include "AutoAgentsPlayerController.h"
#include "AutoAgentsCharacter.h"
#include "UObject/ConstructorHelpers.h"
#include "NavigationDebugHUD.h"

AAutoAgentsGameMode::AAutoAgentsGameMode()
{
#define PLAYER_CAMERA
#ifdef  PLAYER_CAMERA
	// use our custom PlayerController class
	PlayerControllerClass = AAutoAgentsPlayerController::StaticClass();

	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/TopDown/Blueprints/BP_TopDownCharacter"));
	if (PlayerPawnBPClass.Class != nullptr)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}

	// set default controller to our Blueprinted controller
	static ConstructorHelpers::FClassFinder<APlayerController> PlayerControllerBPClass(TEXT("/Game/TopDown/Blueprints/BP_TopDownPlayerController"));
	if(PlayerControllerBPClass.Class != NULL)
	{
		PlayerControllerClass = PlayerControllerBPClass.Class;
	}
#endif

	HUDClass = ANavigationDebugHUD::StaticClass();
}