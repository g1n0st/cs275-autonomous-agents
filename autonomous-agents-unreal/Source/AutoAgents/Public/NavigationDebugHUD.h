#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SlateBasics.h"
#include "SlateExtras.h"
#include "NavigationDebugHUD.generated.h"

UCLASS()
class AUTOAGENTS_API ANavigationDebugHUD : public AHUD
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere)
	UTexture2D* DebugTexture;

	virtual void BeginPlay() override;
	virtual void DrawHUD() override;
	void SetTexture(UTexture2D* Texture);

	// NOTE(changyu): ad-hoc debug tool-kit
	/*
	TSharedPtr<SEditableTextBox> InputTextBox;
	FText InputText;
	*/
};
