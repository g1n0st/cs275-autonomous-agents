#include "NavigationDebugHUD.h"
#include "Kismet/GameplayStatics.h"
#include "NavigationSystemMapVolume.h"
#include "Engine/Canvas.h"
#include "CanvasItem.h"

void ANavigationDebugHUD::BeginPlay()
{
	Super::BeginPlay();

	// NOTE(changyu): ad-hoc debug tool-kit
	/*
	GEngine->GameViewport->AddViewportWidgetContent(
		SNew(SVerticalBox)
		+ SVerticalBox::Slot()
		.AutoHeight()
		.Padding(FMargin(50.f, 100.f, 1200.f, 0.f))
		[
			SNew(SBox)
				.WidthOverride(30.f)
				.VAlign(VAlign_Center)
				[
					SAssignNew(InputTextBox, SEditableTextBox)
						.HintText(FText::FromString(TEXT("Quadtree Node ID")))
				]
		]
	);*/
}

void ANavigationDebugHUD::DrawHUD()
{
	Super::DrawHUD();

	if (DebugTexture != NULL) {
		DebugTexture->UpdateResource();

		FVector2D ImagePosition = FVector2D(50, 200);
		FVector2D ImageSize = FVector2D(DebugTexture->GetSizeX(), DebugTexture->GetSizeY());

		FCanvasTileItem TileItem(ImagePosition, DebugTexture->GetResource(), ImageSize, FLinearColor::White);
		Canvas->DrawItem(TileItem);
	}

	// Draw Neighbor Connections for debug
	// NOTE(changyu): ad-hoc debug tool-kit
	/*
	int NodeID;
	bool bSuccess = LexTryParseString(NodeID, *InputTextBox->GetText().ToString());
	if (bSuccess) {
		TArray<AActor*> NavMap;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANavigationSystemMapVolume::StaticClass(), NavMap);
		Cast<ANavigationSystemMapVolume>(NavMap[0])->VisualizeQuadtreeMap(NodeID);
	} */
}

void ANavigationDebugHUD::SetTexture(UTexture2D* Texture)
{
	DebugTexture = Texture;
}