// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "ReadWriteCoord.generated.h"

/**
 * 
 */
UCLASS()
class CARLAUE4_API UReadWriteCoord : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

		UFUNCTION(BlueprintCallable, Category = "Custom", meta = (Keywords = "Save"))
		static bool SaveArrayText(FString SaveDirectory, FString FileName, TArray<FString> SaveText, bool AllowOverwriting);

		UFUNCTION(BlueprintCallable, Category = "Custom", meta = (Keywords = "Save"))
		static bool SaveText(FString SaveDirectory, FString FileName, FString SaveText, bool AllowOverwriting);

		UFUNCTION(BlueprintCallable, Category = "Custom", meta = (Keywords = "Read"))
		static FString LoadFileToString(FString Directory, FString FileName);
	
};

