// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.


#include "ReadWriteCoord.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformFilemanager.h"

bool UReadWriteCoord::SaveArrayText(FString SaveDirectory, FString FileName, TArray<FString> SaveText, bool AllowOverwriting = true) {

	// Set complete path
	SaveDirectory += "\\";
	SaveDirectory += FileName;

	if (AllowOverwriting) {
		if (FPlatformFileManager::Get().GetPlatformFile().FileExists(*SaveDirectory)) {

			return false;
		}
	}

	FString FinalString = "";
	for (FString& Each : SaveText) {
		FinalString += Each;
		FinalString += LINE_TERMINATOR;
	}

	return FFileHelper::SaveStringToFile(FinalString, *SaveDirectory);
}
