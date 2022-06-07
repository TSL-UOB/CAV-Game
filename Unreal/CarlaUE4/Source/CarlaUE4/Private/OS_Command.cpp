// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.


#include "OS_Command.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformFilemanager.h"

void UOS_Command::ExecuteCustomOS_TerminalCommand(FString PathToPythonScript) {

	// Set complete path
	// PathToPythonScript += "\\";
	// PathToPythonScript += FileName;

	// if (AllowOverwriting) {
	// 	if (FPlatformFileManager::Get().GetPlatformFile().FileExists(*PathToPythonScript)) {

	// 		return false;
	// 	}
	// }

	// FString FinalString = "";
	// for (FString& Each : SaveText) {
	// 	FinalString += Each;
	// 	FinalString += LINE_TERMINATOR;
	// }
	// FString StringExample = TEXT("String example");

	UE_LOG(LogTemp, Warning, TEXT("Output: %s"), *PathToPythonScript);
	// FMonitoredProcess myProc = FMonitoredProcess(TEXT("python3 ~/CAV-Game/scenario_runner_0.9.13/srunner/examples/hello.py"), "", false, true);
	FMonitoredProcess myProc = FMonitoredProcess(*PathToPythonScript, "", false, true);
	myProc.Launch();
	myProc.GetDuration();
	UE_LOG(LogTemp, Warning, TEXT("myProc.GetDuration(): %s"), ( myProc.Launch() ? TEXT("true") : TEXT("false") ));

	// UE_LOG(LogTemp, Warning, TEXT("myProc.GetDuration(): %f"), myProc.GetDuration());

	// const TCHAR* ExePath = L"open -e ~/CAV-Game/GAME_TODO.txt";
	// const TCHAR* Parms = L"";
	// FPlatformProcess::CreateProc(TEXT("nano ~/CAV-Game/GAME_TODO.txt"), nullptr, true, false, false, nullptr, 0, nullptr, nullptr);
	// FPlatformProcess::CreateProc(ExePath, Parms, true, false, false, nullptr, -1, nullptr, nullptr);
	// FPlatformProcess::CreateProc(TEXT("python3 ~/CAV-Game/scenario_runner_0.9.13/srunner/examples/hello.py"), nullptr, true, false, false, nullptr, 0, nullptr, nullptr);
	// FPlatformProcess::CreateProc(*PathToPythonScript, *PathToPythonScript, true, false, false, nullptr, 0, nullptr, nullptr);
	
	// FPlatformProcess::CreateProc(nullptr, nullptr, true, false, false, nullptr, 0, nullptr, nullptr);

	// std::cout << "PathToPythonScript = " << PathToPythonScript;
	return;
}

//FFileHelper::SaveStringToFile(PathToPythonScript, *PathToPythonScript);

// void UOS_Command::ExecuteCommand() {

// 	 FPlatformProcess::CreateProc(TEXT("D:\\University\\3rdYear\\Simulator\\CAV\\CAV-Game\\scenario_runner_0.9.13\\srunner\\examples\\Scenario2.xosc"), TEXT("py D:\\University\\3rdYear\\Simulator\\CAV\\CAV-Game\\scenario_runner_0.9.13\\scenario_runner.py --openscenario"), true, false, false, nullptr, 0, nullptr, nullptr);
// }

// ~/CAV-Game/scenario_runner_0.9.13/srunner/examples/hello.py