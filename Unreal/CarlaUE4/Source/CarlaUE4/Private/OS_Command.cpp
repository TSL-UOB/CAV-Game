// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.


#include "OS_Command.h"

void UOS_Command::ExecuteCommand() {

	 FPlatformProcess::CreateProc(TEXT("D:\\University\\3rdYear\\Simulator\\CAV\\CAV-Game\\scenario_runner_0.9.13\\srunner\\examples\\Scenario2.xosc"), TEXT("py D:\\University\\3rdYear\\Simulator\\CAV\\CAV-Game\\scenario_runner_0.9.13\\scenario_runner.py --openscenario"), true, false, false, nullptr, 0, nullptr, nullptr);
}