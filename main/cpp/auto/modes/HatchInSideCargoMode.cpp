/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/modes/HatchInSideCargoMode.h"

HatchInSideCargoMode::HatchInSideCargoMode(RobotModel *robot, ControlBoard *controlBoard) : AutoMode(robot, controlBoard){
    robot_ = robot;
}

void HatchInSideCargoMode::CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {
    
}

void HatchInSideCargoMode::Init() {
    printf("Hatch In Side Cargo Mode Init\n");
    currentCommand_->Init();
}

HatchInSideCargoMode::~HatchInSideCargoMode() {

}