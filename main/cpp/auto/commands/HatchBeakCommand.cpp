/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/HatchBeakCommand.h"

HatchBeakCommand::HatchBeakCommand(RobotModel *robot, bool hatchOpen) : AutoCommand() {
    printf("intaking hatch from hp station \n");
    isDone_ = false;
    hatchOpen_ = hatchOpen;
    robot_ = robot;
}

void HatchBeakCommand::Init() {
    printf("intake hatch command init\n");
    isDone_ = false;
}

void HatchBeakCommand::Reset() {
    isDone_ = true;
}

void HatchBeakCommand::Update(double currTimeSec, double deltaTimeSec) {
    robot_->SetHatchBeak(hatchOpen_);

    isDone_ = true;
}

bool HatchBeakCommand::IsDone() {
    return isDone_;
}

HatchBeakCommand::~HatchBeakCommand() {
    
}
