/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/IntakeHatchBeakCommand.h"

IntakeHatchBeakCommand::IntakeHatchBeakCommand(RobotModel *robot, bool hatchIn) : AutoCommand() {
    robot_ = robot;
    hatchIn_ = hatchIn;
    isDone_ = false;
}

void IntakeHatchBeakCommand::Init() {}
void IntakeHatchBeakCommand::Update(double currTimeSec, double deltaTimeSec) {
    if (hatchIn_) {
        robot_->SetHatchBeak(true);
    } else {
        robot_->SetHatchBeak(false);
    }
    isDone_ = true;
}

bool IntakeHatchBeakCommand::IsDone() {
    return isDone_;
}

void IntakeHatchBeakCommand::Reset() {}

IntakeHatchBeakCommand::~IntakeHatchBeakCommand() {
}