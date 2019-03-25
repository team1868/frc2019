/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/OuttakeHatchCommand.h"

OuttakeHatchCommand::OuttakeHatchCommand(RobotModel *robot, bool hatchOut) : AutoCommand() {
    robot_ = robot;
    hatchOut_ = hatchOut;
    isDone_ = false;
}

void OuttakeHatchCommand::Init() {}

void OuttakeHatchCommand::Update(double currTimeSec, double deltaTimeSec) {
    if (hatchOut_) {
        robot_->SetHatchOuttake(true);
    } else {
        robot_->SetHatchOuttake(false);
    }
    isDone_ = true;
}

bool OuttakeHatchCommand::IsDone() {
    return isDone_;
}

void OuttakeHatchCommand::Reset() {}

OuttakeHatchCommand::~OuttakeHatchCommand() {
}