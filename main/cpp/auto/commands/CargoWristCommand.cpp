/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/CargoWristCommand.h"
#include <math.h>
#include <cmath>

CargoWristCommand::CargoWristCommand(RobotModel *robot) {
    robot_ = robot;
}

void CargoWristCommand::Init(){
    isDone_ = false;
}

void CargoWristCommand::Reset(){
    robot_->SetCargoIntakeOutput(0.0);
    robot_->SetCargoIntakeWrist(false);
    isDone_ = true;
}

void CargoWristCommand::Update(double currTimeSec, double deltaTimeSec){
    if(deltaTimeSec >= 0.7){
        robot_->SetCargoIntakeWrist(false);
        robot_->SetCargoIntakeOutput(0.0);
        isDone_ = true;
        printf("cargo intake wrist command done by timeout");
    } else {
        robot_->SetCargoIntakeWrist(true);
        robot_->SetCargoIntakeOutput(0.7);
    }
}

bool CargoWristCommand::IsDone(){
    return isDone_;
}

CargoWristCommand::~CargoWristCommand(){}