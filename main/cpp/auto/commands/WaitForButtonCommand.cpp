/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/WaitForButtonCommand.h"

WaitForButtonCommand::WaitForButtonCommand(ControlBoard *controlBoard) : AutoCommand() {
	isDone_ = false;
    humanControl_ = controlBoard;
}

void WaitForButtonCommand::Init() {}

void WaitForButtonCommand::Update(double currTimeSec, double deltaTimeSec) {
	isDone_ = humanControl_->GetQuickTurnDesired();
    if(isDone_){
        printf("Wait for button command done.\n");
    }
}

bool WaitForButtonCommand::IsDone() {
	return isDone_;
}

void WaitForButtonCommand::Reset() {
}

WaitForButtonCommand::~WaitForButtonCommand() {
}
