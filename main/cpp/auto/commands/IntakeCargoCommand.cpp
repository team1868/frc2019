/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
 *  IntakeCargoCommand
 *  Runs polycord inwards, stops when photoelectric sensor sees it.
*/

#include "../../../include/auto/commands/IntakeCargoCommand.h"

IntakeCargoCommand::IntakeCargoCommand(RobotModel *robot, double cargoIntakeMotorOutput) : AutoCommand() {
    printf("Creating cargo intake command \n");
    isDone_ = false;
	robot_ = robot;
	cargoIntakeMotorOutput_ = cargoIntakeMotorOutput;

	startTime_ = robot_->GetTime();
	timeDiff_ = robot_->GetTime() - startTime_;
	wasJustRunning_ = true;
}

void IntakeCargoCommand::Init() {
    printf("In IntakeCargoCommand Init\n");
    startTime_ = robot_->GetTime();
	timeDiff_ = robot_->GetTime() - startTime_;
	wasJustRunning_ = true;

	isDone_ = false;
//	printf("%f Starting chicken tenders timeout with intake\n", startTime_);
}

void IntakeCargoCommand::Reset() {
	isDone_ = true;
}

void IntakeCargoCommand::Update(double currTimeSec, double deltaTimeSec) {
    timeDiff_ = robot_->GetTime() - startTime_;
    // if (robot_->GetCargoInIntake()) {        // TODO PUT THE LIGHT SENSOR IN AND IMPLEMENT THIS IN ROBOT MODEL
    //     robot_->SetCargoIntakeOutput(0.0);
    //     isDone_ = true;
    //     printf("Cargo is in the intake\n");
    // } else...
    if (timeDiff_ <= 2.2) {     // TODO TODO TUNE ALL THESE H*CKIN NUMBERSA;LSKDJF
        robot_->SetCargoIntakeOutput(cargoIntakeMotorOutput_);
    } else {
        robot_->SetCargoIntakeOutput(0.0);
		isDone_ = true;
		printf("Intake Done from TIMEOUT! with time diff %f\n", timeDiff_);
    }
}

bool IntakeCargoCommand::IsDone() {
	return isDone_;
}

IntakeCargoCommand::~IntakeCargoCommand() {
	// TODO Auto-generated destructor stub
	Reset();
}