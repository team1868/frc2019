/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
 *  OuttakeCargoToRocketCommand
 *  Assumes hood is down. Runs flywheel. Rolls polycord up.
*/

#include "../../../include/auto/commands/OuttakeCargoToRocketCommand.h"

OuttakeCargoToRocketCommand::OuttakeCargoToRocketCommand(RobotModel *robot) : AutoCommand() {
    robot_ = robot;
    isDone_ = false;
	outtakeCargoToRocketMotorOutput_ = 0.5; //TODO TEST
    cargoIntakeMotorOutput_ = 1.0;
	startTime_ = 0.0;
    deltaFlywheelStartTime_ = 1.0;  //TODO TEST
	deltaTime_ = 2.5; //TODO TEST
}

void OuttakeCargoToRocketCommand::Init() {
	isDone_ = false;
	startTime_ = robot_->GetTime();
}

void OuttakeCargoToRocketCommand::Reset() {
	robot_->SetCargoFlywheelOutput(0.0);
    robot_->SetCargoIntakeOutput(0.0);
	isDone_ = true;
}

void OuttakeCargoToRocketCommand::Update(double currTimeSec, double deltaTimeSec) {
	double diffTime = robot_->GetTime() - startTime_;

	if (diffTime > deltaTime_){
		robot_->SetCargoFlywheelOutput(0.0);
        robot_->SetCargoIntakeOutput(0.0);
		isDone_ = true;
        printf("outtake cargo to rocket done from timeout \n");
	} else if (diffTime > deltaFlywheelStartTime_ && diffTime < deltaTime_) {
        robot_->SetCargoFlywheelOutput(outtakeCargoToRocketMotorOutput_);
        printf("starting up flywheel first...\n");      // MANY TODOS FLYWHEEL PID FOR GOOD SPEED??!?!??
    } else {
        robot_->SetCargoIntakeOutput(cargoIntakeMotorOutput_);
        robot_->SetCargoFlywheelOutput(outtakeCargoToRocketMotorOutput_);
	}
}

bool OuttakeCargoToRocketCommand::IsDone() {
	return isDone_;
}

OuttakeCargoToRocketCommand::~OuttakeCargoToRocketCommand() {
	// TODO Auto-generated destructor stub
	Reset();
}
