/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "PivotCommand.h"
#include <frc/WPILib.h>

PivotCommand::PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsoluteAngle, NavXPIDSource* navXSource) {
	navXSource_ = navXSource;

	initYaw_ = navXSource_->PIDGet();

	if (isAbsoluteAngle){
		desiredAngle_ = desiredAngle;
	} else {
		desiredAngle_ = initYaw_ + desiredAngle;
		if (desiredAngle_ > 180) {
			desiredAngle_ -= -360;
		} else if (desiredAngle_ < -180) {
			desiredAngle_ += 360;
		}
	}

	isDone_ = false;
	robot_ = robot;
	talonOutput_ = new PivotPIDTalonOutput(robot_);

	pivotCommandStartTime_ = robot_->GetTime();
	pivotTimeoutSec_ = 0.0;
	//TODO INI GetIniValues();


	//TODO delte @lili
	pFac_ = 0.7;
	iFac_ = 0.0;
	dFac_ = 0.2;

//	actualTimeoutSec_ = fabs(desiredAngle) * pivotTimeoutSec_ / 90.0;
	pivotPID_ = new PIDController(pFac_, iFac_, dFac_, navXSource_, talonOutput_);

	maxOutput_ = 0.9;
	tolerance_ = 3.0;

	numTimesOnTarget_ = 0;

	//ERROR (possibly) WARNING NOTE: this is adding every pivot
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("left output", output);
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("right output", -output);
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Pivot Error", pivotPID_->GetError());
	//NOTE: not printing time difference

}

void PivotCommand::Init() {
	//Profiler profiler(robot_, "Pivot Init");
	// Setting PID values (in case they changed)
	//TODO INI GetIniValues();
	pivotPID_->SetPID(pFac_, iFac_, dFac_);

	initYaw_ = navXSource_->PIDGet();

	pivotPID_->SetSetpoint(desiredAngle_);
	pivotPID_->SetContinuous(true);
	pivotPID_->SetInputRange(-180, 180);
	pivotPID_->SetOutputRange(-maxOutput_, maxOutput_);     //adjust for 2018
	pivotPID_->SetAbsoluteTolerance(tolerance_);	 //adjust for 2018
	pivotPID_->Enable();


	isDone_ = false;
	numTimesOnTarget_ = 0;
	pivotCommandStartTime_ = robot_->GetTime();
	actualTimeoutSec_ = fabs(pivotPID_->GetError()) * pivotTimeoutSec_ / 90.0;

	printf("Initial NavX Angle: %f\n"
			"Desired NavX Angle: %f\n"
			"Chicken tenders pivot time starts at %f\n",
			initYaw_, desiredAngle_, pivotCommandStartTime_);
}

void PivotCommand::Reset() {
	robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
	robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);

	if (pivotPID_ != NULL) {
		pivotPID_->Disable();
		delete(pivotPID_);
		pivotPID_ = NULL;
		printf("Disabling pivotcommand %f \n", robot_->GetNavXYaw());

	}
	isDone_ = true;

	printf("DONE FROM RESET \n");
}

void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {
	printf("Updating pivotcommand \n");


	double timeDiff = robot_->GetTime() - pivotCommandStartTime_;
	bool timeOut = (timeDiff > pivotTimeoutSec_);								//test this value

	if (pivotPID_->OnTarget()) {
		numTimesOnTarget_++;
	} else {
		numTimesOnTarget_ = 0;
	}
	if ((pivotPID_->OnTarget() && numTimesOnTarget_ > 1) || timeOut) {
		printf("%f Final NavX Angle from PID Source: %f\n"
				"Final NavX Angle from robot: %f \n"
				"%f Angle NavX Error %f\n",
				robot_->GetTime(), navXSource_->PIDGet(), robot_->GetNavXYaw(), robot_->GetTime(), pivotPID_->GetError());
		Reset();
		isDone_ = true;
		robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
		robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);
		printf("%f PIVOT IS DONE \n", robot_->GetTime());
		if (timeOut) {
			printf("%f FROM PIVOT TIME OUT GO GET CHICKEN TENDERS @ %f\n", robot_->GetTime(), timeDiff);
		}
	} else {
		
		output = talonOutput_->GetOutput();
//		double output = 0.0;
		robot_->SetDriveValues(RobotModel::kLeftWheels, -output); //left inverted, right back and left foward if output positive
		robot_->SetDriveValues(RobotModel::kRightWheels, -output);
		printf("output is %d\n", output);
	}
}

bool PivotCommand::IsDone() {
	return isDone_;
}

/* TODO INI
void PivotCommand::GetIniValues() {

	pFac_ = robot_->pivotPFac_;
	iFac_ = robot_->pivotIFac_;
	dFac_ = robot_->pivotDFac_;
	pivotTimeoutSec_ = robot_->pivotTimeoutSec_;
//	minDrivePivotOutput_ = robot_->pini_->getf("PIVOT PID", "minDrivePivotOutput", 0.0);
	printf("PIVOT COMMAND p: %f, i: %f, d: %f\n", pFac_, iFac_, dFac_);
}
*/


PivotCommand::~PivotCommand() {
	Reset();
//	printf("IS DONE FROM DECONSTRUCTOR\n");
}