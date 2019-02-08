/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "PivotCommand.h"
#include <frc/WPILib.h>

// constructor
PivotCommand::PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsoluteAngle, NavXPIDSource* navXSource) {
	navXSource_ = navXSource;

	initYaw_ = navXSource_->PIDGet();

	// adjust angle is absolute
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

	// initialize variables
	isDone_ = false;
	robot_ = robot;
	
	// initialize PID talon output
	talonOutput_ = new PivotPIDTalonOutput(robot_);

	// initialize time variables
	pivotCommandStartTime_ = robot_->GetTime();
	pivotTimeoutSec_ = 0.0;

	// retrieve pid values from user
	pFac_ = robot_->GetPivotPFac();
	iFac_ = robot_->GetPivotIFac();
	dFac_ = robot_->GetPivotDFac();

//	actualTimeoutSec_ = fabs(desiredAngle) * pivotTimeoutSec_ / 90.0;
	pivotPID_ = new PIDController(pFac_, iFac_, dFac_, navXSource_, talonOutput_);

	maxOutput_ = 0.9;
	tolerance_ = 3.0;

	numTimesOnTarget_ = 0;

	//ERROR (possibly) WARNING NOTE: this is adding every pivot
	//frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Pivot Error", pivotPID_->GetError());

	leftDriveNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Pivot Left Drive", 0.0).GetEntry();
	rightDriveNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Pivot Right Drive", 0.0).GetEntry();
	pivotErrorNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Pivot Error", 0.0).GetEntry();
	//NOTE: not printing time difference

}

void PivotCommand::Init() {
	//Profiler profiler(robot_, "Pivot Init");
	// Setting PID values (in case they changed)
	//TODO INI GetIniValues();
	pivotPID_->SetPID(pFac_, iFac_, dFac_);

	// initliaze NavX angle
	initYaw_ = navXSource_->PIDGet();

	// set settings for PID
	pivotPID_->SetSetpoint(desiredAngle_);
	pivotPID_->SetContinuous(true);
	pivotPID_->SetInputRange(-180, 180);
	pivotPID_->SetOutputRange(-maxOutput_, maxOutput_);     //adjust for 2018
	pivotPID_->SetAbsoluteTolerance(tolerance_);	 //adjust for 2018
	pivotPID_->Enable();

	// target variables
	isDone_ = false;
	numTimesOnTarget_ = 0;
	pivotCommandStartTime_ = robot_->GetTime();
	actualTimeoutSec_ = fabs(pivotPID_->GetError()) * pivotTimeoutSec_ / 90.0;

	printf("Initial NavX Angle: %f\n"
			"Desired NavX Angle: %f\n"
			"Chicken tenders pivot time starts at %f\n",
			initYaw_, desiredAngle_, pivotCommandStartTime_);
}

// theoretical change class back to orginal state
void PivotCommand::Reset() {
	// turn off motors
	robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
	robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);

	// reset shuffleboard values
	leftDriveNet_.SetDouble(0.0);
	rightDriveNet_.SetDouble(0.0);

	// disable PID
	if (pivotPID_ != NULL) {
		pivotPID_->Disable();
		delete(pivotPID_);
		pivotPID_ = NULL;
		printf("Disabling pivotcommand %f \n", robot_->GetNavXYaw());

	}
	isDone_ = true;

	printf("DONE FROM RESET \n");
}

// update time variables
void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {
	printf("Updating pivotcommand \n");

	// calculate time difference
	double timeDiff = robot_->GetTime() - pivotCommandStartTime_;
	bool timeOut = (timeDiff > pivotTimeoutSec_);								//test this value

	// on target
	if (pivotPID_->OnTarget()) {
		numTimesOnTarget_++;
	} else {
		numTimesOnTarget_ = 0;
	}
	if ((pivotPID_->OnTarget() && numTimesOnTarget_ > 1){// TODO TEMP CHANGE, put timeout back in || timeOut) { // done
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
	} else { // not done
		
		double output = talonOutput_->GetOutput();
//		double output = 0.0;
		// adjust motor values according to PID
		robot_->SetDriveValues(RobotModel::kLeftWheels, -output); //left inverted, right back and left foward if output positive
		robot_->SetDriveValues(RobotModel::kRightWheels, -output);

		// update shuffleboard
		rightDriveNet_.SetDouble(-output);
		leftDriveNet_.SetDouble(output);
		pivotErrorNet_.SetDouble(pivotPID_->GetError());

		printf("output is %f\n", output);
	}
}

// done
bool PivotCommand::IsDone() {
	return isDone_;
}

// deinitialize
PivotCommand::~PivotCommand() {
	Reset();
//	printf("IS DONE FROM DECONSTRUCTOR\n");
}