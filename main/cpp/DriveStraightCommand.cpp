/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DriveStraightCommand.h"
#include <frc/WPILib.h>

DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance) : AutoCommand(){
	isAbsoluteAngle_ = false;

	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);
}

DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance, double absoluteAngle) {
	isAbsoluteAngle_ = true;
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);
	desiredAngle_ = absoluteAngle;
}

void DriveStraightCommand::Init() {
	robot_->SetTalonBrakeMode();
	isDone_ = false;

	robot_->ResetDriveEncoders();  //TODO sketch!!! works but we need to fix

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;

	// Setting up PID vals
	anglePID_ = new PIDController(rPFac_, rIFac_, rDFac_, navXSource_, anglePIDOutput_);
	distancePID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderSource_, distancePIDOutput_);

	GetIniValues();

	if (!isAbsoluteAngle_) {
		desiredAngle_ = navXSource_->PIDGet();
	}
	initialAvgDistance_ = talonEncoderSource_->PIDGet();
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	anglePID_->SetPID(rPFac_, rIFac_, rDFac_);
	distancePID_->SetPID(dPFac_, dIFac_, dDFac_);

	anglePID_->SetSetpoint(desiredAngle_);
	distancePID_->SetSetpoint(desiredTotalAvgDistance_);

	anglePID_->SetContinuous(true);
	anglePID_->SetInputRange(-180, 180);
	distancePID_->SetContinuous(false);

	anglePID_->SetOutputRange(-rMaxOutput_, rMaxOutput_);
	distancePID_->SetOutputRange(-dMaxOutput_, dMaxOutput_);

	anglePID_->SetAbsoluteTolerance(rTolerance_);
	distancePID_->SetAbsoluteTolerance(dTolerance_);

	anglePID_->Enable();
	distancePID_->Enable();

	driveTimeoutSec_ = fabs(desiredDistance_ / 3.0); // Assuming 5.0 ft / sec from the low gear speed
	initialDriveTime_ = robot_->GetTime();
	printf("%f Start chicken tenders drivestraight time driveTimeoutSec is %f\n", initialDriveTime_, driveTimeoutSec_);

	numTimesOnTarget_ = 0;

	lastDistance_ = talonEncoderSource_->PIDGet();
	lastDOutput_ = 0.0;
	printf("Initial Right Distance: %f\n "
			"Initial Left Distance: %f\n"
			"Initial Average Distance: %f\n"
			"Desired Distance: %f\n"
			"Desired Angle: %f\n"
			"Initial getPID(): %f\n"
			"Initial angle: %f \n"
			"Distance error: %f\n"
			"Angle error: %f \n",
			robot_->GetRightEncoderValue(), robot_->GetLeftEncoderValue(),
			initialAvgDistance_, desiredTotalAvgDistance_, desiredAngle_,
			talonEncoderSource_->PIDGet(),  navXSource_->PIDGet(),
			distancePID_->GetError(), anglePID_->GetError());
}

void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) {
//	SmartDashboard::PutNumber("Left Motor Output", leftMotorOutput_);
//	SmartDashboard::PutNumber("Right Motor Output", rightMotorOutput_);
//	SmartDashboard::PutNumber("Angle Error", anglePID_->GetError());
//	SmartDashboard::PutNumber("Angle Error Graph", anglePID_->GetError());
//	SmartDashboard::PutNumber("DesiredAngle", desiredAngle_);
//	SmartDashboard::PutNumber("Encoder Error Feet", distancePID_->GetError());
//	SmartDashboard::PutNumber("Encoder Error Feet Graph", distancePID_->GetError());
//	SmartDashboard::PutNumber("Desired Total Feet", desiredTotalAvgDistance_);

	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;
//	SmartDashboard::PutNumber("DriveStraight Time:", diffDriveTime_);
	if (distancePID_->OnTarget() && fabs(talonEncoderSource_->PIDGet() - lastDistance_) < 0.04 ) {
		numTimesOnTarget_++;
		printf("%f Drivestraight error: %f\n", robot_->GetTime(), distancePID_->GetError());
	} else {
		numTimesOnTarget_ = 0;
	}

	if ((fabs(distancePID_->GetError()) < 1.0) && (robot_->CollisionDetected())) {
		numTimesStopped_++;
		printf("%f Collision Detected \n", robot_->GetTime());
	} else {
		numTimesStopped_ = 0;
	}

	lastDistance_ = talonEncoderSource_->PIDGet();
	if((numTimesOnTarget_ > 1) || (diffDriveTime_ > driveTimeoutSec_) || (numTimesStopped_ > 0)) { //LEAVING AS 10.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
		if (diffDriveTime_ > driveTimeoutSec_) { //LEAVING AS 10.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
			printf(" %f DRIVESTRAIGHT TIMED OUT!! :) go get chicken tenders %f\n", robot_->GetTime(), diffDriveTime_);
		}
		printf("%f Final Left Distance: %f\n"
				"Final Right Distance: %f\n"
				"Final Average Distance: %f\n"
				"Final Drivestraight error: %f\n",
				robot_->GetTime(), robot_->GetLeftEncoderValue(), robot_->GetRightEncoderValue(),
				talonEncoderSource_->PIDGet(), distancePID_->GetError());
		Reset();

		leftMotorOutput_ = 0.0;
		rightMotorOutput_ = 0.0;

		isDone_ = true;
	} else {
		double dOutput = distancePIDOutput_->GetPIDOutput();
		double rOutput = anglePIDOutput_->GetPIDOutput();

//		SmartDashboard::PutNumber("rOutput:", rOutput);
//		SmartDashboard::PutNumber("dOutput:", dOutput);
		if (dOutput - lastDOutput_ > 0.5) { // only when accelerating forward
			dOutput = lastDOutput_ + 0.5; //0.4 for KOP

		}
		rightMotorOutput_ = dOutput - rOutput;
		leftMotorOutput_ = dOutput + rOutput;
		lastDOutput_ = dOutput;

//		double maxOutput = fmax(fabs(rightMotorOutput_), fabs(leftMotorOutput_));
	}

	robot_->SetDriveValues(RobotModel::kLeftWheels, -leftMotorOutput_); //TODO: THIS IS FOR ARTEMIS< CHANGE INVERSION
	robot_->SetDriveValues(RobotModel::kRightWheels, rightMotorOutput_);
}

bool DriveStraightCommand::IsDone() {
	return isDone_;
}

void DriveStraightCommand::Reset() {
	robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
	robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);

	if (anglePID_ != NULL) {
		anglePID_->Disable();

		delete(anglePID_);

		anglePID_ = NULL;

		printf("Reset Angle PID %f \n", robot_->GetNavXYaw());
	}
	if (distancePID_ != NULL) {
		distancePID_->Disable();

		delete(distancePID_);

		distancePID_ = NULL;
//		printf("Reset Distance PID");

	}
	isDone_ = true;
}
 
void DriveStraightCommand::GetIniValues() { // Ini values are refreshed at the start of auto

	dPFac_ = 0.6;
	dIFac_ = 0.0;
	dDFac_ = 0.25;

	rPFac_ = 0.8;
	rIFac_ = 0.0;
	rDFac_ = 0.0;

	/* TODO INI
	dPFac_ = robot_->driveDPFac_;
	dIFac_ = robot_->driveDIFac_;
	dDFac_ = robot_->driveDDFac_;

	rPFac_ = robot_->driveRPFac_;
	rIFac_ = robot_->driveRIFac_;
	rDFac_ = robot_->driveRDFac_;
	*/

	//driveTimeoutSec_ = robot_->driveTimeoutSec_;

	printf("DRIVESTRAIGHT COMMAND DRIVE p: %f, i: %f, d: %f\n", dPFac_, dIFac_, dDFac_);
	printf("DRIVESTRAIGHT COMMAND ANGLE p: %f, i: %f, d: %f\n", rPFac_, rIFac_, rDFac_);
}

void DriveStraightCommand::Initializations(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
			AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
			double desiredDistance) {
	robot_ = robot;

	navXSource_ = navXSource;
	talonEncoderSource_ = talonEncoderSource;

	anglePIDOutput_ = anglePIDOutput;
	distancePIDOutput_ = distancePIDOutput;

	desiredAngle_ = navXSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();

	desiredDistance_ = desiredDistance;
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;
	isDone_ = false;
	initialDriveTime_ = robot_->GetTime();
	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;

	// Setting up the PID controllers to NULL
	GetIniValues();
	anglePID_ = NULL;
	distancePID_ = NULL;

	rTolerance_ = 0.5;
	dTolerance_ = 3.0 / 12.0;

	rMaxOutput_ = 0.15;
	dMaxOutput_ = 0.85; // 0.85

	// initializing number of times robot is on target
	numTimesOnTarget_ = 0;
	numTimesStopped_ = 0;

	lastDistance_ = talonEncoderSource_->PIDGet();
	lastDOutput_ = 0.0;
}

DriveStraightCommand::~DriveStraightCommand() {

}