/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/DriveStraightCommand.h"
#include <frc/WPILib.h>

// constructing
DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance) : AutoCommand(){
	isTeleop_ = false;
	isAbsoluteAngle_ = false;

	// initialize dependencies
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);

	//NOTE: adding repetative title, this may be an issue later

	//TODO: this may seem as though error, bc all values start at 0 and shouldn't
	leftStraightNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Output (drivestraight)", 0.0).GetEntry();
	rightStraightNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Output (drivestraight)", 0.0).GetEntry();
	angleErrorNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Angle Error (drivestraight)", 0.0).GetEntry();
	angleErrorGraphNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Angle Error Graph (drivestraight)", 0.0).GetEntry();
	desiredAngleNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Desired Angle (drivestraight)", 0.0).GetEntry();
	encoderErrorNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Encoder Error (drivestraight)", 0.0).GetEntry();
	encoderErrorGraphNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Encoder Error Graph (drivestraight)", 0.0).GetEntry();
	desiredTotalFeetNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Desired Total Feet", 0.0).GetEntry();
	dPIDOutputNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Distance PID Output", 0.0).GetEntry();
	aPIDOutputNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Angle PID Output", 0.0).GetEntry();
	
}

// constructing
DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance, bool isTeleop) : AutoCommand(){
	isTeleop_ = isTeleop;
	isAbsoluteAngle_ = false;

	// initialize dependencies
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);

	//NOTE: adding repetative title, this may be an issue later

	//TODO: this may seem as though error, bc all values start at 0 and shouldn't
	leftStraightNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Output (drivestraight)", 0.0).GetEntry();
	rightStraightNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Output (drivestraight)", 0.0).GetEntry();
	angleErrorNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Angle Error (drivestraight)", 0.0).GetEntry();
	angleErrorGraphNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Angle Error Graph (drivestraight)", 0.0).GetEntry();
	desiredAngleNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Desired Angle (drivestraight)", 0.0).GetEntry();
	encoderErrorNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Encoder Error (drivestraight)", 0.0).GetEntry();
	encoderErrorGraphNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Encoder Error Graph (drivestraight)", 0.0).GetEntry();
	desiredTotalFeetNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Desired Total Feet", 0.0).GetEntry();
	dPIDOutputNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Distance PID Output", 0.0).GetEntry();
	aPIDOutputNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Angle PID Output", 0.0).GetEntry();
	
}

// constructor
DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance, double absoluteAngle) {
	isTeleop_ = false;
	isAbsoluteAngle_ = true;
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);
	desiredAngle_ = absoluteAngle;

	//NOTE: adding repetative title, this may be an issue later
}

// initialize class for run
void DriveStraightCommand::Init() {
	printf("IN DRIVESTRAIGHT INIT\n");
	robot_->SetTalonBrakeMode();
	isDone_ = false;

	robot_->ResetDriveEncoders();  //TODO sketch!!! works but we need to fix

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;

	// Setting up PID vals
	anglePID_ = new PIDController(rPFac_, rIFac_, rDFac_, navXSource_, anglePIDOutput_);
	distancePID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderSource_, distancePIDOutput_);

	GetIniValues();
	// absolute angle
	if (!isAbsoluteAngle_) {
		desiredAngle_ = navXSource_->PIDGet();
	}

	// initialize dependencies settings
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

	 // Assuming 5.0 ft / sec from the low gear speed
	driveTimeoutSec_ = fabs(desiredDistance_ / 3.0)+2; //TODO: add physics, also TODO remove +5
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

// update current values
void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) { 
	// update shuffleboard values
	leftStraightNet_.SetDouble(leftMotorOutput_);
	rightStraightNet_.SetDouble(rightMotorOutput_);
	angleErrorNet_.SetDouble(anglePID_->GetError());
	angleErrorGraphNet_.SetDouble(anglePID_->GetError());
	desiredAngleNet_.SetDouble(desiredAngle_);
	encoderErrorNet_.SetDouble(distancePID_->GetError());
	encoderErrorGraphNet_.SetDouble(distancePID_->GetError());
	desiredTotalFeetNet_.SetDouble(desiredTotalAvgDistance_);

	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;

// on target
	if (distancePID_->OnTarget() && fabs(talonEncoderSource_->PIDGet() - lastDistance_) < 0.04 ) {
		numTimesOnTarget_++;
		printf("%f Drivestraight error: %f\n", robot_->GetTime(), distancePID_->GetError());
	} else {
		numTimesOnTarget_ = 0;
	}

    //  error check
	if ((fabs(distancePID_->GetError()) < 1.0) && (robot_->CollisionDetected())) {
		numTimesStopped_++;
		printf("%f Collision Detected \n", robot_->GetTime());
	} else {
		numTimesStopped_ = 0;
	}

	lastDistance_ = talonEncoderSource_->PIDGet();
	if(!isTeleop_ && ((numTimesOnTarget_ > 1) || (diffDriveTime_ > driveTimeoutSec_) || (numTimesStopped_ > 0))) { //LEAVING AS 10.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
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
	} else { // else run motor
		// receive PID outputs
		double dOutput = distancePIDOutput_->GetPIDOutput();
		double rOutput = anglePIDOutput_->GetPIDOutput();

		// shuffleboard update
		aPIDOutputNet_.SetDouble(rOutput);
		dPIDOutputNet_.SetDouble(dOutput);

		if (dOutput - lastDOutput_ > 0.5) { // only when accelerating forward
			dOutput = lastDOutput_ + 0.5; //0.4 for KOP

		}
		// set drive outputs
		rightMotorOutput_ = dOutput - rOutput; //sketch, check!
		leftMotorOutput_ = dOutput + rOutput; //TODO sketch check! TODODODODODO SKETCH MAKE SURE NOT OVER 1.0
		lastDOutput_ = dOutput;

//		double maxOutput = fmax(fabs(rightMotorOutput_), fabs(leftMotorOutput_));
	}
	// drive motors
	robot_->SetDriveValues(-leftMotorOutput_, rightMotorOutput_);
	//printf("times on target at %f \n\n", numTimesOnTarget_);
}

// repeatedly on target
bool DriveStraightCommand::IsDone() {
	return isDone_;
}

// reset robot to standby
void DriveStraightCommand::Reset() {
	// turn off mototrs
	robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);

	// destroy angle PID
	if (anglePID_ != NULL) {
		anglePID_->Disable();

		delete(anglePID_);

		anglePID_ = NULL;

		printf("Reset Angle PID %f \n", robot_->GetNavXYaw());
	}

	// destroy distance PID
	if (distancePID_ != NULL) {
		distancePID_->Disable();

		delete(distancePID_);

		distancePID_ = NULL;
//		printf("Reset Distance PID");

	}
	isDone_ = true;
}
 
//Get pid values from shuffleboard
void DriveStraightCommand::GetIniValues() { // Ini values are refreshed at the start of auto

	dPFac_ = robot_->GetDPFac();
	dIFac_ = robot_->GetDIFac();
	dDFac_ = robot_->GetDDFac();

	rPFac_ = robot_->GetRPFac();
	rIFac_ = robot_->GetRIFac();
	rDFac_ = robot_->GetRDFac();

	//driveTimeoutSec_ = robot_->driveTimeoutSec_;

	printf("DRIVESTRAIGHT COMMAND DRIVE p: %f, i: %f, d: %f\n", dPFac_, dIFac_, dDFac_);
	printf("DRIVESTRAIGHT COMMAND ANGLE p: %f, i: %f, d: %f\n", rPFac_, rIFac_, rDFac_);
}

// initialize dependencies
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
	desiredTotalAvgDistance_ = 2.0; //TODO CHANGE //initialAvgDistance_ + desiredDistance_;
	printf("Total desired distance is: %f", desiredTotalAvgDistance_);

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
	dMaxOutput_ = 0.85;

	// initializing number of times robot is on target
	numTimesOnTarget_ = 0;
	numTimesStopped_ = 0;

	lastDistance_ = talonEncoderSource_->PIDGet();
	lastDOutput_ = 0.0;
}

void DriveStraightCommand::SetDesiredAngle(double absAngle){
	anglePID_->SetSetpoint(absAngle);
}

void DriveStraightCommand::SetDesiredDistance(double distance){
	distancePID_->SetSetpoint(distance);
}

void DriveStraightCommand::Disable(){
	distancePID_->Disable();
	anglePID_->Disable();
}

void DriveStraightCommand::Enable(){
	distancePID_->Enable();
	anglePID_->Enable();
}

DriveStraightCommand::~DriveStraightCommand() {
	//leftStraightNet_->Remove();
	anglePID_->Disable();
	distancePID_->Disable();
	anglePID_->~PIDController();
	distancePID_->~PIDController();

	leftStraightNet_.Delete();
	rightStraightNet_.Delete();
	angleErrorNet_.Delete();
	angleErrorGraphNet_.Delete();
	desiredAngleNet_.Delete();
	encoderErrorNet_.Delete();
	encoderErrorGraphNet_.Delete();
	desiredTotalFeetNet_.Delete();
	dPIDOutputNet_.Delete();
	aPIDOutputNet_.Delete();
}