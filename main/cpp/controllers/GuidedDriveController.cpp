/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../include/controllers/GuidedDriveController.h"
//#include <frc/WPILib.h>
// constructor
GuidedDriveController::GuidedDriveController(RobotModel *robot, ControlBoard *humanControl,
	NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) {

	robot_ = robot;
	humanControl_ = humanControl;

	//TODO sketch
	navXSource_ = new NavXPIDSource(robot_);
	talonEncoderSource_ = new TalonEncoderPIDSource(robot_);
	anglePIDOutput_ = anglePIDOutput;
	distancePIDOutput_ = distancePIDOutput;
	driveStraight_ = new DriveStraightCommand(navXSource_, talonEncoderSource_,
		anglePIDOutput_, distancePIDOutput_, robot_, 0.0, true);
	driveStraight_->Init(); //TODO BAD PRACTICE BAD BAD

	// driver preferences
    // Set sensitivity to 0
	thrustSensitivity_ = 0.0;
	rotateSensitivity_ = 0.0;
	quickTurnSensitivity_ = 0.0;

	LOW_THRUST_SENSITIVITY= 0.3; // TODO tune this
	LOW_ROTATE_SENSITIVITY = 0.7; // TODO tune this

	isDone_ = false;
	
	// motor outputs
	leftOutput = 0.0;
	rightOutput  = 0.0;

	// shuffleboard initializations
	//TODO coordinate with drivecontroller somehow MEMORY LEAK
	thrustZNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Thrust Z", 0.0).GetEntry();
	rotateZNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Rotate Z", 0.0).GetEntry();
	gearDesireNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("High Gear", humanControl_->GetHighGearDesired()).GetEntry();
	quickturnDesireNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Quick Turn", humanControl_->GetQuickTurnDesired()).GetEntry();
	arcadeDesireNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Arcade Drive", true).withWidget(BuiltInWidgets::kToggleSwitch).GetEntry();// humanControl_->GetArcadeDriveDesired()).GetEntry();
	reverseReverseNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Lili Mode", true).withWidget(BuiltInWidgets::kToggleSwitch).GetEntry();
	leftDriveNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Output (controller)", leftOutput).GetEntry();
	rightDriveNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Output (controller)", rightOutput).GetEntry();
	driveDirectionNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Drive Direction (controller)", GetDriveDirection()).GetEntry();
	navXAngleNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("NavX Angle", robot_->GetNavXYaw()).GetEntry();
	leftDistanceNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Drive Distance", robot_->GetLeftDistance()).GetEntry();
	rightDistanceNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Drive Distance", robot_->GetRightDistance()).GetEntry();
	leftEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Encoder", robot_->GetLeftEncoderValue()).GetEntry();
	rightEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Encoder", robot_->GetRightEncoderValue()).GetEntry();
	thrustDeadbandNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Thrust Deadband", 0.0).GetEntry();
	rotateDeadbandNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Rotate Deadband", 0.0).GetEntry();
}

void GuidedDriveController::Reset() {
//	robot_->SetPercentVBusDriveMode(); check robotmodel

//	thrustSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "thrustSensitivity", 0.3);
//	rotateSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "rotateSensitivity", 0.5);
//	quickTurnSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "quickTurnSensitivity", 0.5);
//	check ini
}

void GuidedDriveController::Disable(){
	driveStraight_->Disable();
}

void GuidedDriveController::Enable(){
	driveStraight_->Enable();
}

// update drive
void GuidedDriveController::Update(double currTimeSec, double deltaTimeSec) {
	PrintDriveValues();

	switch (robot_->GetGameMode()) {
		case (RobotModel::NORMAL_TELEOP):
			//		robot->SetPercentVBusDriveMode(); old talon stuff
			double leftJoyY, leftJoyZ, rightJoyY, rightJoyX, rightJoyZ;

			// Takes joystick values
			leftJoyY = -humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY);	// was neg
			leftJoyZ = humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kZ);
			rightJoyY = -humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kY);	// was neg
			rightJoyX = humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX);
			rightJoyZ = humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kZ);

			// so leftJoyZ and rightJoyZ are from -1 to 1
			thrustSensitivity_ = (leftJoyZ + 1.0) / 2.0;
			rotateSensitivity_ = (rightJoyZ + 1.0) / 2.0;

			// Change gear
			if (humanControl_->GetHighGearDesired()) { //TODO POSSIBLY REMOVE WHY IS THIS ALSO HERE
				robot_->SetHighGear();
			} else {
				robot_->SetLowGear();
			}
		
			// Checks quickturn or arcade drive
			if (humanControl_->GetQuickTurnDesired()) {
				QuickTurn(rightJoyX, 0.0);
			} else {
				//if (humanControl_->GetArcadeDriveDesired()) {
				if(arcadeDesireNet_.GetBoolean(true)){
					ArcadeDrive(rightJoyX, leftJoyY, thrustSensitivity_, rotateSensitivity_);
				} else {
					printf("Using Tank drive------------------------------------------------------------------------------------------------------- \n");
					TankDrive(leftJoyY, rightJoyY);
				}
			}
			break;
		case (RobotModel::SANDSTORM):
			printf("WARNING: GuidedDriveController initialized in sandstorm????");
		default:
			printf("ERROR: Game mode not reconized as either sandstorm or normal teleop.  In GuidedDriveController::Update()\n");
	}
}

// arcade drive
void GuidedDriveController::ArcadeDrive(double myX, double myY, double thrustSensitivity, double rotateSensitivity) {
	double thrustValue = myY;
	double rotateValue = myX;

	//TODO: add safety for deadband values (prevent (-) or >= 0.1)
	thrustValue = HandleDeadband(thrustValue, thrustDeadbandNet_.GetDouble(0.0));	// 0.02 was too low
	rotateValue = HandleDeadband(rotateValue, rotateDeadbandNet_.GetDouble(0.0));

	rotateValue = GetCubicAdjustment(rotateValue, rotateSensitivity);
	thrustValue = GetCubicAdjustment(thrustValue, thrustSensitivity);

	if(reverseReverseNet_.GetBoolean(true) || (!reverseReverseNet_.GetBoolean(true) && thrustValue >= 0.0)){// || reverseReverseNet_.GetBoolean(false) && thrustValue > 0.0){ //lili mode
		//leftOutput = thrustValue + rotateValue;			// CHECK FOR COMP BOT
		//rightOutput = thrustValue - rotateValue;
	} else {
		rotateValue = -rotateValue;
		//leftOutput = thrustValue - rotateValue;
		//rightOutput = thrustValue + rotateValue;
	}

	//printf("Left Output: %f and Right Output: %f", -leftOutput, rightOutput);
	driveStraight_->SetDesiredAngle(robot_->GetNavXYaw() + rotateValue); //TODO CHANGE
	driveStraight_->SetDesiredDistance( driveStraight_->GetPIDDistance() + thrustValue); //TODO CHANGE AND sketch
	driveStraight_->Update(0.0, 0.0); //time does not matter in teleop
}

// tank drive
void GuidedDriveController::TankDrive(double myLeft, double myRight) {
	leftOutput = myLeft * GetDriveDirection();
	rightOutput = myRight * GetDriveDirection();

	robot_->SetDriveValues(-leftOutput, rightOutput); //going to leave the same, so no possible bugs
}

void GuidedDriveController::QuickTurn(double myRight, double turnConstant) {

	double rotateValue = GetCubicAdjustment(myRight, turnConstant);

	robot_->SetDriveValues(rotateValue, -rotateValue); //TODO make guided? maybe idk necessary
}

int GuidedDriveController::GetDriveDirection() {
	if (humanControl_->GetReverseDriveDesired()) {
		return -1;
	} else {
		return 1;
	}
}

// depends on joystick sensitivity
double GuidedDriveController::HandleDeadband(double value, double deadband) {
	if (fabs(value) < deadband) {
		return 0.0;
	} else {
		return value;
	}
}

// Rotation sensitivity: when z == 0 same output; when z==1, output^3
double GuidedDriveController::GetCubicAdjustment(double value, double adjustmentConstant) {
	return adjustmentConstant * std::pow(value, 3.0) + (1 - adjustmentConstant) * value;
}

bool GuidedDriveController::IsDone() {
	return isDone_;
}

// update shuffleboard values
void GuidedDriveController::PrintDriveValues(){
	thrustZNet_.SetDouble(thrustSensitivity_);
	rotateZNet_.SetDouble(rotateSensitivity_);
	gearDesireNet_.SetBoolean(humanControl_->GetHighGearDesired());
	quickturnDesireNet_.SetBoolean(humanControl_->GetQuickTurnDesired());
	//arcadeDesireNet_.SetBoolean(humanControl_->GetArcadeDriveDesired());
	leftDriveNet_.SetDouble(leftOutput); //TODO MOVE INTO METHOD + line after
	rightDriveNet_.SetDouble(rightOutput);
	driveDirectionNet_.SetDouble(GetDriveDirection());
	navXAngleNet_.SetDouble(robot_->GetNavXYaw());
	leftDistanceNet_.SetDouble(robot_->GetLeftDistance());
	rightDistanceNet_.SetDouble(robot_->GetRightDistance());
	leftEncoderNet_.SetDouble(robot_->GetLeftEncoderValue());
	rightEncoderNet_.SetDouble(robot_->GetRightEncoderValue());
}

GuidedDriveController::~GuidedDriveController() {
	//TODO recheck all objects
	//shuffleboard remove
	driveStraight_->~DriveStraightCommand();
}
