/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DriveController.h"
//#include <frc/WPILib.h>

// constructor
DriveController::DriveController(RobotModel *robot, ControlBoard *humanControl) {

	robot_ = robot;
	humanControl_ = humanControl;

	navXSource_ = new NavXPIDSource(robot_);
	talonEncoderSource_ = new TalonEncoderPIDSource(robot_);

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
	thrustZNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Thrust Z", 0.0).GetEntry();
	rotateZNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Rotate Z", 0.0).GetEntry();
	gearDesireNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("High Gear", humanControl_->GetHighGearDesired()).GetEntry();
	quickturnDesireNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Quick Turn", humanControl_->GetQuickTurnDesired()).GetEntry();
	arcadeDesireNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Arcade Drive", humanControl_->GetArcadeDriveDesired()).GetEntry();
	leftDriveNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Output (controller)", leftOutput).GetEntry();
	rightDriveNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Output (controller)", rightOutput).GetEntry();
	driveDirectionNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Drive Direction (controller)", GetDriveDirection()).GetEntry();
	navXAngleNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("NavX Angle", robot_->GetNavXYaw()).GetEntry();
	leftDistanceNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Drive Distance", robot_->GetLeftDistance()).GetEntry();
	rightDistanceNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Drive Distance", robot_->GetRightDistance()).GetEntry();
	leftEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Encoder", robot_->GetLeftEncoderValue()).GetEntry();
	rightEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Encoder", robot_->GetRightEncoderValue()).GetEntry();
}

void DriveController::Reset() {
//	robot_->SetPercentVBusDriveMode(); check robotmodel

//	thrustSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "thrustSensitivity", 0.3);
//	rotateSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "rotateSensitivity", 0.5);
//	quickTurnSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "quickTurnSensitivity", 0.5);
//	check ini
}

// update drive
void DriveController::Update(double currTimeSec, double deltaTimeSec) {
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
			if (humanControl_->GetHighGearDesired()) {
				robot_->SetHighGear();
			} else {
				robot_->SetLowGear();
			}
		
			// Checks quickturn or arcade drive
			if (humanControl_->GetQuickTurnDesired()) {
				QuickTurn(rightJoyX, 0.0);
			} else {
				if (humanControl_->GetArcadeDriveDesired()) {
					ArcadeDrive(rightJoyX, leftJoyY, thrustSensitivity_, rotateSensitivity_);
				} else {
					printf("Using Tank drive------------------------------------------------------------------------------------------------------- \n");
					TankDrive(leftJoyY, rightJoyY);
				}
			}
			break;
		case (RobotModel::SANDSTORM):
			printf("WARNING: DriveController initialized in sandstorm????");
		default:
			printf("ERROR: Game mode not reconized as either sandstorm or normal teleop.  In DriveController::Update()\n");
	}
}

// arcade drive
void DriveController::ArcadeDrive(double myX, double myY, double thrustSensitivity, double rotateSensitivity) {

	double thrustValue = myY * GetDriveDirection();
	double rotateValue = myX;
//	double rotateValue = myX * GetDriveDirection(); // TODO fix if you want chloemode
	leftOutput = 0.0;
	rightOutput = 0.0;

	// Account for small joystick jostles (deadband)
	thrustValue = HandleDeadband(thrustValue, 0.1);	// 0.02 was too low
	rotateValue = HandleDeadband(rotateValue, 0.06);

	// Sensitivity adjustment
	rotateValue = GetCubicAdjustment(rotateValue, rotateSensitivity);
	rotateValue *= fabs(thrustValue);

	thrustValue = GetCubicAdjustment(thrustValue, thrustSensitivity);

	leftOutput = thrustValue + rotateValue;			// CHECK FOR COMP BOT
	rightOutput = thrustValue - rotateValue;

	// Make sure, output values are within range
	if (leftOutput > 1.0) {
		leftOutput = 1.0;
	} else if (rightOutput > 1.0) {
		rightOutput = 1.0;
	} else if (leftOutput < -1.0) {
		leftOutput = -1.0;
	} else if (rightOutput < -1.0) {
		rightOutput = -1.0;
	}

	robot_->SetDriveValues(RobotModel::kLeftWheels, -leftOutput); //TODO ARTEMIS FIX CHANGE INVERSION
	robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);

}

// tank drive
void DriveController::TankDrive(double myLeft, double myRight) {
	leftOutput = myLeft * GetDriveDirection();
	rightOutput = myRight * GetDriveDirection();

	robot_->SetDriveValues(RobotModel::kLeftWheels, -leftOutput); //TODO ARTEMIS FIX CHANGE INVERSION
	robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);
}

void DriveController::QuickTurn(double myRight, double turnConstant) {

	double rotateValue = GetCubicAdjustment(myRight, turnConstant);

	robot_->SetDriveValues(RobotModel::kLeftWheels, rotateValue);		// CHECK FOR COMP BOT
	robot_->SetDriveValues(RobotModel::kRightWheels, -rotateValue);		// CHECK FOR COMP BOT
}

int DriveController::GetDriveDirection() {
	if (humanControl_->GetReverseDriveDesired()) {
		return -1;
	} else {
		return 1;
	}
}

// depends on joystick sensitivity
double DriveController::HandleDeadband(double value, double deadband) {
	if (fabs(value) < deadband) {
		return 0.0;
	} else {
		return value;
	}
}

// Rotation sensitivity: when z == 0 same output; when z==1, output^3
double DriveController::GetCubicAdjustment(double value, double adjustmentConstant) {
	return adjustmentConstant * std::pow(value, 3.0) + (1 - adjustmentConstant) * value;
}

bool DriveController::IsDone() {
	return isDone_;
}

// update shuffleboard values
void DriveController::PrintDriveValues(){
	thrustZNet_.SetDouble(thrustSensitivity_);
	rotateZNet_.SetDouble(rotateSensitivity_);
	gearDesireNet_.SetBoolean(humanControl_->GetHighGearDesired());
	quickturnDesireNet_.SetBoolean(humanControl_->GetQuickTurnDesired());
	arcadeDesireNet_.SetBoolean(humanControl_->GetArcadeDriveDesired());
	leftDriveNet_.SetDouble(leftOutput); //TODO MOVE INTO METHOD + line after
	rightDriveNet_.SetDouble(rightOutput);
	driveDirectionNet_.SetDouble(GetDriveDirection());
	navXAngleNet_.SetDouble(robot_->GetNavXYaw());
	leftDistanceNet_.SetDouble(robot_->GetLeftDistance());
	rightDistanceNet_.SetDouble(robot_->GetRightDistance());
	leftEncoderNet_.SetDouble(robot_->GetLeftEncoderValue());
	rightEncoderNet_.SetDouble(robot_->GetRightEncoderValue());
}

DriveController::~DriveController() {
}
