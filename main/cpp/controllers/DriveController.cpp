/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../include/controllers/DriveController.h"

// constructor
DriveController::DriveController(RobotModel *robot, ControlBoard *humanControl) {

	robot_ = robot;
	humanControl_ = humanControl;

	navXSource_ = new NavXPIDSource(robot_);
	talonEncoderSource_ = new TalonEncoderPIDSource(robot_);

	// driver preferences
	// set sensitivity to 0
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
	thrustZNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Thrust Z", 0.0).GetEntry();
	rotateZNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Rotate Z", 0.0).GetEntry();
	gearDesireNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("High Gear", humanControl_->GetHighGearDesired()).GetEntry();
	quickturnDesireNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Quick Turn", humanControl_->GetQuickTurnDesired()).GetEntry();
	arcadeDesireNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Arcade Drive", true).WithWidget(BuiltInWidgets::kToggleSwitch).GetEntry();// humanControl_->GetArcadeDriveDesired()).GetEntry();
	reverseReverseNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Lili Mode", true).WithWidget(BuiltInWidgets::kToggleSwitch).GetEntry();
	leftDriveNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Left Output (controller)", leftOutput).GetEntry();
	rightDriveNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Right Output (controller)", rightOutput).GetEntry();
	driveDirectionNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Drive Direction (controller)", GetDriveDirection()).GetEntry();
	navXAngleNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("NavX Angle", robot_->GetNavXYaw()).GetEntry();
	leftDistanceNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Left Drive Distance", robot_->GetLeftDistance()).GetEntry();
	rightDistanceNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Right Drive Distance", robot_->GetRightDistance()).GetEntry();
	leftEncoderNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Left Encoder", robot_->GetLeftEncoderValue()).GetEntry();
	rightEncoderNet_ = frc::Shuffleboard::GetTab("Public_Display").Add("Right Encoder", robot_->GetRightEncoderValue()).GetEntry();
	thrustDeadbandNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Thrust Deadband", 0.1).GetEntry();
	rotateDeadbandNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Rotate Deadband", 0.1).GetEntry();
}

void DriveController::Reset() {

}

// update drive
void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	PrintDriveValues();

	switch (robot_->GetGameMode()) {
		case (RobotModel::NORMAL_TELEOP):
			double leftJoyY, leftJoyZ, rightJoyY, rightJoyX, rightJoyZ;

			// takes joystick values
			leftJoyY = -humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY);	// was neg
			leftJoyZ = humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kZ);
			rightJoyY = -humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kY); // was neg
			rightJoyX = humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX);
			rightJoyZ = humanControl_->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kZ);

			// so leftJoyZ and rightJoyZ are from -1 to 1
			thrustSensitivity_ = (leftJoyZ + 1.0) / 2.0;
			rotateSensitivity_ = (rightJoyZ + 1.0) / 2.0;

			// change gear
			if (humanControl_->GetHighGearDesired()) { 
				robot_->SetHighGear();
			} else {
				robot_->SetLowGear();
			}
		
			if(arcadeDesireNet_.GetBoolean(true)){
				ArcadeDrive(rightJoyX, leftJoyY, thrustSensitivity_, rotateSensitivity_);
			} else {
				printf("Using Tank drive------------------------------------------------------------------------------------------------------- \n");
				TankDrive(leftJoyY, rightJoyY);
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
	double thrustValue = myY;
	double rotateValue = myX;

	//TODO: add safety for deadband values (prevent (-) or >= 0.1)
	thrustValue = HandleDeadband(thrustValue, thrustDeadbandNet_.GetDouble(0.0));	// 0.02 was too low
	rotateValue = HandleDeadband(rotateValue, rotateDeadbandNet_.GetDouble(0.0));
	// printf("%f\n", rotateValue);

	rotateValue = GetCubicAdjustment(rotateValue, rotateSensitivity);
	thrustValue = GetCubicAdjustment(thrustValue, thrustSensitivity);

	if(reverseReverseNet_.GetBoolean(true) || (!reverseReverseNet_.GetBoolean(true) && thrustValue >= 0.0)){
	// || reverseReverseNet_.GetBoolean(false) && thrustValue > 0.0){ (for lili mode)
		leftOutput = thrustValue + rotateValue;			// CHECK FOR COMP BOT
		rightOutput = thrustValue - rotateValue;
	} else {
		leftOutput = thrustValue - rotateValue;
		rightOutput = thrustValue + rotateValue;
	}

	//printf("Left Output: %f and Right Output: %f", -leftOutput, rightOutput);
	rightDriveNet_.SetDouble(rightOutput);
	leftDriveNet_.SetDouble(leftOutput);
	//printf("left output: %f, right output %f \n", leftOutput, rightOutput);
	robot_->SetDriveValues(-leftOutput, rightOutput);
	//printf("ROTATE VALUE              --------------------- %f", rotateValue);

}

// tank drive
void DriveController::TankDrive(double myLeft, double myRight) {

	myLeft = HandleDeadband(myLeft, thrustDeadbandNet_.GetDouble(0.0));	// 0.02 was too low
	myRight = HandleDeadband(myRight, rotateDeadbandNet_.GetDouble(0.0));

	leftOutput = myLeft * GetDriveDirection();
	rightOutput = myRight * GetDriveDirection();

	robot_->SetDriveValues(-leftOutput, rightOutput);
}

void DriveController::QuickTurn(double myRight, double turnConstant) {

	double rotateValue = GetCubicAdjustment(myRight, turnConstant);

	robot_->SetDriveValues(rotateValue, -rotateValue);
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

double DriveController::GetThrustDeadband(){
	return thrustDeadbandNet_.GetDouble(0.1);
}

double DriveController::GetRotateDeadband(){
	return rotateDeadbandNet_.GetDouble(0.1);
}


// rotation sensitivity: when z == 0 same output; when z==1, output^3
double DriveController::GetCubicAdjustment(double value, double adjustmentConstant) {
	return adjustmentConstant * std::pow(value, 3.0) + (1.0 - adjustmentConstant) * value;
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
	driveDirectionNet_.SetDouble(GetDriveDirection());
	navXAngleNet_.SetDouble(robot_->GetNavXYaw());
	leftDistanceNet_.SetDouble(robot_->GetLeftDistance());
	rightDistanceNet_.SetDouble(robot_->GetRightDistance());
	leftEncoderNet_.SetDouble(robot_->GetLeftEncoderValue());
	rightEncoderNet_.SetDouble(robot_->GetRightEncoderValue());
}

DriveController::~DriveController() {
}

void DriveController::Enable(){}
void DriveController::Disable(){}