/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
#include "ctre/Phoenix.h" //TODO <>
//#include "Robot.h"
#include "Ports2019.h"
//#include <frc/shuffleboard/Shuffleboard.h>
//#include <frc/shuffleboard/ShuffleboardTab.h>
//#include <TalonSRX.h>
//#include <VictorSPX.h>
//#include <ctre/phoenix/motorcontrol/can/TalonSRX>
//#include "CANTalon.h"

const double WHEEL_DIAMETER = 4.0 / 12.0; //Is this Right?
const double ENCODER_COUNT_PER_ROTATION = 256.0;
const int EDGES_PER_ENCODER_COUNT = 4;

RobotModel::RobotModel() : tab_(frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS")) {

  currentGameMode_ = SANDSTORM;

  frc::Shuffleboard::SelectTab("PRINTSSTUFFSYAYS");

  //pini_ = new Ini("home/lvuser/robot.ini");
  //RefreshIni();

  leftDriveOutput_ = 0.0;
  rightDriveOutput_ = 0.0;

  last_world_linear_accel_x_ = 0.0f;
  last_world_linear_accel_y_ = 0.0f;

  //TODO add to ini
  driveCurrentLimit_ = 0.0;
  intakeCurrentLimit_ = 0.0;
  totalCurrentLimit_ = 0.0;
  voltageFloor_ = 0.0;
  pressureFloor_ = 0.0;
  size_ = 0.0;

  //initializing timer
  timer_ = new frc::Timer();
  timer_->Start();

  //initializing pdp
  pdp_ = new frc::PowerDistributionPanel();

  leftDriveACurrent_ = 0;
  leftDriveBCurrent_ = 0;
  rightDriveACurrent_ = 0;
  rightDriveBCurrent_ = 0;
  roboRIOCurrent_ = 0;
  compressorCurrent_ = 0;

  //TODO: it says pressure sensor?
  
  leftDriveEncoder_ = new frc::Encoder(LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT, LEFT_DRIVE_ENCODER_RED_PWM_PORT, true);		// TODO check if true or false
	leftDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);
	leftDriveEncoder_->SetReverseDirection(true);

  rightDriveEncoder_ = new frc::Encoder(RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT, RIGHT_DRIVE_ENCODER_RED_PWM_PORT, false);
	rightDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);
	rightDriveEncoder_->SetReverseDirection(false);

  leftMaster_ = new WPI_TalonSRX(LEFT_DRIVE_MASTER_ID);
  rightMaster_ = new WPI_TalonSRX(RIGHT_DRIVE_MASTER_ID);
  leftSlave_ = new WPI_VictorSPX(LEFT_DRIVE_SLAVE_ID);
  rightSlave_ = new WPI_VictorSPX(RIGHT_DRIVE_SLAVE_ID);

  // Setting talon control modes and slaves
  leftMaster_->Set(ControlMode::PercentOutput, 0.0);
  rightMaster_->Set(ControlMode::PercentOutput, 0.0);
  leftSlave_->Follow(*leftMaster_);
  rightSlave_->Follow(*rightMaster_);

  // Setting Inversions
  //TODO: make variables for inverted
	rightMaster_->SetInverted(false);
	rightSlave_->SetInverted(false);
	leftMaster_->SetInverted(false);
	leftSlave_->SetInverted(false);

  // Initializing NavX
	navXSpeed_ = 200;
	navX_ = new AHRS(SPI::kMXP, navXSpeed_);
	Wait(1.0); // NavX takes a second to calibrate

  // Initializing pneumatics
	compressor_ = new frc::Compressor(PNEUMATICS_CONTROL_MODULE_ID);
	gearShiftSolenoid_ = new frc::DoubleSolenoid(GEAR_SHIFT_FORWARD_SOLENOID_PORT, GEAR_SHIFT_REVERSE_SOLENOID_PORT);

  //TODO Superstructure

}

void RobotModel::ResetTimer() {
  timer_->Reset();
}

double RobotModel::GetTime() {
  return timer_->Get();
}

WPI_TalonSRX *RobotModel::GetTalon(Talons talon) {
  switch(talon) {
    case(kLeftMaster):
      return leftMaster_;
    case(kRightMaster):
      return rightMaster_;
    default:
      printf("WARNING: Talon not returned from RobotModel::GetTalon()\n");
      return NULL;
  }
}

//TODO intake/outtake get motor speed

double RobotModel::GetWheelSpeed(RobotModel::Wheels wheel){
  switch(wheel) {
		case (kLeftWheels):
			return leftMaster_->Get();
		  break;
	  case (kRightWheels):
		  return rightMaster_->Get();
		  break;
	  case (kAllWheels):
		  return rightMaster_->Get(); //sketch, depending on right
    default:
      printf("WARNING: Wheel speed not returned in RobotModel::GetWheelSpeed()\n");
      return 0.0;
  }
}

void RobotModel::SetDriveValues(RobotModel::Wheels wheel, double value) {
  leftDriveOutput_ = rightDriveOutput_ = value;
	switch (wheel) {
    case (kLeftWheels):
      leftMaster_->Set(value);
      break;
    case (kRightWheels):
      rightMaster_->Set(value);
      break;
    case (kAllWheels):
      rightMaster_->Set(value);
      leftMaster_->Set(value);
    default:
      printf("WARNING: Drive value not set in RobotModel::SetDriveValues()");
	}
}

void RobotModel::SetTalonBrakeMode() {
	printf("In Brake Mode\n");
	rightMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	rightSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void RobotModel::SetTalonCoastMode() {
	printf("In Coast Mode\n");
	rightMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	rightSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void RobotModel::SetHighGear() {
	gearShiftSolenoid_->Set(frc::DoubleSolenoid::kReverse); // TODO Check if right
}

void RobotModel::SetLowGear() {
	gearShiftSolenoid_->Set(frc::DoubleSolenoid::kForward); // TODO Check if right
}

double RobotModel::GetLeftEncoderValue() {
	return leftDriveEncoder_->Get();
}

double RobotModel::GetRightEncoderValue() {
	return rightDriveEncoder_->Get();
}

double RobotModel::GetLeftDistance() {
	return -leftDriveEncoder_->GetDistance();
}

double RobotModel::GetRightDistance() {
	return -rightDriveEncoder_->GetDistance();
}

bool RobotModel::GetLeftEncoderStopped() {
	return leftDriveEncoder_->GetStopped();
}

bool RobotModel::GetRightEncoderStopped() {
	return rightDriveEncoder_->GetStopped();
}

void RobotModel::ResetDriveEncoders() {
	leftDriveEncoder_->Reset();
	rightDriveEncoder_->Reset();
}

double RobotModel::GetNavXYaw() {
	return navX_->GetYaw();
}

void RobotModel::ZeroNavXYaw() {
	//	for (int i = 0; i < 4; i++) {
	navX_->ZeroYaw();
	//	}
	printf("Zeroed Yaw\n");
}

double RobotModel::GetNavXPitch() {
	return navX_->GetPitch();
}

double RobotModel::GetNavXRoll() {
	return navX_->GetRoll();
}

//initializes variables pertaining to current
void RobotModel::UpdateCurrent() {
	leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveACurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	compressorCurrent_ = compressor_->GetCompressorCurrent();
	roboRIOCurrent_ = frc::ControllerPower::GetInputCurrent(); //TODO, this is deprecated use static class method
}

//returns the voltage
double RobotModel::GetVoltage() {
	return pdp_->GetVoltage();
}

//returns the total energy of the PDP
double RobotModel::GetTotalCurrent() {
	return pdp_->GetTotalCurrent();
}
//returns the total current of the PDP
double RobotModel::GetTotalEnergy() {
	return pdp_->GetTotalEnergy();
}

//returns the total power of the PDP
double RobotModel::GetTotalPower() {
	return pdp_->GetTotalPower();
}

double RobotModel::GetCurrent(int channel) {
	UpdateCurrent();
	switch(channel) {
	case RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
		return rightDriveACurrent_;
		break;
	case RIGHT_DRIVE_MOTOR_B_PDP_CHAN:
		return rightDriveBCurrent_;
		break;
	case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
		return leftDriveACurrent_;
		break;
	case LEFT_DRIVE_MOTOR_B_PDP_CHAN:
		return leftDriveBCurrent_;
		break;
	default:
    printf("WARNING: Current not recieved in RobotModel::GetCurrent()\n");
		return -1;
	}
}

//returns the current of the compressor
double RobotModel::GetCompressorCurrent() {
	return compressorCurrent_;
}

//returns the current of the roboRIO
double RobotModel::GetRIOCurrent() {
	return roboRIOCurrent_;
}

//returns the pressure
double RobotModel::GetPressureSensorVal() { //TODO make sensor
  return 0.0;
	//return 250 * (pressureSensor_->GetAverageVoltage() / 5) - 25;
}

bool RobotModel::CollisionDetected() {
	bool collisionDetected = false;

	double curr_world_linear_accel_x = navX_->GetWorldLinearAccelX();
	double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x_;
	last_world_linear_accel_x_ = curr_world_linear_accel_x;
	double curr_world_linear_accel_y = navX_->GetWorldLinearAccelY();
	double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y_;
	last_world_linear_accel_y_ = curr_world_linear_accel_y;

	if(leftDriveEncoder_->GetStopped() && rightDriveEncoder_->GetStopped()) {
		collisionDetected = true;
		printf("From ENCODER\n");
	}
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Jerk Y", last_world_linear_accel_y_);
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Jerk X", last_world_linear_accel_x_);

	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add(  "CollisionDetected", collisionDetected);
	return collisionDetected;
}

void RobotModel::StopCompressor() {
	compressor_->Stop();
}

void RobotModel::StartCompressor() {
	compressor_->Start();
}

//TODO Ini stuff
/*
void RobotModel::RefreshIni() {
	delete pini_;
	const char* usbPath = "insert path here"; // TODO fix
	if(FILE *file = fopen(usbPath, "r")) {
		fclose(file);
		pini_ = new Ini(usbPath);
	} else {
		pini_ = new Ini("/home/lvuser/robot.ini");
	}

	RefreshIniVals();
}

void RobotModel::RefreshIniVals() {
	// Pivot PID's
	pivotPFac_ = pini_->getf("PIVOT PID", "pFac", 0.0);
	pivotIFac_ = pini_->getf("PIVOT PID", "iFac", 0.0);
	pivotDFac_ = pini_->getf("PIVOT PID", "dFac", 0.0);
	pivotTimeoutSec_ = pini_->getf("PIVOT PID", "pivotTimeoutSec", 3.5);

	// Drive PID's
	driveRPFac_ = pini_->getf("DRIVESTRAIGHT PID", "rPFac", 0.0);
	driveRIFac_ = pini_->getf("DRIVESTRAIGHT PID", "rIFac", 0.0);
	driveRDFac_ = pini_->getf("DRIVESTRAIGHT PID", "rDFac", 0.0);
	driveDPFac_ = pini_->getf("DRIVESTRAIGHT PID", "dPFac", 0.0);
	driveDIFac_ = pini_->getf("DRIVESTRAIGHT PID", "dIFac", 0.0);
	driveDDFac_ = pini_->getf("DRIVESTRAIGHT PID", "dDFac", 0.0);
	driveTimeoutSec_ = pini_->getf("DRIVESTRAIGHT PID", "driveTimeoutSec", 2.5);

	// Superstructure teleop stuff
	intakeMotorOutput_ = pini_->getf("SUPERSTRUCTURE", "intakeMotorOutput", 0.0);
	intakeMotorOutputSubtract_ = pini_->getf("SUPERSTRUCTURE", "intakeMotorOutputSubtract_", 0.0);
	outtakeMotorOutput_ = pini_->getf("SUPERSTRUCTURE", "outtakeMotorOutput", 0.0);
	elevatorOutput_ = pini_->getf("SUPERSTRUCTURE", "elevatorOutput", 0.5);
	wristMotorOutput_ = pini_->getf("SUPERSTRUCTURE", "wristMotorOutput", 0.3); //TODO ADD TO INI
	wristPFac_ = pini_->getf("WRIST FAKE PID", "pFac", 0.0);

}
*/

RobotModel::GameMode RobotModel::GetGameMode(){
  return currentGameMode_;
}

void RobotModel::PrintState() {
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Drive Distance", GetLeftDistance());
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Drive Distance", GetRightDistance());
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("NavX Yaw", GetNavXYaw());
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("NavX Pitch", GetNavXPitch());
	frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("NavX Roll", GetNavXRoll());
  frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Pressure", GetPressureSensorVal());
}

RobotModel::~RobotModel() {
}