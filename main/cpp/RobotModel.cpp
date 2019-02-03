/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
#include "ctre/Phoenix.h" 
#include "Ports2019.h"

const double WHEEL_DIAMETER = 4.0 / 12.0; //Is this Right?
const double ENCODER_COUNT_PER_ROTATION = 256.0;
const int EDGES_PER_ENCODER_COUNT = 4;

RobotModel::RobotModel() : tab_(frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS")){

  //initialize base variables
  currentGameMode_ = NORMAL_TELEOP;//SANDSTORM;

  //create private tab to get user input
  frc::Shuffleboard::GetTab("Private_Code_Input"); //ini replacement
  
  // initialize pid value nets
  dPFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Distance P", 0.8).GetEntry();
  dIFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Distance I", 0.0).GetEntry();
  dDFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Distance D", 0.2).GetEntry();

  rPFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Directional P", 0.8).GetEntry();
  rIFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Directional I", 0.0).GetEntry();
  rDFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Directional D", 0.2).GetEntry();

  pivotPFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Pivot Command P", 0.8).GetEntry();
  pivotIFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Pivot Command I", 0.0).GetEntry();
  pivotDFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Pivot Command D", 0.2).GetEntry();

  frc::Shuffleboard::SelectTab("PRINTSSTUFFSYAYS");

  // initialize variables
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

  // initiliaze encoders
  leftDriveEncoder_ = new frc::Encoder(LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT, LEFT_DRIVE_ENCODER_RED_PWM_PORT, true);		// TODO check if true or false
  leftDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);
  leftDriveEncoder_->SetReverseDirection(true);

  rightDriveEncoder_ = new frc::Encoder(RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT, RIGHT_DRIVE_ENCODER_RED_PWM_PORT, false);
  rightDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);
  rightDriveEncoder_->SetReverseDirection(false);

  //initilize motor controllers
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

  //Shuffleboard prints
  jerkYNet_ = tab_.Add("Jerk Y", navX_->GetWorldLinearAccelY()).GetEntry();
  jerkXNet_ = tab_.Add("Jerk X", navX_->GetWorldLinearAccelX()).GetEntry();
  //NOTE: collisions Detected not added here
  leftDistanceNet_ = tab_.Add("Left Drive Distance", GetLeftDistance()).GetEntry();
  rightDistanceNet_ = tab_.Add("Right Drive Distance", GetRightDistance()).GetEntry();
  yawNet_ = tab_.Add("NavX Yaw", GetNavXYaw()).GetEntry();
  pitchNet_ = tab_.Add("NavX Pitch", GetNavXPitch()).GetEntry();
  rollNet_ = tab_.Add("NavX Roll", GetNavXRoll()).GetEntry();
  pressureNet_ = tab_.Add("Pressure", GetPressureSensorVal()).GetEntry();
	
}

// reset timer
void RobotModel::ResetTimer() {
  timer_->Reset();
}

// get current time
double RobotModel::GetTime() {
  return timer_->Get();
}

// get specific motor controller
WPI_TalonSRX *RobotModel::GetTalon(Talons talon) {
  switch(talon) {
    case(kLeftMaster): //left motor
      return leftMaster_;
    case(kRightMaster): //right motor
      return rightMaster_;
    default:
      printf("WARNING: Talon not returned from RobotModel::GetTalon()\n");
      return NULL;
  }
}

//TODO intake/outtake get motor speed

// get specific wheel speed
double RobotModel::GetWheelSpeed(RobotModel::Wheels wheel){
  switch(wheel) {
	case (kLeftWheels): // left wheel
	  return leftMaster_->Get();
	case (kRightWheels): // right wheel
	  return rightMaster_->Get();
	case (kAllWheels): // right wheel
	  return rightMaster_->Get(); //sketch, depending on right
    default:
      printf("WARNING: Wheel speed not returned in RobotModel::GetWheelSpeed()\n");
      return 0.0;
  }
}

// drive specific motor
void RobotModel::SetDriveValues(RobotModel::Wheels wheel, double value) {
  leftDriveOutput_ = rightDriveOutput_ = value;
	switch (wheel) {
	  case (kLeftWheels): // set left
		leftMaster_->Set(value);
		break;
	  case (kRightWheels): // set right
		rightMaster_->Set(value);
		break;
	  case (kAllWheels): // set both
		rightMaster_->Set(value);
		leftMaster_->Set(value);
		break;
	  default:
		printf("WARNING: Drive value not set in RobotModel::SetDriveValues()");
	}
}

// set motor brake mode
void RobotModel::SetTalonBrakeMode() {
	printf("In Brake Mode\n");
	rightMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	rightSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

// set motor coast mode
void RobotModel::SetTalonCoastMode() {
	printf("In Coast Mode\n");
	rightMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	rightSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftSlave_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

// set motor high gear
void RobotModel::SetHighGear() {
	gearShiftSolenoid_->Set(frc::DoubleSolenoid::kReverse); // TODO Check if right
	//printf("Gear shift %d\n", gearShiftSolenoid_->Get());
}

// set motor low gear
void RobotModel::SetLowGear() {
	gearShiftSolenoid_->Set(frc::DoubleSolenoid::kForward); // TODO Check if right
	//printf("Gear shift %d\n", gearShiftSolenoid_->Get());
}

// ------------------------ get drive values--------------------------------------------
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

//----------------------------get Navx values-------------------------------------------------

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
	//TODO PUT THIS BACK IN, use robotcontroller static class method :( it's currently causing errors
	//leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	
	leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveACurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	compressorCurrent_ = compressor_->GetCompressorCurrent();
	roboRIOCurrent_ = frc::RobotController::GetInputCurrent();

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


// get current from channel 
double RobotModel::GetCurrent(int channel) {
	UpdateCurrent();
	switch(channel) {
	case RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
		return rightDriveACurrent_;
	case RIGHT_DRIVE_MOTOR_B_PDP_CHAN:
		return rightDriveBCurrent_;
	case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
		return leftDriveACurrent_;
	case LEFT_DRIVE_MOTOR_B_PDP_CHAN:
		return leftDriveBCurrent_;
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

// if sudden stop
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
	return collisionDetected;
}
//------------------------------------compressors--------------------------------
void RobotModel::StopCompressor() {
	compressor_->Stop();
}

void RobotModel::StartCompressor() {
	compressor_->Start();
}

//--------------------------------get overall game mode-----------------------------

RobotModel::GameMode RobotModel::GetGameMode(){
  return currentGameMode_;
}

//-------------------------------get PID values from user--------------------------
//distance p
double RobotModel::GetDPFac(){
	double dPFac = dPFacNet_.GetDouble(0.8);
	if(dPFac > 1.0 || dPFac < 0.0){
		return 0.0;
	} else {
		return dPFac;
	}
}

// distance I
double RobotModel::GetDIFac(){
	double dIFac = dIFacNet_.GetDouble(0.0);
	if(dIFac > 1.0 || dIFac < 0.0){
		return 0.0;
	} else {
		return dIFac;
	}
}

// distance D
double RobotModel::GetDDFac(){
	double dDFac = dDFacNet_.GetDouble(0.2);
	if(dDFac > 1.0 || dDFac < 0.0){
		return 0.0;
	} else {
		return dDFac;
	}
}

// rotational P
double RobotModel::GetRPFac(){
	double rPFac = rPFacNet_.GetDouble(0.8);
	if(rPFac > 1.0 || rPFac < 0.0){
		return 0.0;
	} else {
		return rPFac;
	}
}

// rotational I
double RobotModel::GetRIFac(){
	double rIFac = rIFacNet_.GetDouble(0.0);
	if(rIFac > 1.0 || rIFac < 0.0){
		return 0.0;
	} else {
		return rIFac;
	}
}

// rotational D
double RobotModel::GetRDFac(){
	double rDFac = rDFacNet_.GetDouble(0.2);
	if(rDFac > 1.0 || rDFac < 0.0){
		return 0.0;
	} else {
		return rDFac;
	}
}

// pivot P
double RobotModel::GetPivotPFac(){
	double pivotPFac = pivotPFacNet_.GetDouble(0.8);
	if(pivotPFac > 1.0 || pivotPFac < 0.0){
		return 0.0;
	} else {
		return pivotPFac;
	}
}

// pivot I
double RobotModel::GetPivotIFac(){
	double pivotIFac = pivotIFacNet_.GetDouble(0.0);
	if(pivotIFac > 1.0 || pivotIFac < 0.0){
		return 0.0;
	} else {
		return pivotIFac;
	}
}

// pivot D
double RobotModel::GetPivotDFac(){
	double pivotDFac = pivotDFacNet_.GetDouble(0.2);
	if(pivotDFac > 1.0 || pivotDFac < 0.0){
		return 0.0;
	} else {
		return pivotDFac;
	}
}

// update shuffleboard values
void RobotModel::PrintState(){
	jerkYNet_.SetDouble(navX_->GetWorldLinearAccelY());
	jerkXNet_.SetDouble(navX_->GetWorldLinearAccelX());
	leftDistanceNet_.SetDouble(GetLeftDistance());
	rightDistanceNet_.SetDouble(GetRightDistance());
	yawNet_.SetDouble(GetNavXYaw());
	pitchNet_.SetDouble(GetNavXPitch());
	rollNet_.SetDouble(GetNavXRoll());
	pressureNet_.SetDouble(GetPressureSensorVal());

}

// deconstructor
RobotModel::~RobotModel() {
}