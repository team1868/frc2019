/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
#include "ctre/Phoenix.h" 
#include "Ports2019.h"
#include <math.h>

#define PI 3.141592

static const double WHEEL_DIAMETER = 4.0 / 12.0; //ft
static const double HIGH_GEAR_ENCODER_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*32/34; //ft
static const double LOW_GEAR_ENCODER_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*16/50;//50;//15; //ft 762
static const double ENCODER_COUNT_PER_ROTATION = 256.0;
static const int EDGES_PER_ENCODER_COUNT = 4;

static const double FLYWHEEL_DIAMETER = 1.0 / 12.0; //CHECK (in ft)

static const double MAX_CURRENT_OUTPUT = 180.0; //Amps //TODO INCORRECT< FIX
static const double MAX_DRIVE_MOTOR_CURRENT = 40.0; //Amps
//ratios work in 5 or 10% increments (accumulative)
static const double MIN_RATIO_ALL_CURRENT = 0.2;//0.7; //TODO add to shuffleboard
static const double MIN_RATIO_DRIVE_CURRENT = 0.7; //TODO add to shuffleboard //NOTE: UNUSED
static const double MIN_RATIO_SUPERSTRUCTURE_CURRENT = 0.5; //TODO add to shuffleboard
static const double MIN_VOLTAGE_BROWNOUT = 7.5;//7.5; //6.8; //brownout protection state; PWM, CAN, 6V, relay outputs, CAN motors disabled

//unused
static const double MAX_CURRENT_DRIVE_PERCENT = 0.8; //per motor, most teams are 40-50 Amps

//currently tuned for Mo Practice Bot
static const double LOW_GEAR_STATIC_FRICTION_POWER = 0.11;
static const double HIGH_GEAR_STATIC_FRICTION_POWER = 0.14;
static const double LOW_GEAR_QUICKTURN_ADDITIONAL_STATIC_FRICTION_POWER =  0.15 - LOW_GEAR_STATIC_FRICTION_POWER;
static const double HIGH_GEAR_QUICKTURN_ADDITIONAL_STATIC_FRICTION_POWER = 0.2 - HIGH_GEAR_STATIC_FRICTION_POWER;

RobotModel::RobotModel() : tab_(frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS")){

  printf("robot model constructor\n");
  //initialize base variables
  currentGameMode_ = NORMAL_TELEOP;//SANDSTORM;

  //create private tab to get user input
  frc::Shuffleboard::GetTab("Private_Code_Input"); //ini replacement
  
  // initialize pid value nets
  dPFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Distance P", 0.8).GetEntry();
  dIFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Distance I", 0.0).GetEntry();
  dDFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Distance D", 0.2).GetEntry();

  rPFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Directional P", 0.02).GetEntry();
  rIFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Directional I", 0.0).GetEntry();
  rDFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Drive Straight Directional D", 0.0).GetEntry();

  pivotPFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Pivot Command P", 0.075).GetEntry();
  pivotIFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Pivot Command I", 0.0).GetEntry();
  pivotDFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Pivot Command D", 0.07).GetEntry();
  pivotPFacNet_.SetDouble(0.07);
  pivotDFacNet_.SetDouble(0.04);

  
  maxOutputNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("MAX DRIVE OUTPUT", 1.0).GetEntry();
  minVoltNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Min Volt", MIN_VOLTAGE_BROWNOUT).GetEntry();
  maxCurrentNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Max Current", MAX_CURRENT_OUTPUT).GetEntry();

  printf("tabs done for pid\n");
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

  //power distribution
  ratioAll_ = 1.0; //no ratio
  ratioDrive_ = 1.0;
  ratioSuperstructure_ = 1.0;
  lastOver_ = false;
  compressorOff_ = false;

  leftDriveACurrent_ = 0;
  leftDriveBCurrent_ = 0;
  rightDriveACurrent_ = 0;
  rightDriveBCurrent_ = 0;
  roboRIOCurrent_ = 0;
  compressorCurrent_ = 0;

  //TODO: it says pressure sensor?

  //encoders initialized later to account for gear shift

  //initilize motor controllers
  leftMaster_ = new WPI_TalonSRX(LEFT_DRIVE_MASTER_ID);
  rightMaster_ = new WPI_TalonSRX(RIGHT_DRIVE_MASTER_ID);
  leftSlaveA_ = new WPI_VictorSPX(LEFT_DRIVE_SLAVE_A_ID);
  rightSlaveA_ = new WPI_VictorSPX(RIGHT_DRIVE_SLAVE_A_ID);
  leftSlaveB_ = new WPI_VictorSPX(LEFT_DRIVE_SLAVE_B_ID);
  rightSlaveB_ = new WPI_VictorSPX(RIGHT_DRIVE_SLAVE_B_ID);

  // Setting talon control modes and slaves
  leftMaster_->Set(ControlMode::PercentOutput, 0.0);
  rightMaster_->Set(ControlMode::PercentOutput, 0.0);
  leftSlaveA_->Follow(*leftMaster_);
  rightSlaveA_->Follow(*rightMaster_);
  leftSlaveB_->Follow(*leftMaster_);
  rightSlaveB_->Follow(*rightMaster_);

  // Setting Inversions
  //TODO: make variables for inverted
  rightMaster_->SetInverted(false);
  rightSlaveA_->SetInverted(false);
  rightSlaveB_->SetInverted(false);
  leftMaster_->SetInverted(false);
  leftSlaveA_->SetInverted(false);
  leftSlaveB_->SetInverted(false);

  // Initializing NavX
  navXSpeed_ = 200;
  navX_ = new AHRS(SPI::kMXP, navXSpeed_);
  Wait(1.0); // NavX takes a second to calibrate

  // Initializing pneumatics
  compressor_ = new frc::Compressor(PNEUMATICS_CONTROL_MODULE_A_ID);
  gearShiftSolenoid_ = new frc::Solenoid(PNEUMATICS_CONTROL_MODULE_A_ID, GEAR_SHIFT_FORWARD_SOLENOID_PORT);

	//TODODODODODOD TUNEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE EEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!
  highGear_ = false; //NOTE: make match with ControlBoard

  //Superstructure

	// Initialzing Gyro 
	gyro_ = new frc::AnalogGyro(GYRO_PORT);
	gyro_->InitGyro();
	gyro_->Calibrate();

	//Initializing Light Sensor
	lightSensor_ = new frc::DigitalInput(LIGHT_SENSOR_PORT);

  cargoIntakeWristSolenoid_ = new frc::DoubleSolenoid(PNEUMATICS_CONTROL_MODULE_A_ID, CARGO_WRIST_UP_DOUBLE_SOLENOID_CHAN, CARGO_WRIST_DOWN_DOUBLE_SOLENOID_CHAN);
  hatchBeakSolenoid_ = new frc::DoubleSolenoid(PNEUMATICS_CONTROL_MODULE_B_ID, HATCH_BEAK_CLOSED_DOUBLE_SOLENOID_CHAN, HATCH_BEAK_OPEN_DOUBLE_SOLENOID_CHAN);
  hatchOuttakeSolenoid_ = new frc::Solenoid(PNEUMATICS_CONTROL_MODULE_A_ID, HATCH_OUTTAKE_OUT_DOUBLE_SOLENOID_CHAN); //TODO possible error?
	habBrakeSolenoid_ = new frc::DoubleSolenoid(PNEUMATICS_CONTROL_MODULE_A_ID, HAB_BRAKE_ENGAGE_DOUBLE_SOLENOID_CHAN, HAB_BRAKE_RELEASE_DOUBLE_SOLENOID_CHAN);

	cargoIntakeMotor_ = new Victor(CARGO_INTAKE_MOTOR_PORT);
	cargoFlywheelMotor_ = new Victor(CARGO_FLYWHEEL_MOTOR_PORT);
	hatchIntakeWheelMotor_ = new Victor(HATCH_INTAKE_WHEEL_MOTOR_PORT);
	hatchWristMotor_ = new Victor(HATCH_WRIST_MOTOR_PORT);

	habSparkMotor_ = new rev::CANSparkMax(HAB_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless); //changed 7 to HAB_MOTOR_PORT, correct?

  cargoFlywheelEncoder_ = new Encoder(FLYWHEEL_ENCODER_A_PWM_PORT, FLYWHEEL_ENCODER_B_PWM_PORT, false);
  cargoFlywheelEncoder_->SetPIDSourceType(PIDSourceType::kRate);
  cargoFlywheelEncoder_->SetDistancePerPulse(FLYWHEEL_DIAMETER * PI / (ENCODER_COUNT_PER_ROTATION * EDGES_PER_ENCODER_COUNT));
	//habSparkMotor_->GetEncoder().GetPosition(); //move later


  //TODO MESS, also TODO check before matches
  cargoWristEngaged_ = false;
  hatchOuttakeEngaged_ = false;
  hatchBeakEngaged_ = false;

  // initiliaze encoders
  leftDriveEncoder_ = new frc::Encoder(LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT, LEFT_DRIVE_ENCODER_RED_PWM_PORT, true);		// TODO check if true or false
  if(highGear_){
  	leftDriveEncoder_->SetDistancePerPulse((HIGH_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION);
  } else {
		leftDriveEncoder_->SetDistancePerPulse((LOW_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION);
  }
  leftDriveEncoder_->SetReverseDirection(false); // WAS TRUE

  rightDriveEncoder_ = new frc::Encoder(RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT, RIGHT_DRIVE_ENCODER_RED_PWM_PORT, false);
  if(highGear_){
		rightDriveEncoder_->SetDistancePerPulse((HIGH_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION);
  } else {
		rightDriveEncoder_->SetDistancePerPulse((LOW_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION);
  }
  rightDriveEncoder_->SetReverseDirection(true); // WAS FALSE
  printf("Distance per pulse: %f", (LOW_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION);

  SetLowGear();
  StartCompressor();
  ResetDriveEncoders();

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

  ratioAllNet_ = tab_.Add("Ratio All", ratioAll_).GetEntry();
  ratioDriveNet_ = tab_.Add("Ratio Drive", ratioDrive_).GetEntry();
  ratioSuperNet_ = tab_.Add("Ratio Superstructure", ratioSuperstructure_).GetEntry();

	testSequence_ = "";
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
//WARNING: no static friction accounting, PID values would differ
void RobotModel::SetDriveValues(RobotModel::Wheels wheel, double value) {
  leftDriveOutput_ = rightDriveOutput_ = value;  //SKETCH TODO INCORRECT, last year's code but makes no sense if setting one wheel at a time
  value = ModifyCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN, value); //drive channels do the same as params
	switch (wheel) {
	  case (kLeftWheels): // set left
		leftMaster_->Set(-value);
		break;
	  case (kRightWheels): // set right
		rightMaster_->Set(-value);
		break;
	  case (kAllWheels): // set both
		rightMaster_->Set(-value);
		leftMaster_->Set(-value);
		break;
	  default:
		printf("WARNING: Drive value not set in RobotModel::SetDriveValues()");
	}
}

// drive motors
void RobotModel::SetDriveValues(double leftValue, double rightValue) {
  double thrust = leftValue + rightValue / 2.0; //find average to find y movement
  leftValue = HandleStaticFriction(leftValue, thrust);
  rightValue = HandleStaticFriction(rightValue, thrust);

  leftValue = ModifyCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN, leftValue);
  rightValue = ModifyCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN, rightValue);

  //TODO (minor) Make sure, output values are within range
  double maxOutput = maxOutputNet_.GetDouble(1.0);
  if (leftValue > maxOutput) {
	  rightValue = rightValue/leftValue;
	  leftValue = maxOutput;
  } else if (leftValue < -maxOutput) {
	  rightValue = rightValue/(-leftValue);
	  leftValue = -maxOutput;
  }
  if (rightValue > maxOutput) {
	  leftValue = leftValue/rightValue;
	  rightValue = maxOutput;
  } else if (rightValue < -maxOutput) {
	  leftValue = leftValue/(-rightValue);
	  rightValue = -maxOutput;
  }

  //printf("      RM left output: %f and right output: %f\n", -leftValue, rightValue);
  leftMaster_->Set(-leftValue);
  rightMaster_->Set(-rightValue);
}

double RobotModel::GetStaticFriction(double thrustValue){ //TODODODODODODOD MAKE A VARIABLE DON'T BE AN IDIOT
	double staticFriction;
	if(IsHighGear()){
		staticFriction = HIGH_GEAR_STATIC_FRICTION_POWER;
		
		if(thrustValue <= 0.1 && thrustValue >= -0.1){ //TODO TUNNNNNNNNNNNNNNNNNNNNNNNNEEEEEEEEEEEEEEEEEEEEEEEEE NNNNNNNNNNEEEEEEEEEEDDDDDDDDDD
			//quick turn
			staticFriction += HIGH_GEAR_QUICKTURN_ADDITIONAL_STATIC_FRICTION_POWER;
		}
	} else {
		staticFriction = LOW_GEAR_STATIC_FRICTION_POWER;

		if(thrustValue <= 0.1 && thrustValue >= -0.1){
			//quick turn
			staticFriction += LOW_GEAR_QUICKTURN_ADDITIONAL_STATIC_FRICTION_POWER;
		}
	}
	return staticFriction;
}

double RobotModel::HandleStaticFriction(double value, double thrustValue){
	double staticFriction = GetStaticFriction(thrustValue);
	if(value > 0.0){
		value += staticFriction;
	} else if(value < 0.0){
		value -= staticFriction;
	} //else don't waste power on static friction or might burn motors
	return value;
}

// set motor brake mode
void RobotModel::SetTalonBrakeMode() {
	printf("In Brake Mode\n");
	rightMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	rightSlaveA_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftSlaveA_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	rightSlaveB_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	leftSlaveB_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

// set motor coast mode
void RobotModel::SetTalonCoastMode() {
	printf("In Coast Mode\n");
	rightMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftMaster_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	rightSlaveA_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftSlaveA_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	rightSlaveB_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
	leftSlaveB_->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

// set motor high gear
void RobotModel::SetHighGear() {
	//gearShiftSolenoid_->Set(frc::DoubleSolenoid::kReverse); // TODO Check if right
	gearShiftSolenoid_->Set(false);
	highGear_ = true;
  //leftDriveEncoder_->SetDistancePerPulse((HIGH_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION); //TODO POSSIBLE DOURCE OF ERROR OR SLOWING CODE
  //rightDriveEncoder_->SetDistancePerPulse((HIGH_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION);
	//printf("Gear shift %d\n", gearShiftSolenoid_->Get());
}

// set motor low gear
void RobotModel::SetLowGear() {
	/* Double Solenoid code
	gearShiftSolenoid_->Set(frc::DoubleSolenoid::kForward); // TODO Check if right
	*/
	gearShiftSolenoid_->Set(true);

	highGear_ = false;

	//leftDriveEncoder_->SetDistancePerPulse((LOW_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION); //TODO POSSIBLE DOURCE OF ERROR OR SLOWING CODE

    //rightDriveEncoder_->SetDistancePerPulse((LOW_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION);
	//printf("Gear shift %d\n", gearShiftSolenoid_->Get());
}

bool RobotModel::IsHighGear(){
	return highGear_;
}

// ------------------------ get drive values--------------------------------------------
double RobotModel::GetLeftEncoderValue() {
	//printf("HELLO BLAH left encoder val???????? %d \n\n\n\n", leftDriveEncoder_->Get());
	return leftDriveEncoder_->Get();
}

double RobotModel::GetRightEncoderValue() {
	return rightDriveEncoder_->Get();
}

double RobotModel::GetLeftDistance() {
	if(IsHighGear()){
		return leftDriveEncoder_->Get()*(HIGH_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION;//GetDistance(); //correct? TODO
	} else {
		return leftDriveEncoder_->Get()*(LOW_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION;
	}
}

double RobotModel::GetRightDistance() {
	if(IsHighGear()){
		return rightDriveEncoder_->Get()*(HIGH_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION;//GetDistance(); //correct? TODO
	} else {
		return rightDriveEncoder_->Get()*(LOW_GEAR_ENCODER_ROTATION_DISTANCE) / ENCODER_COUNT_PER_ROTATION;
	}
}

double RobotModel::GetLeftDistancePerPulse(){
	return leftDriveEncoder_->GetDistancePerPulse();
}

double RobotModel::GetRightDistancePerPulse(){
	return rightDriveEncoder_->GetDistancePerPulse();
}

double RobotModel::GetLeftEncodingScale(){
	return leftDriveEncoder_->GetEncodingScale();
}

double RobotModel::GetRightEncodingScale(){
	return rightDriveEncoder_->GetEncodingScale();
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

//-------------------------SUPERSTRUCTURE control-------------------------------------

void RobotModel::ResetGyro(){
	gyro_->Reset();
}

double RobotModel::GetGyroAngle(){
	gyro_->GetAngle();
}

void RobotModel::CalibrateGyro(){
	gyro_->InitGyro();
	gyro_->Calibrate();
}

bool RobotModel::GetLightSensorStatus(){
	lightSensor_->Get();
}

int RobotModel::GetHabEncoderValue() {
	habRailsEncoderVal_ = habSparkMotor_->GetEncoder().GetPosition();
	return habRailsEncoderVal_;
	//todo: convert from encoder ticks to feet
}

// ****************************REMINDER:::::::: USE POWER CONTROLLER DON'T DO RANDOM MOTOR ON DANG IT
void RobotModel::SetCargoIntakeOutput(double output){
	//output = ModifyCurrent(CARGO_INTAKE_MOTOR_PDP_CHAN, output);
	cargoIntakeMotor_->Set(-output); //motor is negatized
}

void RobotModel::SetCargoUnintakeOutput(double output){
	//output = ModifyCurrent(CARGO_INTAKE_MOTOR_PDP_CHAN, output);
	cargoIntakeMotor_->Set(output); //motor is negatized
}

void RobotModel::SetCargoFlywheelOutput(double output){
	//output = ModifyCurrent(CARGO_FLYWHEEL_MOTOR_PDP_CHAN, output);
	cargoFlywheelMotor_->Set(-output); //motor negatized
}

void RobotModel::SetHatchIntakeWheelOutput(double output){
	hatchIntakeWheelMotor_->Set(-output);
}

void RobotModel::SetHabMotorOutput(double output){
	habSparkMotor_->Set(output);
}

void RobotModel::SetHatchWristOutput(double output){
	hatchWristMotor_->Set(-output);
}

void RobotModel::SetHabBrake(bool change){
	if(!change) {
		habBrakeSolenoid_->Set(DoubleSolenoid::kReverse);
	} else {
		habBrakeSolenoid_->Set(DoubleSolenoid::kForward);
	}
}

void RobotModel::SetCargoIntakeWrist(bool change){ //TODO RENAME
	if(change) {
		cargoIntakeWristSolenoid_->Set(DoubleSolenoid::kReverse);
		printf("forward cargo intake wrist\n");
	} else {
		cargoIntakeWristSolenoid_->Set(DoubleSolenoid::kForward); //TODO check if correct orientation
		//printf("reverse cargo intake wrist\n");
	}
}

void RobotModel::SetHatchOuttake(bool change){
	hatchOuttakeSolenoid_->Set(change);
	/*
	if(change) {
		hatchOuttakeSolenoid_->Set(DoubleSolenoid::kForward);
	} else {
		hatchOuttakeSolenoid_->Set(DoubleSolenoid::kReverse); //TODO check if correct orientation
	}*/
}

void RobotModel::SetHatchBeak(bool change){
	if(change) {
		hatchBeakSolenoid_->Set(DoubleSolenoid::kForward);
	} else {
		hatchBeakSolenoid_->Set(DoubleSolenoid::kReverse);
	}
}

AnalogGyro* RobotModel::GetGyro(){
	return gyro_;
}

Victor* RobotModel::GetHatchWristMotor(){
	return hatchWristMotor_;
}

double RobotModel::GetCargoFlywheelMotorOutput(double output){
	return cargoFlywheelMotor_->Get();
}

/*
double RobotModel::GetFlywheelMotorOutput(){
	return flywheelMotor_->Get();
}
*/
//slkjdflksjdf;liargj;oinlgkvgalkaghapeiu;oskldfjnaldirghoaf;ildzkjxflkcj TODODODODODODODODODODO what the gecko is the following my life is a lie
Encoder* RobotModel::GetCargoFlywheelEncoder(){ //TODO possible error, encoder* or *encoder
	return cargoFlywheelEncoder_;
}

Victor* RobotModel::GetCargoFlywheelMotor(){
	return cargoFlywheelMotor_;
}


double RobotModel::ModifyCurrent(int channel, double value){
	double power = value*ratioAll_;
	double individualPowerRatio = power;
	double tempPowerRatio;

	switch(channel){ //TODO check these constants what want to use? TODO CHANGE CHANGE DANG IT
		case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
		case RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
			power *= ratioDrive_;
			tempPowerRatio = CheckMotorCurrentOver(RIGHT_DRIVE_MOTOR_A_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			tempPowerRatio = CheckMotorCurrentOver(RIGHT_DRIVE_MOTOR_B_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			tempPowerRatio = CheckMotorCurrentOver(RIGHT_DRIVE_MOTOR_C_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			tempPowerRatio = CheckMotorCurrentOver(LEFT_DRIVE_MOTOR_A_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			tempPowerRatio = CheckMotorCurrentOver(LEFT_DRIVE_MOTOR_B_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			tempPowerRatio = CheckMotorCurrentOver(LEFT_DRIVE_MOTOR_C_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			power = individualPowerRatio;
			break;
		/*case CARGO_INTAKE_MOTOR_PDP_CHAN:
			power *= ratioSuperstructure_;
			power = CheckMotorCurrentOver(CARGO_INTAKE_MOTOR_PDP_CHAN, power);
			break;
		case CARGO_FLYWHEEL_MOTOR_PDP_CHAN: //unused, dont want to slow flywheel of wont shoot
			power *= ratioSuperstructure_;
			power = CheckMotorCurrentOver(CARGO_FLYWHEEL_MOTOR_PDP_CHAN, power);
			break;*/
		default:
			printf("WARNING: current not found to modify.  In ModifyCurrents() in RobotModel.cpp");
	}
	//debugging:
	//printf("ratio current %f, drive ratio current %f, super ratio current %d", ratioAll_, ratioDrive_, ratioSuperstructure_);
	return power;
}

double RobotModel::CheckMotorCurrentOver(int channel, double power){
	double motorCurrent = GetCurrent(channel);
	if( motorCurrent > MAX_DRIVE_MOTOR_CURRENT){ //current to individual motor is over, TODO change for super
		power = power*MAX_DRIVE_MOTOR_CURRENT / motorCurrent; //ratio down by percent over
	}
	return power;
}

//initializes variables pertaining to current
void RobotModel::UpdateCurrent() {
	//TODO PUT THIS BACK IN, use robotcontroller static class method :( it's currently causing errors
	//leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	
	leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	leftDriveCCurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_C_PDP_CHAN);
	rightDriveACurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveCCurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_C_PDP_CHAN);
	compressorCurrent_ = compressor_->GetCompressorCurrent();
	roboRIOCurrent_ = frc::RobotController::GetInputCurrent();

	//TODO fix and check logic
	if((GetTotalCurrent() > /*MAX_CURRENT_OUTPUT*/maxCurrentNet_.GetDouble(MAX_CURRENT_OUTPUT) || GetVoltage() <= minVoltNet_.GetDouble(MIN_VOLTAGE_BROWNOUT)) && !lastOver_){
		printf("\nSTOPPING\n\n");
		//StopCompressor();
		compressorOff_ = true;
		if(ratioAll_-0.05 > MIN_RATIO_ALL_CURRENT){
			ratioAll_ -= 0.05;
		} else if (ratioSuperstructure_-0.05 > MIN_RATIO_SUPERSTRUCTURE_CURRENT){
			ratioSuperstructure_ -= 0.05;
		} else if (ratioDrive_-0.05 > MIN_RATIO_DRIVE_CURRENT){
			ratioDrive_ -= 0.05;
		}// else {
		//	cutSlaves_ = true;
		//}
		lastOver_ = true;
	} else if((GetTotalCurrent() > /*MAX_CURRENT_OUTPUT*/maxCurrentNet_.GetDouble(MAX_CURRENT_OUTPUT) || GetVoltage() <= minVoltNet_.GetDouble(MIN_VOLTAGE_BROWNOUT) && lastOver_)){
		//know compressor is off, because lastOver_ is true
		//TODO WARNING THIS MIN IS NOT A MIN
		if(ratioAll_ > MIN_RATIO_ALL_CURRENT){ //sketch, sketch, check this
			ratioAll_ *= ratioAll_;//-= 0.1;
		} else if (ratioSuperstructure_ > MIN_RATIO_SUPERSTRUCTURE_CURRENT){
			ratioSuperstructure_ *= ratioSuperstructure_; //-= 0.1;
		} else if (ratioDrive_ > MIN_RATIO_DRIVE_CURRENT){
			ratioDrive_ *= ratioDrive_;//-= 0.1;
		}// else {
		//	cutSlaves_ = true;
		//}
		lastOver_ = true;
	} else { //good !
		//if(cutSlaves_){
		//	cutSlaves = false;
		//} else
		if(compressorOff_){
			StartCompressor();
			compressorOff_ = false;
		}
		if(ratioDrive_+0.001 < 1.0){
			ratioDrive_ += 0.001;
		} else if (ratioDrive_ < 1.0){
			ratioDrive_ = 1.0;
		} else if(ratioSuperstructure_+0.001 < 1.0){
			ratioSuperstructure_ += 0.001;
		} else if(ratioSuperstructure_ < 1.0){
			ratioSuperstructure_ = 1.0;
		} else if(ratioAll_+0.001 < 1.0){
			ratioAll_ += 0.001;
		} else if(ratioAll_ < 1.0){
			ratioAll_ = 1.0;
		} //else don't make it greater than one!
		lastOver_ = false;
	}


	ratioAllNet_.SetDouble(ratioAll_);
	ratioDriveNet_.SetDouble(ratioDrive_);
	ratioSuperNet_.SetDouble(ratioSuperstructure_);

	//printf("current updated\n");

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
	case RIGHT_DRIVE_MOTOR_C_PDP_CHAN:
		return rightDriveCCurrent_;
	case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
		return leftDriveACurrent_;
	case LEFT_DRIVE_MOTOR_B_PDP_CHAN:
		return leftDriveBCurrent_;
	case LEFT_DRIVE_MOTOR_C_PDP_CHAN:
		return leftDriveCCurrent_;
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
	//compressor_->Stop();
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
	double rPFac = rPFacNet_.GetDouble(0.02);
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
	double rDFac = rDFacNet_.GetDouble(0.0);
	if(rDFac > 1.0 || rDFac < 0.0){
		return 0.0;
	} else {
		return rDFac;
	}
}

// pivot P
double RobotModel::GetPivotPFac(){
	double pivotPFac = pivotPFacNet_.GetDouble(0.07);
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
	double pivotDFac = pivotDFacNet_.GetDouble(0.04);
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

std::string RobotModel::GetTestSequence() {
	return testSequence_;
}

void RobotModel::SetTestSequence(std::string testSequence) {
	testSequence_ = testSequence;
}

// deconstructor
RobotModel::~RobotModel() {
}