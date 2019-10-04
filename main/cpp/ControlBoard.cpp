/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ControlBoard.h"
#include "Ports2019.h"

ControlBoard::ControlBoard() {
	printf("in control board\n");

	leftZNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("thrust sensitivity", 0.2).GetEntry();
	smallTurnSensitivityNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("rotate sensitivity smallTurn", 0.0).GetEntry();
	rightZNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("rotate sensitivity", 0.7).GetEntry();
	joyModeNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("GamePad", false).GetEntry(); //hm, consider for ooperator
	opJoyModeNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("GamePad op", false).GetEntry();

    leftJoyX_ = 0.0;
	leftJoyY_ = 0.0;
	leftJoyZ_ = 0.0;
	rightJoyX_ = 0.0;
	rightJoyY_ = 0.0;
	rightJoyZ_ = 0.0;

    leftJoy_ = new frc::Joystick(LEFT_JOY_USB_PORT);
	rightJoy_ = new frc::Joystick(RIGHT_JOY_USB_PORT); //if gamepad mode, just initialized and not used

	operatorJoy_ = new frc::Joystick(OPERATOR_JOY_USB_PORT);
	operatorJoyB_ = new frc::Joystick(OPERATOR_JOY_B_USB_PORT); //if gamepad mode, just initialized and not used
	
	//Drive Buttons
	highGearDesired_ = false;
	smallTurnDesired_ = false;
	arcadeDriveDesired_ = true;
	quickTurnDesired_ = true;
	alignTapeDesired_ = false;
	reverseDriveDesired_ = false;

	smallTurnButton_ = new ButtonReader(leftJoy_, SMALL_TURN_BUTTON_PORT);
	arcadeDriveButton_ = new ButtonReader(operatorJoy_, ARCADE_DRIVE_BUTTON_PORT); 
	quickTurnButton_ = new ButtonReader(rightJoy_, 4); //FOR WAIT FOR BUTTON COMMAND
	alignTapeButton_ = new ButtonReader(rightJoy_, ALIGN_TAPE_BUTTON_PORT);
	driveDirectionButton_ = new ButtonReader(leftJoy_, DRIVE_DIRECTION_BUTTON_PORT);
	
	//Superstructure Buttons
	cargoIntakeDesired_ = false;
	cargoUnintakeDesired_ = false;
	cargoFlywheelDesired_ = false; 
	cargoFlywheelDesiredRocket_ = false;
	cargoFlywheelUnintakeDesired_ = false;
	cargoIntakeWristDesired_ = false;
	hatchBeakDesired_ = false;
	hatchOuttakeDesired_ = false;
	hatchWristUpDesired_ = false;
	hatchWristDownDesired_ = false;
	hatchIntakeWheelDesired_ = false;
	hatchUnintakeWheelDesired_ = false;
	habDeployDesired_ = false;
	habPrepDesired_ = false;
	habBrakeDesired_ = false;
	habBrakeLevel2Desired_ = false;
	habArmsRetractDesired_ = false;

	cargoIntakeButton_ = new ButtonReader(operatorJoy_, CARGO_INTAKE_BUTTON_PORT); 
	cargoUnintakeButton_ = new ButtonReader(operatorJoy_, CARGO_UNINTAKE_BUTTON_PORT); 
	hatchWristUpButton_ = new ButtonReader(operatorJoy_, HATCH_WRIST_UP_BUTTON_PORT);
	hatchWristDownButton_ = new ButtonReader(operatorJoy_, HATCH_WRIST_DOWN_BUTTON_PORT);
	hatchIntakeWheelButton_ = new ButtonReader(operatorJoy_, HATCH_INTAKE_WHEEL_BUTTON_PORT);
	hatchUnintakeWheelButton_ = new ButtonReader(operatorJoy_, HATCH_UNINTAKE_WHEEL_BUTTON_PORT);
	cargoIntakeWristButton_ = new ButtonReader(operatorJoy_, CARGO_INTAKE_WRIST_BUTTON_PORT);
	cargoFlywheelUnintakeButton_ = new ButtonReader(operatorJoy_, CARGO_FLYWHEEL_UNINTAKE_BUTTON_PORT);

	cargoFlywheelButton_ = new ButtonReader(operatorJoyB_, CARGO_FLYWHEEL_BUTTON_PORT);
	cargoFlywheelRocketButton_ = new ButtonReader(operatorJoyB_, CARGO_FLYWHEEL_ROCKET_BUTTON_PORT);
	hatchOuttakeButton_ = new ButtonReader(operatorJoyB_, HATCH_OUTTAKE_BUTTON_PORT);
	hatchBeakButton_ = new ButtonReader(operatorJoyB_, HATCH_BEAK_BUTTON_PORT);
	habDeployButton_ = new ButtonReader(operatorJoyB_, HAB_DEPLOY_BUTTON_PORT); 
	habRetractButton_ = new ButtonReader(operatorJoyB_, HAB_RETRACT_BUTTON_PORT); 
	habPrepButton_ = new ButtonReader(operatorJoyB_, HAB_PREP_BUTTON_PORT); //unused
	habBrakeButton_ = new ButtonReader(operatorJoyB_, HAB_BREAK_BUTTON_PORT);
	habBrakeLevel2Button_ = new ButtonReader(operatorJoyB_, HAB_BRAKE_LEVEL2_BUTTON_PORT);
	habArmsRetractButton_ = new ButtonReader(operatorJoyB_, HAB_ARMS_RETRACT_BUTTON_PORT);

	testButton_ = new ButtonReader(leftJoy_, 2);
	testButton2 = new ButtonReader(leftJoy_, 7); //unused
	testButton3 = new ButtonReader(rightJoy_, 2); 

	gearHighShiftButton_ = new ButtonReader(leftJoy_, HIGH_GEAR_BUTTON_PORT);
	// gearLowShiftButton_ = new ButtonReader(leftJoy_, 0); //no low gear button

    ReadControls();
}

void ControlBoard::ReadControls() {

	ReadAllButtons();

	//Reading joystick values
	leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = leftJoy_->GetY();

	leftJoyLTrigger_ = leftJoy_->GetRawAxis(2); //TODO FIX, gamepad or not (error prob for not)

	leftJoyZ_ = leftZNet_.GetDouble(0.0);
	if(!smallTurnDesired_){
		rightJoyZ_ = rightZNet_.GetDouble(0.0);
	} else {
		rightJoyZ_ = smallTurnSensitivityNet_.GetDouble(2*rightZNet_.GetDouble(0.0));
	}

	rightJoyX_ = rightJoy_->GetX();
	rightJoyY_ = rightJoy_->GetY();

	//no switch, so using buttons
	reverseDriveDesired_ = driveDirectionButton_->IsDown();
	
	if(gearHighShiftButton_->IsDown()){
		highGearDesired_ = true;
	} else {
		highGearDesired_ = false;
	}

	arcadeDriveDesired_ = arcadeDriveButton_->IsDown(); //DEAD CODE, now on shuffleboard
	quickTurnDesired_ = quickTurnButton_->IsDown();
	alignTapeDesired_ = alignTapeButton_->WasJustPressed(); //TODO POSSIBLE ERROR
	smallTurnDesired_ = smallTurnButton_->IsDown();


	//-----------SUPERSTRUCTURE-------------------
	cargoIntakeDesired_ = cargoIntakeButton_->IsDown();
	cargoUnintakeDesired_ = cargoUnintakeButton_->IsDown();

	cargoFlywheelDesired_ = cargoFlywheelButton_->IsDown();
	cargoFlywheelUnintakeDesired_ = cargoFlywheelUnintakeButton_->IsDown();
	cargoFlywheelDesiredRocket_ = cargoFlywheelRocketButton_->IsDown();

	cargoIntakeWristDesired_ = cargoIntakeWristButton_->IsDown();

	hatchBeakDesired_ = hatchBeakButton_->IsDown();
	hatchOuttakeDesired_ = hatchOuttakeButton_->IsDown();
	
	hatchWristUpDesired_ = hatchWristUpButton_->IsDown();
	hatchWristDownDesired_ = hatchWristDownButton_->IsDown();

	hatchIntakeWheelDesired_ = hatchIntakeWheelButton_->IsDown();
	hatchUnintakeWheelDesired_ = hatchUnintakeWheelButton_->IsDown();

	habBrakeDesired_ = habBrakeButton_->IsDown();
	habDeployDesired_ = habDeployButton_->IsDown();
	habRetractDesired_ = habRetractButton_->IsDown();
	habBrakeLevel2Desired_ = habBrakeLevel2Button_->IsDown();
	habArmsRetractDesired_ = habArmsRetractButton_->IsDown();
}

double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
	  case (kLeftJoy):
		switch(a) {
			case(kX):
				return leftJoyX_;
			case(kY):
				return leftJoyY_;
			case(kZ):
				return leftJoyZ_;
			case(kLT):
				return leftJoyLTrigger_;
		}
	  	break;
	  case (kRightJoy):
		switch(a){
	  	  case(kX):
			return rightJoyX_;
	  	  case(kY):
			return rightJoyY_;
	  	  case(kZ):
			return rightJoyZ_;
		}
		break;
	  default:
      printf("WARNING: Joystick value not received in ControlBoard::GetJoystickValue\n");
	}
	return 0;
}

bool ControlBoard::GetReverseDriveDesired() {
	return reverseDriveDesired_;
}

bool ControlBoard::GetHighGearDesired() {
	return highGearDesired_;
}

bool ControlBoard::GetSmallTurnDesired(){
	return smallTurnDesired_;
}

bool ControlBoard::GetArcadeDriveDesired() {
	return arcadeDriveDesired_;
}

bool ControlBoard::GetQuickTurnDesired() {
	return quickTurnDesired_;
}

bool ControlBoard::GetAlignTapeDesired(){
	return alignTapeDesired_;
}

//TODO DELETE
bool ControlBoard::GetTestDesired() {
	testButton_->ReadValue();
	return testButton_->IsDown(); // don't follow this example: is test
}

bool ControlBoard::GetTest2Desired(){
	testButton2->ReadValue();
	return testButton2->IsDown();
}

bool ControlBoard::GetTest3Desired(){
	testButton3->ReadValue();
	return testButton3->IsDown();
}

bool ControlBoard::GetCargoIntakeDesired(){
	return cargoIntakeDesired_;
}

bool ControlBoard::GetCargoUnintakeDesired(){
	return cargoUnintakeDesired_;
}

bool ControlBoard::GetCargoFlywheelDesired(){
	return cargoFlywheelDesired_;
}

bool ControlBoard::GetCargoFlywheelDesiredRocket(){
	return cargoFlywheelDesiredRocket_;
}

bool ControlBoard::GetCargoFlywheelUnintakeDesired() {
	return cargoFlywheelUnintakeDesired_;
}

bool ControlBoard::GetCargoIntakeWristDesired(){
	return cargoIntakeWristDesired_;
}

bool ControlBoard::GetHatchBeakDesired(){
	return hatchBeakDesired_;
}

bool ControlBoard::GetHatchOuttakeDesired(){
	return hatchOuttakeDesired_;
}

bool ControlBoard::GetHatchIntakeWheelDesired(){ 
	return hatchIntakeWheelDesired_;
}

bool ControlBoard::GetHatchUnintakeWheelDesired(){
	return hatchUnintakeWheelDesired_;
}

bool ControlBoard::GetHatchWristUpDesired(){ 
	return hatchWristUpDesired_;
}

bool ControlBoard::GetHatchWristDownDesired(){
	return hatchWristDownDesired_;
}

bool ControlBoard::GetHabDeployDesired(){
	return habDeployDesired_;
}

bool ControlBoard::GetHabPrepDesired(){
	return habPrepDesired_;
}

bool ControlBoard::GetHabBrakeDesired(){
	return habBrakeDesired_;
}

bool ControlBoard::GetHabRetractDesired(){
	return habRetractDesired_;
}

bool ControlBoard::GetHabBrakeLevel2Desired(){
	return habBrakeLevel2Desired_;
}

bool ControlBoard::GetHabArmsRetractDesired(){
	return habArmsRetractDesired_;
}

void ControlBoard::ReadAllButtons() {
	driveDirectionButton_->ReadValue();
	gearHighShiftButton_->ReadValue();
	// gearLowShiftButton_->ReadValue();
	smallTurnButton_->ReadValue();
	arcadeDriveButton_->ReadValue();
	quickTurnButton_->ReadValue();
	alignTapeButton_->ReadValue();

	cargoIntakeButton_->ReadValue();
	cargoUnintakeButton_->ReadValue();
	cargoFlywheelButton_->ReadValue();
	cargoFlywheelRocketButton_->ReadValue();
	cargoFlywheelUnintakeButton_->ReadValue();
	cargoIntakeWristButton_->ReadValue();
	hatchOuttakeButton_->ReadValue();
	hatchBeakButton_->ReadValue();
	hatchIntakeWheelButton_->ReadValue();
	hatchUnintakeWheelButton_->ReadValue();
	hatchWristUpButton_->ReadValue();
	hatchWristDownButton_->ReadValue();
	habDeployButton_->ReadValue();
	habRetractButton_->ReadValue();
	habPrepButton_->ReadValue();
	habBrakeButton_->ReadValue();
	habBrakeLevel2Button_->ReadValue();
	habArmsRetractButton_->ReadValue();
}

ControlBoard::~ControlBoard() {
}