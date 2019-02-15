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
	curJoyMode = gamePad; 
	curOpJoyMode_ = gamePad;

  leftJoyX_ = 0.0;
	leftJoyY_ = 0.0;
	leftJoyZ_ = 0.0;
	rightJoyX_ = 0.0;
	rightJoyY_ = 0.0;
	rightJoyZ_ = 0.0;

  leftJoy_ = new frc::Joystick(LEFT_JOY_USB_PORT);
	rightJoy_ = new frc::Joystick(RIGHT_JOY_USB_PORT); //if gamepad mode, just initialized and not used

	/*switch(curJoyMode){
		case(twoJoy):
			rightJoy_ = new frc::Joystick(RIGHT_JOY_USB_PORT);
			break;
		case(gamePad):
			printf("GamePad mode, right joy not initialized (good thing)\n");
			break;
		default:
			printf("ERROR: mode of numjoysticks (curJoyMode of type JoystickMode) not set in ControlBoard constructer.\n");
	}*/

	operatorJoy_ = new frc::Joystick(OPERATOR_JOY_USB_PORT);
	operatorJoyB_ = new frc::Joystick(OPERATOR_JOY_B_USB_PORT); //if gamepad mode, just initialized and not used
	
	//Drive Buttons
	highGearDesired_ = false;
	arcadeDriveDesired_ = true;
	quickTurnDesired_ = true;
	reverseDriveDesired_ = false;

	gearHighShiftButton_ = new ButtonReader(leftJoy_, HIGH_GEAR_BUTTON_PORT);
	gearLowShiftButton_ = new ButtonReader(leftJoy_, LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton_ = new ButtonReader(operatorJoy_, ARCADE_DRIVE_BUTTON_PORT); // TODO change this, delete actually
	quickTurnButton_ = new ButtonReader(rightJoy_, QUICK_TURN_BUTTON_PORT); //TODO FIX, aka add to shuffleboard
	driveDirectionButton_ = new ButtonReader(leftJoy_, DRIVE_DIRECTION_BUTTON_PORT);
	
	//Superstructure Buttons
	cargoIntakeDesired_ = false;
	cargoUnintakeDesired_ = false;
	cargoFlywheelDesired_ = false; 
	cargoFlywheelDesiredRocket_ = false;
	hatchBeakDesired_ = false;
	hatchOuttakeDesired_ = false;
	hatchWristUpDesired_ = false;
	hatchWristDownDesired_ = false;
	hatchIntakeWheelDesired_ = false;
	hatchUnintakeWheelDesired_ = false;
	
	cargoIntakeWristEngaged_ = false;
	hatchOuttakeEngaged_ = false;

	cargoIntakeButton_ = new ButtonReader(operatorJoy_, CARGO_INTAKE_BUTTON_PORT); //TODO make op
	cargoUnintakeButton_ = new ButtonReader(operatorJoy_, CARGO_UNINTAKE_BUTTON_PORT); //TODO make op
	cargoFlywheelButton_ = new ButtonReader(operatorJoy_, CARGO_FLYWHEEL_BUTTON_PORT); //TODO make op
	cargoFlywheelRocketButton_ = new ButtonReader(operatorJoy_, CARGO_FLYWHEEL_BUTTON_PORT);
	cargoIntakeWristButton_ = new ButtonReader(operatorJoy_, CARGO_INTAKE_WRIST_BUTTON_PORT);
	hatchOuttakeButton_ = new ButtonReader(operatorJoyB_, HATCH_OUTTAKE_BUTTON_PORT);
	hatchBeakButton_ = new ButtonReader(operatorJoyB_, HATCH_BEAK_BUTTON_PORT);
	hatchWristUpButton_ = new ButtonReader(operatorJoyB_, HATCH_WRIST_UP_BUTTON_PORT);
	hatchWristDownButton_ = new ButtonReader(operatorJoyB_, HATCH_WRIST_DOWN_BUTTON_PORT);
	hatchIntakeWheelButton_ = new ButtonReader(operatorJoyB_, HATCH_INTAKE_WHEEL_BUTTON_PORT);
	hatchUnintakeWheelButton_ = new ButtonReader(operatorJoyB_, HATCH_UNINTAKE_WHEEL_BUTTON_PORT);
	
	//TODO DELETE
	testButton_ = new ButtonReader(leftJoy_, 2);

	leftZNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Left Joy Z (thrust sensitivity)", 0.0).GetEntry();
	rightZNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Right Joy Z (rotate sensitivity)", 0.0).GetEntry();
	joyModeNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("GamePad", true).GetEntry(); //hm, consider for ooperator

    ReadControls();
}

void ControlBoard::ReadControls() {

	ReadAllButtons();

	if(joyModeNet_.GetBoolean(true)){
		curJoyMode = gamePad;
	} else {
		curJoyMode = twoJoy;
	}

	//Reading joystick values
	leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = leftJoy_->GetY();

	leftJoyLTrigger_ = leftJoy_->GetRawAxis(2); //TODO FIX, gamepad or not (error prob for not)

	leftJoyZ_ = leftZNet_.GetDouble(0.0);
	rightJoyZ_ = rightZNet_.GetDouble(0.0);
	switch(curJoyMode) {
		case(twoJoy):
			rightJoyX_ = rightJoy_->GetX();
			rightJoyY_ = rightJoy_->GetY();
			break;
		case(gamePad):
			rightJoyX_ = leftJoy_->GetRawAxis(4);
			rightJoyY_ = leftJoy_->GetRawAxis(5);
			break;
		default:
			printf("ERROR: mode of numjoysticks (curJoyMode of type JoystickMode) not set in ControlBoard read controls.\n");
	}

	//no switch, so using buttons
	reverseDriveDesired_ = driveDirectionButton_->IsDown();
	if(gearLowShiftButton_->IsDown()){
		highGearDesired_ = false;
	} else if (gearHighShiftButton_->IsDown()){
		highGearDesired_ = true;
	} //else remain as is
	//gearHighShiftButton_->IsDown();
	arcadeDriveDesired_ = arcadeDriveButton_->IsDown();
	quickTurnDesired_ = quickTurnButton_->IsDown();
	//TODO check all of this for logic
	cargoIntakeDesired_ = cargoIntakeButton_->IsDown();
	cargoUnintakeDesired_ = cargoUnintakeButton_->IsDown();

	cargoFlywheelDesired_ = cargoFlywheelButton_->IsDown();
	cargoFlywheelDesiredRocket_ = cargoFlywheelRocketButton_->IsDown();

	cargoIntakeWristDesired_ = cargoIntakeWristButton_->IsDown();

	hatchBeakDesired_ = hatchBeakButton_->IsDown();
	hatchOuttakeDesired_ = hatchOuttakeButton_->IsDown();

	hatchWristUpDesired_ = hatchWristUpButton_->IsDown();
	hatchWristDownDesired_ = hatchWristDownButton_->IsDown();

	hatchIntakeWheelDesired_ = hatchIntakeWheelButton_->IsDown();
	hatchUnintakeWheelDesired_ = hatchUnintakeWheelButton_->IsDown();

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

bool ControlBoard::GetArcadeDriveDesired() {
	return arcadeDriveDesired_;
}

bool ControlBoard::GetQuickTurnDesired() {
	return quickTurnDesired_;
}

//TODO DELETE
bool ControlBoard::GetTestDesired() {
	testButton_->ReadValue();
	return testButton_->IsDown(); // don't follow this example: is test
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

bool ControlBoard::GetCargoIntakeWristDesired(){
	printf("cargo intake wrist desired\n");
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

void ControlBoard::ReadAllButtons() {
	driveDirectionButton_->ReadValue();
	gearHighShiftButton_->ReadValue();
	gearLowShiftButton_->ReadValue();
	arcadeDriveButton_->ReadValue();
	quickTurnButton_->ReadValue();

	cargoIntakeButton_->ReadValue();
	cargoUnintakeButton_->ReadValue();
	cargoFlywheelButton_->ReadValue();
	cargoFlywheelRocketButton_->ReadValue();
	cargoIntakeWristButton_->ReadValue();
	hatchOuttakeButton_->ReadValue();
	hatchBeakButton_->ReadValue();
	hatchIntakeWheelButton_->ReadValue();
	hatchUnintakeWheelButton_->ReadValue();
	hatchWristUpButton_->ReadValue();
	hatchWristDownButton_->ReadValue();

}

ControlBoard::~ControlBoard() {
}