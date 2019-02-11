/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ControlBoard.h"
#include "Ports2019.h"

ControlBoard::ControlBoard() {
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
	
	hatchOuttakeEngaged_ = false;
	hatchPickupEngaged_ = false;

	cargoIntakeButton_ = new ButtonReader(leftJoy_, CARGO_INTAKE_BUTTON_PORT); //TODO make op
	cargoUnintakeButton_ = new ButtonReader(leftJoy_, CARGO_UNINTAKE_BUTTON_PORT); //TODO make op
	cargoFlywheelButton_ = new ButtonReader(leftJoy_, CARGO_FLYWHEEL_BUTTON_PORT); //TODO make op
	cargoFlywheelRocketButton_ = new ButtonReader(leftJoy_, CARGO_FLYWHEEL_BUTTON_PORT);
	hatchOuttakeButton_ = new ButtonReader(operatorJoy_, HATCH_OUTTAKE_BUTTON_PORT);
	hatchBeakButton_ = new ButtonReader(operatorJoy_, HATCH_BEAK_BUTTON_PORT);
	hatchPickupButton_ = new ButtonReader(leftJoy_, HATCH_DOUBLE_SOLENOID_BUTTON_PORT);
	
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

	cargoIntakeDesired_ = cargoIntakeButton_->IsDown();
	cargoUnintakeDesired_ = cargoUnintakeButton_->IsDown();


	if(cargoFlywheelButton_->IsDown()){ //button clicked, flip polarity (same button for off and on)
		cargoFlywheelDesired_ = !cargoFlywheelDesired_;
	}

	if(cargoFlywheelRocketButton_->IsDown()){
		cargoFlywheelDesiredRocket_ = !cargoFlywheelDesiredRocket_;
	}

	if(cargoIntakeWristButton_->WasJustPressed()){ //? or IsDown()
		cargoIntakeWristDesired_ = !cargoIntakeWristDesired_;
	}

	hatchBeakDesired_ = hatchBeakButton_->IsDown();

	if(hatchPickupButton_->WasJustPressed()){ 
		hatchPickupEngaged_ = !hatchPickupEngaged_;
	}

	if(hatchOuttakeButton_->WasJustPressed()){
		hatchOuttakeDesired_ = !hatchOuttakeDesired_;
	}

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
	return cargoIntakeWristDesired_;
}

bool ControlBoard::GetHatchBeakDesired(){
	return hatchBeakDesired_;
}

bool ControlBoard::GetHatchOuttakeDesired(){
	return hatchOuttakeDesired_;
}

bool ControlBoard::GetHatchPickupDesired(){ //actually switch from engage->disengage and disengage->engage
	return hatchPickupEngaged_;
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
	hatchOuttakeButton_->ReadValue();
}

ControlBoard::~ControlBoard() {
}