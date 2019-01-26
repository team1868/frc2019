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

  leftJoyX_ = 0.0;
	leftJoyY_ = 0.0;
	leftJoyZ_ = 0.0;
	rightJoyX_ = 0.0;
	rightJoyY_ = 0.0;
	rightJoyZ_ = 0.0;
    
	reverseDriveDesired_ = false;
	highGearDesired_ = true; // TODO may want to fix this
	arcadeDriveDesired_ = true;
	quickTurnDesired_ = true;

  leftJoy_ = new frc::Joystick(LEFT_JOY_USB_PORT);
	rightJoy_ = new frc::Joystick(RIGHT_JOY_USB_PORT);

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
	operatorJoyB_ = new frc::Joystick(OPERATOR_JOY_B_USB_PORT);

	gearShiftButton_ = new ButtonReader(rightJoy_, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton_ = new ButtonReader(operatorJoy_, ARCADE_DRIVE_BUTTON_PORT); // TODO change this
	quickTurnButton_ = new ButtonReader(rightJoy_, QUICK_TURN_BUTTON_PORT);
	driveDirectionButton_ = new ButtonReader(leftJoy_, DRIVE_DIRECTION_BUTTON_PORT);

    ReadControls();
}

void ControlBoard::ReadControls() {
	ReadAllButtons();

	//Reading joystick values
	leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = leftJoy_->GetY();
	leftJoyZ_ = leftJoy_->GetZ();

	switch(curJoyMode) {
		case(twoJoy):
			rightJoyX_ = rightJoy_->GetX();
			rightJoyY_ = rightJoy_->GetY();
			rightJoyZ_ = rightJoy_->GetZ();
			break;
		case(gamePad):
			rightJoyX_ = leftJoy_->GetRawAxis(4);
			rightJoyY_ = leftJoy_->GetRawAxis(5);
			break;
		default:
			printf("ERROR: mode of numjoysticks (curJoyMode of type JoystickMode) not set in ControlBoard read controls.\n");
	}
    
	reverseDriveDesired_ = driveDirectionButton_->IsDown();
	highGearDesired_ = gearShiftButton_->IsDown();
	arcadeDriveDesired_ = arcadeDriveButton_->IsDown();
	quickTurnDesired_ = quickTurnButton_->IsDown();

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

void ControlBoard::ReadAllButtons() {
	driveDirectionButton_->ReadValue();
	gearShiftButton_->ReadValue();
	arcadeDriveButton_->ReadValue();
	quickTurnButton_->ReadValue();
}

ControlBoard::~ControlBoard() {
}