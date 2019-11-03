/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#ifndef SRC_DRIVERSTATION_CONTROLBOARD_H_
#define SRC_DRIVERSTATION_CONTROLBOARD_H_

#include <frc/WPILib.h>
#include "ButtonReader.h"
#include <networktables/NetworkTableEntry.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

class ControlBoard {
public:
	enum Joysticks{ kLeftJoy, kRightJoy };
	enum Axes{ kX, kY, kZ, kLT};
	enum JoystickMode{ gamePad, twoJoy};

	// ControlBoard constructor initializes joysticks, buttons, and all other values
	ControlBoard();

	// reads all control values for buttons and joysticks
	void ReadControls();

	// returns joystick value according to specified joystick and axis
	double GetJoystickValue(Joysticks j, Axes a);
	bool GetReverseDriveDesired();
	bool GetHighGearDesired();
	bool GetSmallTurnDesired();
	bool GetArcadeDriveDesired();
	bool GetQuickTurnDesired();
	bool GetAlignTapeDesired();

	//TODO: MAKE REAL METHODS AND DELETE
	bool GetTestDesired();
	bool GetTest2Desired();
	bool GetTest3Desired();
	
	// returns desired values for superstructure controls
	bool GetCargoIntakeDesired();
	bool GetCargoUnintakeDesired();
	bool GetCargoFlywheelDesired();
	bool GetCargoFlywheelDesiredRocket();
	bool GetCargoFlywheelUnintakeDesired();
	bool GetCargoIntakeWristDesired();
	bool GetHatchOuttakeDesired();
	bool GetHatchBeakDesired();
	bool GetHatchIntakeWheelDesired();
	bool GetHatchUnintakeWheelDesired();
	bool GetHatchWristUpDesired();
	bool GetHatchWristDownDesired();
	bool GetHabDeployDesired();
	bool GetHabRetractDesired();
	bool GetHabPrepDesired(); //hab break
	bool GetHabBrakeDesired();
	bool GetHabBrakeLevel2Desired();
	bool GetHabArmsRetractDesired();

	virtual ~ControlBoard();
private:
	void ReadAllButtons();

	JoystickMode curJoyMode, curOpJoyMode_;

	// joystick values
	double leftJoyX_, leftJoyY_, leftJoyZ_, rightJoyX_, rightJoyY_, rightJoyZ_;

	// for gamepad
	double leftJoyLTrigger_;

	// Drive Modes
	bool reverseDriveDesired_, highGearDesired_, smallTurnDesired_, arcadeDriveDesired_, quickTurnDesired_;

	// joysticks for drive
	frc::Joystick *leftJoy_, *rightJoy_;

	// joysticks for operator
	frc::Joystick *operatorJoy_, *operatorJoyB_; 

	// buttons for drive
	ButtonReader *driveDirectionButton_, *gearHighShiftButton_, *smallTurnButton_, *arcadeDriveButton_, *quickTurnButton_;

	nt::NetworkTableEntry leftZNet_, rightZNet_, joyModeNet_, opJoyModeNet_, smallTurnSensitivityNet_;
	
	// buttons for superstructure
	ButtonReader *cargoFlywheelButton_, *cargoFlywheelRocketButton_, *cargoFlywheelUnintakeButton_, *cargoIntakeButton_, *cargoUnintakeButton_,
		*cargoIntakeWristButton_, *hatchOuttakeButton_, *hatchBeakButton_, *hatchIntakeWheelButton_, *hatchUnintakeWheelButton_,
		*hatchWristUpButton_, *hatchWristDownButton_, *habDeployButton_, *habRetractButton_, *habPrepButton_ , *habBrakeButton_, 
		*testButton_, *testButton2, *testButton3, *habBrakeLevel2Button_, *habArmsRetractButton_, *alignTapeButton_;

	// variables for superstructure
	bool cargoIntakeDesired_, cargoUnintakeDesired_, cargoFlywheelDesired_, cargoFlywheelDesiredRocket_, cargoFlywheelUnintakeDesired_, cargoIntakeWristDesired_,
		hatchBeakDesired_, hatchOuttakeDesired_, hatchIntakeWheelDesired_, hatchUnintakeWheelDesired_, hatchWristUpDesired_,  
		hatchWristDownDesired_, habDeployDesired_, habRetractDesired_, habPrepDesired_,  habBrakeDesired_, habBrakeLevel2Desired_, habArmsRetractDesired_,
		alignTapeDesired_;


};

#endif /* SRC_DRIVERSTATION_CONTROLBOARD_H_ */