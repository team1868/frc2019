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
#include <NetworkTables/NetworkTableEntry.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h> //is this one necessary
//#include "Ports2019.h"

class ControlBoard {
public:
	enum Joysticks{ kLeftJoy, kRightJoy };
	enum Axes{ kX, kY, kZ, kLT};
	enum JoystickMode{ gamePad, twoJoy};

	/**
	 * ControlBoard constructor initializes joysticks, buttons, and all other values
	 */
	ControlBoard();

	/**
	 * Reads all control values for buttons and joysticks
	 */
	void ReadControls();

	/**
	 * Returns joystick value according to specified joystick and axis
	 */
	double GetJoystickValue(Joysticks j, Axes a);
	bool GetReverseDriveDesired();
	bool GetHighGearDesired();
	bool GetArcadeDriveDesired();
	bool GetQuickTurnDesired();

	//TODO DELETE
	bool GetTestDesired();

	bool GetCargoIntakeDesired();
	bool GetCargoUnintakeDesired();
	bool GetCargoFlywheelDesired();
	bool GetHatchEngageDesired();

	/**
	 * Returns desired values for superstructure controls
	 */


	virtual ~ControlBoard();
private:
	void ReadAllButtons();

	JoystickMode curJoyMode, curOpJoyMode_; //TODO curJoyMode to curDriverJoyMode_

	// Joystick values
	double leftJoyX_, leftJoyY_, leftJoyZ_, rightJoyX_, rightJoyY_, rightJoyZ_;

	//for gamepad
	double leftJoyLTrigger_;

	// Drive Modes
	bool reverseDriveDesired_, highGearDesired_, arcadeDriveDesired_, quickTurnDesired_;

	//Superstructure
	bool cargoIntakeDesired_, cargoUnintakeDesired_, cargoFlywheelDesired_, hatchEngaged_;

	// Joysticks for drive
	frc::Joystick *leftJoy_, *rightJoy_;

	// Joysticks for operator
	frc::Joystick *operatorJoy_, *operatorJoyB_; //TODO implement

	// Buttons for drive
	ButtonReader *driveDirectionButton_, *gearHighShiftButton_, *gearLowShiftButton_, *arcadeDriveButton_, *quickTurnButton_,
		*cargoIntakeButton_, *cargoUnintakeButton_, *cargoFlywheelButton_, *hatchDoubleSolenoidButton_;

	nt::NetworkTableEntry leftZNet_, rightZNet_, joyModeNet_;

  //Buttons for superstructure

  //Variables for superstructure

  //TODO delete BELOW ALL GO AWAY DELETE DELETE DANG IT
   ButtonReader *testButton_;

};

#endif /* SRC_DRIVERSTATION_CONTROLBOARD_H_ */