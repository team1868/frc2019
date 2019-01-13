/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "RobotModel.h"

//#define SRC_CONTROLLERS_DRIVECONTROLLER_H_
//#ifndef SRC_CONTROLLERS_DRIVECONTROLLER_H_

#include <frc/WPILib.h>
#include "ControlBoard.h"
#include "PIDInputSource.h"
#include "PIDOutputSource.h"


class DriveController {
public:
	/**
	 * Initializes all variables
	 * takes in RobotModel and ControlBoard
	 */
	DriveController(RobotModel *robot, ControlBoard *humanControl);

	/**
	 * Destructor
	 */
	~DriveController();

	void DriveStraightInit();


	void Reset();

	void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * Checks if align with cube command is done (if we use)
	 */
	bool IsDone();

	/**
	 * Prints direction, state, angle, etc.
	 */
	void PrintDriveValues();

private:
	/**
	 * Drives robot in Arcade
	 */
	void ArcadeDrive(double myX, double myY, double thrustSensitivity, double rotateSensitivity);

	/**
	 * Drives robot in Tank
	 */
	void TankDrive(double myLeft, double myRight);

	/**
	 * Quick Turn
	 */
	void QuickTurn(double myRight, double turnConstant);

	/**
	 * Returns -1 for reverse drive, 1 otherwise
	 */
	int GetDriveDirection();

	/**
	 * Adjusts joystick value if too small
	 */
	double HandleDeadband(double value, double deadband);

	/**
	 * Adjusts sensitivity for turn
	 */
	double GetCubicAdjustment(double value, double adjustmentConstant);

	RobotModel *robot_;
	ControlBoard *humanControl_;

	uint32_t currState_;
	uint32_t nextState_;

	double thrustSensitivity_, rotateSensitivity_, quickTurnSensitivity_;
	double LOW_THRUST_SENSITIVITY;
	double LOW_ROTATE_SENSITIVITY;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;

	bool isDone_;
};

//#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */

