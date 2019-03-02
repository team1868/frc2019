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
#include "../auto/PIDSource/PIDInputSource.h"
#include "../auto/PIDSource/PIDOutputSource.h"
#include "../auto/commands/DriveStraightCommand.h"
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/BuiltInWidgets.h>


class GuidedDriveController {
public:
	/**
	 * Initializes all variables
	 * takes in RobotModel and ControlBoard
	 */
	GuidedDriveController(RobotModel *robot, ControlBoard *humanControl,
		NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput);

	/**
	 * Destructor
	 */
	~GuidedDriveController();

	void DriveStraightInit();


	void Reset();

	void Disable();
	void Enable();

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

	double leftOutput, rightOutput;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;
	AnglePIDOutput* anglePIDOutput_;
	DistancePIDOutput* distancePIDOutput_;

	DriveStraightCommand *driveStraight_;

	bool isDone_;

	nt::NetworkTableEntry thrustZNet_, rotateZNet_, gearDesireNet_, quickturnDesireNet_, arcadeDesireNet_,
		leftDriveNet_, rightDriveNet_, driveDirectionNet_, navXAngleNet_, leftDistanceNet_, rightDistanceNet_,
		leftEncoderNet_, rightEncoderNet_, thrustDeadbandNet_, rotateDeadbandNet_, reverseReverseNet_;
};

//#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */
