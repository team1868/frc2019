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
#include "DriveController.h"
#include "../auto/PIDSource/PIDInputSource.h"
#include "../auto/PIDSource/PIDOutputSource.h"
#include "../auto/commands/DriveStraightCommand.h"
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/BuiltInWidgets.h>


class GuidedDriveController : public DriveController{ //TODO make protected? why is this public
public:
	/**
	 * Initializes all variables
	 * takes in RobotModel and ControlBoard
	 */
	GuidedDriveController(RobotModel *robot, ControlBoard *humanControl,
		NavXPIDSource* navXSource, AnglePIDOutput* anglePIDOutput);

	/**
	 * Destructor
	 */
	virtual ~GuidedDriveController();

	virtual void Disable();
	virtual void Enable();

private:
	/**
	 * Drives robot in Arcade
	 */
	virtual void ArcadeDrive(double myX, double myY, double thrustSensitivity, double rotateSensitivity);

	uint32_t currState_;
	uint32_t nextState_;

	double pFac_, iFac_, dFac_;

	NavXPIDSource *navXSource_;
	AnglePIDOutput* anglePIDOutput_;

	PIDController *anglePID_;

	nt::NetworkTableEntry pFacNet_, iFacNet_, dFacNet_, errorNet_;

};

//#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */

