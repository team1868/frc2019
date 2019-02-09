/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#ifndef SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_
#define SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_

#include "RobotModel.h"
#include "ControlBoard.h"


class SuperstructureController {
public:
	SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl);

	void Reset();

	void Update(double currTimeSec, double deltaTimeSec);

	void RefreshIni();

	virtual ~SuperstructureController();

	enum SuperstructureState {
		kInit, kIdle
	};

private:
	RobotModel *robot_;
	ControlBoard *humanControl_;

	double cargoIntakeOutput_, cargoFlywheelOutput_;
	

	uint32_t currState_;
	uint32_t nextState_;
};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */