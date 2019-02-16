/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#define SRC_AUTO_COMMANDS_INTAKEHATCHBEAKCOMMAND_H_
#define SRC_AUTO_COMMANDS_INTAKEHATCHBEAKCOMMAND_H_

#include <frc/WPILib.h> // needed?
#include "../AutoCommand.h"
#include "RobotModel.h" //maybe?

class IntakeHatchBeakCommand : public AutoCommand {
public:
	IntakeHatchBeakCommand(RobotModel *robot, bool hatchIn);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
    void Reset();
    virtual ~IntakeHatchBeakCommand();

private:
	RobotModel *robot_;
	bool hatchIn_;
	bool isDone_;
};

// #endif /* SRC_AUTO_COMMANDS_INTAKEHATCHBEAKCOMMAND_H_ */

