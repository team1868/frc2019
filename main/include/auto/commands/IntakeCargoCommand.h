/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef SRC_AUTO_COMMANDS_INTAKECARGOCOMMAND_H_
#define SRC_AUTO_COMMANDS_INTAKECARGOCOMMAND_H_

#include "../AutoCommand.h"
#include "../../RobotModel.h"

class IntakeCargoCommand: public AutoCommand {
public:
    IntakeCargoCommand(RobotModel *robot, double cargoIntakeMotorOutput = 1.0);
    virtual ~IntakeCargoCommand();

    void Init();

    void Reset();

    void Update(double currTimeSec, double deltaTimeSec);

    bool IsDone();

private:
    bool isDone_;

	RobotModel *robot_;

	double cargoIntakeMotorOutput_;
	double startTime_;
	double timeDiff_;
	bool wasJustRunning_;
};

#endif /* SRC_AUTO_COMMANDS_INTAKECOMMAND_H_ */