/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef SRC_AUTO_COMMANDS_OUTTAKECARGOTOROCKETCOMMAND_H_
#define SRC_AUTO_COMMANDS_OUTTAKECARGOTOROCKETCOMMAND_H_

#include "../AutoCommand.h"
#include "../../RobotModel.h"

class OuttakeCargoToRocketCommand: public AutoCommand {
public:
    OuttakeCargoToRocketCommand(RobotModel *robot);

    virtual ~OuttakeCargoToRocketCommand();

    void Init();
    void Reset();
    void Update(double currTimeSec, double deltaTimeSec);
    bool IsDone();

private:
    bool isDone_;
	RobotModel *robot_;
    double cargoIntakeMotorOutput_;
	double outtakeCargoToRocketMotorOutput_;
	double startTime_;
	double deltaTime_;
    double deltaFlywheelStartTime_;
};

#endif /* SRC_AUTO_COMMANDS_OUTTAKECARGOTOROCKETCOMMAND_H_ */