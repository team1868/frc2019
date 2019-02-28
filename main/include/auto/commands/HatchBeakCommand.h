#pragma once

#ifndef SRC_AUTO_COMMANDS_HATCHBEAKCOMMAND_H_
#define SRC_AUTO_COMMANDS_HATCHBEAKCOMMAND_H_

#include <frc/WPILib.h>
#include <math.h>
#include "RobotModel.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"
#include "../AutoCommand.h"
#include <networktables/NetworkTableEntry.h>

class HatchBeakCommand : public AutoCommand {
public:
    HatchBeakCommand(RobotModel *robot, bool hatchOpen);
    virtual ~HatchBeakCommand();

    void Init();

    void Reset();

    void Update(double currTimeSec, double deltaTimeSec);

    bool IsDone();

private:
    bool isDone_;
    bool hatchOpen_;

	RobotModel *robot_;
};

#endif /* SRC_AUTO_COMMANDS_HATCHBEAKCOMMAND_H_ */