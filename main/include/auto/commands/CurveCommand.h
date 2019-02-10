/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>
#include <AHRS.h>
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"
#include "RobotModel.h"
#include "../AutoCommand.h"

//NOTE: refer to 2016 code
class CurveCommand : public AutoCommand {
  public:
    CurveCommand(RobotModel *robot, double curveRadius, double desiredAngle, bool isAbsoluteAngle, NavXPIDSource* navXSource);
  private:
    RobotModel *robot_;
    NavXPIDSource *navXSource_;
    double initYaw_;
    double desiredAngle_;
    double pFac_, iFac_, dFac_;

    bool isDone;
    double lastYaw, curYaw, deltaYaw, accumulatedYaw;
};
