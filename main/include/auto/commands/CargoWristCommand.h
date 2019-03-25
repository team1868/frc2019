/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include <math.h>
#include "RobotModel.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"
#include "../AutoCommand.h"
#include <networktables/NetworkTableEntry.h>

class CargoWristCommand : public AutoCommand {
 public:
  CargoWristCommand(RobotModel *robot);
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();
  ~CargoWristCommand();
 private:
  RobotModel *robot_;
  bool isDone_;
};
