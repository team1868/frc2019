/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//TODO CONSIDER USING INHERITENCE FOR FOLLOWING CLASSES

#include "CurveCommand.h"
#include "EllipseCommand.h"
#include "PivotCommand.h"
#include "DriveStraightCommand.h"

//NEEDED? TODO
#include <frc/WPILib.h>
#include <math.h>
#include "RobotModel.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"
#include "../AutoCommand.h"
#include <networktables/NetworkTableEntry.h>
/*
class DriveToCommand : public AutoCommand{
  public:
    DriveToCommand();
  private:
}
*/

/*
class DriveToWithPivotCommand : public AutoCommand{
 public:
  DriveToWithPivotCommand();
};

class DriveToWithCurveCommand : public AutoCommand{
 public:
  DriveToWithCurveCommand();
};
*/
/*
class DriveToWithEllipseCommand : public AutoCommand{
 public:
  DriveToWithEllipseCommand();
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();
 private:
  RobotModel *robot_;
  double finalDesiredAngle_;
  double relativeDesiredAngle_;
  bool twoTurn_;
};
*/

