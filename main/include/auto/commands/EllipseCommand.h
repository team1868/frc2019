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

class EllipseCommand : public AutoCommand{
 public:
  EllipseCommand(RobotModel *robot, double desiredA, double desiredB, double desiredAngle, bool turnLeft,
    NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
	  AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput);
  
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();

  ~EllipseCommand();

 private:
  double CalcCurDesiredAngle(double curDistance);
  void LoadPIDValues();

  NavXPIDSource *navXPIDSource_;
  TalonEncoderPIDSource *talonEncoderPIDSource_;
  AnglePIDOutput *anglePIDOutput_;
  DistancePIDOutput *distancePIDOutput_;
  PIDController *dPID_, *tPID_;

  double dPFac_, dIFac_, dDFac_;
  double tPFac_, tIFac_, tDFac_;

  RobotModel *robot_;
  double initAngle_;

  double desiredTotalDistance_;
  double desiredAngle_; //angle going across the ellipse
  double curDistance_, curAngle_, curDesiredAngle_;
  double curAngleError_;

  double a_, b_; //a is horizontal

  bool turnLeft_;

  bool isDone_;

  nt::NetworkTableEntry dOutputNet_, tOutputNet_, lOutputNet_, rOutputNet_,
    dErrorNet_, tErrorNet_, dPFacNet_, dIFacNet_, dDFacNet_, tPFacNet_, tIFacNet_, tDFacNet_;

};
