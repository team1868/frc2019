/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "RobotModel.h"
#include "ControlBoard.h"
#include "../include/controllers/DriveController.h"
#include "../include/controllers/SuperstructureController.h"
#include "../include/auto/PIDSource/PIDInputSource.h"
#include "../include/auto/commands/DriveStraightCommand.h"
#include "../include/auto/commands/PivotCommand.h"
#include "../include/auto/commands/CurveCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include "rev/CANSparkMax.h"

class Robot : public frc::TimedRobot { //TODO CHANGE
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void ResetTimerVariables();
  void UpdateTimerVariables();
  void ResetControllers();

 private:
  RobotModel *robot_;
  ControlBoard *humanControl_;
  DriveController *driveController_;
  SuperstructureController *superstructureController_;
  TalonEncoderPIDSource *talonEncoderSource_;


  double currTimeSec_;
	double lastTimeSec_;
	double deltaTimeSec_;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto"; //TODO: Delete?? idk whatevs
  std::string m_autoSelected;


  DriveStraightCommand *driveStraight_;//TODO delete
  PivotCommand *pivot_; //testing TODO delte
  CurveCommand *curve_; //TODO DELETE

  nt::NetworkTableEntry leftEncoderNet_, rightEncoderNet_, leftEncoderStopNet_, rightEncoderStopNet_, testerPowerNet_;
};
