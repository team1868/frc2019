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
#include "DriveController.h"
#include "SuperstructureController.h"
#include "PIDInputSource.h"

class Robot : public frc::IterativeRobot {
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
};
