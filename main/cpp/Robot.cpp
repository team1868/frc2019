/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Ports2019.h"
#include "DriveController.h"
#include "SuperstructureController.h"
#include "ControlBoard.h"
#include "RobotModel.h"
#include "Logger.h"
#include "PIDInputSource.h"
#include <frc/WPILib.h>

void Robot::RobotInit() {
  
  printf("In robot init.\n");

  robot_ = new RobotModel();
  robot_->ZeroNavXYaw();
	//robot_->RefreshIni(); //TODO INI
  
  humanControl_ = new ControlBoard();
  driveController_ = new DriveController(robot_, humanControl_);
  superstructureController_ = new SuperstructureController(robot_, humanControl_);
  talonEncoderSource_ = new TalonEncoderPIDSource(robot_);


  //Sandstorm stuffs Here, Grace

  ResetTimerVariables();

  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  //RobotInit();
  robot_->ResetTimer();
	robot_->SetTalonCoastMode();
	ResetTimerVariables();
	ResetControllers();
	robot_->StartCompressor();
}

void Robot::TeleopPeriodic() {
  switch(robot_->GetGameMode()){
    case RobotModel::SANDSTORM:
      //Do override via checking joystick values (not 0)
      break;
    case RobotModel::NORMAL_TELEOP:
      UpdateTimerVariables();
		  robot_->PrintState();
		  humanControl_->ReadControls();
		  driveController_->Update(currTimeSec_, deltaTimeSec_);
		  superstructureController_->Update(currTimeSec_, deltaTimeSec_);
		  Logger::LogState(robot_, humanControl_);
      break;
    default:
      printf("ERROR: Mode not found in Robot::TeleopPeriodic\n");
  }
}

void Robot::TestPeriodic() {}

void Robot::ResetTimerVariables() {
  currTimeSec_ = robot_->GetTime();
	lastTimeSec_ = currTimeSec_;
	deltaTimeSec_ = 0.0;
}

void Robot::UpdateTimerVariables(){
  lastTimeSec_ = currTimeSec_;
	currTimeSec_ = robot_->GetTime();
	deltaTimeSec_ = currTimeSec_ - lastTimeSec_;
}

void Robot::ResetControllers() {
	driveController_->Reset();
	superstructureController_->Reset();
}


//TODO get game data? needed?


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
