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
#include "../include/controllers/DriveController.h"
#include "../include/controllers/SuperstructureController.h"
#include "ControlBoard.h"
#include "RobotModel.h"
#include "Logger.h"
#include "../include/auto/PIDSource/PIDInputSource.h"
#include <frc/WPILib.h>

#include "../include/auto/commands/DriveStraightCommand.h"
#include "../include/auto/PIDSource/PIDInputSource.h"
#include "../include/auto/PIDSource/PIDOutputSource.h"

void Robot::RobotInit()  {
  
  printf("In robot init.\n");

  //initialize RobotModel
  robot_ = new RobotModel();

  robot_->ZeroNavXYaw();
  robot_->CalibrateGyro();
  robot_->ResetGyro();
  
  //initialize controllers
  humanControl_ = new ControlBoard();
  driveController_ = new DriveController(robot_, humanControl_);

  superstructureController_ = new SuperstructureController(robot_, humanControl_); //TODO COMMENT OUT
  talonEncoderSource_ = new TalonEncoderPIDSource(robot_);


  //Sandstorm stuffs Here, Grace

  ResetTimerVariables();

  printf("Main program initialized\n");

  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  //TODODODOD MOVE TO ROBOT MODEL
	leftEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Encoder (RM)", robot_->GetLeftEncoderValue()).GetEntry();
	rightEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Encoder (RM)", robot_->GetRightEncoderValue()).GetEntry();
  leftEncoderStopNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Encoder Stopped (RM)", false).GetEntry();
	rightEncoderStopNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Encoder Stopped (RM)", false).GetEntry();
  testerPowerNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("TESTER power", 0.1).GetEntry();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  leftEncoderNet_.SetDouble(robot_->GetLeftEncoderValue());
  rightEncoderNet_.SetDouble(robot_->GetRightEncoderValue());
  leftEncoderStopNet_.SetBoolean(robot_->GetLeftEncoderStopped());
  rightEncoderStopNet_.SetBoolean(robot_->GetRightEncoderStopped());
}

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
  frc::Shuffleboard::StartRecording();

  printf("IN AUTONOMOUS \n");

  ResetTimerVariables();
  NavXPIDSource *navXSource = new NavXPIDSource(robot_);
  TalonEncoderPIDSource* talonEncoderSource = new TalonEncoderPIDSource(robot_);
  AnglePIDOutput* anglePIDOutput = new AnglePIDOutput();
  DistancePIDOutput* distancePIDOutput = new DistancePIDOutput();
  driveStraight_ = new DriveStraightCommand(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot_, 2);
  driveStraight_->Init();
  /*pivot_ = new PivotCommand(robot_, 90.0, true, navXSource);
  pivot_->Init();*/
  //curve_ = new CurveCommand(robot_, 3, 90, navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput);
  //curve_->Init();


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
  //frc::Shuffleboard::Update();

  UpdateTimerVariables();
  if(!driveStraight_->IsDone()){
    driveStraight_->Update(currTimeSec_, deltaTimeSec_);
  }
  //pivot_->Update(currTimeSec_, deltaTimeSec_);
  //curve_->Update(currTimeSec_, deltaTimeSec_);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}
// reset timer and controller
void Robot::TeleopInit() {
  frc::Shuffleboard::StartRecording();
  printf("Start teleop\n");
  robot_->ResetTimer();
	robot_->SetTalonCoastMode();
	ResetTimerVariables();
	ResetControllers();
	robot_->StartCompressor();

}

// read controls and get current time from controllers
void Robot::TeleopPeriodic() {
  leftEncoderNet_.SetDouble(robot_->GetLeftEncoderValue());
  rightEncoderNet_.SetDouble(robot_->GetRightEncoderValue());
  leftEncoderStopNet_.SetBoolean(robot_->GetLeftEncoderStopped());
  rightEncoderStopNet_.SetBoolean(robot_->GetRightEncoderStopped());



  switch(robot_->GetGameMode()){ //TODO MAKE INTO SEMIAUTO OR REG
    case RobotModel::SANDSTORM:
      //Do override via checking joystick values (not 0)
      break;
    case RobotModel::NORMAL_TELEOP:
      //printf("In normal teleop periodic\n");
      UpdateTimerVariables();
		  robot_->PrintState();
      robot_->UpdateCurrent();
		  humanControl_->ReadControls();
		  driveController_->Update(currTimeSec_, deltaTimeSec_);
		  superstructureController_->Update(currTimeSec_, deltaTimeSec_); //TODO timer variables not being used, comment out
		  Logger::LogState(robot_, humanControl_);
      break;
    default:
      printf("ERROR: Mode not found in Robot::TeleopPeriodic\n");
  }
}

void Robot::TestPeriodic() {}

// set last time to current time
void Robot::ResetTimerVariables() {
  currTimeSec_ = robot_->GetTime();
	lastTimeSec_ = currTimeSec_;
	deltaTimeSec_ = 0.0;
}

// calculate time difference
void Robot::UpdateTimerVariables(){
  lastTimeSec_ = currTimeSec_;
	currTimeSec_ = robot_->GetTime();
	deltaTimeSec_ = currTimeSec_ - lastTimeSec_;
}
// reset controllers
void Robot::ResetControllers() {
	driveController_->Reset();
	//superstructureController_->Reset();
}


//TODO get game data? needed?


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
