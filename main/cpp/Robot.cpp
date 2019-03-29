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
#include "../include/controllers/GuidedDriveController.h"
#include "../include/controllers/SuperstructureController.h"
#include "ControlBoard.h"
#include "RobotModel.h"
#include "Logger.h"
#include "../include/auto/PIDSource/PIDInputSource.h"
#include <frc/WPILib.h>

#include "../include/auto/commands/DriveStraightCommand.h"
#include "../include/auto/commands/CurveCommand.h"
#include "../include/auto/PIDSource/PIDInputSource.h"
#include "../include/auto/PIDSource/PIDOutputSource.h"

void Robot::RobotInit()  {

  printf("In robot init.\n");

  frc::Shuffleboard::GetTab("AUTO CHOOSER");

  //initialize RobotModel
  robot_ = new RobotModel();

  robot_->ZeroNavXYaw();
  robot_->CalibrateGyro();
  robot_->ResetGyro();

  //NOTE: POSSIBLE ERROR bc making multiple sources teleop vs auto
  NavXPIDSource *navXSource = new NavXPIDSource(robot_);
  //TalonEncoderPIDSource* talonEncoderSource = new TalonEncoderPIDSource(robot_);
  AnglePIDOutput* anglePIDOutput = new AnglePIDOutput();
  //DistancePIDOutput* distancePIDOutput = new DistancePIDOutput();
  
  
  //initialize controllers
  humanControl_ = new ControlBoard();
  driveController_ = new DriveController(robot_, humanControl_);
  guidedDriveController_ = new GuidedDriveController(robot_, humanControl_, navXSource, anglePIDOutput);

  superstructureController_ = new SuperstructureController(robot_, humanControl_); //TODO COMMENT OUT
  //talonEncoderSource_ = new TalonEncoderPIDSource(robot_);
  autoController_ = new AutoController();
  autoMode_ = NULL;

  habLimitSwitch_ = new DigitalInput(4);

  // testHabPiston = new DoubleSolenoid(0, 7, 1);
  testHabPiston = new DoubleSolenoid(0,5,2);
  testHabPiston->Set(DoubleSolenoid::kReverse);

  sandstormOverride_ = false;
  autoJoyVal_ = 0.0;

  robot_->SetLowGear();
  robot_->ResetDriveEncoders(); //needed?


  ResetTimerVariables();
  Wait(1.0);
  cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera.SetResolution(320,240);
  //Wait(1.0);


  printf("Main program initialized\n");

  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    // robot_->SetTestSequence("h 0 d 10.9"); //  b 1 s 0.4 h 1 straight forward hatch deploy
  // robot_->SetTestSequence("h 0 d 12.0 t 90.0 d 2.8 t 0.0 d 2.3"); // left hab 2 to left front hatch deploy working
  // robot_->SetTestSequence("h 0 a");
  // robot_->SetTestSequence("h 0 d 4.0 t -90.0 d 4.9 t 0.0 d 4.5 t -23.0 d 1.5"); // rocket near in progress lol left
  // robot_->SetTestSequence("h 0 d 19.1 t 90.0"); // cargo ship from hab 2 near cargo shot left
  // robot_->SetTestSequence("h 0 d 16.1 t 90.0"); // chargo ship from hab 1 near cargo shot left
  // robot_->SetTestSequence("h 0 d 19.1 t 90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t 90.0");  // 1.5 ish cargo shoot left hab 2

  //autoSendableChooser_ = new frc::SendableChooser<std::string>();
  // autoSendableChooser_.InitSendable();
  autoSendableChooser_.SetDefaultOption("0: blank", "h 0");
  autoSendableChooser_.AddOption("1: hab 1, straight, hatch", "h 0 d 10.9");
  autoSendableChooser_.AddOption("2: left, hab 2, left front hatch", "h 0 d 12.0 t 90.0 d 2.8 t 0.0 d 2.3");
  autoSendableChooser_.AddOption("3: right, hab 2, right front hatch", "h 0 d 12.0 t -90.0 d 2.8 t 0.0 d 2.3");
  autoSendableChooser_.AddOption("4: left, hab 2, left near cargo ship, 1 cargo", "h 0 d 19.1 t 90.0 ^");
  autoSendableChooser_.AddOption("5: left, hab 2, left near cargo ship, 1 cargo + pickup", "h 0 d 19.1 t 90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t 90.0");
  autoSendableChooser_.AddOption("6: right, hab 2, right near cargo ship, 1 cargo", "h 0 d 19.1 t -90.0 ^");
  autoSendableChooser_.AddOption("7: right, hab 2, right near cargo ship, 1 cargo + pickup", "h 0 d 19.1 t -90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t -90.0");
  autoSendableChooser_.AddOption("8: other, input your string", "h 0");
  
  //TODODODOD MOVE TO ROBOT MODEL
	leftEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Encoder (RM)", robot_->GetLeftEncoderValue()).GetEntry();
	rightEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Encoder (RM)", robot_->GetRightEncoderValue()).GetEntry();
  leftDistanceNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Distance (RM)", robot_->GetLeftEncoderValue()).GetEntry();
	rightDistanceNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Distance (RM)", robot_->GetRightEncoderValue()).GetEntry();
  leftEncoderStopNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Encoder Stopped (RM)", false).GetEntry();
	rightEncoderStopNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Encoder Stopped (RM)", false).GetEntry();
  testerPowerNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("TESTER power", 0.4).GetEntry();
  habRisePowerNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("TESTER - power", 0.4).GetEntry();
  guidedDriveNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Guided Drive", false).WithWidget(BuiltInWidgets::kToggleSwitch).GetEntry();
  frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("lala info", 0.0);
  // frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("Choose yo auto here fam", autoSendableChooser_).WithWidget(BuiltInWidgets::kComboBoxChooser);
  frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("Choose yo auto here fam", autoSendableChooser_).WithWidget(BuiltInWidgets::kSplitButtonChooser);
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
  leftDistanceNet_.SetDouble(robot_->GetLeftDistance());
  rightDistanceNet_.SetDouble(robot_->GetRightDistance());
  SmartDashboard::PutNumber("left distance per pulse", robot_->GetLeftDistancePerPulse());
  SmartDashboard::PutNumber("right distance per pulse", robot_->GetRightDistancePerPulse());
  SmartDashboard::PutNumber("left encoder scale", robot_->GetLeftEncodingScale());
  SmartDashboard::PutNumber("right encoder scale", robot_->GetRightEncodingScale());
  SmartDashboard::PutBoolean("is high gear?", robot_->IsHighGear());
  robot_->PrintState();
  // std::cout << "INFORMATION:            " << autoSendableChooser_.GetSelected() << std::endl;
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
  std::string autoModeString = autoSendableChooser_.GetSelected();

  printf("IN AUTONOMOUS \n");
  robot_->ResetDriveEncoders();
  robot_->ZeroNavXYaw();

  //TODO BAD FORM but whatev
  NavXPIDSource *navXSource = new NavXPIDSource(robot_);
  TalonEncoderPIDSource* talonEncoderSource = new TalonEncoderPIDSource(robot_);
  AnglePIDOutput* anglePIDOutput = new AnglePIDOutput();
  DistancePIDOutput* distancePIDOutput = new DistancePIDOutput();

  ResetTimerVariables();
  //driveStraight_ = new DriveStraightCommand(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot_, 2);
  //autoStartTime = currTimeSec_;
  //driveStraight_->Init();
  //printf("\n\n AUTO BEGINS AT %f\n\n", autoStartTime);
  //pivot_ = new PivotCommand(robot_, -90.0, true, navXSource);
  //pivot_->Init();
  //curve_ = new CurveCommand(robot_, 2, 90, true, navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput);
  //curve_->Init();
  // ellipse_ = new EllipseCommand(robot_, 1, 3, 90, false, navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput);
  // ellipse_->Init();

  // robot_->SetTestSequence("h 0 d 10.9"); //  b 1 s 0.4 h 1 straight forward hatch deploy
  // robot_->SetTestSequence("h 0 d 12.0 t 90.0 d 2.8 t 0.0 d 2.3"); // left hab 2 to left front hatch deploy working
  // robot_->SetTestSequence("h 0 a");
  // robot_->SetTestSequence("h 0 d 4.0 t -90.0 d 4.9 t 0.0 d 4.5 t -23.0 d 1.5"); // rocket near in progress lol left
  // robot_->SetTestSequence("h 0 d 19.1 t 90.0"); // cargo ship from hab 2 near cargo shot left
  // robot_->SetTestSequence("h 0 d 16.1 t 90.0"); // chargo ship from hab 1 near cargo shot left
  // robot_->SetTestSequence("h 0 d 19.1 t 90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t 90.0");  // 1.5 ish cargo shoot left hab 2
  // robot_->SetTestSequence(autoModeString);
  robot_->SetTestSequence("a h 0");
  // robot_->SetTestSequence("h 0 d 10.0 t 90.0 d 2.8 t 0.0 d 1.3 b 1 s 1.0 h 1"); // left hab 1 to front left
  
  autoMode_ = new TestMode(robot_);
  autoController_->SetAutonomousMode(autoMode_);
  autoController_->Init(AutoMode::AutoPositions::kBlank, AutoMode::HabLevel::k1);

  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  sandstormAuto_ = true;
}

void Robot::AutonomousPeriodic() {
  //frc::Shuffleboard::Update();

  UpdateTimerVariables();
  /*if(!driveStraight_->IsDone()){
    driveStraight_->Update(currTimeSec_, deltaTimeSec_);
  }*/
  /*if(!pivot_->IsDone()){
    pivot_->Update(currTimeSec_, deltaTimeSec_);
  }*/
  //if (!curve_->IsDone()) curve_->Update(currTimeSec_, deltaTimeSec_);
  // if(!ellipse_->IsDone()) ellipse_->Update(currTimeSec_, deltaTimeSec_);

  if(sandstormAuto_){
    humanControl_->ReadControls();
    autoJoyVal_ = humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY);
    autoJoyVal_ = driveController_->HandleDeadband(autoJoyVal_, driveController_->GetThrustDeadband()); //TODO certain want this deadband?
    if(autoJoyVal_ != 0.0){ //TODO mild sketch, check deadbands more
      printf("WARNING: EXITED SANDSTORM.  autoJoyVal_ is %f after deadband, not == 0\n\n",autoJoyVal_);
      autoController_->~AutoController(); //TODO check that these are being destructed
      sandstormAuto_ = false;
      TeleopInit();
    } else if (!autoController_->IsDone()) {
      autoController_->Update(currTimeSec_, deltaTimeSec_);
    }
  } else {
    TeleopPeriodic();
  }
  //if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  //} else {
    // Default Auto goes here
  //}
}
// reset timer and controller
void Robot::TeleopInit() {
  guidedDrive_ = false;
  frc::Shuffleboard::StartRecording();
  printf("Start teleop\n");
  robot_->ResetTimer();
	robot_->SetTalonCoastMode();
	ResetTimerVariables();
	ResetControllers();
	robot_->StartCompressor();
  robot_->ResetDriveEncoders();
  robot_->ZeroNavXYaw();
  robot_->SetHabBrake(true);
  printf("setting hab arms to reverse\n");
  testHabPiston->Set(DoubleSolenoid::kReverse);
}

// read controls and get current time from controllers
void Robot::TeleopPeriodic() {

  if(humanControl_->GetHabBrakeDesired()){ //hab arms, change this name or put in superstructure
    printf("greetings hab brake has been released\n");
    testHabPiston->Set(DoubleSolenoid::kForward);
  } /*else {
    testHabPiston->Set(DoubleSolenoid::kReverse);
  }*/

  if(humanControl_->GetTestDesired() && habLimitSwitch_->Get()){ //NOTE IMPORTANT TODO if delete, reenable the one commented out in superstructure and add a backwards //habdeploy
    //printf("\n\n\n hab limit is %f \n\n", habLimitSwitch_->Get());
    // robot_->SetHabBrake(false);
    robot_->SetHabMotorOutput(testerPowerNet_.GetDouble(0.4));
    printf("Hab downing at %f power.\n", testerPowerNet_.GetDouble(0.4));
  } else if (humanControl_->GetTest3Desired()){
    robot_->SetHabMotorOutput(-habRisePowerNet_.GetDouble(0.2));
    printf("Hab rising at %f power.\n", -habRisePowerNet_.GetDouble(0.2));
  } else {
    robot_->SetHabMotorOutput(0.0);
  }

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

      if(!guidedDriveNet_.GetBoolean(false)){ //regular drive
        
        if(guidedDrive_){ //switching modes
          guidedDrive_ = false;
          guidedDriveController_->Disable();
        } else {
		      driveController_->Update(currTimeSec_, deltaTimeSec_);
        }
      
      } else { //guided drive
        
        if(!guidedDrive_){ //switching modes
          guidedDriveController_->Enable();
          guidedDrive_ = true;
        } else {
          guidedDriveController_->Update(currTimeSec_, deltaTimeSec_);
        }
      
      }

      //guidedDrive_ = guidedDriveNet_.GetBoolean(false); //TODO change and make better
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
	superstructureController_->Reset();
}


//TODO get game data? needed?


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif