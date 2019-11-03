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

  // initialize RobotModel
  robot_ = new RobotModel();
  robot_->CreateNavX();

  robot_->ZeroNavXYaw();

  aligningTape_ = false;

  // NOTE: POSSIBLE ERROR bc making multiple sources teleop vs auto
  navX_ = robot_->GetNavXSource();
  AnglePIDOutput* anglePIDOutput = new AnglePIDOutput();
  
  
  // initialize controllers
  humanControl_ = new ControlBoard();
  driveController_ = new DriveController(robot_, humanControl_);
  guidedDriveController_ = new GuidedDriveController(robot_, humanControl_, navX_, anglePIDOutput);

  superstructureController_ = new SuperstructureController(robot_, humanControl_);

  habLimitSwitch_ = new DigitalInput(4);

  testHabPiston = new DoubleSolenoid(0,5,2);
  testHabPiston->Set(DoubleSolenoid::kReverse);

  autoJoyVal_ = 0.0;

  robot_->SetLowGear();
  robot_->ResetDriveEncoders();


  ResetTimerVariables();
  // Wait(1.0);
  // NOTE camera commented out
  // cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(0);
  // camera.SetResolution(320,240);
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
  //autoSendableChooser_.InitSendable();
  /*autoSendableChooser_.SetDefaultOption("0: blank", "h 0");
  autoSendableChooser_.AddOption("1: hab 1, straight, hatch", "h 0 d 10.9");
  autoSendableChooser_.AddOption("2: left, hab 2, left front hatch", "h 0 d 12.0 t 90.0 d 2.8 t 0.0 d 2.3");
  autoSendableChooser_.AddOption("3: right, hab 2, right front hatch", "h 0 d 12.0 t -90.0 d 2.8 t 0.0 d 2.3");
  autoSendableChooser_.AddOption("4: left, hab 2, left near cargo ship, 1 cargo", "h 0 d 19.1 t 90.0 ^");
  autoSendableChooser_.AddOption("5: left, hab 2, left near cargo ship, 1 cargo + pickup", "h 0 d 19.1 t 90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t 90.0");
  autoSendableChooser_.AddOption("6: right, hab 2, right near cargo ship, 1 cargo", "h 0 d 19.1 t -90.0 ^");
  autoSendableChooser_.AddOption("7: right, hab 2, right near cargo ship, 1 cargo + pickup", "h 0 d 19.1 t -90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t -90.0");
  autoSendableChooser_.AddOption("8: other, input your string", "h 0");*/

  // //NASA field code
  // autoSendableChooser_.SetDefaultOption("0: blank", "h 0");
  // autoSendableChooser_.AddOption("1:1S,H", "h 0 d 10.9 a 1 b 1 s 0.1 h 1 d -0.5");
  // autoSendableChooser_.AddOption("2:2L,Lfront,H", "h 0 d 12.0 t 90.0 d 2.8 t 0.0 d 2.3");
  // autoSendableChooser_.AddOption("3:2R,Rfront,H", "h 0 d 12.0 t -90.0 d 2.8 t 0.0 d 2.3");
  // autoSendableChooser_.AddOption("4:2L,Lship,H", "t -1.0 d 19.05 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -1.4");
  // autoSendableChooser_.AddOption("5:2L,Lship,1.5C", "h 0 d 19.1 t 90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t 90.0");
  // autoSendableChooser_.AddOption("6:2R,Rship,1C", "h 0 d 19.1 t -90.0 ^");
  // autoSendableChooser_.AddOption("7:2R,Rship,1.5C", "h 0 d 19.1 t -90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t -90.0");
  // autoSendableChooser_.AddOption("9:2L,Lship,1.2H", "t -1.0 d 19.85 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -1.4 h 0 t -164.0 d 18.2 t -180.0 b 1 a 0");
  // autoSendableChooser_.AddOption("10:2L,Lship,HC", "t -1.0 d 19.6 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -3.4 h 0 t 0.0 d -16.0 t -15.0 w b 0");
  // autoSendableChooser_.AddOption("11:2R,Rship,1.2H", "t 1.0 d 19.85 t -90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -1.4 h 0 t 164.0 d 18.2 t 180.0 b 1 a 0"); //flipped of 9
  // autoSendableChooser_.AddOption("12:2R,Rship,HC", "t 1.0 d 19.6 t -90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -3.4 h 0 t 0.0 d -16.0 t 15.0 w b 0"); //flipped of 10
  // autoSendableChooser_.AddOption("13:2L,Lfront,H", "d 14.5 t 90.0 d 2.8 t 0.0 a 1 b 1 s 0.1 h 1 s 0.3 d -2.3 t -90.0 d 7.0 t 0.0 d -8.0 t -10.0 w b 0"); //UNTESTED sketch
  // autoSendableChooser_.AddOption("14:2R,Rfront,H", "d 14.5 t -90.0 d 2.8 t 0.0 a 1 b 1 s 0.1 h 1 s 0.3 d -2.3 t 90.0 d 7.0 t 0.0 d -8.0 t 10.0 w b 0"); //UNTESTED sketch
  // autoSendableChooser_.AddOption("8:other", "");

  // //Houston field code TODO
  // autoSendableChooser_.SetDefaultOption("0: blank", "h 0 b 0");
  // autoSendableChooser_.AddOption("1:1S,H", "h 0 d 10.817 a 1 b 1 s 0.1 h 1 d -0.5");
  // autoSendableChooser_.AddOption("2:2L,Lfront,H", "h 0 d 11.916 t 90.0 d 2.8 t 0.0 d 2.3");
  // autoSendableChooser_.AddOption("3:2R,Rfront,H", "h 0 d 11.916 t -90.0 d 2.8 t 0.0 d 2.3");
  // autoSendableChooser_.AddOption("4:2L,Lship,H", "t -1.0 d 18.97 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -1.4");
  // autoSendableChooser_.AddOption("5:2L,Lship,1.5C", "h 0 d 19.016 t 90.0 ^ d -2.883 t 0.0 d -17.0 w d 17.0 t 90.0");
  // autoSendableChooser_.AddOption("6:2R,Rship,1C", "h 0 d 19.016 t -90.0 ^");
  // autoSendableChooser_.AddOption("7:2R,Rship,1.5C", "h 0 d 19.016 t -90.0 ^ d -2.883 t 0.0 d -17.0 w d 17.0 t -90.0");
  // // autoSendableChooser_.AddOption("9:2L,Lship,1.2H", "t -1.0 d 19.85 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -1.4 h 0 t -164.0 d 18.2 t -180.0 b 1 a 0");
  // autoSendableChooser_.AddOption("10:2L,Lship,HC", "t -1.0 d 18.8 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -3.4 h 0 t 0.0 d -16.0 t -15.0 w b 0");
  // // autoSendableChooser_.AddOption("11:2R,Rship,1.2H", "t 1.0 d 19.85 t -90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -1.4 h 0 t 164.0 d 18.2 t 180.0 b 1 a 0"); //flipped of 9
  // autoSendableChooser_.AddOption("12:2R,Rship,HC", "t 1.0 d 18.8 t -90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -3.4 h 0 t 0.0 d -16.0 t 15.0 w b 0"); //flipped of 10
  // // autoSendableChooser_.AddOption("13:2L,Lfront,H", "d 14.5 t 90.0 d 2.8 t 0.0 a 1 b 1 s 0.1 h 1 s 0.3 d -2.3 t -90.0 d 7.0 t 0.0 d -8.0 t -10.0 w b 0"); //UNTESTED sketch
  // // autoSendableChooser_.AddOption("14:2R,Rfront,H", "d 14.5 t -90.0 d 2.8 t 0.0 a 1 b 1 s 0.1 h 1 s 0.3 d -2.3 t 90.0 d 7.0 t 0.0 d -8.0 t 10.0 w b 0"); //UNTESTED sketch


  //--------------------CHEZY-----------------------------------------------
  autoSendableChooser_.SetDefaultOption("0: blank", "h 0 b 0");
  autoSendableChooser_.AddOption("R1:1S,H", "h 0 d 10.85 a 1 b 1 s 0.1 h 1 d -0.5");
  autoSendableChooser_.AddOption("R2:2L,Lfr,H", "h 0 d 11.916 t 90.0 d 2.8 t 0.0 d 2.3");
  autoSendableChooser_.AddOption("R3:2R,Rfr,H", "h 0 d 11.916 t -90.0 d 2.8 t 0.0 d 2.3");
  autoSendableChooser_.AddOption("R4:2L,Lsh,H", "t -1.0 d 18.55 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -1.4");
  autoSendableChooser_.AddOption("R5:2L,Lsh,1.5C", "h 0 d 18.55 t 90.0 ^ d -2.883 t 0.0 d -17.0 w d 17.0 t 90.0");
  autoSendableChooser_.AddOption("R6:2R,Rsh,1C", "h 0 d 18.55 t -90.0 ^ h 0");
  autoSendableChooser_.AddOption("R7:2R,Rsh,1.5C", "h 0 d 18.55 t -90.0 ^ d -2.883 t 0.0 d -17.0 w d 17.0 t -90.0");
  autoSendableChooser_.AddOption("R10:2L,Lsh,HC", "t -1.0 d 18.55 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -3.4 h 0 t 0.0 d -16.0 t -15.0 w b 0");
  autoSendableChooser_.AddOption("R12:2R,Rsh,HC", "t 1.0 d 18.55 t -90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -3.4 h 0 t 0.0 d -16.0 t 15.0 w b 0"); //flipped of 10

  autoSendableChooser_.AddOption("B1:1S,H", "h 0 d 10.817 a 1 b 1 s 0.1 h 1 d -0.5");
  autoSendableChooser_.AddOption("B2:2L,Lfr,H", "h 0 d 11.916 t 90.0 d 2.8 t 0.0 d 2.3");
  autoSendableChooser_.AddOption("B3:2R,Rfr,H", "h 0 d 11.916 t -90.0 d 2.8 t 0.0 d 2.3");
  autoSendableChooser_.AddOption("B4:2L,Lsh,H", "t -1.4 d 19.35 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -1.4");
  autoSendableChooser_.AddOption("B5:2L,Lsh,1.5C", "t -0.5 d 19.35 t 90.0 ^ d -2.883 t 0.0 d -17.0 w d 17.0 t 90.0");
  autoSendableChooser_.AddOption("B6:2R,Rsh,1C", "h 0 d 19.35 t -90.0 ^ h 0");
  autoSendableChooser_.AddOption("B7:2R,Rsh,1.5C", "h 0 d 19.35 t -90.0 ^ d -2.883 t 0.0 d -17.0 w d 17.0 t -90.0");
  autoSendableChooser_.AddOption("B10:2L,Lsh,HC", "t -1.4 d 19.35 t 90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -3.4 h 0 t 0.0 d -16.0 t -15.0 w b 0");
  autoSendableChooser_.AddOption("B12:2R,Rsh,HC", "t 1.0 d 19.35 t -90.0 a 1 b 1 s 0.1 h 1 s 0.3 d -3.4 h 0 t 0.0 d -16.0 t 15.0 w b 0"); //flipped of 10
  autoSendableChooser_.AddOption("8:other", "h 0");  

  //TODO MOVE TO ROBOT MODEL
	leftEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Encoder (RM)", robot_->GetLeftEncoderValue()).GetEntry();
	rightEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Encoder (RM)", robot_->GetRightEncoderValue()).GetEntry();
  leftDistanceNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Distance (RM)", robot_->GetLeftEncoderValue()).GetEntry();
	rightDistanceNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Distance (RM)", robot_->GetRightEncoderValue()).GetEntry();
  leftEncoderStopNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Left Encoder Stopped (RM)", false).GetEntry();
	rightEncoderStopNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("Right Encoder Stopped (RM)", false).GetEntry();
  testerPowerNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("TESTER power", 0.6).GetEntry();
  habDeployAccelNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("hab deploy accel", 1.0005).GetEntry();
  habRaiseAccelNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("hab raise accel", 1.001).GetEntry();
  habRisePowerNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("TESTER - power", 0.9).GetEntry();
  guidedDriveNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Guided Drive", false).WithWidget(BuiltInWidgets::kToggleSwitch).GetEntry();
  lightSensorDisplayNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("light sensor read", false).GetEntry();
  //frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("lala info", 0.0);
  // frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("Choose yo auto here fam", autoSendableChooser_).WithWidget(BuiltInWidgets::kComboBoxChooser);
  frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("Choose auto", autoSendableChooser_).WithWidget(BuiltInWidgets::kSplitButtonChooser);
  sparkEncoderNet_ = frc::Shuffleboard::GetTab("PRINTSSTUFFSYAYS").Add("hab encoder val", 0.0).GetEntry();
  
  //autoChooserType_ = frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("SendableChooser", true).WithWidget(BuiltInWidgets::kToggleSwitch).GetEntry();
  auto8Val_ = frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("8:other string seq", "h 0 b 0").GetEntry();
  
  auto8Val_ = frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("8:other string seq", "h 0").GetEntry();
  
}

/*
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
  sparkEncoderNet_.SetDouble(robot_->GetHabEncoderValue());

  lightSensorDisplayNet_.SetBoolean(superstructureController_->CargoInIntake());

  SmartDashboard::PutNumber("left distance per pulse", robot_->GetLeftDistancePerPulse());
  SmartDashboard::PutNumber("right distance per pulse", robot_->GetRightDistancePerPulse());
  SmartDashboard::PutNumber("left encoder scale", robot_->GetLeftEncodingScale());
  SmartDashboard::PutNumber("right encoder scale", robot_->GetRightEncodingScale());
  SmartDashboard::PutBoolean("is high gear?", robot_->IsHighGear());
  robot_->PrintState();
  
  //printf("----------------------------%s\n", autoSendableChooser_.GetSelected().c_str());
  // std::cout << "INFORMATION:            " << autoSendableChooser_.GetSelected() << std::endl;
}

/*
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
  // std::string autoModeString = autoSendableChooser_.GetSelected();
  // if(autoMode_!=NULL){
  //   printf("deleted previous auto mode, ptr reset to null");
  //   delete autoMode_;
  //   autoMode_ = NULL;
  // }

  printf("IN AUTONOMOUS \n");
  //frc::Shuffleboard::GetTab("AUTO CHOOSER").Add("Choose auto", autoSendableChooser_).WithWidget(BuiltInWidgets::kSplitButtonChooser);


  autoController_ = new AutoController();
  autoMode_ = NULL;
  autoJoyVal_ = 0.0;

  robot_->ResetDriveEncoders();
  robot_->ZeroNavXYaw();
  robot_->SetLowGear(); //faster, for 2 hatch, so cargo does not fall out
  robot_->EngageHook();

  // TODO BAD FORM but whateva
  // NavXPIDSource *navXSource = new NavXPIDSource(robot_);
  TalonEncoderPIDSource* talonEncoderSource = new TalonEncoderPIDSource(robot_);
  AnglePIDOutput* anglePIDOutput = new AnglePIDOutput();
  DistancePIDOutput* distancePIDOutput = new DistancePIDOutput();

  ResetTimerVariables();



  // tuning pid
  // robot_->SetTestSequence("h 0 t -90.0 s 2.0 t 0.0");
  // robot_->SetTestSequence("h 0 t 8.0");

  // really sketch needs fixing:
  // blank
  // robot_->SetTestSequence("h 0");

  // RED ALLIANCE
  // hatches:
  // robot_->SetTestSequence("h 0 d 10.9"); //  b 1 s 0.4 h 1 straight forward hatch deploy
  // robot_->SetTestSequence("h 0 d 12.0 t 90.0 d 2.9 t 0.0 d 2.3"); // left hab 2 to left front hatch deploy included offset WORKS actually go back to 2.8
  // robot_->SetTestSequence("h 0 d 12.0 t -90.0 d 2.9 t 0.0 d 2.3"); // right hab 2 to right front hatch deploy included offset

  // cargo:
  // left:
  // robot_->SetTestSequence("h 0 d 15.8 t 90.0"); // chargo ship from hab 1 near cargo shot left
  // robot_->SetTestSequence("h 0 d 19.0 t 90.0"); // cargo ship from hab 2 near cargo shot left
  // robot_->SetTestSequence("h 0 d 18.8 t 90.0 ^ d -2.8 t 0.0 d -16.6 w d 16.6 t 90.0");  // 1.5 ish cargo shoot left hab 2
  // right:
  // robot_->SetTestSequence("h 0 d 15.8 t -90.0"); // chargo ship from hab 1 near cargo shot right
  // robot_->SetTestSequence("h 0 d 19.0 t -90.0"); // cargo ship from hab 2 near cargo shot right
  // robot_->SetTestSequence("h 0 d 18.8 t -90.0 ^ d -2.8 t 0.0 d -16.6 w d 16.6 t -90.0");  // 1.5 ish cargo shoot left hab 2


  // BLUE ALLIANCE
  // hatches:
  // robot_->SetTestSequence("h 0 d 10.9"); //  b 1 s 0.4 h 1 straight forward hatch deploy
  // robot_->SetTestSequence("h 0 d 12.0 t 90.0 d 3.05 t 0.0 d 2.3"); // left hab 2 to left front hatch deploy included offset
  // robot_->SetTestSequence("h 0 d 12.0 t -90.0 d 2.8 t 0.0 d 2.3"); // right hab 2 to right front hatch deploy included offset

  // cargo:
  // left:
  // robot_->SetTestSequence("h 0 d 15.8 t 90.0"); // chargo ship from hab 1 near cargo shot left
  // robot_->SetTestSequence("h 0 d 18.8 t 90.0"); // cargo ship from hab 2 near cargo shot left
  // robot_->SetTestSequence("h 0 d 19.0 t 90.0"); // cargo ship from hab 2 near cargo shot left
  // robot_->SetTestSequence("h 0 d 18.8 t 90.0 ^ d -2.8 t 0.0 d -16.6 w d 16.6 t 90.0");  // 1.5 ish cargo shoot left hab 2
  // right:
  // robot_->SetTestSequence("h 0 d 15.8 t -90.0"); // chargo ship from hab 1 near cargo shot right
  // robot_->SetTestSequence("h 0 d 18.8 t -90.0"); // cargo ship from hab 2 near cargo shot right
  // robot_->SetTestSequence("h 0 d 18.8 t -90.0 ^ d -2.8 t 0.0 d -16.6 w d 16.6 t -90.0");  // 1.5 ish cargo shoot left hab 2
  

  // NASA FIELD
  // robot_->SetTestSequence("h 0 d 12.0 t 90.0 d 3.05 t 0.0 d 2.3"); // left hab 2 to left front hatch deploy included offset, should work
  // robot_->SetTestSequence("h 0 d 4.0 t -90.0 d 4.9 t 0.0 d 4.5 t -23.0 d 1.5"); // rocket near in progress lol left
  
  // robot_->SetTestSequence("h 0 d 19.1 t 90.0"); // cargo ship from hab 2 near cargo shot left
  // robot_->SetTestSequence("h 0 d 16.1 t 90.0"); // chargo ship from hab 1 near cargo shot left
  // robot_->SetTestSequence("h 0 d 19.1 t 90.0 ^ d -2.8 t 0.0 d -17.0 w d 17.0 t 90.0");  // 1.5 ish cargo shoot left hab 2
  // robot_->SetTestSequence(autoModeString);
  // robot_->SetTestSequence("a h 0");

  // robot_->SetTestSequence("h 0 t -90.0");

  // robot_->SetTestSequence("h 0 d 10.0 t 90.0 d 2.8 t 0.0 d 1.3 b 1 s 1.0 h 1"); // left hab 1 to front left
  // if(autoChooserType_.GetBoolean(true)){
    printf("auto sequence is %s from autoChooser (which is being used)\n", autoSendableChooser_.GetSelected().c_str());
  if(autoSendableChooser_.GetSelected()=="h 0"){ //auto 8: other
    printf("using auto8 which is %s\n", auto8Val_.GetString("h 0").c_str());
    robot_->SetTestSequence(auto8Val_.GetString("h 0"));
  } else { //valid chosen auto
    robot_->SetTestSequence(autoSendableChooser_.GetSelected());
  }
  // if(autoSendableChooser_.GetSelected()!=""){
  //   robot_->SetTestSequence(autoSendableChooser_.GetSelected());
  // } else if(autoSendableChooser_.GetSelected()){
  //   printf("WARNING: could not get auto test sequence!  Set auto to h 0 b 0\n");
  //   robot_->SetTestSequence(auto8Val_.GetString("h 0 b 0"));
  // }
  printf("done choosing auto sequence\n");  

  // if(autoChooserType_.GetBoolean(true)){
  //   printf("selected auto: %s\n", autoSendableChooser_.GetSelected());
  //   robot_->SetTestSequence(autoSendableChooser_.GetSelected());
  // }

  // robot_->ZeroNavXYaw();
  autoMode_ = new TestMode(robot_, humanControl_);
  printf("STATS:  %d %d\n", autoMode_==NULL, autoController_==NULL);
  // robot_->ZeroNavXYaw();
  autoController_->SetAutonomousMode(autoMode_);
  autoController_->Init(AutoMode::AutoPositions::kBlank, AutoMode::HabLevel::k1);

  // m_autoSelected = m_chooser.GetSelected();
  // // m_autoSelected = SmartDashboard::GetString(
  // //     "Auto Selector", kAutoNameDefault);
  // std::cout << "Auto selected: " << m_autoSelected << std::endl;

  // if (m_autoSelected == kAutoNameCustom) {
  //   // Custom Auto goes here
  // } else {
  //   // Default Auto goes here
  // }
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
  //printf("is auto mode done %d\n\n", autoMode_->IsDone());
  
  if(sandstormAuto_){
    printf("enter sandstorm auto\n");
    printf("STATS:  %d %d\n", autoMode_==NULL, autoController_==NULL);

    humanControl_->ReadControls();
    autoJoyVal_ = humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY);
    autoJoyVal_ = driveController_->HandleDeadband(autoJoyVal_, driveController_->GetThrustDeadband()); //TODO certain want this deadband?
    printf("checkpoint");
    if(autoJoyVal_ != 0.0 || autoMode_->IsDone() || autoController_->Abort()){ //TODO mild sketch, check deadbands more
      printf("WARNING: EXITED SANDSTORM.\n\n");
      //autoController_->~AutoController(); //TODO check that these are being destructed
      delete autoController_;
      autoController_ = NULL; 
      sandstormAuto_ = false;
      TeleopInit();
    } else if (!autoController_->IsDone()) {
      printf("updating auto controller\n");
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
  aligningTape_ = false;
  navX_ = new NavXPIDSource(robot_);
  curHabPowerDeploy_ = testerPowerNet_.GetDouble(0.6);
  habStartTime_ = -1.0;
  wasJustDeployingHab_ = false;
  wasJustRaisingHab_ = false;
}

// read controls and get current time from controllers
void Robot::TeleopPeriodic() {


  if(!aligningTape_ && humanControl_->GetAlignTapeDesired()){
    printf("in align tape\n");
    aligningTape_ = true;
    aCommand = new AlignWithTapeCommand(robot_, navX_, talonEncoderSource_, false);
    aCommand->Init();
    printf("initing teleop tape align\n");
    // robot_->SetTestSequence("= h 0"); //assuming alignwithtape handles everything including outtake

    // autoMode_ = new TestMode(robot_);
    // autoController_->SetAutonomousMode(autoMode_);
    // autoController_->Init(AutoMode::AutoPositions::kBlank, AutoMode::HabLevel::k1); //TODO super sketch

    return;
  } else if (aligningTape_){
    // printf("in part align tape :))\n");
    humanControl_->ReadControls();
    autoJoyVal_ = humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY);
    autoJoyVal_ = driveController_->HandleDeadband(autoJoyVal_, driveController_->GetThrustDeadband()); //TODO certain want this deadband?
    if(autoJoyVal_ != 0.0 || aCommand == NULL){ //TODO mild sketch, check deadbands more
      printf("WARNING: EXITED align.  autoJoyVal_ is %f after deadband or aCommand is NULL %d\n\n",autoJoyVal_, aCommand==NULL);
      delete aCommand;
      aCommand = NULL;
      aligningTape_ = false;
    } else if(!aCommand->IsDone()){
      aCommand->Update(currTimeSec_, deltaTimeSec_);
      printf("updated align tape command\n");
    } else { //isDone() is true
      delete aCommand;
      aCommand = NULL;
      aligningTape_ = false;
      printf("destroyed align tape command\n");
    }
    return;
  }

  if(humanControl_->GetHabBrakeDesired()){ //hab arms, change this name or put in superstructure
    printf("greetings hab brake has been released\n");
    testHabPiston->Set(DoubleSolenoid::kForward);
  } else if(humanControl_->GetHabArmsRetractDesired()){
    testHabPiston->Set(DoubleSolenoid::kReverse);
  }
  /*else {
    testHabPiston->Set(DoubleSolenoid::kReverse);
  }*/

  if(humanControl_->GetTestDesired() && habLimitSwitch_->Get()){ //NOTE IMPORTANT TODO if delete, reenable the one commented out in superstructure and add a backwards //habdeploy
    //printf("\n\n\n hab limit is %f \n\n", habLimitSwitch_->Get());
    // robot_->SetHabBrake(false);
    if(!wasJustDeployingHab_){
      habStartTime_ = robot_->GetTime();
    } else if(robot_->GetTime()-habStartTime_ >= 0.5){
      robot_->SetHabBrake(false);      
    }

    if (robot_->GetHabEncoderValue() >= 202.0) { //5 val slippage, so ends at val 207
      // printf("high enough...stopping hab retract\n");
      robot_->SetHabMotorOutput(0.0);
      wasJustRaisingHab_ = false;
      wasJustDeployingHab_ = false;
    } else {
      if(!wasJustDeployingHab_){
        curHabPowerDeploy_ = testerPowerNet_.GetDouble(0.6);
      }
      robot_->SetHabMotorOutput(curHabPowerDeploy_);
      printf("Hab downing at %f power after accel.\n", curHabPowerDeploy_);
      //TODO reach 1.0 eventually
      if(/*robot_->GetTime() - habStartTime_ > 2 && */curHabPowerDeploy_*habDeployAccelNet_.GetDouble(1.001) <= 1.0){ //2 sec slow deploy, then accel
        curHabPowerDeploy_ *= habDeployAccelNet_.GetDouble(1.001); //exponential increase
      } else if (/*robot_->GetTime() - habStartTime_ > 2 && */curHabPowerDeploy_ >= 1.0){
        curHabPowerDeploy_ = 1.0;
      }
      wasJustRaisingHab_ = false;
      wasJustDeployingHab_ = true;
    }  
  } else if (humanControl_->GetTest3Desired()){ /* && robot_->GetHabEncoderValue() >= 4.0*/ // TODO TEST
    if (robot_->GetHabEncoderValue() <= 10.0) {
      // printf("high enough...stopping hab retract\n");
      robot_->SetHabMotorOutput(0.0);
      wasJustRaisingHab_ = false;
      wasJustDeployingHab_ = false;
    } else {
      if(!wasJustRaisingHab_){
         curHabPowerDeploy_ = -habRisePowerNet_.GetDouble(0.9);
      }
      robot_->SetHabMotorOutput(curHabPowerDeploy_);
      printf("Hab rising at %f power.\n", curHabPowerDeploy_);
      if (curHabPowerDeploy_ * habRaiseAccelNet_.GetDouble(1.0005) >= -1.0){ //TODO reach 1.0 eventually
        curHabPowerDeploy_ *= habRaiseAccelNet_.GetDouble(1.0005);
      } else if (curHabPowerDeploy_<= -1.0){
        curHabPowerDeploy_ = -1.0;
      }
      wasJustRaisingHab_ = true;
      wasJustDeployingHab_ = false;
    }
  } else {
    robot_->SetHabMotorOutput(0.0);
    curHabPowerDeploy_ = 0.0;
    wasJustRaisingHab_ = false;
    wasJustDeployingHab_ = false;

    //if(habStartTime_ > 0.0){ //sketch code, change this to using wasJust
    habStartTime_ = -1.0;
    //}
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