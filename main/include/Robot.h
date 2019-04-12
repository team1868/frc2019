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
#include "../include/controllers/GuidedDriveController.h"
#include "../include/controllers/SuperstructureController.h"
#include "../include/auto/PIDSource/PIDInputSource.h"
#include "../include/auto/commands/DriveStraightCommand.h"
#include "../include/auto/commands/EllipseCommand.h"
#include "../include/auto/commands/PivotCommand.h"
#include "../include/auto/commands/CurveCommand.h"
#include "auto/AutoController.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include "rev/CANSparkMax.h"

class Robot : public frc::TimedRobot {
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
  NavXPIDSource *navX_;

  DoubleSolenoid *testHabPiston;
  DigitalInput *habLimitSwitch_;


  double currTimeSec_;
	double lastTimeSec_;
	double deltaTimeSec_;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto"; //TODO: Delete?? idk whatevs
  std::string m_autoSelected;

  GuidedDriveController *guidedDriveController_;
  DriveStraightCommand *driveStraight_;//TODO delete
  EllipseCommand *ellipse_;
  bool guidedDrive_;
  PivotCommand *pivot_; //testing TODO delte
  CurveCommand *curve_; //TODO DELETE
  bool aligningTape_;
  //double autoStartTime;

  bool sandstormOverride_;
  double autoJoyVal_;

  AutoController *autoController_;
  AutoMode *autoMode_;

  AlignWithTapeCommand *aCommand;

  int curvesDone;// delete plz
  bool sandstormAuto_;

  double curHabPowerDeploy_;
  double habStartTime_;
  bool wasJustRaisingHab_, wasJustDeployingHab_;

  nt::NetworkTableEntry leftEncoderNet_, rightEncoderNet_, leftEncoderStopNet_, rightEncoderStopNet_,
    testerPowerNet_, guidedDriveNet_, habRisePowerNet_, leftDistanceNet_, rightDistanceNet_, sparkEncoderNet_,
    habRaiseAccelNet_, habDeployAccelNet_;
  // nt::NetworkTableEntry usingOffset_, side_, redShipHorizOffset_, redShipVertiOffset_, redRocketHorizOffset_, redRocketVertiOffset_,
  //   blueShipHorizOffset_, blueShipVertiOffset_, blueRocketHorizOffset_, blueRocketVertiOffset_;

  uint64_t auto0Change_, auto1Change_, auto2Change_, auto3Change_, auto4Change_, auto5Change_, auto6Change_, auto7Change_, auto8Change_;
  frc::SendableChooser<std::string> autoSendableChooser_;
  nt::NetworkTableEntry autoChooser_, autoChooserType_, auto0_, auto1_, auto2_, auto3_, auto4_, auto5_, auto6_, auto7_, auto8_, auto8Val_;
};
