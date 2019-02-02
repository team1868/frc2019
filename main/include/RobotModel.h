/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "frc/WPILib.h" //TODO <>
#include <string>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
//#include "frc/smartdashboard/SmartDashboard.h"
//#include "frc/Encoder.h"
#include <AHRS.h>
#include "ctre/Phoenix.h" //TODO <>
#include <NetworkTables/NetworkTableEntry.h>
//#include "../ext/ini/ini.h"
//#include <TalonSRX.h>
//#include <VictorSPX.h>
//#include "CANTalon.h"

static const double MAX_CURRENT_OUTPUT = 180.0; //Amps //TODO INCORRECT< FIX
static const double MIN_VOLTAGE_BROWNOUT = 6.8; //brownout protection state; PWM, CAN, 6V, relay outputs, CAN motors disabled
static const double MAX_CURRENT_DRIVE_PERCENT = 0.8; //per motor, most teams are 40-50 Amps

class RobotModel {
 public:
  enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
  enum Talons {kLeftMaster, kRightMaster};
  
  //constructor
  RobotModel();

  //Destructor
  ~RobotModel();

  void ResetTimer(); //TODO: implement ALL TIME
  void UpdateCurrent();
  double GetTime();

  double GetVoltage();
  double GetTotalCurrent();
  double GetTotalEnergy();
  double GetTotalPower();
  double GetCurrent(int channel);
  double GetCompressorCurrent();
  double GetRIOCurrent();
  double GetPressureSensorVal();

  void StopCompressor();
  void StartCompressor();

  void PrintState();

  //---------------DRIVE------------------
  WPI_TalonSRX *GetTalon(Talons talon); //TODO

  bool CollisionDetected();

  double GetWheelSpeed(Wheels wheel);

  void SetDriveValues(Wheels wheel, double value); //TODO
  void SetTalonBrakeMode(); //TODO
  void SetTalonCoastMode(); //TODO

  void SetHighGear(); //TODO
  void SetLowGear(); //TODO

  double GetLeftEncoderValue(); //TODO
  double GetRightEncoderValue(); //TODO

  double GetLeftDistance(); //TODO
  double GetRightDistance(); //TODO

  bool GetLeftEncoderStopped(); //TODO
  bool GetRightEncoderStopped(); //TODO

  void ResetDriveEncoders(); //TODO
  
  double GetNavXYaw(); //TODO
  double GetNavXPitch(); //TODO
  double GetNavXRoll(); //TODO

  void ZeroNavXYaw(); //TODO

  enum GameMode{SANDSTORM, NORMAL_TELEOP};
  
  RobotModel::GameMode GetGameMode();
  double GetDPFac();
  double GetDIFac();
  double GetDDFac();

  double GetRPFac();
  double GetRIFac();
  double GetRDFac();

  double GetPivotPFac();
  double GetPivotIFac();
  double GetPivotDFac();

 private:
 
  GameMode currentGameMode_;

  frc::ShuffleboardTab &tab_;

  //Ini *pini_;
  double pivotPFac_, pivotIFac_, pivotDFac_; //TODO
	double driveDPFac_, driveDIFac_, driveDDFac_; //TODO

  double leftDriveOutput_, rightDriveOutput_;
  float last_world_linear_accel_x_, last_world_linear_accel_y_;
  double driveCurrentLimit_, intakeCurrentLimit_, totalCurrentLimit_, voltageFloor_, pressureFloor_, size_;
  double leftDriveACurrent_, leftDriveBCurrent_, rightDriveACurrent_, rightDriveBCurrent_,
		roboRIOCurrent_, compressorCurrent_, leftIntakeCurrent_, rightIntakeCurrent_;

  int autoPos_, autoMode_; //TODO
  std::string testMode_; //TODO

  WPI_TalonSRX *leftMaster_, *rightMaster_;
  WPI_VictorSPX *leftSlave_, *rightSlave_;

  frc::Timer *timer_;
  AHRS *navX_; 
  int navXSpeed_; //in Hz

  frc::Compressor *compressor_; //TODO
  frc::DoubleSolenoid *gearShiftSolenoid_; //TODO
  frc::Encoder *leftDriveEncoder_, *rightDriveEncoder_; //TODO
  frc::PowerDistributionPanel *pdp_; //TODO

  nt::NetworkTableEntry jerkYNet_, jerkXNet_, leftDistanceNet_, rightDistanceNet_, yawNet_, pitchNet_, rollNet_, pressureNet_,
    dPFacNet_, dIFacNet_, dDFacNet_, rPFacNet_, rIFacNet_, rDFacNet_, pivotPFacNet_, pivotIFacNet_, pivotDFacNet_;
};
