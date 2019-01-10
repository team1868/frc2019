/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
//#include "../ext/ini/ini.h"
//#include <TalonSRX.h>
//#include <VictorSPX.h>
//#include "CANTalon.h"

class RobotModel {
 public:
  enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
  enum Talons {kLeftMaster, kRightMaster};
  
  //constructor
  RobotModel();

  //Destructor
  ~RobotModel();

  void ResetTimer(); //TODO: implement ALL TIME
  double GetTime();

  //---------------DRIVE------------------
  WPI_TalonSRX *GetTalon(Talons talon); //TODO

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

 private:
  //Ini *pini_;
  double pivotPFac_, pivotIFac_, pivotDFac_; //TODO
	double driveDPFac_, driveDIFac_, driveDDFac_; //TODO

  double leftDriveOutput_, rightDriveOutput_;
  double driveCurrentLimit_, intakeCurrentLimit_, totalCurrentLimit_, voltageFloor_, pressureFloor_, size_;
  double leftDriveACurrent_, leftDriveBCurrent_, rightDriveACurrent_, rightDriveBCurrent_,
		roboRIOCurrent_, compressorCurrent_, leftIntakeCurrent_, rightIntakeCurrent_;

  int autoPos_, autoMode_; //TODO
  //string testMode_; //TODO

  WPI_TalonSRX *leftMaster_, *rightMaster_;
  WPI_VictorSPX *leftSlave_, *rightSlave_;

  //Timer *timer_; //TODO
  //AHRS *navX_; //TODO, --------------------------WARNING ahrs?
  int navXSpeed_; //in Hz

  //Compressor *compressor_; //TODO
  //DoubleSolenoid *gearShiftSolenoid_; //TODO
  //Encoder *leftDriveEncoder_, *rightDriveEncoder_; //TODO
  //PowerDistributionPanel *pdp_; //TODO
  
};
