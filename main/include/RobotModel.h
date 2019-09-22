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
#include <networktables/NetworkTableEntry.h>

#include "../include/auto/PIDSource/PIDInputSource.h"
#include "../include/auto/PIDSource/PIDOutputSource.h"
//#include "../ext/ini/ini.h"
//#include <TalonSRX.h>
//#include <VictorSPX.h>
//#include "CANTalon.h"
#include "rev/CANSparkMax.h"

#define PI 3.141592

static const double WHEEL_DIAMETER = 4.0 / 12.0; //ft
static const double HIGH_GEAR_ENCODER_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*32/34; //ft
static const double LOW_GEAR_ENCODER_ROTATION_DISTANCE = WHEEL_DIAMETER*PI*16/50;
static const double ENCODER_COUNT_PER_ROTATION = 256.0;
static const int EDGES_PER_ENCODER_COUNT = 4;

static const double MAX_VELOCITY = 5.75; //low gear ft/s

static const double FLYWHEEL_DIAMETER = 1.0 / 12.0; //CHECK (in ft)

static const double MAX_CURRENT_OUTPUT = 180.0; //Amps //TODO INCORRECT< FIX
static const double MAX_DRIVE_MOTOR_CURRENT = 40.0; //Amps
//ratios work in 5 or 10% increments (accumulative)
static const double MIN_RATIO_ALL_CURRENT = 0.2;//0.7; //TODO add to shuffleboard
static const double MIN_RATIO_DRIVE_CURRENT = 0.7; //TODO add to shuffleboard //NOTE: UNUSED
static const double MIN_RATIO_SUPERSTRUCTURE_CURRENT = 0.5; //TODO add to shuffleboard
static const double MIN_VOLTAGE_BROWNOUT = 7.5;//7.5; //6.8; //brownout protection state; PWM, CAN, 6V, relay outputs, CAN motors disabled

//unused
static const double MAX_CURRENT_DRIVE_PERCENT = 0.8; //per motor, most teams are 40-50 Amps

//currently tuned for Mo Practice Bot
static double LOW_GEAR_STATIC_FRICTION_POWER = 0.06;//0.11;
static double HIGH_GEAR_STATIC_FRICTION_POWER = 0.09;//0.14;
static double LOW_GEAR_QUICKTURN_ADDITIONAL_STATIC_FRICTION_POWER =  /*0.15*/0.09 - LOW_GEAR_STATIC_FRICTION_POWER;
static double HIGH_GEAR_QUICKTURN_ADDITIONAL_STATIC_FRICTION_POWER = /*0.2*/0.19 - HIGH_GEAR_STATIC_FRICTION_POWER;

class RobotModel {
 public:
  enum Wheels {kLeftWheels, kRightWheels, kAllWheels};
  enum Talons {kLeftMaster, kRightMaster};
  
  //constructor
  RobotModel();

  //Destructor
  ~RobotModel();

  void ResetTimer(); //TODO: implement ALL TIME
  double ModifyCurrent(int channel, double value);
  double CheckMotorCurrentOver(int channel, double power);
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
  void SetDriveValues(double leftValue, double rightValue);
  void SetTalonBrakeMode(); //TODO
  void SetTalonCoastMode(); //TODO

  void SetHighGear(); //TODO
  void SetLowGear(); //TODO
  bool IsHighGear();

  double GetLeftEncoderValue(); //TODO
  double GetRightEncoderValue(); //TODO

  double GetLeftDistance(); //TODO
  double GetRightDistance(); //TODO

  double GetLeftDistancePerPulse();
  double GetRightDistancePerPulse();

  double GetLeftEncodingScale();
  double GetRightEncodingScale();

  bool GetLeftEncoderStopped(); //TODO
  bool GetRightEncoderStopped(); //TODO

  void ResetDriveEncoders(); //TODO
  
  double GetNavXYaw(); //TODO
  double GetNavXPitch(); //TODO
  double GetNavXRoll(); //TODO
  
  void ZeroNavXYaw(); //TODO
  
  //---------------SUPERSTRUCTURE------------------
  void ResetGyro();
  void CalibrateGyro();
  double GetGyroAngle();
  int GetHabEncoderValue();
  bool GetLightSensorStatus();
  
  void SetCargoIntakeOutput(double output);
  void SetCargoUnintakeOutput(double output);
  void SetCargoFlywheelOutput(double output);
  void SetHatchWristOutput(double output);
  void SetHatchIntakeWheelOutput(double output);
  void SetHabMotorOutput(double output);

  void SetCargoIntakeWrist(bool change);
  void SetHatchOuttake(bool change);
  void SetHatchBeak(bool change);
  void SetHabBrake(bool change);
  void EngageHook();
  void DisengageHook();

  double GetCargoFlywheelMotorOutput(double output);

  //blahslkdjfslkdjflksjdfklsj
  Encoder* GetCargoFlywheelEncoder();
  Victor* GetCargoFlywheelMotor();

  AnalogGyro* GetGyro();

  Victor* GetHatchWristMotor();

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

  std::string GetTestSequence();
  void SetTestSequence(std::string testSequence);

  NavXPIDSource* GetNavXSource();
  void CreateNavX();

 private:

  double GetStaticFriction(double thrustValue);
  double HandleStaticFriction(double value, double thrustValue);
 
  GameMode currentGameMode_;

  frc::ShuffleboardTab &tab_;

  //Ini *pini_;
  double pivotPFac_, pivotIFac_, pivotDFac_; //TODO
	double driveDPFac_, driveDIFac_, driveDDFac_; //TODO

  double leftDriveOutput_, rightDriveOutput_;
  float last_world_linear_accel_x_, last_world_linear_accel_y_;
  double driveCurrentLimit_, intakeCurrentLimit_, totalCurrentLimit_, voltageFloor_, pressureFloor_, size_;
  double leftDriveACurrent_, leftDriveBCurrent_, leftDriveCCurrent_, rightDriveACurrent_, rightDriveBCurrent_, rightDriveCCurrent_,
		roboRIOCurrent_, compressorCurrent_, leftIntakeCurrent_, rightIntakeCurrent_;

  //Power Distribution
  bool lastOver_;
  double ratioAll_, ratioDrive_, ratioSuperstructure_;
  bool compressorOff_;

  int habRailsEncoderVal_;
  
  //bool cutSlaves_; //remove one slave from drive

  int autoPos_, autoMode_; //TODO
  std::string testMode_; //TODO

  WPI_TalonSRX *leftMaster_, *rightMaster_; //TODO this is weird, frc:: or not?
  WPI_VictorSPX *leftSlaveA_, *leftSlaveB_, *rightSlaveA_, *rightSlaveB_;

  Victor *cargoIntakeMotor_, *cargoFlywheelMotor_, *hatchIntakeWheelMotor_, *hatchWristMotor_;
  rev::CANSparkMax *habSparkMotor_;
  //rev::CANEncoder *habSparkMotor_;
  Encoder *cargoFlywheelEncoder_;
  DoubleSolenoid *cargoIntakeWristSolenoid_, *hatchBeakSolenoid_, *habBrakeSolenoid_;
  Solenoid *hatchOuttakeSolenoid_, *hookSolenoid_;

  frc::Timer *timer_;
  AHRS *navX_; 
  AnalogGyro *gyro_; //gyro for hatch wrist
  DigitalInput *lightSensor_; //limit switch for cargo
  int navXSpeed_; //in Hz

  frc::Compressor *compressor_; //TODO
  frc::Solenoid *gearShiftSolenoid_;
  frc::Encoder *leftDriveEncoder_, *rightDriveEncoder_; //TODO
  frc::PowerDistributionPanel *pdp_; //TODO

  bool highGear_, cargoWristEngaged_, hatchOuttakeEngaged_, hatchBeakEngaged_;

  std::string testSequence_;
  NavXPIDSource *navXSource_;

  nt::NetworkTableEntry jerkYNet_, jerkXNet_, leftDistanceNet_, rightDistanceNet_, yawNet_, pitchNet_, rollNet_, pressureNet_,
    dPFacNet_, dIFacNet_, dDFacNet_, rPFacNet_, rIFacNet_, rDFacNet_, pivotPFacNet_, pivotIFacNet_, pivotDFacNet_,
    ratioAllNet_, ratioDriveNet_, ratioSuperNet_, maxOutputNet_, minVoltNet_, maxCurrentNet_, lowGearStaticFric_, lowGearTurnStaticFric_, highGearStaticFric_, highGearTurnStaticFric_;
};
