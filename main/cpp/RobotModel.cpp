/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include "Ports2019.h"
//#include <TalonSRX.h>
//#include <VictorSPX.h>
//#include <ctre/phoenix/motorcontrol/can/TalonSRX>
//#include "CANTalon.h"

const double WHEEL_DIAMETER = 4.0; //Is this Right?
const double ENCODER_COUNT_PER_ROTATION = 256.0;
const int EDGES_PER_ENCODER_COUNT = 4;

RobotModel::RobotModel() {

  //pini_ = new Ini("home/lvuser/robot.ini");
  //RefreshIni();

  leftMaster_ = new WPI_TalonSRX(LEFT_DRIVE_MASTER_ID);
  rightMaster_ = new WPI_TalonSRX(RIGHT_DRIVE_MASTER_ID);
  leftSlave_ = new WPI_VictorSPX(LEFT_DRIVE_SLAVE_ID);
  rightSlave_ = new WPI_VictorSPX(RIGHT_DRIVE_SLAVE_ID);

  leftMaster_->Set(ControlMode::PercentOutput, 0.0);
  rightMaster_->Set(ControlMode::PercentOutput, 0.0);
  leftSlave_->Follow(*leftMaster_);
  rightSlave_->Follow(*rightMaster_);
}
