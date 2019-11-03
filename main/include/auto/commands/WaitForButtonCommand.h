/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "RobotModel.h"
#include "ControlBoard.h"
#include "../AutoCommand.h"

class WaitForButtonCommand : public AutoCommand {
 public:
  WaitForButtonCommand(ControlBoard *controlBoard);
  virtual ~WaitForButtonCommand();
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();
 private:
	bool isDone_;
  ControlBoard *humanControl_;
<<<<<<< HEAD
};
=======
};
>>>>>>> 8ad727fb82cd4b6206784afb75a1bbd5407cdc10
