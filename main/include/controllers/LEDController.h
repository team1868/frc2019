/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*#pragma once

#ifndef SRC_CONTROLLERS_LEDCONTROLLER_H_
#define SRC_CONTROLLERS_LEDCONTROLLER_H_

#include "RobotModel.h"
#include "ControlBoard.h"

class LEDController {
public:
    LEDController(RobotModel *myRobot, ControlBoard *myHumanControl);
    
    virtual ~LEDController();
};

 private:
    RobotModel *robot_;
	ControlBoard *humanControl_;
    Solenoid *LEDLight_;
};  

#endif /* SRC_CONTROLLERS_LEDCONTROLLER_H_ */ 


