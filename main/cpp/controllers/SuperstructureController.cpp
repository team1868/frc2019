/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../include/controllers/SuperstructureController.h"

SuperstructureController::SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl) {
    robot_ = myRobot;
	humanControl_ = myHumanControl;

	currState_ = kInit;
	nextState_ = kIdle;

    cargoIntakeOutput_ = 1.0; //TODO PID
    cargoFlywheelOutput_ = 0.5; //TODO PID, also, btw not using

    hatchPickupEngaged_ = false;

    //TODO INIT highgear disengaged whatever whatever (like last year's wrist engage asap)
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

    robot_->SetCargoIntakeOutput(0.0);
    robot_->SetCargoFlywheelOutput(0.0);
    //TODO add here @katie
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	switch(currState_) {
        case kInit:
            nextState_ = kIdle;
            robot_->SetCargoIntakeOutput(0.0);
            robot_->SetCargoFlywheelOutput(0.0);
            break;
        case kIdle:
            nextState_ = kIdle;
            if(humanControl_->GetCargoIntakeDesired()){
                printf("cargo intake activated\n");
                robot_->SetCargoIntakeOutput(cargoIntakeOutput_);
            } else if(humanControl_->GetCargoUnintakeDesired()){
                robot_->SetCargoUnintakeOutput(cargoIntakeOutput_);
                printf("cargo unintake activated\n");
            } else {
                robot_->SetCargoIntakeOutput(0.0);
            }
            if(humanControl_->GetHatchPickupEngageDesired() && !hatchPickupEngaged_){
                hatchPickupEngaged_ = true;
                robot_->SetHatchDoubleSolenoid(true); //TODO engage/diengage hatch at constructor
            } else if(!humanControl_->GetHatchPickupEngageDesired() && hatchPickupEngaged_){
                hatchPickupEngaged_ = false;
                robot_->SetHatchDoubleSolenoid(false);
            }
            printf("hatch engaged : %d\n", hatchPickupEngaged_);

            robot_->SetCargoFlywheelOutput(humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kLT));
            printf("cargo flywheel at %d speed\n", humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kLT));
           /* if cargo flywheel button
            if(humanControl_->GetCargoFlywheelDesired()){
                printf("Cargo Flywheel activated\n");
                robot_->SetCargoFlywheelOutput(cargoFlywheelOutput_);
                 } else {
                robot_->SetCargoFlywheelOutput(0.0);
            }
            */

            if(humanControl_->GetHighGearDesired()){
                robot_->SetHighGear();
            } else {
                robot_->SetLowGear();
            }
            break;
        //TODO add here @katie
        default:
            printf("WARNING: State not found in SuperstructureController::Update()\n");
    }
	currState_ = nextState_;
}

void SuperstructureController::RefreshIni() { //TODO remove

}

SuperstructureController::~SuperstructureController() {
}