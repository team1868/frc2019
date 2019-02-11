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

    desiredFlywheelVelocCargo_ = 0.56; //tune 
    desiredFlywheelVelocRocket_ = 0.25; //tune

    cargoVelocNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("cargo veloc", 0.56).GetEntry(); //0.56
    cargoRocketVelocNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("rocket veloc", 0.25).GetEntry(); //0.25

    cargoIntakeOutput_ = 1.0; 

    flywheelStartTime_ = 0.0;
    flywheelStarted_ = false;

    cargoIntakeWristEngaged_ = false;
    hatchOuttakeEngaged_ = false;
    hatchPickupEngaged_ = false;

    //get PID vals for cargo ship
    cargoPFac_ = 0.8;
    cargoIFac_ = 0.0;
    cargoDFac_ = 0.2;

    //get PID vals for rocket ship
    rocketPFac_ = 0.8;
    rocketIFac_ = 0.0;
    rocketDFac_ = 0.2;

    cargoPNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("cargo P", 0.8).GetEntry();
    cargoINet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("cargo I", 0.0).GetEntry();
    cargoDNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("cargo D", 0.2).GetEntry();

    rocketPNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("rocket P", 0.8).GetEntry();
    rocketINet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("rocket I", 0.0).GetEntry();
    rocketDNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("rocket D", 0.2).GetEntry();

    //shuffleboard PID values
    cargoFlyPID_  = new PIDController(cargoPFac_, cargoIFac_, cargoDFac_, robot_->GetCargoFlywheelEncoder(),
        robot_->GetCargoFlywheelMotor());
    cargoFlyPID_->SetSetpoint(desiredFlywheelVelocCargo_);  
    cargoFlyPID_->SetOutputRange(0.0, 1.0); //TODO
    cargoFlyPID_->SetAbsoluteTolerance(0.05); //TODO
    cargoFlyPID_->SetContinuous(false);

    rocketFlyPID_ = new PIDController(rocketPFac_, rocketIFac_, rocketDFac_, robot_->GetCargoFlywheelEncoder(),
        robot_->GetCargoFlywheelMotor());
    rocketFlyPID_->SetSetpoint(desiredFlywheelVelocRocket_);  
    rocketFlyPID_->SetOutputRange(0.0, 1.0); //TODO
    rocketFlyPID_->SetAbsoluteTolerance(0.05); //TODO
    rocketFlyPID_->SetContinuous(false);

    //TODO INIT highgear disengaged whatever whatever (like last year's wrist engage asap)
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

    cargoFlyPID_->Reset();
    rocketFlyPID_->Reset();

    robot_->SetCargoIntakeOutput(0.0);
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {

    SetOutputs(); 
    RefreshShuffleboard();

	switch(currState_) {
        case kInit:
            cargoFlyPID_->Reset();
            cargoFlyPID_->Disable();
            
            rocketFlyPID_->Reset();
            rocketFlyPID_->Disable();

            robot_->SetCargoIntakeOutput(0.0);
            robot_->SetCargoFlywheelOutput(0.0);

            nextState_ = kIdle;
        case kIdle:
            nextState_ = kIdle;

            //HATCH STUFF
            if(humanControl_->GetHatchPickupDesired()){ //toggle button
                if(!hatchPickupEngaged_){
                    robot_->SetHatchPickup(true); //TODO engage/diengage hatch at constructor
                } else {
                    robot_->SetHatchPickup(false);
                }
                hatchPickupEngaged_ = !hatchPickupEngaged_;
                printf("hatch Pickup engaged : %d\n", hatchPickupEngaged_);
            }

            if(humanControl_->GetHatchBeakDesired()){
                robot_->SetHatchBeak(true);
            }

            if(humanControl_->GetHatchOuttakeDesired()){
                hatchOuttakeEngaged_ = !hatchOuttakeEngaged_;
                if(hatchOuttakeEngaged_){
                    robot_->SetHatchOuttake(true);
                } else {
                    robot_->SetHatchOuttake(false);
                }
                robot_->SetHatchOuttake(true);
            }

            //CARGO STUFF

            if(humanControl_->GetCargoIntakeWristDesired()){
                cargoIntakeWristEngaged_ = !cargoIntakeWristEngaged_;
                if(cargoIntakeWristEngaged_){
                    robot_->SetCargoIntakeWrist(true);
                } else {
                    robot_->SetCargoIntakeWrist(false);
                }
            }

            if(humanControl_->GetCargoIntakeDesired()){
                printf("cargo intaking\n");
                robot_->SetCargoIntakeOutput(cargoIntakeOutput_);
            } else if(humanControl_->GetCargoUnintakeDesired()){
                robot_->SetCargoUnintakeOutput(cargoIntakeOutput_);
                printf("cargo unintaking\n");
            } else {
                robot_->SetCargoIntakeOutput(0.0);
            }

            if(humanControl_->GetCargoFlywheelDesired()){ //flywheel for cargo ship
                cargoFlyPID_->Enable();
                nextState_ = kFlywheelCargo;
            }
            if(humanControl_->GetCargoFlywheelDesiredRocket()){ //flywheel for rocket ship
                rocketFlyPID_->Enable();
                nextState_ = kFlywheelRocket;
            }

           /* robot_->SetCargoFlywheelOutput(humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kLT));
           // printf("cargo flywheel at %d speed\n", humanControl_->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kLT));
           // if cargo flywheel button
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
        case kFlywheelCargo:
            printf("outtaking cargo into cargo ship bay\n");

            if(!flywheelStarted_){
                flywheelStartTime_ = robot_->GetTime();
                flywheelStarted_ = true;
                nextState_ = kFlywheelCargo;
            } else if(robot_->GetTime() - flywheelStartTime_ < 1.5){ //test for time delay
                nextState_ = kFlywheelCargo;
                robot_->SetHood(true);
            } else if(humanControl_->GetCargoFlywheelDesired()){
                robot_->SetCargoIntakeOutput(cargoIntakeOutput_);
				nextState_ = kFlywheelCargo;
            } else {
                robot_->SetCargoIntakeOutput(0.0);
				cargoFlyPID_->Disable();
				flywheelStarted_ = false;
				nextState_ = kIdle;
            }
            break;
        case kFlywheelRocket:
            printf("outtaking cargo into rocket\n");

            if(!flywheelStarted_){
                flywheelStartTime_ = robot_->GetTime();
                flywheelStarted_ = true;
                nextState_ = kFlywheelRocket;
            } else if(robot_->GetTime() - flywheelStartTime_ < 2){ //test for time delay
                nextState_ = kFlywheelRocket;
                robot_->SetHood(false);
            } else if(humanControl_->GetCargoFlywheelDesiredRocket()){
                robot_->SetCargoIntakeOutput(cargoIntakeOutput_);
				nextState_ = kFlywheelRocket;
            } else {
                robot_->SetCargoIntakeOutput(0.0);
				cargoFlyPID_->Disable();
				flywheelStarted_ = false;
				nextState_ = kIdle;
            }
            break;
        default:
            printf("WARNING: State not found in SuperstructureController::Update()\n");
    }
	currState_ = nextState_;
}

void SuperstructureController::SetOutputs() {
    cargoFlyPID_->SetSetpoint(desiredFlywheelVelocCargo_);
}

void SuperstructureController::RefreshShuffleboard() {
    desiredFlywheelVelocCargo_ = cargoVelocNet_.GetDouble(0.56);
    desiredFlywheelVelocRocket_ = cargoRocketVelocNet_.GetDouble(0.25);

    cargoPFac_ = cargoPNet_.GetDouble(0.8);
    cargoIFac_ = cargoINet_.GetDouble(0.0);
    cargoDFac_ = cargoDNet_.GetDouble(0.2);

    rocketPFac_ = rocketPNet_.GetDouble(0.8);
    rocketIFac_ = rocketINet_.GetDouble(0.0);
    rocketDFac_ = rocketDNet_.GetDouble(0.2);
}

void SuperstructureController::RefreshIni() { //TODO remove

}

SuperstructureController::~SuperstructureController() {
}