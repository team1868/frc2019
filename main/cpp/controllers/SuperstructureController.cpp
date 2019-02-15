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

    desiredFlywheelVelocCargo_ = 0.7; //TODO this is /1 not actual velocity
    desiredFlywheelVelocRocket_ = 0.25; //TODO this is /1 not actual velocity

    cargoVelocNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("cargo veloc", 0.7).GetEntry(); //0.7
    cargoRocketVelocNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("rocket veloc", 0.25).GetEntry(); //0.25

    cargoIntakeOutput_ = 1.0; 

    flywheelStartTime_ = 0.0;
    flywheelStarted_ = false;

    cargoIntakeWristEngaged_ = false;
    hatchOuttakeEngaged_ = false;

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
    cargoFlyPID_->SetOutputRange(-1.0, 1.0); //TODO
    cargoFlyPID_->SetAbsoluteTolerance(0.05); //TODO
    cargoFlyPID_->SetContinuous(false);

    rocketFlyPID_ = new PIDController(rocketPFac_, rocketIFac_, rocketDFac_, robot_->GetCargoFlywheelEncoder(),
        robot_->GetCargoFlywheelMotor());
    rocketFlyPID_->SetSetpoint(desiredFlywheelVelocRocket_);  
    rocketFlyPID_->SetOutputRange(-1.0, 1.0); //TODO
    rocketFlyPID_->SetAbsoluteTolerance(0.05); //TODO
    rocketFlyPID_->SetContinuous(false);

    //TODO INIT highgear disengaged whatever whatever (like last year's wrist engage asap)
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

    cargoFlyPID_->Reset();
    rocketFlyPID_->Reset();

    RefreshShuffleboard();

    robot_->SetCargoIntakeOutput(0.0);
    robot_->SetHatchIntakeWheelOutput(0.0);
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {

    SetOutputs(); 
    
	switch(currState_) {
        case kInit:
            cargoFlyPID_->Reset();
            cargoFlyPID_->Disable();
            
            rocketFlyPID_->Reset();
            rocketFlyPID_->Disable();

            robot_->SetCargoIntakeOutput(0.0); 
            robot_->SetCargoFlywheelOutput(0.0);
            robot_->SetHatchIntakeWheelOutput(0.0);

            nextState_ = kIdle;
        case kIdle:
            nextState_ = kIdle;
            //HATCH STUFF
            
            if(humanControl_->GetHatchIntakeWheelDesired()){
                printf("hatch intaking\n");
                robot_->SetHatchIntakeWheelOutput(0.8);
            } else if (humanControl_->GetHatchUnintakeWheelDesired()){
                printf("hatch intaking\n");
                robot_->SetHatchIntakeWheelOutput(-0.8);
            } else {
                robot_->SetHatchIntakeWheelOutput(0.8);
            }


            //TODO INTEGRATE GYRO
            if (humanControl_->GetHatchWristUpDesired()) {
			    printf("hatch intake up\n");
			    robot_->SetHatchWristOutput(-0.3);
            } else if (humanControl_->GetHatchWristDownDesired()) {
                printf("hatch intake down\n");
			    robot_->SetHatchWristOutput(0.3);
		    } else {
			    robot_->SetHatchWristOutput(0.0);
	    	}

            if(humanControl_->GetHatchBeakDesired()){
                robot_->SetHatchBeak(true);
            } else {
                robot_->SetHatchBeak(false);
            }

            if(humanControl_->GetHatchOuttakeDesired()){ //TODO different state: time delay
                robot_->SetHatchOuttake(true);
            } else {
                robot_->SetHatchOuttake(false);
            }

            //CARGO STUFF

            if(humanControl_->GetCargoIntakeWristDesired()){
                robot_->SetCargoIntakeWrist(true);
            } else {
                robot_->SetCargoIntakeWrist(false);
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
                printf("cargo shooting into cargo ship\n");
                /*if(!flywheelStarted_){
                    flywheelStarted_ = true;
                    cargoFlyPID_->Enable();
                }*/
                robot_->SetCargoFlywheelOutput(desiredFlywheelVelocCargo_);
            } else {
                /*
                cargoFlyPID_->Disable(); 
                flywheelStarted_ = false;
                */
               robot_->SetCargoFlywheelOutput(0.0);
            }

            if(humanControl_->GetCargoFlywheelDesiredRocket()){ //flywheel for rocket ship
                printf("cargo shooting into rocket ship\n");
                /*
                if(!flywheelStarted_){
                    rocketFlyPID_->Enable();
                    flywheelStarted_=true;
                }*/
                robot_->SetCargoFlywheelOutput(desiredFlywheelVelocRocket_);
            } else {
                /*rocketFlyPID_->Disable(); 
                flywheelStarted_ = false;*/
                robot_->SetCargoFlywheelOutput(0.0);
            }

            if(humanControl_->GetHighGearDesired()){
                robot_->SetHighGear();
            } else {
                robot_->SetLowGear();
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

            break;
        default:
            printf("WARNING: State not found in SuperstructureController::Update()\n");
    }
	currState_ = nextState_;
}

void SuperstructureController::SetOutputs() {
    cargoFlyPID_->SetSetpoint(desiredFlywheelVelocCargo_);
    rocketFlyPID_->SetSetpoint(desiredFlywheelVelocRocket_);
}

void SuperstructureController::RefreshShuffleboard() {
    desiredFlywheelVelocCargo_ = cargoVelocNet_.GetDouble(0.7);
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