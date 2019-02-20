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

    desiredHatchWristAngle_ = 90;
    hatchWristNewAngle_ = true;

    cargoVelocNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("cargo veloc", 0.7).GetEntry(); //0.7
    cargoRocketVelocNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("rocket veloc", 0.25).GetEntry(); //0.25
    
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

    //PID vals for hatch wrist
    hatchPDownFac_ = 0.8;
    hatchIDownFac_ = 0.0;
    hatchDDownFac_ = 0.2;

    hatchPUpFac_ = 0.8;
    hatchIUpFac_ = 0.0;
    hatchDUpFac_ = 0.2;

    cargoPNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("cargo P", 0.8).GetEntry();
    cargoINet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("cargo I", 0.0).GetEntry();
    cargoDNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("cargo D", 0.2).GetEntry();

    rocketPNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("rocket P", 0.8).GetEntry();
    rocketINet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("rocket I", 0.0).GetEntry();
    rocketDNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("rocket D", 0.2).GetEntry();

    hatchPDownNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Pd", 0.8).GetEntry(); //down pid
    hatchIDownNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Id", 0.0).GetEntry();
    hatchDDownNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Dd", 0.2).GetEntry();

    hatchPUpNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Pu", 0.8).GetEntry(); //up pid
    hatchIUpNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Iu", 0.0).GetEntry();
    hatchDUpNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Du", 0.2).GetEntry();

    //shuffleboard PID values
    cargoFlyPID_  = new PIDController(cargoPFac_, cargoIFac_, cargoDFac_, robot_->GetCargoFlywheelEncoder(),
        robot_->GetCargoFlywheelMotor());
    cargoFlyPID_->SetSetpoint(desiredFlywheelVelocCargo_);  
    cargoFlyPID_->SetOutputRange(-1.0, 1.0); 
    cargoFlyPID_->SetAbsoluteTolerance(0.05); //TODO
    cargoFlyPID_->SetContinuous(false);

    rocketFlyPID_ = new PIDController(rocketPFac_, rocketIFac_, rocketDFac_, robot_->GetCargoFlywheelEncoder(),
        robot_->GetCargoFlywheelMotor());
    rocketFlyPID_->SetSetpoint(desiredFlywheelVelocRocket_);  
    rocketFlyPID_->SetOutputRange(-1.0, 1.0); 
    rocketFlyPID_->SetAbsoluteTolerance(0.05); //TODO
    rocketFlyPID_->SetContinuous(false);

    /**
     * for hatch PID: currently 2 positions
     * 
     * down - stowed position to floor
     * up - floor to stowed position
    */

    hatchWristPID_ = new PIDController(hatchPUpFac_, hatchIUpFac_, hatchDUpFac_, robot_->GetGyro(), robot_->GetHatchWristMotor());
    hatchWristPID_->SetSetpoint(desiredHatchWristAngle_);
    hatchWristPID_->SetOutputRange(-1.0, 1.0);
    hatchWristPID_->SetAbsoluteTolerance(0.1); //TODO
    hatchWristPID_->SetContinuous(true);

    //TODO INIT highgear disengaged whatever whatever (like last year's wrist engage asap)
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

    cargoFlyPID_->Reset();
    rocketFlyPID_->Reset();
    hatchWristPID_->Reset();

    RefreshShuffleboard();

    robot_->SetCargoIntakeOutput(0.0);
    robot_->SetHatchIntakeWheelOutput(0.0);
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {

    SetOutputs(); 

    //TODO TAKE THIS OUT
    HatchWristAngleTest();

	switch(currState_) {
        case kInit:
            cargoFlyPID_->Reset();
            cargoFlyPID_->Disable();
            
            rocketFlyPID_->Reset();
            rocketFlyPID_->Disable();

            hatchWristPID_->Enable();

            robot_->SetCargoIntakeOutput(0.0); 
            robot_->SetCargoFlywheelOutput(0.0);
            robot_->SetHatchIntakeWheelOutput(0.0);

            nextState_ = kIdle;
        case kIdle:
            nextState_ = kIdle;
            //HATCH STUFF

            //TODO INTEGRATE GYRO - THIS IS SO NOT DONE RIGHT NOW thanks
            if (humanControl_->GetHatchWristDownDesired()) { 
			    printf("hatch intake wrist to floor\n");
                HatchWristControllerUpdate(0, hatchPDownFac_, hatchIDownFac_, hatchDDownFac_);
                if(humanControl_->GetHatchIntakeWheelDesired()){ //only run wheels if wrist down (otherwise wheels are irrelevant)
                printf("hatch intaking\n");
                    robot_->SetHatchIntakeWheelOutput(0.8);
                } else if (humanControl_->GetHatchUnintakeWheelDesired()){
                    printf("hatch intaking\n");
                    robot_->SetHatchIntakeWheelOutput(-0.8);
                } else {
                    robot_->SetHatchIntakeWheelOutput(0.0);
                }
			    //robot_->SetHatchWristOutput(-0.3); 
            } else if (humanControl_->GetHatchWristUpDesired()) { 
                HatchWristControllerUpdate(95, hatchPUpFac_, hatchIUpFac_, hatchDUpFac_);
			    //robot_->SetHatchWristOutput(0.3);
		    } else { //otherwise, keep in past 90 degree point
                HatchWristControllerUpdate(95, hatchPUpFac_, hatchIUpFac_, hatchDUpFac_);
			    //robot_->SetHatchWristOutput(0.0);
            }
            //CARGO STUFF

            //note: combined wrist and intake/unintake (so if wrist down and not unintaking, auto intake + no two controllers on same motor)
            if(humanControl_->GetCargoIntakeWristDesired()){
                robot_->SetCargoIntakeWrist(true);
                if(!humanControl_->GetCargoUnintakeDesired()){
                    robot_->SetCargoIntakeOutput(cargoIntakeOutput_);
                } else {
                    robot_->SetCargoUnintakeOutput(cargoIntakeOutput_);
                }
            } else {
                robot_->SetCargoUnintakeOutput(false);
                if(humanControl_->GetCargoIntakeDesired()){
                    robot_->SetCargoIntakeOutput(cargoIntakeOutput_);
                } else if (humanControl_->GetCargoUnintakeDesired()){
                    robot_->SetCargoUnintakeOutput(cargoIntakeOutput_);
                } else {
                    robot_->SetCargoIntakeOutput(0.0);
                }
            }        

            if (humanControl_->GetCargoFlywheelDesiredRocket()){ //Note: check if less power first, don't want accident more power
               robot_->SetHatchBeak(true);
               robot_->SetCargoFlywheelOutput(desiredFlywheelVelocRocket_);
            } else if(humanControl_->GetCargoFlywheelDesired()){ //flywheel for cargo ship
                printf("cargo shooting into cargo ship\n");
                robot_->SetHatchBeak(false);
                robot_->SetCargoFlywheelOutput(desiredFlywheelVelocCargo_);
            } else {
                robot_->SetCargoFlywheelOutput(0.0);

                //so beak not messed up if multiple activated at a time
                if(humanControl_->GetHatchOuttakeDesired()){ //TODO different state: time delay
                    robot_->SetHatchBeak(true);
                    robot_->SetHatchOuttake(true);
                } else if(humanControl_->GetHatchBeakDesired()){
                    robot_->SetHatchBeak(true);
                    //robot_->SetHatchOuttake(false); //TODO prob not needed bc not immediete switch between buttons, but otherwise needed
                } else {
                    robot_->SetHatchBeak(false);
                    robot_->SetHatchOuttake(false);
                }

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

           /*TODODODODODOD ENTER BACK IN
           if(humanControl_->GetHabDeployDesired()){
               robot_->SetHabMotorOutput(0.4); //WHAT IS THIS VALUE CHECK W LILLIAN
           } else {
               robot_->SetHabMotorOutput(0.0);
           }*/

            break;
        default:
            printf("WARNING: State not found in SuperstructureController::Update()\n");
    }
	currState_ = nextState_;
}

void SuperstructureController::SetOutputs() {
    cargoFlyPID_->SetSetpoint(desiredFlywheelVelocCargo_);
    rocketFlyPID_->SetSetpoint(desiredFlywheelVelocRocket_);
    hatchWristPID_->SetSetpoint(desiredHatchWristAngle_);
}

void SuperstructureController::HatchWristAngleTest() {
    currHatchWristAngle_ = robot_->GetGyroAngle();
    //printf("Gryo Angle is the following %f\n", currHatchWristAngle_);
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
    
    hatchPUpFac_ = hatchPUpNet_.GetDouble(0.8);
    hatchIUpFac_ = hatchIUpNet_.GetDouble(0.0);
    hatchDUpFac_ = hatchDUpNet_.GetDouble(0.2);

    hatchPDownFac_ = hatchPDownNet_.GetDouble(0.8);
    hatchIDownFac_ = hatchIDownNet_.GetDouble(0.0);
    hatchDDownFac_ = hatchDDownNet_.GetDouble(0.2);
}

void SuperstructureController::HatchWristControllerUpdate(double newAngle_, double pFac_, double iFac_, double dFac_) { 
    hatchWristPID_->SetPID(pFac_, iFac_, dFac_); 
    hatchWristPID_->SetSetpoint(newAngle_);
}

void SuperstructureController::RefreshIni() { //TODO remove

}

SuperstructureController::~SuperstructureController() {
}