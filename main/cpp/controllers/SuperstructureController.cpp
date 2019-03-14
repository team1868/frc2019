#include "../../include/controllers/SuperstructureController.h"

SuperstructureController::SuperstructureController(RobotModel *myRobot, ControlBoard *myHumanControl) {
    robot_ = myRobot;
	humanControl_ = myHumanControl;

	currState_ = kInit;
	nextState_ = kIdle;

    desiredFlywheelVelocCargo_ = 0.7; 
    desiredFlywheelVelocRocket_ = 0.25; 

    desiredHatchWristAngle_ = 90;
    hatchWristNewAngle_ = true;

    cargoVelocNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("cargo veloc", 0.7).GetEntry(); //0.7
    cargoRocketVelocNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("rocket veloc", 0.25).GetEntry(); //0.25
    
    cargoIntakeOutput_ = 0.8; 

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
    hatchPDownFac_ = 0.00;
    hatchIDownFac_ = 0.0;
    hatchDDownFac_ = 0.02;

    hatchPUpFac_ = 0.00;
    hatchIUpFac_ = 0.0;
    hatchDUpFac_ = 0.02;

    cargoPNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("cargo P", 0.8).GetEntry();
    cargoINet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("cargo I", 0.0).GetEntry();
    cargoDNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("cargo D", 0.2).GetEntry();

    rocketPNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("rocket P", 0.8).GetEntry();
    rocketINet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("rocket I", 0.0).GetEntry();
    rocketDNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("rocket D", 0.2).GetEntry();

    hatchPDownNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Pd", 0.00).GetEntry(); //down pid
    hatchIDownNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Id", 0.0).GetEntry();
    hatchDDownNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Dd", 0.02).GetEntry();

    hatchPUpNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Pu", 0.00).GetEntry(); //up pid
    hatchIUpNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Iu", 0.0).GetEntry();
    hatchDUpNet_ = frc::Shuffleboard::GetTab("Operator_Input").Add("hatch Du", 0.02).GetEntry();

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

    //hatchWristPID_ = new PIDController(hatchPUpFac_, hatchIUpFac_, hatchDUpFac_, robot_->GetGyro(), robot_->GetHatchWristMotor());
    //hatchWristPID_->SetSetpoint(desiredHatchWristAngle_);
    //hatchWristPID_->SetOutputRange(-1.0, 1.0);
    //hatchWristPID_->SetAbsoluteTolerance(0.1); //TODO
    //hatchWristPID_->SetContinuous(true);

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

    //TODO TAKE THIS OUT
    HatchWristAngleTest();
    printf("current flywheel veloc cargo ship %f\n", desiredFlywheelVelocCargo_);
    printf("curr flywheel veloc rocket ship %f\n", desiredFlywheelVelocRocket_);

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
            RefreshShuffleboard();
            //HATCH STUFF

            //TODO INTEGRATE GYRO - THIS IS SO NOT DONE RIGHT NOW thanks
            if (humanControl_->GetHatchWristDownDesired()) { 
			    printf("hatch intake wrist to floor\n");
                robot_->SetHatchWristOutput(-0.3);
            } else if (humanControl_->GetHatchWristUpDesired()) { 
			    robot_->SetHatchWristOutput(0.3);
		    } else { //otherwise, keep in past 90 degree point
			    robot_->SetHatchWristOutput(0.0);
            }

            // TODO FIX BELOW SO THAT WHEELS ONLY RUN IF GYRO IS 90 ISH
            if(humanControl_->GetHatchIntakeWheelDesired()){ //only run wheels if wrist down (otherwise wheels are irrelevant)
                    printf("hatch intaking\n");
                    robot_->SetHatchIntakeWheelOutput(0.8);
            } else if (humanControl_->GetHatchUnintakeWheelDesired()){
                printf("hatch unintaking\n");
                robot_->SetHatchIntakeWheelOutput(-0.8);
            } else {
                robot_->SetHatchIntakeWheelOutput(0.0);
            } 
            //CARGO STUFF

            //note: combined wrist and intake/unintake (so if wrist down and not unintaking, auto intake + no two controllers on same motor)
            if(humanControl_->GetCargoIntakeWristDesired()){
                printf("wrist down\n");
                robot_->SetCargoIntakeWrist(true);
                if(!humanControl_->GetCargoUnintakeDesired()){  //!humanControl_->GetCargoUnintakeDesired()){ 
                    if (!CargoInIntake()) {
                       printf("hello friendz\n");
                       robot_->SetCargoIntakeOutput(cargoIntakeOutput_);
                    } else {
                        robot_->SetCargoIntakeOutput(0.0);
                    }
                } else {
                    printf("?????\n");
                    robot_->SetCargoUnintakeOutput(cargoIntakeOutput_);
                }
            } else {
                robot_->SetCargoIntakeWrist(false);
                if(humanControl_->GetCargoIntakeDesired()){ //if(humanControl_->GetCargoIntakeDesired()){ 
                    robot_->SetCargoIntakeOutput(cargoIntakeOutput_);
                } else if (humanControl_->GetCargoUnintakeDesired()){
                    robot_->SetCargoUnintakeOutput(cargoIntakeOutput_);
                } else {
                    robot_->SetCargoIntakeOutput(0.0);
                }
            }        

            if (humanControl_->GetCargoFlywheelDesiredRocket()){ //Note: check if less power first, don't want accident more power
               robot_->SetHatchBeak(true);
               robot_->SetCargoFlywheelOutput(RatioFlywheel(desiredFlywheelVelocRocket_)); 
               printf("flywheel speed is following %f\n", RatioFlywheel(desiredFlywheelVelocRocket_));
            } else if(humanControl_->GetCargoFlywheelDesired()){ //flywheel for cargo ship
                printf("cargo shooting into cargo ship\n");
                robot_->SetHatchBeak(false);
                robot_->SetCargoFlywheelOutput(RatioFlywheel(desiredFlywheelVelocCargo_)); 
                printf("flywheel speed is following %f\n", RatioFlywheel(desiredFlywheelVelocRocket_));
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

            if(humanControl_->GetHabBrakeDesired()){ //&& !humanControl_->GetTestDesired() && !humanControl_->GetTest3Desired()
                robot_->SetHabBrake(false);
                printf("hab brake not on\n");
            } else {
                robot_->SetHabBrake(true);
                printf("hab brake activate\n");
            }
            
            //TODO SUPER SKETCH 2 messages random everywhere see robot

            if(humanControl_->GetHighGearDesired()){
                robot_->SetHighGear();
                printf("High Gear \n");
            } else {
                robot_->SetLowGear();
                printf("Low Gear \n");
            }

            break;
        default:
            printf("WARNING: State not found in SuperstructureController::Update()\n");
    }
	currState_ = nextState_;
}

void SuperstructureController::SetOutputs() {
    cargoFlyPID_->SetSetpoint(desiredFlywheelVelocCargo_);
    rocketFlyPID_->SetSetpoint(desiredFlywheelVelocRocket_);
    //hatchWristPID_->SetSetpoint(desiredHatchWristAngle_);
}

void SuperstructureController::HatchWristAngleTest() {
    currHatchWristAngle_ = robot_->GetGyroAngle();
    //printf("Gryo Angle is the following %f\n", currHatchWristAngle_);
}

void SuperstructureController::HabEncoderTest() {
    currHabEncoderVal_ = robot_->GetHabEncoderValue();
    printf("Hab Encoder Value is the follwoing %f\n", currHabEncoderVal_);
}

void SuperstructureController::LightSensorTest(){
    currLightSensorStatus_ = robot_->GetLightSensorStatus();
    printf("Light Sensor Value is the following %d\n", currLightSensorStatus_);
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
    
    hatchPUpFac_ = hatchPUpNet_.GetDouble(0.00);
    hatchIUpFac_ = hatchIUpNet_.GetDouble(0.0);
    hatchDUpFac_ = hatchDUpNet_.GetDouble(0.02);

    hatchPDownFac_ = hatchPDownNet_.GetDouble(0.00);
    hatchIDownFac_ = hatchIDownNet_.GetDouble(0.0);
    hatchDDownFac_ = hatchDDownNet_.GetDouble(0.02);
}

void SuperstructureController::HatchWristControllerUpdate(double newAngle_, double pFac_, double iFac_, double dFac_) { 
    hatchWristPID_->SetPID(pFac_, iFac_, dFac_); 
    hatchWristPID_->SetSetpoint(newAngle_);
}

bool SuperstructureController::CargoInIntake(){
    currLightSensorStatus_ = robot_->GetLightSensorStatus();
    printf("Light Sensor Value is the following %d\n", currLightSensorStatus_);
    return currLightSensorStatus_;
}

double SuperstructureController::RatioFlywheel(double value){
    double ratioFlywheelOutput = 12.5/robot_->GetVoltage()*value;
    if(ratioFlywheelOutput > 1){
        ratioFlywheelOutput = 1;
    } else if(ratioFlywheelOutput < -1){
        ratioFlywheelOutput = -1;
    }
    return ratioFlywheelOutput;
}

void SuperstructureController::RefreshIni() { //TODO remove

}

SuperstructureController::~SuperstructureController() {
}