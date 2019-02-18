/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "../../../include/auto/commands/EllipseCommand.h"
#include <math.h>
#define PI 3.141592653589 //TODO CHANGE, this is a mess

//NOTEE:  IMPORTANT START ANGLE PARAMETER keep relative

EllipseCommand::EllipseCommand(RobotModel *robot, double a, double b, double desiredAngle, bool turnLeft,
    NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(){

    robot_ = robot;
    a_ = a;
    b_ = b;
    desiredAngle_ = desiredAngle;
    double t = atan(a/b*tan((desiredAngle * PI/180.0)));
    desiredTotalDistance_ = (pow((b_*b_+a_*a_)/2.0 + (b_*b_-a_*a_)/2.0*cos(2*t), 3.0/2.0)-b_*b_*b_)/3.0;
    turnLeft_ = turnLeft;

    navXPIDSource_ = navXSource;
    talonEncoderPIDSource_ = talonEncoderSource;
    anglePIDOutput_ = anglePIDOutput;
    distancePIDOutput_ = distancePIDOutput;

    dPFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Ellipse dP", 0.8).GetEntry();
    dIFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Ellipse dI", 0.0).GetEntry();
    dDFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Ellipse dD", 0.2).GetEntry();

    tPFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Ellipse tP", 0.8).GetEntry();
    tIFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Ellipse tI", 0.0).GetEntry();
    tDFacNet_ =  frc::Shuffleboard::GetTab("Private_Code_Input").Add("Ellipse tD", 0.2).GetEntry();
}

void EllipseCommand::Init(){
    
    dOutputNet_ = frc::Shuffleboard::GetTab("PRINTSTUFFSYAYS").Add("Ellipse dO", 0.0).GetEntry(); 
    tOutputNet_ = frc::Shuffleboard::GetTab("PRINTSTUFFSYAYS").Add("Ellipse tO", 0.0).GetEntry(); 
    lOutputNet_ = frc::Shuffleboard::GetTab("PRINTSTUFFSYAYS").Add("Ellipse lO", 0.0).GetEntry();
    rOutputNet_ = frc::Shuffleboard::GetTab("PRINTSTUFFSYAYS").Add("Ellipse rO", 0.0).GetEntry();
    dErrorNet_ = frc::Shuffleboard::GetTab("PRINTSTUFFSYAYS").Add("Ellipse dErr", 0.0).GetEntry(); 
    tErrorNet_ = frc::Shuffleboard::GetTab("PRINTSTUFFSYAYS").Add("Ellipse tErr", 0.0).GetEntry(); 

    
    initAngle_ = robot_->GetNavXYaw();

    curAngle_ = initAngle_;
    curDistance_ = 0.0;
    curDesiredAngle_ = curAngle_;

    curAngleError_ = 0.0;

    robot_->SetTalonCoastMode();
    robot_->SetHighGear(); //TODO tune/fix
    robot_->ResetDriveEncoders(); //TODODODODODODOD FIXXXXXXXXXXXXXXXXXXXXXX, use dem mathsd

    LoadPIDValues();

    dPID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderPIDSource_, distancePIDOutput_);
    tPID_ = new PIDController(tPFac_, tIFac_, tDFac_, navXPIDSource_, anglePIDOutput_);

    tPID_->SetPID(tPFac_, tIFac_, tDFac_);
	dPID_->SetPID(dPFac_, dIFac_, dDFac_);

	tPID_->SetSetpoint(curDesiredAngle_);
	dPID_->SetSetpoint(desiredTotalDistance_);

    tPID_->SetContinuous(true);
	tPID_->SetInputRange(-180, 180);
	dPID_->SetContinuous(false);

    tPID_->SetOutputRange(-0.9, 0.9);
	dPID_->SetOutputRange(-0.9, 0.9);

    dPID_->SetAbsoluteTolerance(3.0/12.0); //this too U DUDE   
    tPID_->SetAbsoluteTolerance(0.5); //HM TUNE TODODODODODOD

	tPID_->Enable();
	dPID_->Enable();


}

void EllipseCommand::Reset(){
  robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
  robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);
  
	// destroy angle PID
	if (tPID_ != NULL) {
		tPID_->Disable();
        tPID_->~PIDController();

		delete(tPID_);

		tPID_ = NULL;

		printf("Reset Angle PID %f \n", robot_->GetNavXYaw());
	}

	// destroy distance PID
	if (dPID_ != NULL) {
		dPID_->Disable();
        dPID_->~PIDController();

		delete(dPID_);

		dPID_ = NULL;
//		printf("Reset Distance PID");

	}
	isDone_ = true;
}

void EllipseCommand::Update(double currTimeSec, double deltaTimeSec){
    
    if(dPID_->OnTarget() && tPID_->OnTarget()){ //TODO add timeout here, also TODO possible source of error if one done and one not?
    printf("%f Final NavX Angle from PID Source: %f\n"
            "Final NavX Angle from robot: %f \n"
            "Final Distance from PID Source: %f\n",
            robot_->GetTime(), navXPIDSource_->PIDGet(), robot_->GetNavXYaw(),
            talonEncoderPIDSource_->PIDGet());
    if(turnLeft_){
        printf("Final Distance from robot: %f\n", robot_->GetRightDistance());//robot_->GetLeftDistance()); /fixed inversion
    } else {
        printf("Final Distance from robot: %f\n", robot_->GetLeftDistance());
    }
        //Reset();
        isDone_ = true;
        robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
        robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);
        printf("%f Ellipse Command IS DONE \n", robot_->GetTime());
        //if (timeOut) {
        //	printf("%f FROM CURVE TIME OUT GO LEAVEEEEEE %f\n", robot_->GetTime(), timeDiff);
        //}
    } else {

    if(turnLeft_){
        curDistance_ = robot_->GetRightDistance();//robot_->GetLeftDistance();
    } else {
        curDistance_ = robot_->GetLeftDistance();
    }
    curDesiredAngle_ = CalcCurDesiredAngle(curDistance_);
    curAngleError_ = curDesiredAngle_ - curAngle_;

    tPID_->SetSetpoint(curDesiredAngle_);

    double dOutput = distancePIDOutput_->GetPIDOutput();
    double tOutput = anglePIDOutput_->GetPIDOutput();

    double lOutput;
    double rOutput;

    if(turnLeft_){
        lOutput = dOutput - tOutput; //TODO SKETCH
        rOutput = dOutput + tOutput;
    } else {
        lOutput = dOutput + tOutput; //TODO SKETCH
        rOutput = dOutput - tOutput;
    }

    //TODODODODO NEEDED OR IS THIS MESSING WITH THE PID????
    //power output checks
    if(lOutput > 1.0){
        rOutput = rOutput/lOutput;
        lOutput = 1.0;
    } else if (lOutput < -1.0){
        rOutput = rOutput/(-lOutput);
        lOutput = -1.0;
    }
    if(rOutput > 1.0) {
        lOutput = lOutput/rOutput;
        rOutput = 1.0;
    } else if (rOutput < -1.0) {
        lOutput = lOutput/(-rOutput);
        rOutput = -1.0;
    }

    robot_->SetDriveValues(RobotModel::kLeftWheels, -lOutput);
    robot_->SetDriveValues(RobotModel::kRightWheels, rOutput);

    dOutputNet_.SetDouble(dOutput);
    tOutputNet_.SetDouble(tOutput);
    lOutputNet_.SetDouble(lOutput);
    rOutputNet_.SetDouble(rOutput);
    if(turnLeft_){
        dErrorNet_.SetDouble(2*PI/(360/desiredAngle_) - robot_->GetRightDistance());
    } else {
        dErrorNet_.SetDouble((2*PI/(360/desiredAngle_) - robot_->GetLeftDistance()));
    }
    tErrorNet_.SetDouble(curAngleError_);
    }
}

bool EllipseCommand::IsDone(){
    return isDone_;
}

double EllipseCommand::CalcCurDesiredAngle(double curDistance){
    double t = acos((2.0*pow(3.0*curDistance+b_*b_*b_, 2.0/3.0) - b_*b_ + a_*a_) / (b_*b_-a_*a_))/2.0;
    double angle = 90 - (atan((b_*cos(t)) / (-a_*sin(t))) * 180/PI);
    if(turnLeft_){
        angle = -angle;
    }
    return angle;
}

void EllipseCommand::LoadPIDValues(){
    dPFac_ = dPFacNet_.GetDouble(0.8);
    dIFac_ = dIFacNet_.GetDouble(0.0);
    dDFac_ = dDFacNet_.GetDouble(0.0);

    tPFac_ = tPFacNet_.GetDouble(0.8);
    tIFac_ = tIFacNet_.GetDouble(0.0);
    tDFac_ = tDFacNet_.GetDouble(0.0);
}

EllipseCommand::~EllipseCommand(){
    Reset();
    
    dOutputNet_.Delete();
    tOutputNet_.Delete();
    lOutputNet_.Delete();
    rOutputNet_.Delete();
    dErrorNet_.Delete();
    tErrorNet_.Delete();

    dPFacNet_.Delete();
    dIFacNet_.Delete();
    dDFacNet_.Delete();

    tPFacNet_.Delete();
    tIFacNet_.Delete();
    tDFacNet_.Delete();
}