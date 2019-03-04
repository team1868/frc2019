/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../include/controllers/GuidedDriveController.h"
//#include <frc/WPILib.h>
// constructor
GuidedDriveController::GuidedDriveController(RobotModel *robot, ControlBoard *humanControl,
	NavXPIDSource* navXSource, AnglePIDOutput* anglePIDOutput) : DriveController(robot, humanControl){

    //done in super()
	//robot_ = robot;
	//humanControl_ = humanControl;
	
	pFacNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Guided P", 0.07).GetEntry(); //TODO MOVE TO SEPERAT
	iFacNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Guided I", 0.0).GetEntry();
	dFacNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Guided D", 0.02).GetEntry();
	
	errorNet_ = frc::Shuffleboard::GetTab("Private_Code_Input").Add("Guided error", 0.0).GetEntry();

	pFac_ = pFacNet_.GetDouble(0.07);
	iFac_ = iFacNet_.GetDouble(0.0);
	dFac_ = dFacNet_.GetDouble(0.02);

	navXSource_ = navXSource;
	anglePIDOutput_ = anglePIDOutput;
	anglePID_ = new PIDController(pFac_, iFac_, dFac_, navXSource_, anglePIDOutput);

	anglePID_->SetPID(pFac_, iFac_, dFac_);
	anglePID_->SetSetpoint(robot_->GetNavXYaw());
	anglePID_->SetAbsoluteTolerance(0.5); //HM TUNE TODODODODODOD
	anglePID_->SetContinuous(true);
	anglePID_->SetInputRange(-180, 180);
	anglePID_->SetOutputRange(-0.4, 0.4); //TODO TUNEEEEEEEEEEEE
	anglePID_->Enable();

	// motor outputs, done in super() but done again anyways
	leftOutput = 0.0;
	rightOutput  = 0.0;

}

void GuidedDriveController::Disable(){
	anglePID_->Disable();
}

void GuidedDriveController::Enable(){
	anglePID_->Enable();
}

// arcade drive
void GuidedDriveController::ArcadeDrive(double myX, double myY, double thrustSensitivity, double rotateSensitivity) {
	double thrustValue = myY;
	double rotateValue = myX;

	//TODO: add safety for deadband values (prevent (-) or >= 0.1)
	thrustValue = HandleDeadband(thrustValue, thrustDeadbandNet_.GetDouble(0.0));	// 0.02 was too low
	rotateValue = HandleDeadband(rotateValue, rotateDeadbandNet_.GetDouble(0.0));

	rotateValue = GetCubicAdjustment(rotateValue, rotateSensitivity);
	thrustValue = GetCubicAdjustment(thrustValue, thrustSensitivity);

	if(reverseReverseNet_.GetBoolean(true) || (!reverseReverseNet_.GetBoolean(true) && thrustValue >= 0.0)){// || reverseReverseNet_.GetBoolean(false) && thrustValue > 0.0){ //lili mode
		leftOutput = thrustValue + rotateValue;			// CHECK FOR COMP BOT
		rightOutput = thrustValue - rotateValue;
		anglePID_->SetSetpoint(robot_->GetNavXYaw()+rotateValue);
	} else {
		leftOutput = thrustValue - rotateValue;
		rightOutput = thrustValue + rotateValue;
		anglePID_->SetSetpoint(robot_->GetNavXYaw()-rotateValue);
	}
	
	
    errorNet_.SetDouble(navXSource_->PIDGet());

	robot_->SetDriveValues(-leftOutput, rightOutput);

}

//NOTE: tank drive not overloaded
//quick turn not overloaded

//unimportanish note: cubic adjustment not overloaded

GuidedDriveController::~GuidedDriveController() { //POSSIBLE ERROR: destroy super but not small class
	//TODO needed?
}
