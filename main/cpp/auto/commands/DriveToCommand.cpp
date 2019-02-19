/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//TODO CONSIDER USING INHERITENCE FOR FOLLOWING CLASSES

#include "../../../include/auto/commands/DriveToCommand.h"

/*
DriveToCommand::DriveToCommand(){
    
}
*/

/*
DriveToWithPivotCommand::DriveToWithPivotCommand() : AutoCommand(){}

DriveToWithCurveCommand::DriveToWithCurveCommand() : AutoCommand(){}
*/

/*
//TODO NEED TUNING
DriveToWithEllipseCommand::DriveToWithEllipseCommand(RobotModel *robot, double x, double y, double finalDesiredAngle,
    NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(){
    //purpose of class: end at x, y (relative to command init's position) and facing angle angle (degrees and not relative to init's position)
    //twoTurn if:
        //((target angle on right) and (target angle facing (left or straight on))) or
        //((targetangle on left) and (target angle facing (right or straight on)))
    
    robot_ = robot;
    finalDesiredAngle_ = finalDesiredAngle;

}

void DriveToWithEllipseCommand::Init(){
    relativeDesiredAngle_ = finalDesiredAngle_ - robot_->GetNavXYaw();
    //double ellipse1_a_ = 
    if(( x > 0.0 && relativeDesiredAngle_ >= 0.0) || x < 0.0 && relativeDesiredAngle_ <= 0.0){
        twoTurn_ = true;
    } else {
        twoTurn_ = false;
    }

    ellipse1_ = new EllipseCommand(robot_, double a, double b, double desiredAngle, bool turnLeft_,
        navXSource_, talonEncoderSource_, anglePIDOutput_, distancePIDOutput_);
    if(twoTurn)
}

void DriveToWithEllipseCommand::Update(double currTimeSec, double deltaTimeSec){

}
*/