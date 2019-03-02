// #include "../../../include/auto/commands/AlignWithTapeCommand.h"

// AlignWithTapeCommand::AlignWithTapeCommand(RobotModel* robot, NavXPIDSource* navXSource, TalonEncoderPIDSource* talonSource, bool driveStraightDesired) {
//     printf("constructing align with tape command\n");
    
//     context_ = NULL;
//     subscriber_ = NULL;

//     robot_ = robot;
//     navXSource_ = navXSource;
//     talonSource_ = talonSource;

//     driveStraightDesired_ = driveStraightDesired;

//     anglePIDOutput_ = new AnglePIDOutput();
//     distancePIDOutput_ = new DistancePIDOutput();

//     pivotCommand_ = NULL;
//     driveStraightCommand_ = NULL;

//     isDone_ = false;

//     desiredDeltaAngle_ = 0.0;
//     desiredDistance_ = 0.0;

//     currState_ = kPivotInit;
//     nextState_ = kPivotInit;

//     initTimeVision_ = 0.0;
//     initTimeAlign_ = 0.0;
// }
