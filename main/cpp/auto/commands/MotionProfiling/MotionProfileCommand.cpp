/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/MotionProfiling/MotionProfileCommand.h"
// #include "../../RobotModel.cpp"

MotionProfileCommand::MotionProfileCommand(RobotModel *robot) : AutoCommand(){
    robot_ = robot;
}

void MotionProfileCommand::Init(){
    LoadTrajectory();
    
    leftFollower_ = (EncoderFollower*)calloc(1, sizeof(EncoderFollower));
    rightFollower_ = (EncoderFollower*)calloc(1, sizeof(EncoderFollower));

    leftFollower_->last_error = 0; leftFollower_->segment = 0; leftFollower_->finished = 0;
    rightFollower_->last_error = 0; rightFollower_->segment = 0; rightFollower_->finished = 0;

    leftConfig_->initial_position = robot_->GetLeftEncoderValue();
    leftConfig_->ticks_per_revolution = ENCODER_COUNT_PER_ROTATION;
    leftConfig_->wheel_circumference = WHEEL_DIAMETER*M_PI;
    leftConfig_->kp = 1.0;
    leftConfig_->ki = 0.0;
    leftConfig_->kd = 0.0;
    leftConfig_->kv = 1.0 / (MAX_VELOCITY*METERS_IN_FOOT);
    leftConfig_->ka = 0.0;
    
    rightConfig_->initial_position = robot_->GetRightEncoderValue();
    rightConfig_->ticks_per_revolution = ENCODER_COUNT_PER_ROTATION;
    rightConfig_->wheel_circumference = WHEEL_DIAMETER*M_PI;
    rightConfig_->kp = 1.0;
    rightConfig_->ki = 0.0;
    rightConfig_->kd = 0.0;
    rightConfig_->kv = 1.0 / (MAX_VELOCITY*METERS_IN_FOOT);
    rightConfig_->ka = 0.0;
}

void MotionProfileCommand::Update(double currTimeSec, double deltaTimeSec){

    double l = pathfinder_follow_encoder(*leftConfig_, leftFollower_, leftTrajectory_, trajectoryLen, robot_->GetLeftEncoderValue());
    double r = pathfinder_follow_encoder(*rightConfig_, rightFollower_, rightTrajectory_, trajectoryLen, robot_->GetRightEncoderValue());

    double gyro_heading = robot_->GetNavXYaw();
    double desired_heading = r2d(leftFollower_->heading);

    double angle_difference = desired_heading - gyro_heading;    // Make sure to bound this from -180 to 180, otherwise you will get super large values

    // This allows the angle distance to respect 'wrapping', where 360 and 0 are the same value.
    angle_difference = std::fmod(angle_difference, 360.0);
    if (std::abs(angle_difference) > 180.0) {
        angle_difference = (angle_difference > 0) ? angle_difference - 360 : angle_difference + 360;
    } 

    double turn = 0.8 * (-1.0/80.0) * angle_difference;

    robot_->SetDriveValues(RobotModel::kLeftWheels, l + turn);
    robot_->SetDriveValues(RobotModel::kRightWheels, r - turn);
}

bool MotionProfileCommand::IsDone(){
    return leftFollower_->finished && rightFollower_->finished;
}

void MotionProfileCommand::Reset(){
    free(leftFollower_);
    free(rightFollower_);
    delete [] leftTrajectory_;
    delete [] rightTrajectory_;
    Init();
}

void MotionProfileCommand::LoadTrajectory(){
    trajectoryLen = sizeof(data)/sizeof(double)/8/2;
    leftTrajectory_ = new Segment[trajectoryLen];
    rightTrajectory_ = new Segment[trajectoryLen];

    int dataI;
    for(int i = 0, dataI = 0; i<trajectoryLen; i++){
        leftTrajectory_[i].dt = data[dataI++];
        leftTrajectory_[i].x = data[dataI++];
        leftTrajectory_[i].y = data[dataI++];
        leftTrajectory_[i].position = data[dataI++];
        leftTrajectory_[i].velocity = data[dataI++];
        leftTrajectory_[i].acceleration = data[dataI++];
        leftTrajectory_[i].jerk = data[dataI++];
        leftTrajectory_[i].heading = data[dataI++];

        rightTrajectory_[i].dt = data[dataI++];
        rightTrajectory_[i].x = data[dataI++];
        rightTrajectory_[i].y = data[dataI++];
        rightTrajectory_[i].position = data[dataI++];
        rightTrajectory_[i].velocity = data[dataI++];
        rightTrajectory_[i].acceleration = data[dataI++];
        rightTrajectory_[i].jerk = data[dataI++];
        rightTrajectory_[i].heading = data[dataI++];
    }
}

MotionProfileCommand::~MotionProfileCommand(){
    free(leftFollower_);
    free(rightFollower_);
    delete [] leftTrajectory_;
    delete [] rightTrajectory_;
}