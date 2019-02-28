#include "../../../include/auto/commands/HatchBeakCommand.h"

HatchBeakCommand::HatchBeakCommand(RobotModel *robot, bool hatchOpen) : AutoCommand() {
    printf("intaking hatch from hp station \n");
    isDone_ = false;
    hatchOpen_ = hatchOpen;
    robot_ = robot;
}

void HatchBeakCommand::Init() {
    printf("intake hatch command init\n");
    isDone_ = false;
}

void HatchBeakCommand::Reset() {
    isDone_ = true;
}

void HatchBeakCommand::Update(double currTimeSec, double deltaTimeSec) {
    robot_->SetHatchBeak(hatchOpen_);

    isDone_ = true;
}

bool HatchBeakCommand::IsDone() {
    return isDone_;
}

HatchBeakCommand::~HatchBeakCommand() {
    
}
