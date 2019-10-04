#include "../../../include/auto/modes/HatchInSideCargoMode.h"

HatchInSideCargoMode::HatchInSideCargoMode(RobotModel *robot, ControlBoard *controlBoard) : AutoMode(robot, controlBoard){
    robot_ = robot;
}

void HatchInSideCargoMode::CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {
    
}

void HatchInSideCargoMode::Init() {
    printf("Hatch In Side Cargo Mode Init\n");
    currentCommand_->Init();
}

HatchInSideCargoMode::~HatchInSideCargoMode() {

}