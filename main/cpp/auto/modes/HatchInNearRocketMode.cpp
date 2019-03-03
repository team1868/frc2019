#include "../../../include/auto/modes/HatchInNearRocketMode.h"

HatchInNearRocketMode::HatchInNearRocketMode(RobotModel *robot) : AutoMode(robot){
    robot_ = robot;
}

void HatchInNearRocketMode::CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {
    
}

void HatchInNearRocketMode::Init() {
    printf("Hatch In Near Rocket Mode Init\n");
    currentCommand_->Init();
}

HatchInNearRocketMode::~HatchInNearRocketMode() {

}