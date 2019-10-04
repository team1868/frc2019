#include "../../../include/auto/modes/BlankMode.h"

BlankMode::BlankMode(RobotModel *robot, ControlBoard *controlBoard) : AutoMode(robot, controlBoard) {
	printf("In Blank Mode\n");
}

void BlankMode::CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {
	printf("In Blank Mode Queue\n");
}

void BlankMode::Init() {
	printf("In Blank Mode Init\n");
}

BlankMode::~BlankMode() {
}
