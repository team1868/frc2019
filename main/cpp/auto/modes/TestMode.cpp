#include "../../../include/auto/modes/TestMode.h"

TestMode::TestMode(RobotModel *robot) : AutoMode(robot) {
    printf("in test mode constructor \n");

}

void TestMode::CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {
    string testSequence = robot_->GetTestSequence();
    QueueFromString(testSequence);
}

void TestMode::Init() {
	printf("Initializing Test mode\n");
	currentCommand_->Init();
	printf("Finished initializing\n");
}

TestMode::~TestMode() {

}