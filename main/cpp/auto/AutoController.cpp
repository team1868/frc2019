#include "../../include/auto/AutoController.h"

// Blank constructor
AutoController::AutoController() {
	autoMode = nullptr;
}

// Constructor that sets auto mode
AutoController::AutoController(AutoMode *myAutoMode){
	autoMode = myAutoMode;
}

// Setting auto mode
void AutoController::SetAutonomousMode(AutoMode *myAutoMode) {
	autoMode = myAutoMode;
}

void AutoController::Init(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {
	printf("in autocontroller init\n");
	if (autoMode == NULL) {
		printf("autoMode is null\n");
	} else {
		autoMode->CreateQueue(pos, hablvl);
		printf("done making queue\n");
		autoMode->Init();
		printf("init finished\n");
	}
}

void AutoController::Update(double currTimeSec, double deltaTimeSec) {
//	printf("Auto controller update\n");
	autoMode->Update(currTimeSec, deltaTimeSec);
}

bool AutoController::IsDone() {
	return autoMode->IsDone();
}

bool AutoController::Abort() {
	return autoMode->Abort();
}