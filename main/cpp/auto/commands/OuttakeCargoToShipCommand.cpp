#include "../../../include/auto/commands/OuttakeCargoToShipCommand.h"

OuttakeCargoToShipCommand::OuttakeCargoToShipCommand(RobotModel *robot) : AutoCommand() {
    robot_ = robot;
    isDone_ = false;
	outtakeCargoToCargoShipMotorOutput_ = 0.7; //TODO TEST
    cargoIntakeMotorOutput_ = 1.0;
	startTime_ = 0.0;
    deltaFlywheelStartTime_ = 1.0;  //TODO TEST
	deltaTime_ = 1.8; //TODO TEST
}

void OuttakeCargoToShipCommand::Init() {
	isDone_ = false;
	startTime_ = robot_->GetTime();
}

void OuttakeCargoToShipCommand::Reset() {
	robot_->SetCargoFlywheelOutput(0.0);
    robot_->SetCargoIntakeOutput(0.0);
	isDone_ = true;
}

void OuttakeCargoToShipCommand::Update(double currTimeSec, double deltaTimeSec) {
	double diffTime = robot_->GetTime() - startTime_;

	if (diffTime > deltaTime_){
		robot_->SetCargoFlywheelOutput(0.0);
        robot_->SetCargoIntakeOutput(0.0);
		isDone_ = true;
        printf("outtake cargo to rocket done from timeout \n");
	} else if (diffTime > deltaFlywheelStartTime_ && diffTime < deltaTime_) {
        robot_->SetCargoFlywheelOutput(outtakeCargoToCargoShipMotorOutput_);
        printf("starting up flywheel first...\n");      // MANY TODOS FLYWHEEL PID FOR GOOD SPEED??!?!??
    } else {
        robot_->SetCargoIntakeOutput(cargoIntakeMotorOutput_);
        robot_->SetCargoFlywheelOutput(outtakeCargoToCargoShipMotorOutput_);
	}
}

bool OuttakeCargoToShipCommand::IsDone() {
	return isDone_;
}

OuttakeCargoToShipCommand::~OuttakeCargoToShipCommand() {
	// TODO Auto-generated destructor stub
	Reset();
}
