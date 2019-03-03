#include "../../../include/auto/commands/AlignWithTapeCommand.h"

AlignWithTapeCommand::AlignWithTapeCommand(RobotModel* robot, NavXPIDSource* navXSource, TalonEncoderPIDSource* talonSource, bool driveStraightDesired) : AutoCommand() {
    printf("constructing align with tape command\n");
    
    context_ = NULL;
    subscriber_ = NULL;

    robot_ = robot;
    navXSource_ = navXSource;
    talonSource_ = talonSource;

    driveStraightDesired_ = driveStraightDesired;

    anglePIDOutput_ = new AnglePIDOutput();
    distancePIDOutput_ = new DistancePIDOutput();

    pivotCommand_ = NULL;
    driveStraightCommand_ = NULL;

    isDone_ = false;

    desiredDeltaAngle_ = 0.0;
    desiredDistance_ = 0.0;

    currState_ = kPivotInit;
    nextState_ = kPivotInit;

    initTimeVision_ = 0.0;
    initTimeAlign_ = 0.0;
}

void AlignWithTapeCommand::Init() {
    printf("in align with tape command init\n");

    context_ = new zmq::context_t(1);

    try {
        subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
        subscriber_->connect("tcp://10.18.68.15:5563"); // TODO CHECK ID
        int confl = 1;
		subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
		subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000);
		subscriber_->setsockopt(ZMQ_SUBSCRIBE, "MESSAGE", 0);
    } catch(const zmq::error_t &exc) {
		printf("TRY CATCH FAILED IN ALIGNWITHTAPECOMMAND INIT\n");
		std::cerr << exc.what();
	}

    desiredDeltaAngle_ = 0.0;
    desiredDistance_ = 0.0;
    isDone_ = false;
    currState_ = kPivotInit;
	nextState_ = kPivotInit;

	anglePIDOutput_ = new AnglePIDOutput();
	distancePIDOutput_ = new DistancePIDOutput();

	initTimeVision_ = robot_->GetTime();
}

void AlignWithTapeCommand::Update(double currTimeSec, double deltaTimeSec) {
    double lastDesiredAngle = desiredDeltaAngle_;
	double lastDesiredDistance = desiredDistance_;
	double diffInAngle;

	switch (currState_) {
		case (kPivotInit) :
			printf("In kPivotInit\n");

			ReadFromJetson();
			SmartDashboard::PutNumber("Vision pivot delta angle", desiredDeltaAngle_);
			SmartDashboard::PutNumber("Vision desired distance", desiredDistance_);

			diffInAngle = fabs(lastDesiredAngle - desiredDeltaAngle_);
			printf("Difference in Angle: %f\n", diffInAngle);
			if (diffInAngle > 2.0 || (diffInAngle == 0.0 && desiredDeltaAngle_ == 0.0)) {
				printf("diff in angle is > 2\n");
				nextState_ = kPivotInit;
			} else if (fabs(desiredDeltaAngle_) > 2.0) {
				printf("vision done at: %f\n", robot_->GetTime() - initTimeVision_);

				printf("ANGLE FOR PIVOT COMMAND: %f\n", -desiredDeltaAngle_);
				pivotCommand_ = new PivotCommand(robot_, -desiredDeltaAngle_, false, navXSource_);
				printf("pivotCommand constructed: %f\n", robot_->GetTime() - initTimeVision_);
				pivotCommand_->Init();
				printf("pivotCommand inited: %f\n", robot_->GetTime() - initTimeVision_);
				nextState_ = kPivotUpdate;
			} else {
				printf("vision done at: %f\n", robot_->GetTime() - initTimeVision_);
				printf("ANGLE THAT WAS GOOD NO PIVOT: %f\n", -desiredDeltaAngle_);
				if(driveStraightDesired_) {
					nextState_ = kDriveInit;
				} else {
					isDone_ = true;
				}
			}
			break;

		case (kPivotUpdate) :
			if (!pivotCommand_->IsDone()) {
				pivotCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kPivotUpdate;
			} else {
				ReadFromJetson();
				printf("Final Vision Angle: %f\n", desiredDeltaAngle_);
				printf("Pivot To Angle Is Done\n");
				if (driveStraightDesired_) {
					nextState_ = kDriveInit;
				} else {
					isDone_ = true;
					printf("AlighWithTape Done \n");
				}
			}
			break;

		case (kDriveInit) :
			printf("In DriveStraightInit\n");

			ReadFromJetson();
            // TODO: REDO ALLLLLLL THE MATH HERE
			if (fabs(desiredDistance_) > 2.0/12.0) {	// 2 in threshold
				printf("DISTANCE FOR COMMAND: %f\n", desiredDistance_);
				// Jetson returns in inches, so /12.0
				// Subtract 10 inches bc of length of peg
				// negative because technically driving backwards
				driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, anglePIDOutput_, distancePIDOutput_,
						robot_, -(desiredDistance_ - 12.5)/12.0);	// converting to feet
				driveStraightCommand_->Init();
				nextState_ = kDriveUpdate;
			} else {
				isDone_ = true;
				printf("Done with AlignWithTape \n");
			}
			break;

		case (kDriveUpdate) :
			if (!driveStraightCommand_->IsDone()) {
				driveStraightCommand_->Update(0.0, 0.0); 	// add timer later
				nextState_ = kDriveUpdate;
			} else {
				isDone_ = true;
				// no next state
				printf("Done with AlignWithTape \n");
			}
			break;
	}
	currState_ = nextState_;

	if (robot_->GetTime() - initTimeAlign_ > 3.5) {	// Timeout for align with tape command
		isDone_ = true;
	}
}

bool AlignWithTapeCommand::IsDone() {
    return isDone_;
}

void AlignWithTapeCommand::ReadFromJetson() {
    printf("starting read from jetson\n");

	try {
		string contents = s_recv(*subscriber_);

		stringstream ss(contents);
		vector<string> result;

		while(ss.good()) {
			string substr;
			getline( ss, substr, ' ' );
			result.push_back( substr );
		}

		desiredDeltaAngle_ = stod(result.at(0));
		desiredDistance_ = stod(result.at(1));
		printf("contents from jetson: %s\n", contents.c_str());
	} catch (const std::exception &exc) {
		printf("TRY CATCH FAILED IN READFROMJETSON\n");
		std::cerr << exc.what();
		desiredDeltaAngle_ = 0.0;
		desiredDistance_ = 0.0;
	}
	printf("end of read from jetson\n");
}

void AlignWithTapeCommand::Reset() {
	isDone_ = true;
}

AlignWithTapeCommand::~AlignWithTapeCommand() {

}