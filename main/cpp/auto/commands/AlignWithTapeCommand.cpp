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
	abort_ = false;

    desiredDeltaAngle_ = 0.0;
    desiredDistance_ = 0.0;

    currState_ = kPivotInit;
    nextState_ = kPivotInit;

    initTimeVision_ = 0.0;
    initTimeAlign_ = 0.0;
}

void AlignWithTapeCommand::Init() {
    printf("in align with tape command init\n");
	initTimeAlign_ = robot_->GetTime();
	initTimeVision_ = robot_->GetTime(); //TODO is this right

    context_ = new zmq::context_t(1);

    try {
		printf("in try connect to jetson\n");
        subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
        subscriber_->connect("tcp://10.18.68.12:5808");
		printf("connected to socket\n");
        int confl = 1;
		subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
		subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000);
		subscriber_->setsockopt(ZMQ_SUBSCRIBE, "MESSAGE", 0);
		printf("done with try\n");
    } catch(const zmq::error_t &exc) {
		printf("TRY CATCH FAILED IN ALIGNWITHTAPECOMMAND INIT\n");
		std::cerr << exc.what();
	}

    desiredDeltaAngle_ = 0.0;
    desiredDistance_ = 0.0;
    isDone_ = false;
	abort_ = false;
    currState_ = kPivotInit;
	nextState_ = kPivotInit;

	anglePIDOutput_ = new AnglePIDOutput();
	distancePIDOutput_ = new DistancePIDOutput();

	initTimeVision_ = robot_->GetTime();
}

void AlignWithTapeCommand::Update(double currTimeSec, double deltaTimeSec) {
	printf("in align with tape update()");
    double lastDesiredAngle = desiredDeltaAngle_;
	double lastDesiredDistance = desiredDistance_;
	double diffInAngle;

	switch (currState_) {
		case (kPivotInit) :
			printf("In kPivotInit\n");

			ReadFromJetson();
			if (abort_) {
				isDone_ = true;
				nextState_ = kPivotInit;
				break;
			}
			printf("Vision pivot delta angle %f\n", desiredDeltaAngle_);
			printf("Vision desired distance %f\n", desiredDistance_);

			diffInAngle = fabs(lastDesiredAngle - desiredDeltaAngle_);
			printf("Difference in Angle: %f\n", diffInAngle);
			// if (fabs(diffInAngle) < 2.0 || (diffInAngle == 0.0 && desiredDeltaAngle_ == 0.0)) {
			// 	printf("diff in angle is < 2\n");
			// 	nextState_ = kPivotInit;
			// } else 
			if (fabs(desiredDeltaAngle_) > 2.0) {
				printf("vision done at: %f\n", robot_->GetTime() - initTimeVision_);

				printf("ANGLE turning FOR PIVOT COMMAND: %f, abs angle turning to is %f (includes orig angle)\n", desiredDeltaAngle_, robot_->GetNavXYaw()+desiredDeltaAngle_);
				pivotCommand_ = new PivotCommand(robot_, robot_->GetNavXYaw()+desiredDeltaAngle_, true, navXSource_, 1); //last digit is pivot tolerance
				printf("pivotCommand constructed at time: %f\n", robot_->GetTime() - initTimeVision_);
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

		case (kPivotUpdate):
			printf("in kPivot update");
			if (!pivotCommand_->IsDone()) {
				pivotCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kPivotUpdate;
				printf("in alignwithtape update() pivot command on, updated pivot command\n");
			} else {
				ReadFromJetson();
				if (abort_) {
					isDone_ = true;
					nextState_ = kPivotInit;
					break;
				}
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
			if (abort_) {
				isDone_ = true;
				nextState_ = kPivotInit;
				break;
			}
            // TODO: REDO ALLLLLLL THE MATH HERE
			if (fabs(desiredDistance_) > 2.0/12.0) {	// 2 in threshold
				printf("DISTANCE FOR COMMAND: %f\n", desiredDistance_);
				driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, anglePIDOutput_, distancePIDOutput_,
						robot_, desiredDistance_);
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
		default:
			printf("default in align tape\n");
			break; //unecessary?
	}
	currState_ = nextState_;
	//printf("moving to next state in align with tape\n");

	if (robot_->GetTime() - initTimeAlign_ > 4.0) {	// Timeout for align with tape command
		isDone_ = true;
		printf("align with tape is DONE TIMEOUT\n robot time is %f, init time is %f, diff time is %f\n",
			robot_->GetTime(), initTimeAlign_, robot_->GetTime() - initTimeAlign_);
	}
}

bool AlignWithTapeCommand::IsDone() {
    return isDone_;
}

bool AlignWithTapeCommand::Abort() {
	return abort_;
}

void AlignWithTapeCommand::ReadFromJetson() {
    printf("starting read from jetson\n");

	//try {
	string contents = s_recv(*subscriber_);
	printf("contents from jetson: %s\n", contents.c_str());
	stringstream ss(contents);
	vector<string> result;

	while(ss.good()) {
		string substr;
		getline( ss, substr, ' ' );
		if (substr == "") {
			continue;
		}
		result.push_back( substr );
	}
	
	contents = contents.c_str();
	if(!contents.empty() && result.size() > 1) {
		desiredDeltaAngle_ = stod(result.at(0));
		desiredDistance_ = stod(result.at(1));
	} else {
		abort_ = true;
		printf("contents empty in alignwithtape\n");
	}

	if(result.size() > 1) {
		desiredDeltaAngle_ = stod(result.at(0));
		desiredDistance_ = stod(result.at(1))-1.5;//1.6;
	} else {
		abort_ = true;
		desiredDeltaAngle_ = 0.0;
		desiredDistance_ = 0.0;
	}
	printf("desired delta angle at %f in AlignWithTapeCommand\n", desiredDeltaAngle_);
		
		
	/*} catch (const std::exception &exc) {
		printf("TRY CATCH FAILED IN READFROMJETSON\n");
		std::cout << exc.what() << std::endl;
		desiredDeltaAngle_ = 0.0;
		// desiredDistance_ = 0.0;
	}*/
	printf("end of read from jetson\n");
}

void AlignWithTapeCommand::Reset() {
	isDone_ = true;
}

AlignWithTapeCommand::~AlignWithTapeCommand() {

}