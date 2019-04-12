#include "../../../include/auto/modes/AutoMode.h"

AutoMode::AutoMode(RobotModel* robot) {
    printf("constructing automode\n");

        firstCommand_ = NULL;
		currentCommand_ = NULL;
		robot_ = robot;
		navX_ = robot_->GetNavXSource();
		//navX_ = new NavXPIDSource(robot_);
		talonEncoder_ = new TalonEncoderPIDSource(robot_);
		angleOutput_ = new AnglePIDOutput();
		distanceOutput_ = new DistancePIDOutput();
		breakDesired_ = false;
		currAngle_ = 0.0;//robot_->GetNavXYaw();

	printf("Done constructing AutoMode\n");
}

void AutoMode:: QueueFromString(string autoSequence) {
    firstCommand_ = NULL;
		currentCommand_ = NULL;
		AutoCommand *lastCommand = NULL;
		iss.str (autoSequence);
		cout << string ("autosequence" ) + autoSequence << endl;
		breakDesired_ = false;
		currAngle_ = 0.0;//robot_->GetNavXYaw();

		if (autoSequence == "") {
			printf("NO SEQUENCE ! TRY AGAIN KID");
		}

		//printf("AUto sequence: %s", autoSequence.c_str());

		while (!iss.eof() && !breakDesired_) {
			AutoCommand* tempCommand = NULL;
			char command;
			iss >> command;
			printf("Command: %c, ", command);

			tempCommand = GetStringCommand(command);

			if (firstCommand_ == NULL) {
				firstCommand_ = tempCommand;
				currentCommand_ = firstCommand_;
				lastCommand = currentCommand_;
			} else {
				lastCommand->SetNextCommand(tempCommand);
				lastCommand = lastCommand->GetNextCommand();
			}
		}
		iss.clear();
}

AutoCommand* AutoMode::GetStringCommand(char command) {
		AutoCommand* tempCommand = NULL;
		AutoCommand* commandA = NULL;

		switch(command) {
		case '[':
			char charA;
			iss >> charA;
			printf("Command %c ", charA);
			commandA = GetStringCommand(charA);
			tempCommand = commandA;

			charA = NULL;
			iss >> charA;
			while (charA != ']') {
				printf("bleh %c\n", charA);
				AutoCommand* memeCommand  = GetStringCommand(charA);

				commandA->SetNextCommand(memeCommand);
				commandA = commandA->GetNextCommand();
				charA = NULL;
				iss >> charA;
			}

			double rand;
			iss >> rand;
			break;
		case 't':	// Pivots with absolute position
			double angle;
			iss >> angle;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				currAngle_ = angle;
				printf("Angle: %f\n", angle);
				tempCommand = new PivotCommand(robot_, angle, true, navX_);
			}
			break;
		case 'd':	// Drive straight
			double distance;
			iss >> distance;
			if(IsFailed(command)) {
				tempCommand = NULL;
			} else {
				printf("Distance: %f\n", distance);
				tempCommand = new DriveStraightCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_, distance, currAngle_);
			}
			break;
		case 'c':	// curve command
			double curveRadius;
			double curveAngle;
			int turnLeft;
			iss >> curveRadius;
			iss >> curveAngle;
			iss >> turnLeft;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				printf("radius: %f\n, angle: %f\n, turnleft: %d\n", curveRadius, curveAngle, turnLeft);
				if (turnLeft == 0) {
					tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, false, navX_, talonEncoder_, angleOutput_, distanceOutput_);
				} else {
					tempCommand = new CurveCommand(robot_, curveRadius, curveAngle, true, navX_, talonEncoder_, angleOutput_, distanceOutput_);
				}
			}
			break;
		case 'b':	// hatch beak command
			int beakOpenDesired;	// 1 = true = open, 0 = false = closed
			iss >> beakOpenDesired;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				if (beakOpenDesired == 0) {
					tempCommand = new HatchBeakCommand(robot_, false);
				} else {
					tempCommand = new HatchBeakCommand(robot_, true);
				}
			}
			break;
		case 'h':	// hatch outtake popper command
			int hatchOutDesired;	// 1 = true = out, 0 = false = in
			iss >> hatchOutDesired;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				if (hatchOutDesired == 0) {
					tempCommand = new OuttakeHatchCommand(robot_, false);
				} else {
					tempCommand = new OuttakeHatchCommand(robot_, true);
				}
			}
			break;
		case 'a':	// align with tape command
			int driveStraightDesired;	// 1 = true = drive, 0 = false = just pivot
			iss >> driveStraightDesired;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				if (driveStraightDesired == 0) {
					tempCommand = new AlignWithTapeCommand(robot_, navX_, talonEncoder_, false); //, false
				} else {
					tempCommand = new AlignWithTapeCommand(robot_, navX_, talonEncoder_, true);
				}
			}
			break;
		case '^':
			printf("Outtake cargo to cargo ship command\n");
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new OuttakeCargoToShipCommand(robot_);
			}
			break;
		case 'w':
			printf("intaking with cargo wrist command\n");
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new CargoWristCommand(robot_);
			}
			break;
		case 's':
			printf("Wait Command\n");
			double waitTime;
			iss >> waitTime;
			if (IsFailed(command)) {
				tempCommand = NULL;
			} else {
				tempCommand = new WaitingCommand(waitTime);
			}
			break;
		default:	// When it's not listed, don't do anything :)
			printf("Unexpected character %c detected. Terminating queue", command);
			firstCommand_ = NULL;
			currentCommand_ = NULL;
			tempCommand = NULL;
			breakDesired_ = true;
			break;
		}

		return tempCommand;
}

bool AutoMode::IsFailed(char command) {
    if (iss.fail()) {
        iss.clear();
        printf("Unexpected character detected after %c. Terminating queue", command);
        firstCommand_ = NULL;
        currentCommand_ = NULL;
        breakDesired_ = true;
        return true;
    }
    return false;
}

void AutoMode::Update(double currTimeSec, double deltaTimeSec) {
    if (currentCommand_ != NULL) {
        //			printf("Update in automode running\n");
		// if (currentCommand_->Abort()) {
		// 	currentCommand_->Reset();
		// 	printf("aborting auto sequence. start driver control\n");
		// }
        if (currentCommand_->IsDone()) {
            //				DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
            currentCommand_->Reset();
			AutoCommand *nextCommand = currentCommand_->GetNextCommand();
			// currentCommand_->~AutoCommand();
            // delete currentCommand_;
			currentCommand_ = nextCommand;
            if (currentCommand_ != NULL) {
                //					DO_PERIODIC(1, printf("Command start at: %f \n", currTimeSec));
                currentCommand_->Init();
                printf("Initializing current commmand\n");
				printf("AM I DONE???? %d\n", currentCommand_->IsDone());
            } else {
				printf("DONE WITH LIFE HAIHSIDFALKSDJFLASKJDFLAKSHDFLKSJF\n");
			}
        } else {
            //				printf("Update current command\n");
            currentCommand_->Update(currTimeSec, deltaTimeSec);
        }
    } else {
        //			printf("Done with auto mode update\n");
    }
}

bool AutoMode::IsDone() {
    return (currentCommand_ == NULL);
}

bool AutoMode::Abort() {
	return currentCommand_->Abort();
}

void AutoMode::Disable() {
    printf("Disabling\n");
    if (!IsDone()) {
        printf("Resetting current command\n");
        currentCommand_->Reset();
    }
    if (firstCommand_ != NULL) {
        currentCommand_ = firstCommand_;
        AutoCommand* nextCommand;
        while (currentCommand_ != NULL) {
            nextCommand = currentCommand_->GetNextCommand();
            delete(currentCommand_);
            currentCommand_ = nextCommand;
        }
    }

    printf("Successfully disabled\n");
}