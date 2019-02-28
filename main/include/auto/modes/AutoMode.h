#include "../../RobotModel.h"
#include "../commands/CurveCommand.h"
#include "../commands/DriveStraightCommand.h"
#include "../commands/PivotCommand.h"
#include "../commands/WaitingCommand.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"

class AutoMode {
public:
    enum AutoPositions {kBlank, kLeft, kMiddle, kRight};

    AutoMode(RobotModel *robot) {
        printf("constructing automode\n");

        firstCommand_ = NULL;
		currentCommand_ = NULL;
		robot_ = robot;
		navX_ = new NavXPIDSource(robot_);
		talonEncoder_ = new TalonEncoderPIDSource(robot_);
		angleOutput_ = new AnglePIDOutput();
		distanceOutput_ = new DistancePIDOutput();
		breakDesired_ = false;
		currAngle_ = robot_->GetNavXYaw();

		printf("Done constructing AutoMode\n");
    };

    virtual ~AutoMode() {};

    virtual void CreateQueue(AutoMode::AutoPositions pos) {};

    void QueueFromString(string autoSequence) {
        firstCommand_ = NULL;
		currentCommand_ = NULL;
		AutoCommand *lastCommand = NULL;
		iss.str (autoSequence);
		cout << string ("autosequence" ) + autoSequence << endl;
		breakDesired_ = false;
		currAngle_ = robot_->GetNavXYaw();

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

    AutoCommand* GetStringCommand(char command) {
		AutoCommand* tempCommand = NULL;

		switch(command) {
		case '[':
			char charA;
			iss >> charA;
			printf("Command %c ", charA);
			AutoCommand* commandA = GetStringCommand(charA);
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
		// case 'i':
		// 	printf("Intake hatch command\n");
		// case 'i':
		// 	printf("Inttake Command\n");
		// 	double intakeOutput;
		// 	iss >> intakeOutput;
		// 	if(IsFailed(command)) {
		// 		tempCommand = NULL;
		// 	} else {
		// 		tempCommand = new IntakeCommand(robot_, intakeOutput);
		// 	}
		// 	break;
		// case 'o':   // Outtake
		// 	printf("Outtake Command\n");
		// 	tempCommand = new OuttakeCommand(robot_);
		// 	break;
		// case 'e':
		// 	printf("Elevator Command\n");
		// 	double height;
		// 	iss >> height;
		// 	if (IsFailed(command)) {
		// 		tempCommand = NULL;
		// 	} else {
		// 		tempCommand = new ElevatorHeightCommand(robot_, height);
		// 	}
		// 	break;
		// case 'w':
		// 	printf("Wrist Command\n");
		// 	int wUp; // wrist up
		// 	iss >> wUp;
		// 	if (IsFailed(command)) {
		// 		tempCommand = NULL;
		// 	} else {
		// 		if (wUp == 1) {
		// 			tempCommand = new WristCommand(robot_, true);
		// 		} else {
		// 			tempCommand = new WristCommand(robot_, false);
		// 		}
		// 	}
		// 	break;
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
		// case 'z':
		// {
		// 	printf("Drive to Cube Command\n");
		// 	double num;
		// 	iss >> num;	// Can be any arbitrary value. Apparently without this it'll run the command twice. Might want to fix this someday
		// 	if (IsFailed(command)) {
		// 		tempCommand = NULL;
		// 	} else {
		// 		tempCommand = new DriveToCubeCommand(robot_, navX_, talonEncoder_, angleOutput_, distanceOutput_);
		// 	}
		// 	break;
		// }
		// case 'b':
		// {
		// 	printf("Intake Drive to Cube Command\n");
		// 	double num = 0;
		// 	iss >> num;	// Can be any arbitrary value. Apparently without this it'll run the command twice. Might want to fix this someday
		// 	if (IsFailed(command)) {
		// 		tempCommand = NULL;
		// 	} else {
		// 		tempCommand = new DriveIntakeCubeCommand(navX_, talonEncoder_, angleOutput_, distanceOutput_, robot_);
		// 	}
		// 	break;
		// }
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

    bool IsFailed(char command) {
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

    virtual void Init() = 0;

    void Update(double currTimeSec, double deltaTimeSec) {
		if (currentCommand_ != NULL) {
			//			printf("Update in automode running\n");
			if (currentCommand_->IsDone()) {
				//				DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
				currentCommand_->Reset();
				currentCommand_ = currentCommand_->GetNextCommand();
				if (currentCommand_ != NULL) {
					//					DO_PERIODIC(1, printf("Command start at: %f \n", currTimeSec));
					currentCommand_->Init();
					printf("Initializing current commmand\n");
				}
			} else {
				//				printf("Update current command\n");
				currentCommand_->Update(currTimeSec, deltaTimeSec);
			}
		} else {
			//			printf("Done with auto mode update\n");
		}
	}

    bool IsDone() {
		return (currentCommand_ == NULL);
	}

	void Disable(){
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

private:
    AutoCommand *firstCommand_;
	AutoCommand *currentCommand_;
	RobotModel* robot_;

	NavXPIDSource* navX_;
	TalonEncoderPIDSource* talonEncoder_;

	AnglePIDOutput *angleOutput_;
	DistancePIDOutput *distanceOutput_;

	istringstream iss;
	bool breakDesired_;
	double currAngle_;
};