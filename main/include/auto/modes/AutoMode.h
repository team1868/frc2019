#include "../../RobotModel.h"
#include "../commands/CurveCommand.h"
#include "../commands/DriveStraightCommand.h"
#include "../commands/PivotCommand.h"
#include "../commands/WaitingCommand.h"
#include "../commands/HatchBeakCommand.h"
#include "../commands/AlignWithTapeCommand.h"
#include "../commands/OuttakeHatchCommand.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"

class AutoMode {
public:
    enum AutoPositions {kBlank, kLeft, kMiddle, kRight};
	enum HabLevel {k1, k2};

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

    virtual void CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {};

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
					tempCommand = new AlignWithTapeCommand(robot_, navX_, talonEncoder_, false);
				} else {
					tempCommand = new AlignWithTapeCommand(robot_, navX_, talonEncoder_, true);
				}
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

protected:
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