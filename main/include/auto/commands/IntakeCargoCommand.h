#ifndef SRC_AUTO_COMMANDS_INTAKECARGOCOMMAND_H_
#define SRC_AUTO_COMMANDS_INTAKECARGOCOMMAND_H_

#include "../AutoCommand.h"
#include "../../RobotModel.h"

class IntakeCargoCommand: public AutoCommand {
public:
    IntakeCargoCommand(RobotModel *robot, double cargoIntakeMotorOutput = 1.0);
    virtual ~IntakeCargoCommand();

    void Init();

    void Reset();

    void Update(double currTimeSec, double deltaTimeSec);

    bool IsDone();

private:
    bool isDone_;

	RobotModel *robot_;

	double cargoIntakeMotorOutput_;
	double startTime_;
	double timeDiff_;
	bool wasJustRunning_;
};

#endif /* SRC_AUTO_COMMANDS_INTAKECOMMAND_H_ */