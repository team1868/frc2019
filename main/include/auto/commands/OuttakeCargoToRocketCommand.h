#ifndef SRC_AUTO_COMMANDS_OUTTAKECARGOTOROCKETCOMMAND_H_
#define SRC_AUTO_COMMANDS_OUTTAKECARGOTOROCKETCOMMAND_H_

#include "../AutoCommand.h"
#include "../../RobotModel.h"

class OuttakeCargoToRocketCommand: public AutoCommand {
public:
    OuttakeCargoToRocketCommand(RobotModel *robot);

    virtual ~OuttakeCargoToRocketCommand();

    void Init();
    void Reset();
    void Update(double currTimeSec, double deltaTimeSec);
    bool IsDone();

private:
    bool isDone_;
	RobotModel *robot_;
    double cargoIntakeMotorOutput_;
	double outtakeCargoToRocketMotorOutput_;
	double startTime_;
	double deltaTime_;
    double deltaFlywheelStartTime_;
};

#endif /* SRC_AUTO_COMMANDS_OUTTAKECARGOTOROCKETCOMMAND_H_ */