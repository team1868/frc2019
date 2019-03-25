#ifndef SRC_AUTO_COMMANDS_OUTTAKECARGOTOSHIPCOMMAND_H_
#define SRC_AUTO_COMMANDS_OUTTAKECARGOTOCARGOSHIPCOMMAND_H_

#include "../AutoCommand.h"
#include "../../RobotModel.h"

class OuttakeCargoToShipCommand: public AutoCommand {
public:
    OuttakeCargoToShipCommand(RobotModel *robot);

    virtual ~OuttakeCargoToShipCommand();

    void Init();
    void Reset();
    void Update(double currTimeSec, double deltaTimeSec);
    bool IsDone();

private:
    bool isDone_;
	RobotModel *robot_;
    double cargoIntakeMotorOutput_;
	double outtakeCargoToCargoShipMotorOutput_;
	double startTime_;
	double deltaTime_;
    double deltaFlywheelStartTime_;
};

#endif /* SRC_AUTO_COMMANDS_OUTTAKECARGOTOROCKETCOMMAND_H_ */