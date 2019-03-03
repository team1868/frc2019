#ifndef SRC_AUTO_COMMANDS_ALIGNWITHTAPECOMMAND_H
#define SRC_AUTO_COMMANDS_ALIGNWITHTAPECOMMAND_H

#include "../AutoCommand.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"
#include "PivotCommand.h"
#include "DriveStraightCommand.h"
#include "../../../../ext/zmq.hpp"
#include "../../../../ext/zhelpers.hpp"
#include <string>
#include <chrono>
#include <thread>
#include <memory>
#include <fstream>
#include <iostream>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif

using namespace std;

class AlignWithTapeCommand : public AutoCommand {
public:
    AlignWithTapeCommand(RobotModel* robot, NavXPIDSource* navXSource, TalonEncoderPIDSource* talonSource, bool driveStraightDesired);
    virtual ~AlignWithTapeCommand();

    void Init();
    void Update(double currTimeSec, double deltaTimeSec);
    void Reset();
    
    bool IsDone();

    void ReadFromJetson();

private:
    zmq::context_t *context_;
	zmq::socket_t *subscriber_;

    RobotModel* robot_;
    NavXPIDSource* navXSource_;
    TalonEncoderPIDSource* talonSource_;
    AnglePIDOutput* anglePIDOutput_;
    DistancePIDOutput* distancePIDOutput_;

    PivotCommand* pivotCommand_;
    DriveStraightCommand* driveStraightCommand_;

    // TODO the below is for after a pretty good dead reckon, and would work for curve command
    double desiredDeltaAngle_;
    double desiredDistance_;

    enum AlignState{ kPivotInit, kPivotUpdate, kDriveInit, kDriveUpdate };
    uint32_t currState_;
    uint32_t nextState_;

    double initTimeVision_, initTimeAlign_;

    bool driveStraightDesired_;

    bool isDone_;
};

#endif /* SRC_AUTO_COMMANDS_ALIGNWITHTAPECOMMAND_H */
