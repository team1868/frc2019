/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef SRC_AUTO_MODES_AUTOMODE_H
#define SRC_AUTO_MODES_AUTOMODE_H

#include "../../RobotModel.h"
#include "../../ControlBoard.h"
#include "../commands/CurveCommand.h"
#include "../commands/DriveStraightCommand.h"
#include "../commands/PivotCommand.h"
#include "../commands/WaitingCommand.h"
#include "../commands/WaitForButtonCommand.h"
#include "../commands/HatchBeakCommand.h"
#include "../commands/AlignWithTapeCommand.h"
#include "../commands/OuttakeHatchCommand.h"
#include "../commands/OuttakeCargoToShipCommand.h"
#include "../commands/CargoWristCommand.h"
#include "../commands/MotionProfiling/MotionProfileCommand.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"

class AutoMode {
public:
    enum AutoPositions {kBlank, kLeft, kMiddle, kRight};
	enum HabLevel {kNone, k1, k2};

    AutoMode(RobotModel *robot, ControlBoard *controlBoard);

    virtual ~AutoMode() {};

    virtual void CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {};

    void QueueFromString(string autoSequence);

    AutoCommand* GetStringCommand(char command);

    bool IsFailed(char command);

    virtual void Init() = 0;

    void Update(double currTimeSec, double deltaTimeSec);

    bool IsDone();

    bool Abort();

	void Disable();

protected:
    AutoCommand *firstCommand_;
	AutoCommand *currentCommand_;
	RobotModel* robot_;
    ControlBoard *humanControl_;

	NavXPIDSource* navX_;
	TalonEncoderPIDSource* talonEncoder_;

	AnglePIDOutput *angleOutput_;
	DistancePIDOutput *distanceOutput_;

	istringstream iss;
	bool breakDesired_;
	double currAngle_;
};

#endif /*SRC_AUTO_MODES_AUTOMODE_H*/