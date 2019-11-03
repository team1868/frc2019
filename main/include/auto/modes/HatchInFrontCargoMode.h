/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef SRC_AUTO_MODES_HATCHINFRONTCARGOMODE_H
#define SRC_AUTO_MODES_HATCHINFRONTCARGOMODE_H

#include "AutoMode.h"

class HatchInFrontCargoMode : public AutoMode {
public:
    HatchInFrontCargoMode(RobotModel *robot);
    void CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) override;
    void Init();
    virtual ~HatchInFrontCargoMode();
};

#endif /* SRC_AUTO_MODES_HATCHINFRONTCARGOMODE_H */