#ifndef SRC_AUTO_MODES_HATCHINSIDECARGMODE_H
#define SRC_AUTO_MODES_HATCHINSIDECARGMODE_H

#include "AutoMode.h"

class HatchInSideCargoMode : public AutoMode {
public:
    HatchInSideCargoMode(RobotModel *robot, ControlBoard *controlBoard);
    void CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) override;
    void Init();
    virtual ~HatchInSideCargoMode();
};

#endif /* SRC_AUTO_MODES_HATCHINSIDECARGMODE_H */