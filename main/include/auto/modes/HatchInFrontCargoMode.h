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