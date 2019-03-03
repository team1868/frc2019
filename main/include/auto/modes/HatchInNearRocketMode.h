#ifndef SRC_AUTO_MODES_HATCHINNEARROCKETMODE_H
#define SRC_AUTO_MODES_HATCHINNEARROCKETMODE_H

#include "AutoMode.h"

class HatchInNearRocketMode : public AutoMode {
public:
    HatchInNearRocketMode(RobotModel *robot);
    void CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) override;
    void Init();
    virtual ~HatchInNearRocketMode();
};

#endif /* SRC_AUTO_MODES_HATCHINNEARROCKETMODE_H */