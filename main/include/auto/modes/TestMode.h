#ifndef SRC_AUTO_MODES_TESTMODE_H_
#define SRC_AUTO_MODES_TESTMODE_H_

#include "AutoMode.h"

class TestMode : public AutoMode {
public:
    TestMode(RobotModel *robot);
    void CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) override;
    void Init();

    virtual ~TestMode();
private:

};

#endif /* SRC_AUTO_MODES_TESTMODE_H_ */