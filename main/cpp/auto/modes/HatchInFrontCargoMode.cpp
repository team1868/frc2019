#include "../../../include/auto/modes/HatchInFrontCargoMode.h"

HatchInFrontCargoMode::HatchInFrontCargoMode(RobotModel *robot) : AutoMode(robot){
    robot_ = robot;
}

void HatchInFrontCargoMode::CreateQueue(AutoMode::AutoPositions pos, AutoMode::HabLevel hablvl) {
    AutoPositions autoPos = pos;
    HabLevel habLvl = hablvl;
    string autoSequence = "";

    switch(autoPos) {
        case kLeft:
            printf("kLeft for autopos\n");
            switch (habLvl) {
                case k1:
                    printf("kLeft and kHab1\n");
                    autoSequence = "h 0 d 10.0 t 90.0 d 3.0 t -90.0 d 1.3";
                    break;
                case k2:
                    printf("kLeft and kHab2\n");
                    autoSequence = "h 0 d 12.0 t 90.0 d 3.0 t -90.0 d 2.3";
                    break;
                case kNone:
                    printf("kLeft but kBlank hab\n");
                    autoSequence = "";
                    break;
                default:
                    printf("kLeft hab default\n");
                    autoSequence = "";
                    break;
            }
            // need to account for zoom off hab 2 though test 
            // need to travel 14.3ish feet forward
            // start 36.13 - robot_wdith from absolute middle
            // robot center starts 63.88 - robot_width/2.0 from absolute middle
            // need to move 53.005 - robot_width/2.0 in to the right
            // 10.875 in from absolute middle
            break;
        case kMiddle:
            printf("kMiddle autopos\n");
            autoSequence = "h 0 d 11.3 b 1 s 0.4 h 1";  // use align with beak.
            break;
        case kRight:
            printf("kRight for autopos\n");
            switch (habLvl) {
                case k1:
                    printf("kRight and kHab1\n");
                    autoSequence = "h 0 d 10.0 t -90.0 d 3.0 t 90.0 d 1.3";
                    break;
                case k2:
                    printf("kRight and kHab2\n");
                    autoSequence = "h 0 d 12.0 t -90.0 d 3.0 t 90.0 d 2.3";
                    break;
                case kNone:
                    printf("kRight but kBlank hab\n");
                    autoSequence = "";
                    break;
                default:
                    printf("kRight, no hab lvl indicated.treating as blank\n");
                    autoSequence = "";
                    break;
            }
            break;
        case kBlank:
            printf("blank autopos\n");
            autoSequence = "";
            break;
        default:
            printf("default autopos\n");
            autoSequence = "";
            break;
    }
}

void HatchInFrontCargoMode::Init() {
    printf("Hatch In Front Cargo Mode Init\n");
    currentCommand_->Init();
}

HatchInFrontCargoMode::~HatchInFrontCargoMode() {

}