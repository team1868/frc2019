
//NOTE: Ports changed, look into Phoenix tuner to update

//Currently for practice bot - Valkarie

static const int LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT  = 0;
static const int RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT = 2;
static const int LEFT_DRIVE_ENCODER_RED_PWM_PORT     = 1;
static const int RIGHT_DRIVE_ENCODER_RED_PWM_PORT    = 3;

static const int PNEUMATICS_CONTROL_MODULE_ID        = 0;
static const int GEAR_SHIFT_FORWARD_SOLENOID_PORT    = 0;
static const int GEAR_SHIFT_REVERSE_SOLENOID_PORT    = 1;

static const int LEFT_DRIVE_MOTOR_A_PDP_CHAN  = 15;
static const int LEFT_DRIVE_MOTOR_B_PDP_CHAN  = 14;
static const int LEFT_DRIVE_MOTOR_C_PDP_CHAN  = 13;
static const int RIGHT_DRIVE_MOTOR_A_PDP_CHAN = 0;
static const int RIGHT_DRIVE_MOTOR_B_PDP_CHAN = 1;
static const int RIGHT_DRIVE_MOTOR_C_PDP_CHAN = 2;

//-----------------------Drive Talon IDs---------------------
// ASSUMING INTAKE IS FRONT
static const int RIGHT_DRIVE_MASTER_ID        = 1;
static const int RIGHT_DRIVE_SLAVE_A_ID       = 2;
static const int RIGHT_DRIVE_SLAVE_B_ID       = 3;
static const int LEFT_DRIVE_MASTER_ID         = 4;
static const int LEFT_DRIVE_SLAVE_A_ID        = 5;
static const int LEFT_DRIVE_SLAVE_B_ID        = 6;

//--------------------Joystick USB Ports---------------------

static const int LEFT_JOY_USB_PORT            = 0;
static const int RIGHT_JOY_USB_PORT           = 1;
static const int OPERATOR_JOY_USB_PORT        = 2;
static const int OPERATOR_JOY_B_USB_PORT      = 3;
//top 4 actually correct :)
static const int HIGH_GEAR_BUTTON_PORT        = 4;
static const int LOW_GEAR_BUTTON_PORT         = 1;
static const int ARCADE_DRIVE_BUTTON_PORT     = 5;
static const int QUICK_TURN_BUTTON_PORT       = 6;
static const int DRIVE_DIRECTION_BUTTON_PORT  = 7;

static const int CARGO_INTAKE_BUTTON_PORT     = 5;
static const int CARGO_UNINTAKE_BUTTON_PORT   = 6;
static const int CARGO_FLYWHEEL_BUTTON_PORT   = 6; //unused
static const int CARGO_FLYWHEEL_ROCKET_BUTTON_PORT = -1; //TODO CHANGE THIS
static const int CARGO_INTAKE_WRIST_BUTTON_PORT    = -1;
static const int HATCH_DOUBLE_SOLENOID_BUTTON_PORT = 8; //NOTE 8 GUYS FOR HATCH    CHANGE THIS  TO HATCH PICK UP
static const int HATCH_OUTTAKE_BUTTON_PORT     = -1;
static const int HATCH_BEAK_BUTTON_PORT        = -1;

//----------------------Superstructure-----------------------
static const int CARGO_INTAKE_MOTOR_PORT                    = 8;
static const int CARGO_FLYWHEEL_MOTOR_PORT                  = 9;

static const int CARGO_INTAKE_MOTOR_PDP_CHAN                = 11; //TODO DOESN'T WORKKK, incorrect
static const int CARGO_FLYWHEEL_MOTOR_PDP_CHAN              = 12; //TODO DOESN'T WORKKK, incorrect

static const int CARGO_WRIST_UP_DOUBLE_SOLENOID_CHAN        = -1; //TODO CHANGE THESE TO ACTUAL
static const int CARGO_WRIST_DOWN_DOUBLE_SOLENOID_CHAN      = -1;

static const int HOOD_UP_DOUBLE_SOLENOID_CHAN               = -1; 
static const int HOOD_DOWN_DOUBLE_SOLENOID_CHAN             = -1;

static const int HATCH_OUTTAKE_DOUBLE_SOLENOID_FORWARD_CHAN = 2; //TODO DOESNT WORK - CHANGE THIS TO MOTOR, PICK UP NOT OUTTAKE
static const int HATCH_OUTTAKE_DOUBLE_SOLENOID_REVERSE_CHAN = 3; //TODO DOESNT WORK

static const int HATCH_OUTTAKE_OUT_DOUBLE_SOLENOID_CHAN     = -1;
static const int HATCH_OUTTAKE_DOWN_DOUBLE_SOLENOID_CHAN    = -1;

static const int HATCH_BEAK_CLOSED_DOUBLE_SOLENOID_CHAN     = -1;
static const int HATCH_BEAK_OPEN_DOUBLE_SOLENOID_CHAN       = -1;

static const int FLYWHEEL_ENCODER_A_PWM_PORT = -1; //INCORRECT, TODO
static const int FLYWHEEL_ENCODER_B_PWM_PORT = -1; //INCORRECT, TODO