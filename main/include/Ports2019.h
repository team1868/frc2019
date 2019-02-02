
//NOTE: Ports changed, look into Phoenix tuner to update

//Currently for practice bot - Valkarie

static const int LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT = 0;
static const int RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT = 2;
static const int LEFT_DRIVE_ENCODER_RED_PWM_PORT     = 1;
static const int RIGHT_DRIVE_ENCODER_RED_PWM_PORT    = 3;

static const int PNEUMATICS_CONTROL_MODULE_ID        = 4; //INCORRECT, TODO
static const int GEAR_SHIFT_FORWARD_SOLENOID_PORT    = 5; //INCORRECT, TODO
static const int GEAR_SHIFT_REVERSE_SOLENOID_PORT    = 6; //INCORRECT, TODO

static const int LEFT_DRIVE_MOTOR_A_PDP_CHAN = 15; //INCORRECT, TODO
static const int LEFT_DRIVE_MOTOR_B_PDP_CHAN = 14; //INCORRECT, TODO
static const int RIGHT_DRIVE_MOTOR_A_PDP_CHAN = 0;
static const int RIGHT_DRIVE_MOTOR_B_PDP_CHAN = 1;

//-----------------------Drive Talon IDs---------------------
static const int LEFT_DRIVE_MASTER_ID          = 4;
static const int LEFT_DRIVE_SLAVE_ID           = 5;
static const int RIGHT_DRIVE_MASTER_ID         = 1;
static const int RIGHT_DRIVE_SLAVE_ID          = 2;

//--------------------Joystick USB Ports---------------------

static const int LEFT_JOY_USB_PORT             = 0;
static const int RIGHT_JOY_USB_PORT            = 1;
static const int OPERATOR_JOY_USB_PORT         = 2;
static const int OPERATOR_JOY_B_USB_PORT       = 3;
//top 4 actually correct :)
static const int HIGH_LOW_GEAR_BUTTON_PORT     = 4;
static const int ARCADE_DRIVE_BUTTON_PORT      = 5;
static const int QUICK_TURN_BUTTON_PORT        = 6;
static const int DRIVE_DIRECTION_BUTTON_PORT   = 7;
