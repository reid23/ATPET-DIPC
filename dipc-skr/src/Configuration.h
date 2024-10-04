#define DIAG_PIN 0x3C
#define TMC_SW_MOSI 0x31
#define TMC_SW_MISO 0x05
#define TMC_SW_SCK 0x04
#define CS_PIN 0x28
#define RSENSE 0.022F
#define EN_PIN 0x48

#define STEPS_PER_MM 150.0
#define USTEPS 128

#define ACC_UNIT_CONVERSION 0.015270994830222222
#define VEL_UNIT_CONVERSION 1.3981013333333334

#define ENCODER_TICKS 16384
#define RADS_PER_TICK PI/(ENCODER_TICKS/2)

#define MAX_VEL 5000
#define MAX_ACC 20000.0 // all accelerations are 1000x bigger
#define HOMING_POS 800 // position after hitting limit switch
#define HOMING_SPEED 100.0
#define HOMING_ACC 500.0 // all accelerations are 1000x bigger