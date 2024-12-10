#include <TMCStepper.h>

// pins for encoders
#define CS_PIN_MOTOR 10
#define RSENSE 0.022F
#define DIAG_PIN 4
#define EN_PIN 3
#define BUF_LEN 50
#define CS_PIN_ENCODER 0

#define STEPS_PER_MM 1.25
#define USTEPS 64.0

#define NEW_ROT_THRESH 10000

#define DEBUG
#define ACC_UNIT_CONVERSION 0.015270994830222222
#define VEL_UNIT_CONVERSION 1.3981013333333334

#define ENCODER_TICKS 16384
#define RADS_PER_TICK PI/(ENCODER_TICKS/2)

#define MAX_POS 800.0
#define MAX_VEL 5000.0
#define MAX_ACC 30000.0
#define HOMING_POS 800.0 // position after hitting limit switch
#define HOMING_SPEED 80.0
#define HOMING_ACC 500.0

int16_t top_home = 0;
int16_t end_home = 0;

const uint16_t angle_bitmask = 0b0011111111111111;
const uint16_t clear_errors = 0b0100000000000001;
uint16_t read_angle = 0x3FFF;
const uint16_t read_diag = 0b1011111111111101;

struct State {
  unsigned long t;
  float pos;
  float vel;
  float top;
  float topvel;
  float end;
  float endvel;
};
union SixVec {
  char buf[24];
  float values[6];
};
enum Mode { RESET, USB, CLOSED_LOOP };

SixVec gains;
SixVec setpoint;

TMC5160Stepper motor = TMC5160Stepper(CS_PIN_MOTOR, RSENSE);
SPISettings settings = SPISettings(3000000, MSBFIRST, SPI_MODE1);

State state;
int32_t acc = 0;
float oldtopf = 0.0;
float oldendf = 0.0;
int16_t top = 0;
int16_t end = 0;
int16_t oldtop = 0;
int16_t oldend = 0;
int16_t toprots = 0;
int16_t endrots = 0;
uint16_t toperr = 0;
uint16_t enderr = 0;

static float topbuf[BUF_LEN] = {0.0};
static float endbuf[BUF_LEN] = {0.0};
uint16_t buf_ptr = 0;

unsigned long looptimer;
unsigned long looptimer2;
unsigned long dt;

bool stalled, limits_hit;
Mode mode = Mode::USB;

void clear_error_flag() {
  // Serial.println("clearing error flag");
  SPI1.beginTransaction(settings);
  //tell it to clear errors
  digitalWrite(CS_PIN_ENCODER, LOW);
  SPI1.transfer16(clear_errors);
  SPI1.transfer16(clear_errors);
  digitalWrite(CS_PIN_ENCODER, HIGH);
  delayMicroseconds(1);
  // then request angle again so other code still works
  digitalWrite(CS_PIN_ENCODER, LOW);
  toperr = SPI1.transfer16(read_diag);
  enderr = SPI1.transfer16(read_diag);
  digitalWrite(CS_PIN_ENCODER, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS_PIN_ENCODER, LOW);
  SPI1.transfer16(read_angle);
  SPI1.transfer16(read_angle);
  digitalWrite(CS_PIN_ENCODER, HIGH);
  SPI1.endTransaction();
}

byte spiCalcEvenParity(word value){
  byte cnt = 0;
  byte i;

  for (i = 0; i < 16; i++) {
    if (value & 0x1) {
      cnt++;
    }
    value >>= 1;
  }
  return cnt & 0x1;
}

void update_encoder_data() {
  oldend = end;
  oldtop = top;

  SPI1.beginTransaction(settings);
  digitalWrite(CS_PIN_ENCODER, LOW);
  end = (int16_t)(SPI1.transfer16(read_angle) & angle_bitmask) - end_home;
  top = (int16_t)(SPI1.transfer16(read_angle) & angle_bitmask) - top_home;
  digitalWrite(CS_PIN_ENCODER, HIGH);
  SPI1.endTransaction();
  looptimer2 = ARM_DWT_CYCCNT;
  dt = looptimer2-looptimer;
  looptimer = looptimer2;
 
  if (top-oldtop > NEW_ROT_THRESH) {
    toprots--;
  } else if (top-oldtop < -NEW_ROT_THRESH) {
    toprots++;
  }
  if (end-oldend > NEW_ROT_THRESH) {
    endrots--;
  } else if (end-oldend < -NEW_ROT_THRESH) {
    endrots++;
  }
  oldtopf = state.top;
  oldendf = state.end;
  state.top = toprots*2*PI + (float)top*RADS_PER_TICK;
  state.end = endrots*2*PI + (float)end*RADS_PER_TICK;

  state.topvel -= topbuf[buf_ptr];
  topbuf[buf_ptr] = ((state.top-oldtopf)/((float)dt/(float)F_CPU))/(float)BUF_LEN;
  state.topvel += topbuf[buf_ptr];
  state.endvel -= endbuf[buf_ptr];
  endbuf[buf_ptr] = ((state.end-oldendf)/((float)dt/(float)F_CPU))/(float)BUF_LEN;
  state.endvel += endbuf[buf_ptr];
  buf_ptr = (buf_ptr+1)%BUF_LEN;
  state.t = micros();
}

float clamp_acc(float a) {
  const double t = a < -MAX_ACC ? -MAX_ACC : a;
  return t > MAX_ACC ? MAX_ACC : t;
}
float clamp_vel(float v) {
  const double t = v < -MAX_VEL ? -MAX_VEL : v;
  return t > MAX_VEL ? MAX_VEL : t;
}
float clamp_pos(float v) {
  const double t = v < -MAX_POS ? -MAX_POS : v;
  return t > MAX_POS ? MAX_POS : t;
}

void tmc_init() {
    motor.begin();

    CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01;
    chopconf.toff = 5;
    chopconf.intpol = true;
    chopconf.hend = 0 + 3;
    chopconf.hstrt = 0 - 1;
    // TERN_(SQUARE_WAVE_STEPPING, chopconf.dedge = true);
    motor.CHOPCONF(chopconf.sr);

    motor.rms_current(1800, 0.5);
    motor.microsteps(USTEPS);
    motor.iholddelay(10);
    motor.TPOWERDOWN(128); // ~2s until driver lowers to hold current
    motor.diag0_stall(true);
    motor.en_pwm_mode(0);
    // motor.stored.stealthChop_enabled = 0;

    TMC2160_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    motor.PWMCONF(pwmconf.sr);
    // TERN(HYBRID_THRESHOLD, motor.set_pwm_thrs(hyb_thrs), UNUSED(hyb_thrs));
    motor.GSTAT(); // Clear GSTAT
}
void setup() {
  Serial.begin(250000);
  word command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | read_angle;
	//Add a parity bit on the the MSB
	command |= ((word)spiCalcEvenParity(command)<<15);
  read_angle = (uint16_t)(command & 0xFFFF);
  pinMode(DIAG_PIN, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(CS_PIN_ENCODER, OUTPUT);
  pinMode(CS_PIN_MOTOR, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(CS_PIN_ENCODER, HIGH);
  digitalWrite(CS_PIN_MOTOR, HIGH);

  SPI.begin();

  #ifdef DEBUG
  Serial.println(motor.test_connection());
  Serial.println(motor.DRV_STATUS(), BIN);
  #endif
  digitalWrite(EN_PIN, LOW);
  tmc_init();

  SPI1.begin();
  clear_error_flag();
  update_encoder_data();
  update_encoder_data();
  // encoder_timer.begin(update_encoder_data, 500);
}

void enter_reset() {
  digitalWrite(EN_PIN, HIGH);
  motor.VMAX(0);
  motor.AMAX((uint16_t)(MAX_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
  motor.RAMPMODE(1);
  mode = Mode::RESET;
  acc = 0.0;
}

void set_motor_acc() { 
  if (mode==Mode::CLOSED_LOOP) {
    acc =(- (state.pos    - setpoint.values[0]) * gains.values[0]
          - (state.vel    - setpoint.values[1]) * gains.values[1]
          - (state.top    - setpoint.values[2]) * gains.values[2]
          - (state.topvel - setpoint.values[3]) * gains.values[3]
          - (state.end    - setpoint.values[4]) * gains.values[4]
          - (state.endvel - setpoint.values[5]) * gains.values[5]);
  } else if (mode==Mode::RESET) {
    return;
  }

  motor.AMAX((uint16_t)(abs(acc)*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
  if (acc>0) {
    motor.RAMPMODE(1);
  } else if (acc<0) {
    motor.RAMPMODE(2);
  }

  // now gooo!!
  motor.VMAX((uint32_t)(MAX_VEL*STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION));
}
void deal_with_serial() {
  if (Serial.available()) {
    // first, reply with state. do this first so it's quick
    #ifdef DEBUG
      Serial.printf("{'t': %u, 'u': %f, 'cp': %f, 'cv': %f, 'tp': %f, 'tv': %f, 'ep': %f, 'ev': %f, 'dt': %u}\n", 
        micros(),
        (float)acc, 
        state.pos,
        state.vel,
        state.top,
        state.topvel,
        state.end,
        state.endvel,
        dt);
    #endif
    Serial.write((byte*)&state, sizeof(state));

    char cmd = Serial.read();
    if (mode==Mode::RESET) {
      digitalWrite(EN_PIN, HIGH);
      motor.VMAX(0);
      motor.AMAX((uint16_t)(MAX_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.RAMPMODE(1);
      // driver.RAMPMODE(1); // in case it wasn't already like this
      if (cmd!=3) {
        Serial.println("RESET:");
        if (stalled) { Serial.println("    MOTOR STALLED!"); }
        if (limits_hit) { Serial.println("    LIMITS HIT!"); }
        Serial.println("Use CLEAR RESET (0x03) to clear.");
        return;
      }
    }
    //* command 0x00 = SET ACCELERATION (enter usb mode)
    if (cmd==0) {
      mode = Mode::USB;
      char buf[4];
      Serial.readBytes(buf, 4);
      acc = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);
      acc = clamp_acc(*(float*)&acc);
    } 
    //* command 0x01 = SET POSITION
    else if (cmd==1) {
      char buf[4];
      Serial.readBytes(buf, 4);
      motor.RAMPMODE(0);
      motor.AMAX((uint16_t)(HOMING_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.DMAX((uint16_t)(HOMING_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.d1((uint16_t)(HOMING_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.a1((uint16_t)(HOMING_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      uint32_t data = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);
      data = clamp_pos(*(float*)&data);
      motor.XTARGET((int32_t)(data*STEPS_PER_MM*USTEPS));
      
      do {
        delay(5);
      } while (!motor.position_reached());
      

      motor.VMAX(0);
      motor.RAMPMODE(1);
      
    }
    //* command 0x02 = STAHP (RESET)
    else if (cmd==2) {
      enter_reset();
    }
    //* command 0x03 = CLEAR RESET
    else if (cmd==3) {
      digitalWrite(EN_PIN, LOW);
      motor.VMAX(0);
      motor.RAMPMODE(1);
      motor.XACTUAL(0);
      stalled = false;
      limits_hit = false;
      mode = Mode::USB;
    }
    //* command >= 0x04 = ZERO X AXIS
    else if (cmd==4) {
      motor.AMAX((uint16_t)(MAX_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.VMAX(0);
      motor.RAMPMODE(1);
      delay(5);
      motor.XACTUAL(0);
      // driver.sg_stop(true);
      // driver.VMAX((uint16_t)(HOMING_SPEED*STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION));
      // driver.AMAX((uint16_t)(HOMING_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION));
      // // motor now going towards limit, will stallguard stop
      // while (driver.VACTUAL()>0);
      // driver.sg_stop(false);
      // driver.XACTUAL((int32_t)(HOMING_POS*STEPS_PER_MM*USTEPS));
      // driver.RAMPMODE(0);
      // driver.XACTUAL(0);
      // while (driver.VACTUAL()>0);
      // driver.VMAX(0);
      // driver.RAMPMODE(1);
    }
    //* command 0x05 = SOFT STOP
    else if (cmd==5) {
      motor.AMAX((uint16_t)(0.5*MAX_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.VMAX(0);
      acc = 0.0;
    }
    //* command 0x06 = NOOP (get state)
    else if (cmd==6) {}
    //* command 0x07 = SET FEEDBACK GAINS
    else if (cmd==7) {
      Serial.readBytes(gains.buf, 24);
      #ifdef DEBUG
        Serial.printf("gains: %f, %f, %f, %f, %f, %f\n", 
        gains.values[0], gains.values[1], gains.values[2], 
        gains.values[3], gains.values[4], gains.values[5]);
      #endif
    }
    //* command 0x08 = SET SETPOINT
    else if (cmd==8) {
      Serial.readBytes(setpoint.buf, 24);
      #ifdef DEBUG
        Serial.printf("setpoint: %f, %f, %f, %f, %f, %f\n", 
        setpoint.values[0], setpoint.values[1], setpoint.values[2], 
        setpoint.values[3], setpoint.values[4], setpoint.values[5]);
      #endif
    }
    //* command 0x09 = RUN CLOSED LOOP
    else if (cmd==9) {
      mode = Mode::CLOSED_LOOP;
    }
  }
}

void update_motor_data() {
  state.vel = (float)(motor.VACTUAL())/(STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION);
  state.pos = (float)(motor.XACTUAL())/(STEPS_PER_MM*USTEPS);
  if (abs(state.pos)>MAX_POS) {
    limits_hit = true;
    enter_reset();
  }
}

void loop() {
  deal_with_serial();
  update_encoder_data();
  update_motor_data();
}