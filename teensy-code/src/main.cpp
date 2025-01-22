#include <TMCStepper.h>
#include <EEPROM.h>

// pins for encoders
#define CS_PIN_MOTOR 10
#define RSENSE 0.022F
#define DIAG_PIN 4
#define EN_PIN 3
#define BUF_LEN 20
#define VEL_BUF_LEN 35
#define CS_PIN_ENCODER 0

#define STEPS_PER_MM 1.25
#define USTEPS 64.0

#define NEW_ROT_THRESH 10000

// #define DEBUG
#define ACC_UNIT_CONVERSION 0.015270994830222222
#define VEL_UNIT_CONVERSION 1.3981013333333334

#define ENCODER_TICKS 16384
#define RADS_PER_TICK PI/(ENCODER_TICKS/2)

#define MAX_POS 800.0
#define MAX_VEL 5000.0
#define MAX_ACC 20000.0
#define HOMING_POS 800.0 // position after hitting limit switch
#define HOMING_SPEED 80.0
#define HOMING_ACC 500.0

#define X_OFFSET 3.0*MAX_POS*STEPS_PER_MM*USTEPS

float top_home = 1.9822291272403798;
float end_home = -2.6149389132870287;

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
enum Mode { RESET, USB, CLOSED_LOOP, SOFT_STOP };

SixVec gains;
SixVec setpoint;

TMC5160Stepper motor = TMC5160Stepper(CS_PIN_MOTOR, RSENSE);
SPISettings settings = SPISettings(3000000, MSBFIRST, SPI_MODE1);

State state;
float acc = 0;
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
static float topvelbuf[VEL_BUF_LEN] = {0.0};
static float endvelbuf[VEL_BUF_LEN] = {0.0};
uint16_t buf_ptr = 0;
uint16_t vel_buf_ptr = 0;

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
  end = -((int16_t)(SPI1.transfer16(read_angle) & angle_bitmask) - end_home);
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
  state.top -= topbuf[buf_ptr];
  topbuf[buf_ptr] = (toprots*2*PI + (float)top*RADS_PER_TICK - top_home)/(float)BUF_LEN;
  state.top += topbuf[buf_ptr];
  state.end -= endbuf[buf_ptr];
  endbuf[buf_ptr] = (endrots*2*PI + (float)end*RADS_PER_TICK - end_home)/(float)BUF_LEN;
  state.end += endbuf[buf_ptr];
  buf_ptr = (buf_ptr + 1)%BUF_LEN;

  state.topvel -= topvelbuf[vel_buf_ptr];
  topvelbuf[vel_buf_ptr] = ((state.top-oldtopf)/((float)dt/(float)F_CPU))/(float)VEL_BUF_LEN;
  state.topvel += topvelbuf[vel_buf_ptr];
  state.endvel -= endvelbuf[vel_buf_ptr];
  endvelbuf[vel_buf_ptr] = ((state.end-oldendf)/((float)dt/(float)F_CPU))/(float)VEL_BUF_LEN;
  state.endvel += endvelbuf[vel_buf_ptr];
  vel_buf_ptr = (vel_buf_ptr+1)%VEL_BUF_LEN;
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
    chopconf.hend = 1 + 3;
    chopconf.hstrt = 1 - 1;
    // TERN_(SQUARE_WAVE_STEPPING, chopconf.dedge = true);
    motor.CHOPCONF(chopconf.sr);

    motor.rms_current(2500, 0.5);
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
    motor.RAMPMODE(1);
    motor.VMAX(0);
    motor.XACTUAL(X_OFFSET);
}
void setup() {
  Serial.begin(115200);
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
  gains.values[0] = 100;
  gains.values[1] = 10;

  // EEPROM.read(0);

  uint32_t top_home_int = (((uint32_t)EEPROM.read(0) << 24) + ((uint32_t)EEPROM.read(1) << 16) + ((uint32_t)EEPROM.read(2) << 8) + (uint32_t)EEPROM.read(3));
  top_home = *(float*)&top_home_int;
  uint32_t end_home_int = (((uint32_t)EEPROM.read(4) << 24) + ((uint32_t)EEPROM.read(5) << 16) + ((uint32_t)EEPROM.read(6) << 8) + (uint32_t)EEPROM.read(7));
  end_home = *(float*)&end_home_int;
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
float fix_angle(float theta) {
  return theta - 2*PI * floorf((theta+PI)/(2*PI));
}
void set_motor_acc() { 
  if (mode==Mode::CLOSED_LOOP) {
    acc =(- (state.pos    - setpoint.values[0]) * gains.values[0] * 0.001
          - (state.vel    - setpoint.values[1]) * gains.values[1] * 0.001
          - fix_angle(state.top    - setpoint.values[2]) * gains.values[2]
          - (state.topvel - setpoint.values[3]) * gains.values[3]
          - fix_angle(state.end    - setpoint.values[4]) * gains.values[4]
          - (state.endvel - setpoint.values[5]) * gains.values[5]) * 1000;
  } else if (mode==Mode::RESET) {
    return;
  }

  if (mode==Mode::SOFT_STOP) {
    if (acc != 0.0) {
      mode=Mode::USB;
    } else {
      motor.AMAX((uint16_t)(0.5*MAX_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.VMAX(0);
      return;
    }
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
        acc, 
        state.pos,
        state.vel,
        state.top,
        state.topvel,
        state.end,
        state.endvel,
        dt);
    #endif
    Serial.write((byte*)&state, sizeof(state));
    Serial.send_now();

    char cmd = Serial.read();
    if (mode==Mode::RESET) {
      digitalWrite(EN_PIN, HIGH);
      motor.VMAX(0);
      motor.AMAX((uint16_t)(MAX_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.RAMPMODE(1);
      // driver.RAMPMODE(1); // in case it wasn't already like this
      if (cmd!=3) {
        #ifdef DEBUG
        Serial.println("RESET:");
        if (stalled) { Serial.println("    MOTOR STALLED!"); }
        if (limits_hit) { Serial.println("    LIMITS HIT!"); }
        Serial.println("Use CLEAR RESET (0x03) to clear.");
        #endif
        return;
      }
    }
    //* command 0x00 = SET ACCELERATION (enter usb mode)
    if (cmd==0) {
      mode = Mode::USB;
      char buf[4];
      Serial.readBytes(buf, 4);
      uint32_t acc_int = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);
      acc = clamp_acc(*(float*)&acc_int);
      #ifdef DEBUG
      Serial.print("got acc: ");
      Serial.println(acc);
      #endif
    } 
    //* command 0x01 = SET POSITION
    else if (cmd==1) {
      char buf[4];
      Serial.readBytes(buf, 4);
      #ifdef DEBUG
      Serial.print("VSTART: ");
      Serial.println(motor.VSTART());
      Serial.print("VSTOP: ");
      Serial.println(motor.VSTOP());
      #endif
      motor.AMAX((uint16_t)(HOMING_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.DMAX((uint16_t)(HOMING_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.d1((uint16_t)(HOMING_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.a1((uint16_t)(HOMING_ACC*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      motor.v1(0);
      motor.VSTART(10);
      motor.VSTOP(20);
      motor.RAMPMODE(0);
      uint32_t data_int = ((buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3]);
      float data_f = clamp_pos(*(float*)&data_int);
      motor.XTARGET((int32_t)(data_f*STEPS_PER_MM*USTEPS + X_OFFSET));
      
      do {
        delay(5);
      } while (!motor.position_reached());
      
      motor.VSTART(0);
      motor.VSTOP(1);
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
      motor.XACTUAL(X_OFFSET);
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
      motor.XACTUAL(X_OFFSET);
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
      mode = Mode::SOFT_STOP;
    }
    //* command 0x06 = NOOP (get state)
    else if (cmd==6) {}
    //* command 0x07 = SET FEEDBACK GAINS
    else if (cmd==7) {
      char buf[24];
      if (Serial.readBytes(buf, 24)<24) {
        #ifdef DEBUG
        Serial.println("ERROR!");
        #endif
      }
      uint32_t data_int;
      for (int i=0; i<6; i++) {
        data_int = ((buf[4*i] << 24) + (buf[4*i+1] << 16) + (buf[4*i+2] << 8) + buf[4*i+3]);
        gains.values[i] = *(float*)&data_int;
      }
      #ifdef DEBUG
        Serial.printf("gains: %f, %f, %f, %f, %f, %f\n", 
        gains.values[0], gains.values[1], gains.values[2], 
        gains.values[3], gains.values[4], gains.values[5]);
      #endif
    }
    //* command 0x08 = SET SETPOINT
    else if (cmd==8) {
      char buf[24];
      if (Serial.readBytes(buf, 24)<24) {
        #ifdef DEBUG
        Serial.println("ERROR!");
        #endif
      }
      uint32_t data_int;
      for (int i=0; i<6; i++) {
        data_int = ((buf[4*i] << 24) + (buf[4*i+1] << 16) + (buf[4*i+2] << 8) + buf[4*i+3]);
        setpoint.values[i] = *(float*)&data_int;
      }
      #ifdef DEBUG
        Serial.println(setpoint.values[0], HEX);
        Serial.printf("setpoint: %f, %f, %f, %f, %f, %f\n", 
        setpoint.values[0], setpoint.values[1], setpoint.values[2], 
        setpoint.values[3], setpoint.values[4], setpoint.values[5]);
      #endif
    }
    //* command 0x09 = RUN CLOSED LOOP
    else if (cmd==9) {
      mode = Mode::CLOSED_LOOP;
    }
    //* command 0x10 = SET ENCODER ZEROS
    else if (cmd==10) {
      top_home += state.top;
      end_home += state.end;
      uint32_t top_home_int = *(uint32_t*)&top_home;
      uint32_t end_home_int = *(uint32_t*)&end_home;
      EEPROM.write(0, (top_home_int >> 24) % 0xFF);
      EEPROM.write(1, (top_home_int >> 16) % 0xFF);
      EEPROM.write(2, (top_home_int >>  8) % 0xFF);
      EEPROM.write(3, (top_home_int >>  0) % 0xFF);
      EEPROM.write(4, (end_home_int >> 24) % 0xFF);
      EEPROM.write(5, (end_home_int >> 16) % 0xFF);
      EEPROM.write(6, (end_home_int >>  8) % 0xFF);
      EEPROM.write(7, (end_home_int >>  0) % 0xFF);
    }
  }
}

void update_motor_data() {
  float dt_f = (float)dt/(float)F_CPU;
  // delayMicroseconds(5);
  int32_t xpos = motor.XACTUAL();
  #ifdef DEBUG
  // Serial.print("XPOS: ");
  // Serial.println(xpos);
  #endif
  if (xpos!=0) {
    state.pos = (float)(xpos-X_OFFSET)/(STEPS_PER_MM*USTEPS);
  } else {
    state.pos += dt_f * state.vel + 0.5*sq(dt_f)*acc;
  }
  // delayMicroseconds(5);
  int32_t xvel = motor.VACTUAL();
  #ifdef DEBUG
  // Serial.print("XVEL: ");
  // Serial.println(xvel);
  #endif
  if (xvel==0) {
    if (abs(state.vel + acc*dt_f) > 0.1) {
      state.vel += acc*dt_f;
    } else {
      state.vel = 0.0;
    }
  } else {
    state.vel = (float)(xvel)/(STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION);
  }
  if (abs(state.pos)>MAX_POS) {
    limits_hit = true;
    enter_reset();
  }
}

void loop() {
  deal_with_serial();
  update_encoder_data();
  update_motor_data();
  set_motor_acc();
}