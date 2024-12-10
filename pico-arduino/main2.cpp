#include <Arduino.h>
// #include <SerialUART.h>
// #include <SerialUSB.h>
// #include <TMCStepper.h>
#include <SPI.h>
#include "pico/stdlib.h"
#include "pico/stdlib.h"
#include "RunningMedian.h"
#include "TMC5160.h"

#define STEPS_PER_MM 150.0
#define USTEPS 128.0

#define ACC_UNIT_CONVERSION 0.015270994830222222
#define VEL_UNIT_CONVERSION 1.3981013333333334


#define SPIMODE SPI_MODE1

// pins & settings for TMC5160 stepper driver
#define STEPPER_CS            17
#define STEPPER_MOSI          19
#define STEPPER_MISO          16
#define STEPPER_SCK           18

#define DIAG_PIN 9
#define EN_PIN 8
#define LIMIT_SWITCH_PIN 7

// pins for encoders
#define CS_PIN 13
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 10
#define RSENSE 0.022F


// buffers for encoder filtering
#define BUF_LEN 50 // second buffer: where calculation actually happens. accounts for lag more
#define MED_BUF_LEN 10 // first buffer: get median and feed into second buffer. just gets rid of ouliers.

#define ENCODER_TICKS 16384
#define RADS_PER_TICK PI/(ENCODER_TICKS/2)

#define MAX_VEL 5000.0
#define MAX_ACC 20000000.0 // all accelerations are 1000x bigger
#define HOMING_POS 800.0 // position after hitting limit switch
#define HOMING_SPEED 100.0
#define HOMING_ACC 500000.0 // all accelerations are 1000x bigger
#define ACC_SCALE 0.001

#define LOOP_PERIOD 1 // microseconds, minimum

RunningMedian topmed = RunningMedian(MED_BUF_LEN);
RunningMedian endmed = RunningMedian(MED_BUF_LEN);

// TMC5160Stepper driver((uint16_t)STEPPER_CS, RSENSE, (uint16_t)STEPPER_MOSI, (uint16_t)STEPPER_MISO, (uint16_t)STEPPER_SCK);
TMC5160_SPI stepper(STEPPER_CS, 10000000, SPISettings(10000000, MSBFIRST, SPI_MODE3), SPI);
// TMC5160Stepper driver((uint16_t)STEPPER_CS, (float)RSENSE);
bool stalled = false;
void stallInterrupt(){
  stalled = true;
}

bool limits_hit = false;
void limitsInterrupt(){
  limits_hit = true;
}


const uint16_t angle_bitmask = 0b0011111111111111;
const uint16_t comp_hi_bitmask = 0b0010000000000;
const uint16_t comp_lo_bitmask = 0b0001000000000;
const uint16_t COF_bitmask = 0b0000100000000;
const uint16_t OCF_bitmask = 0b0000010000000;
const uint16_t AGC_bitmask = 0b0000001111111;

const uint16_t clear_errors = 0b0100000000000001;
const uint16_t read_angle = 0xFFFF;
const uint16_t read_mag = 0b1011111111111110;
const uint16_t read_diag = 0b1011111111111101;
int16_t top = 0;
int16_t end = 0;
int16_t oldtop = 0;
int16_t oldend = 0;
int16_t toprots = 0;
int16_t endrots = 0;
int16_t bufptr = 0;
uint16_t toperr = 0;
uint16_t enderr = 0;
float toppos = 0.0;
float endpos = 0.0;
float topvel = 0.0;
float endvel = 0.0;

float vel = 0;
float pos = 0;

unsigned long looptime = 0;
unsigned long t = 0;
unsigned long dt = 0;
unsigned long led_turned_on_time = 0;

int32_t acc = 0;
// getweights = lambda BUF_LEN: (np.linalg.inv((A:=np.array([[1, i-BUF_LEN] for i in range(BUF_LEN)])).T@A).T@(A.T)).tolist()
static float pos_weights[BUF_LEN] = {-0.04000000000000001, -0.03755102040816327, -0.03510204081632655, -0.03265306122448981, -0.030204081632653076, -0.02775510204081634, -0.025306122448979604, -0.022857142857142868, -0.02040816326530613, -0.017959183673469395, -0.015510204081632659, -0.013061224489795922, -0.010612244897959186, -0.008163265306122464, -0.005714285714285727, -0.003265306122448991, -0.0008163265306122547, 0.0016326530612244816, 0.004081632653061218, 0.006530612244897954, 0.00897959183673469, 0.011428571428571427, 0.013877551020408163, 0.0163265306122449, 0.018775510204081622, 0.021224489795918365, 0.023673469387755094, 0.02612244897959183, 0.028571428571428567, 0.031020408163265303, 0.03346938775510204, 0.035918367346938776, 0.038367346938775505, 0.04081632653061224, 0.04326530612244898, 0.045714285714285714, 0.04816326530612245, 0.05061224489795918, 0.053061224489795916, 0.05551020408163265, 0.05795918367346939, 0.060408163265306125, 0.06285714285714286, 0.0653061224489796, 0.06775510204081633, 0.07020408163265307, 0.07265306122448979, 0.07510204081632653, 0.07755102040816327, 0.08};
static float vel_weights[BUF_LEN] = {-0.0023529411764705885, -0.002256902761104442, -0.0021608643457382954, -0.002064825930372149, -0.0019687875150060023, -0.0018727490996398558, -0.00177671068427371, -0.0016806722689075636, -0.001584633853541417, -0.0014885954381752705, -0.001392557022809124, -0.0012965186074429774, -0.0012004801920768309, -0.0011044417767106843, -0.0010084033613445378, -0.0009123649459783917, -0.0008163265306122451, -0.0007202881152460986, -0.0006242496998799521, -0.0005282112845138055, -0.000432172869147659, -0.00033613445378151245, -0.00024009603841536635, -0.0001440576230492198, -4.801920768307327e-05, 4.801920768307327e-05, 0.0001440576230492198, 0.00024009603841536635, 0.00033613445378151245, 0.000432172869147659, 0.0005282112845138055, 0.0006242496998799521, 0.0007202881152460986, 0.0008163265306122449, 0.0009123649459783915, 0.001008403361344538, 0.0011044417767106843, 0.0012004801920768309, 0.0012965186074429774, 0.0013925570228091237, 0.0014885954381752703, 0.0015846338535414168, 0.0016806722689075631, 0.0017767106842737097, 0.0018727490996398562, 0.0019687875150060027, 0.0020648259303721493, 0.002160864345738296, 0.002256902761104442, 0.0023529411764705885};
static float topbuf[BUF_LEN] = {0.0};
static float endbuf[BUF_LEN] = {0.0};

SPISettings settings(10000000, MSBFIRST, SPI_MODE1);

void clear_error_flag() {
  // Serial.println("clearing error flag");
  SPI1.beginTransaction(settings);
  //tell it to clear errors
  digitalWrite(CS_PIN, LOW);
  SPI1.transfer16(clear_errors);
  SPI1.transfer16(clear_errors);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  // then request angle again so other code still works
  digitalWrite(CS_PIN, LOW);
  toperr = SPI1.transfer16(read_angle);
  enderr = SPI1.transfer16(read_angle);
  digitalWrite(CS_PIN, HIGH);
  SPI1.endTransaction();
  // Serial.println(toperr);
  // Serial.println(enderr);
}

void setup() {
  pinMode(DIAG_PIN, INPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLDOWN);
  // attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallInterrupt, RISING);
  // attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitsInterrupt, RISING);

  pinMode(EN_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  // pinMode(MISO_PIN, INPUT);
  // pinMode(MOSI_PIN, OUTPUT);
  // pinMode(SCK_PIN, OUTPUT);
  // pinMode(STEPPER_CS, OUTPUT);

  pinMode(25, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(CS_PIN, HIGH);
  // digitalWrite(STEPPER_CS, HIGH);
  // SPI.setCS(STEPPER_CS);
  // SPI.setRX(STEPPER_MISO);
  // SPI.setTX(STEPPER_MOSI);
  // SPI.setSCK(STEPPER_SCK);
  // SPI.begin();

  // SPI1.setCS(CS_PIN);
  SPI1.setRX(MISO_PIN);
  SPI1.setTX(MOSI_PIN);
  SPI1.setSCK(SCK_PIN);
  SPI1.begin();
  // SPI1.setDataMode(SPI_MODE1);
  // SPI1.setBitOrder(MSBFIRST);

  // Serial.setTX(12);
  // Serial.setRX(13);
  
  Serial.begin(115200);
  // while (!Serial.available());
  // Serial.begin();
  TMC5160::PowerStageParameters powerStageParams;
  // powerStageParams.drvStrength
  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 30;
  motorParams.ihold = 16;
  motorParams.irun = 31;
  stepper.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);
  stepper.writeRegister(TMC5160_Reg::RAMPMODE, TMC5160_Reg::VELOCITY_MODE_POS);
  stepper.writeRegister(TMC5160_Reg::VMAX, 0);
  // stepper.writeRegister(TMC5160_Reg::CHOPCONF, )
  // set toff to 5 and mres to 1 (128 usteps)
  // Serial.println(stepper.readRegister(TMC5160_Reg::CHOPCONF));
  stepper.writeRegister(TMC5160_Reg::CHOPCONF, stepper.readRegister(TMC5160_Reg::CHOPCONF) | 0b00000001000000000000000000000101);
  // Serial.println(stepper.readRegister(TMC5160_Reg::CHOPCONF));
  stepper.writeRegister(TMC5160_Reg::VSTART, 10);
  stepper.writeRegister(TMC5160_Reg::VSTOP, 10);
  stepper.writeRegister(TMC5160_Reg::XACTUAL, 5);
  // driver.begin();
  // driver.toff(5);
  // driver.rms_current(1200);
  // driver.microsteps(USTEPS);
  // driver.pwm_autoscale(true);
  // driver.VMAX(0);
  // driver.RAMPMODE(1);
  // driver.diag0_stall(true);
  // driver.en_softstop(false);
  // driver.VSTOP(10);
  // driver.VSTART(10);
  // driver.XACTUAL(0);

  // SPI1.begin();
  SPI1.beginTransaction(settings);
  clear_error_flag();
  delayMicroseconds(1);
  digitalWrite(CS_PIN, LOW);
  (int16_t)(SPI1.transfer16(read_angle) & angle_bitmask);
  (int16_t)(SPI1.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS_PIN, LOW);
  top = (int16_t)(SPI1.transfer16(read_angle) & angle_bitmask);
  end = (int16_t)(SPI1.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  SPI1.endTransaction();
  looptime = micros();
  t = micros();
}
uint16_t topread;
uint16_t endread;

void update_encoder_data() {

  SPI1.beginTransaction(settings);
  // spi_write16_read16_blocking(SPI1.)
  digitalWrite(CS_PIN, LOW);
  topread = SPI1.transfer16(read_angle);
  endread = SPI1.transfer16(read_angle);
  digitalWrite(CS_PIN, HIGH);
  SPI1.endTransaction();
  delayMicroseconds(1);
  if ((topread & (uint16_t)0b0100000000000000) > 0) { 
    clear_error_flag(); 
    // topread = top; 
  }
  if ((endread & (uint16_t)0b0100000000000000) > 0) { 
    clear_error_flag();
    // endread = end; 
  }
  oldtop = top;
  oldend = end;
  top = (int16_t)(topread & angle_bitmask);
  end = (int16_t)(endread & angle_bitmask);
  if      (top-oldtop > ENCODER_TICKS/2) { toprots--; }
  else if (oldtop-top > ENCODER_TICKS/2) { toprots++; }
  if      (end-oldend > ENCODER_TICKS/2) { endrots--; } 
  else if (oldend-end > ENCODER_TICKS/2) { endrots++; }

  topmed.add((top+ENCODER_TICKS*toprots) * RADS_PER_TICK);
  endmed.add((end+ENCODER_TICKS*endrots) * RADS_PER_TICK);
  topbuf[bufptr] = topmed.getMedian();
  endbuf[bufptr] = endmed.getMedian();
  bufptr++;
  if (bufptr == BUF_LEN) { bufptr = 0; }
  
  toppos = 0.0;
  endpos = 0.0;
  topvel = 0.0;
  endvel = 0.0;
  for (int i = 0; i<BUF_LEN; i++) {
    toppos += topbuf[(bufptr+i)%BUF_LEN]*pos_weights[i];
    endpos += endbuf[(bufptr+i)%BUF_LEN]*pos_weights[i];
    topvel += topbuf[(bufptr+i)%BUF_LEN]*vel_weights[i];
    endvel += endbuf[(bufptr+i)%BUF_LEN]*vel_weights[i];
  }
  // Serial.println(endread, BIN);
}

void loop() {
  // Serial.println(driver.VMAX());
  // Serial.println(driver.AMAX());
  // Serial.println(driver.RAMPMODE());
  // Serial.println(driver.test_connection());
  // Serial.println();
  // Serial.println((uint32_t)(MAX_VEL*STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION));
  if (dt>LOOP_PERIOD){
    digitalWrite(LED_BUILTIN, HIGH);
    led_turned_on_time = millis();
  } else if (millis() - led_turned_on_time < 200) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  update_encoder_data();
  // Serial.println(top);

  if (Serial.available()) {
    uint8_t buf[5];
    Serial.readBytes(buf, 5);
    // Serial.printf("read command %d", buf[0]);
    if (stalled || limits_hit) {
      digitalWrite(EN_PIN, HIGH);
      stepper.writeRegister(TMC5160_Reg::VMAX, 0);
      stepper.writeRegister(TMC5160_Reg::AMAX, (uint32_t)(MAX_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      stepper.writeRegister(TMC5160_Reg::RAMPMODE, 1);
      // driver.RAMPMODE(1); // in case it wasn't already like this
      if (buf[0]!=3) {
        if (stalled) { Serial.println("RESET: MOTOR STALLED! use CLEAR RESET (0x03) to clear."); }
        if (limits_hit) { Serial.println("RESET: LIMITS HIT! use CLEAR RESET (0x03) to clear."); }
        return;
      }
    }
    //* command 0x00 = SET ACCELERATION
    if (buf[0]==0) {
      acc = ((buf[1] << 24) + (buf[2] << 16) + (buf[3] << 8) + buf[4]);
      acc = (*(int32_t*)&acc);

      stepper.writeRegister(TMC5160_Reg::AMAX, (uint32_t)(acc*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      if (acc>0) {
        stepper.writeRegister(TMC5160_Reg::RAMPMODE, 1);
      } else if (acc<0) {
        stepper.writeRegister(TMC5160_Reg::RAMPMODE, 2);
      }

      // now gooo!!
      stepper.writeRegister(TMC5160_Reg::VMAX, (uint32_t)(MAX_VEL*STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION));

      // driver.VMAX((uint32_t)(MAX_VEL*STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION));
    } 
    //* command 0x01 = SET POSITION
    else if (buf[0]==1) {
      stepper.writeRegister(TMC5160_Reg::RAMPMODE, 0);
      stepper.writeRegister(TMC5160_Reg::AMAX, (uint32_t)(HOMING_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      stepper.writeRegister(TMC5160_Reg::DMAX, (uint32_t)(HOMING_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      stepper.writeRegister(TMC5160_Reg::D_1, 0);
      // driver.AMAX((uint16_t)(HOMING_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      // driver.DMAX((uint16_t)(HOMING_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      // driver.d1(0);
      uint32_t data = ((buf[1] << 24) + (buf[2] << 16) + (buf[3] << 8) + buf[4]);
      // driver.XTARGET((*(int32_t*)&data)*STEPS_PER_MM*USTEPS);
      stepper.writeRegister(TMC5160_Reg::XTARGET, (*(int32_t*)&data)*STEPS_PER_MM*USTEPS);
      while (stepper.readRegister(TMC5160_Reg::VACTUAL)>0);
      stepper.writeRegister(TMC5160_Reg::VMAX, 0);
      stepper.writeRegister(TMC5160_Reg::RAMPMODE, 1);

      // driver.VMAX(0);
      // driver.RAMPMODE(1);
    }
    //* command 0x02 = STAHP (RESET)
    else if (buf[0]==2) {
      digitalWrite(EN_PIN, HIGH);
      stepper.writeRegister(TMC5160_Reg::VMAX, 0);
      stepper.writeRegister(TMC5160_Reg::AMAX, (uint32_t)(MAX_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      stepper.writeRegister(TMC5160_Reg::RAMPMODE, 1);
      
      // driver.VMAX(0);
      // driver.AMAX((uint16_t)(MAX_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      // driver.RAMPMODE(1); // in case it wasn't already like this
    }
    //* command 0x03 = CLEAR RESET
    else if (buf[0]==3) {
      digitalWrite(EN_PIN, LOW);
      stepper.writeRegister(TMC5160_Reg::VMAX, 0);
      stepper.writeRegister(TMC5160_Reg::RAMPMODE, 1);

      // driver.VMAX(0);
      // driver.RAMPMODE(1);
      stalled = false;
      limits_hit = false;
    }
    //* command >= 0x04 = HOME
    else if (buf[0]==4) {
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
    //* commad 0x05 = SOFT STOP
    else if (buf[0]==5) {
      stepper.writeRegister(TMC5160_Reg::AMAX, (uint32_t)(MAX_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      stepper.writeRegister(TMC5160_Reg::VMAX, 0);

      // driver.AMAX((uint16_t)(MAX_ACC*ACC_SCALE*STEPS_PER_MM*USTEPS*ACC_UNIT_CONVERSION));
      // driver.VMAX(0);
    }
    
    vel = (float)(stepper.readRegister(TMC5160_Reg::VACTUAL))/(STEPS_PER_MM*USTEPS*VEL_UNIT_CONVERSION);
    pos = (float)(stepper.readRegister(TMC5160_Reg::XACTUAL))/(STEPS_PER_MM*USTEPS);

    Serial.printf("{'t': %u, 'u': %f, 'cp': %f, 'cv': %f, 'tp': %f, 'tv': %f, 'ep': %f, 'ev': %f}\n", 
      micros(), 
      acc*ACC_SCALE, 
      pos,
      vel,
      toppos, topvel,
      endpos, endvel);
  }
  Serial.print("top: ");
  Serial.print(topread);
  Serial.print(", end: ");
  Serial.println(endread);
  dt = micros()-looptime;
  do {
    t = micros();
  } while (t-looptime < LOOP_PERIOD);
  looptime = t;
}