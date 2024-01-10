#include <Arduino.h>
#include <SerialUART.h>
#include <SerialUSB.h>
#include <TMCStepper.h>
#include <SPI.h>

#define VEL_BUF_LEN 20
#define DRIVER_UPDATE_INTERVAL 500UL // in microseconds

#define SPIMODE SPI_MODE1

#define EN_PIN           20 // Enable
// #define DIR_PIN          6  // Direction
// #define STEP_PIN         5  // Step
#define CS_PIN            17 // Chip select
#define MOSI_PIN          19 // Software Master Out Slave In (MOSI)
#define MISO_PIN          16 // Software Master In Slave Out (MISO)
#define SCK_PIN           18 // Software Slave Clock (SCK)
// #define SW_RX            63 // TMC2208/TMC2224 SoftwareSerial receive pin
// #define SW_TX            40 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2208 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);
const unsigned short angle_bitmask = 0b0011111111111111;
const unsigned short clear_errors = 0b0100000000000001;
const unsigned short read_angle = 0xFFFF;
bool done = true;
unsigned short top = 0;
unsigned short end = 0;
short oldtop = 0;
short oldend = 0;
short toprots = 0; // if it's done more than 32767 revolutions we've got bigger problems
short endrots = 0;
short dtop = 0;
short dend = 0;
int topvel = 0;
int endvel = 0;
int pos = 0;
int vel = 0;
int acc = 0;
unsigned long accsettime = 0;
int accsetvel = 0;

unsigned long curtime = 0;
unsigned long oldtime = 0;
unsigned long last_driver_update_time = 0;
int last_driver_update_pos = 0;
// SPISettings settings = SPISettings(14000000, MSBFIRST, SPIMODE);

static int velbuftop[VEL_BUF_LEN] = {0};
static int velbufend[VEL_BUF_LEN] = {0};
int curveltop = 0;
int curvelend = 0;
int velidx = 0;
void add_vel(int dtop, int dend, unsigned long dt) {
  int topnewvel = dtop*(1000000/dt);
  int endnewvel = dend*(1000000/dt);
  topvel += topnewvel-velbuftop[velidx];
  endvel += endnewvel-velbufend[velidx];
  if (velidx >= VEL_BUF_LEN-1) {
    velidx = 0;
  } else {
    velidx++;
  }
  velbuftop[velidx] = topnewvel;
  velbufend[velidx] = endnewvel;
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  pinMode(25, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(CS_PIN, HIGH);
  SPISettings settings(5000000, MSBFIRST, SPIMODE);
  SPI.setCS(CS_PIN);
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);

  Serial1.setTX(12);
  Serial1.setRX(13);
  
  Serial1.begin(500000);
  Serial.begin();
  driver.begin();

  driver.en_spreadCycle(true);
  driver.pdn_disable(true);
  // driver.toff(5);                 // Enables driver in software
  driver.rms_current(1200);
  driver.microsteps(128);
  driver.VACTUAL(vel);

  SPI.begin();
  SPI.beginTransaction(settings);
  //clear error flag
  digitalWrite(CS_PIN, LOW);
  SPI.transfer16(clear_errors);
  SPI.transfer16(clear_errors);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS_PIN, LOW);
  (short)(SPI.transfer16(read_angle) & angle_bitmask);
  (short)(SPI.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(CS_PIN, LOW);
  top = (short)(SPI.transfer16(read_angle) & angle_bitmask);
  end = (short)(SPI.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

union message {
  uint8_t bytes[32];
  int32_t nums[8];
  uint32_t unums[8];
};

void loop() {
  if (Serial.available()) {
    uint8_t buf[5];
    Serial.readBytes(buf, 5);
    uint32_t val1 = (buf[1] << 24) + (buf[2] << 16) + (buf[3] << 8) + buf[4];
    if (buf[0]==0) {
      acc = (int32_t)val1;
      message msg;
      msg.unums[0] = micros();
      msg.nums[1] = acc;
      msg.nums[2] = pos;
      msg.nums[3] = vel;
      msg.nums[4] = (int)top;// + (int)toprots*16384;
      msg.nums[5] = topvel;
      msg.nums[6] = (int)end;// + (int)endrots*16384;
      msg.nums[7] = endvel;

      // Serial.write(msg.bytes, 32);
      Serial.printf("%d, %d,  %d, %d, %d, %d, %d, %d, %d", micros(), acc, pos, vel, (int)top, topvel, (int)end, endvel, (int)(curtime-oldtime));
    } else if (buf[0]==1) {
      acc = 0;
      vel = (int32_t)val1;
    } else if (buf[0]==2) {
      acc = 0;
      vel = 0;
      driver.VACTUAL(0);
      digitalWrite(EN_PIN, HIGH);
    }
  }
  
  oldtop = top;
  oldend = end;
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPIMODE));
  digitalWrite(CS_PIN, LOW);
  top = (short)(SPI.transfer16(read_angle) & angle_bitmask);
  end = (short)(SPI.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  // SPI.endTransaction();

  // if ((top>>14)%2 > 0) {
  //   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPIMODE));
  //   digitalWrite(CS_PIN, LOW);
  //   SPI.transfer16(clear_errors);
  //   SPI.transfer16(clear_errors);
  //   digitalWrite(CS_PIN, HIGH);
  //   SPI.endTransaction();
  // }
  oldtime = curtime;
  curtime = micros();

  if (done) {
    oldtop = top;
    oldend = end;
    done = false;
  }
  dtop = top-oldtop;
  if (dtop > 1000) { toprots--; dtop = oldtop+16384-top; } 
  else if (dtop < -1000) { toprots++; dtop = top+16384-oldtop; }
  
  dend = end-oldend;
  if (dend > 1000) { endrots--; dend = oldend+16384-end; }
  else if (dend < -1000) { endrots++; dend = end+16384-oldend; }


  add_vel((int)dtop, (int)dend, curtime-oldtime);

  unsigned long notimeissues = micros();
  pos = last_driver_update_pos + ((int)(notimeissues-last_driver_update_time)*vel)/1000000; //todo: figure out why this divide is too much
  if (notimeissues-last_driver_update_time < DRIVER_UPDATE_INTERVAL) { return; }
  
  last_driver_update_time = notimeissues;
  last_driver_update_pos = pos;
  bool dir = vel>0;
  if (acc != 0) {
    vel = accsetvel + ((int)(notimeissues-accsettime)*acc)/1000000;
  }
  if (dir != (vel>0)) {
    driver.shaft(vel>0);
  }
  driver.VACTUAL(vel);
}