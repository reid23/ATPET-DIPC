#include <Arduino.h>
#include <SerialUART.h>
#include <SerialUSB.h>
#include <TMCStepper.h>
#include <SPI.h>

#define EN_PIN           18 // Enable
// #define DIR_PIN          6  // Direction
// #define STEP_PIN         5  // Step
#define CS_PIN           42 // Chip select
#define MOSI_PIN          66 // Software Master Out Slave In (MOSI)
#define MISO_PIN          44 // Software Master In Slave Out (MISO)
#define SCK_PIN           64 // Software Slave Clock (SCK)
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

bool done = true;
short top = 0;
short end = 0;
short oldtop = 0;
short oldend = 0;
short toprots = 0; // if it's done more than 32767 revolutions we've got bigger problems
short endrots = 0;
int topvel = 0;
int endvel = 0;
int pos = 0;
int vel = 0;
int acc = 0;
unsigned long accsettime = 0;
int accsetvel = 0;
// SPISettings settings = SPISettings(14000000, MSBFIRST, SPI_MODE0);
void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, OUTPUT);

  pinMode(25, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  SPISettings settings(1000000, MSBFIRST, SPI_MODE0);
  SPI.setCS(CS_PIN);
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);
  // SPI.setClockDivider(20);
  // SPI.setDataMode(SPI_MODE0);

  Serial1.setTX(12);
  Serial1.setRX(13);
  
  Serial1.begin();
  Serial.begin();
  driver.begin();

  driver.en_spreadCycle(true);
  driver.pdn_disable(true);
  // driver.toff(5);                 // Enables driver in software
  driver.rms_current(1200);
  driver.microsteps(128);
  driver.VACTUAL(vel);

  SPI.begin();
  digitalWrite(CS_PIN, LOW);
  SPI.transfer16(0xFFFF);
  SPI.transfer16(0xFFFF);
  digitalWrite(CS_PIN, HIGH);
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
      msg.nums[4] = (int)top + (int)toprots*16384;
      msg.nums[5] = topvel;
      msg.nums[6] = (int)end + (int)endrots*16384;
      msg.nums[7] = endvel;

      Serial.write(msg.bytes, 32);
    } else if (buf[0]==1) {
      acc = 0;
      vel = (int32_t)val1;
    } else if (buf[0]==2) {
      acc = 0;
      vel = 0;
      driver.VACTUAL(0);
      digitalWrite(EN_PIN, HIGH);
    }
    
    digitalWrite(25, !digitalRead(25));
    

  }
  
  oldtop = top;
  oldend = end;
  digitalWrite(CS_PIN, LOW);
  top = (int16_t)(SPI.transfer16(0xFFFF)&0b0011111111111111);
  end = (int16_t)(SPI.transfer16(0xFFFF)&0b0011111111111111);
  digitalWrite(CS_PIN, HIGH);
  if (done) {
    oldtop = top;
    oldend = end;
    done = false;
  }
  if (top-oldtop > 1000) { toprots--; } 
  else if (top-oldtop < -1000) { toprots++; }
  
  if (end-oldend > 1000) { endrots--; }
  else if (end-oldend < -1000) {endrots++; }

  if (acc != 0) {
    vel = accsetvel + ((int)(micros()-accsettime)*acc)/1000000;
  }
  driver.VACTUAL(vel);
  driver.shaft(vel>0);
}