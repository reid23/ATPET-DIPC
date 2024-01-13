#include <Arduino.h>
#include <SerialUART.h>
#include <SerialUSB.h>
#include <TMCStepper.h>
#include <SPI.h>
#include "pico/stdlib.h"
#include "pico/stdlib.h"
// #include "hardware/pio.h"
// // Our assembled program:
// #include "counter.pio.h"
// #include "hardware/dma.h"

#define VEL_BUF_LEN 40
#define DRIVER_UPDATE_INTERVAL 500UL // in microseconds

#define SPIMODE SPI_MODE1

#define INDEX_PIN 21
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
const uint16_t angle_bitmask = 0b0011111111111111;
const uint16_t clear_errors = 0b0100000000000001;
const uint16_t read_angle = 0xFFFF;
bool done = true;
int16_t top = 0;
int16_t end = 0;
int16_t oldtop = 0;
int16_t oldend = 0;
int16_t toprots = 0; // if it's done more than 32767 revolutions we've got bigger problems
int16_t endrots = 0;
int16_t dtop = 0;
int16_t dend = 0;
int32_t topvel = 0;
int32_t endvel = 0;
int32_t pos = 0;
int64_t finepos = 0;
int32_t vel = 0;
int32_t acc = 0;
uint32_t accsettime = 0;
int32_t accsetvel = 0;
int32_t accsetpos = 0;
uint32_t curtime = 0;
uint32_t oldtime = 0;
uint32_t last_driver_update_time = 0;
int64_t last_driver_update_pos = 0;
int32_t old_top_full_for_vel = 0;
int32_t old_end_full_for_vel = 0;
// SPISettings settings = SPISettings(14000000, MSBFIRST, SPIMODE);

static int32_t velbuftop[VEL_BUF_LEN] = {0};
static int32_t velbufend[VEL_BUF_LEN] = {0};

int32_t curveltop = 0;
int32_t curvelend = 0;
int32_t velidx = 0;

// PIO pio = pio0;
// uint offset = pio_add_program(pio, &counter_program);
// uint sm = pio_claim_unused_sm(pio, true);
// counter_program_init(pio, sm, offset, INDEX_PIN);
void add_vel(int32_t dtop, int32_t dend, uint32_t dt) {
  int32_t topnewvel = dtop*(int32_t)(1000000/dt);
  int32_t endnewvel = dend*(int32_t)(1000000/dt);
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
  //testing silly
  // pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_x));
  // pio_sm_exec(pio, sm, pio_encode_push(false, true));
  // pos = pio_sm_get(pio, sm);

  // dma_channel_config c = dma_channel_get_default_config(0);
  // channel_config_set_read_increment(&c, false);
  // channel_config_set_write_increment(&c, false);
  // channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
  // dma_channel_configure(0, &c,
  //   &pos,
  //   &pio->rxf[sm],
  //   -1,
  //   true
  // );


  //end testing silly
  pinMode(EN_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  pinMode(25, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(CS_PIN, HIGH);
  SPISettings settings(1000000, MSBFIRST, SPIMODE);
  SPI.setCS(CS_PIN);
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);

  Serial1.setTX(12);
  Serial1.setRX(13);
  
  Serial1.begin(250000);
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
  (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(CS_PIN, LOW);
  top = (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  end = (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void clear_error_flag() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer16(clear_errors);
  SPI.transfer16(clear_errors);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS_PIN, LOW);
  (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
}

union message {
  uint8_t bytes[32];
  int32_t nums[8];
  uint32_t unums[8];
};

void setacc(int32_t newacc){
  if (acc != newacc) {
    acc = newacc;
    accsettime = micros();
    // accsetpos = pos;
    accsetvel = vel;
  }
}
void loop() {
  if (Serial.available()) {
    uint8_t buf[5];
    Serial.readBytes(buf, 5);
    //* command 0x00 = SET ACCELERATION
    if (buf[0]==0) {
      setacc((int32_t)((buf[1] << 24) + (buf[2] << 16) + (buf[3] << 8) + buf[4]));
    } 
    //* command 0x01 = SET VELOCITY
    else if (buf[0]==1) {
      acc = 0;
      vel = (int32_t)((buf[1] << 24) + (buf[2] << 16) + (buf[3] << 8) + buf[4]);
    }
    //* command 0x02 = STAHP (RESET)
    else if (buf[0]==2) {
      acc = 0;
      vel = 0;
      driver.VACTUAL(0);
      digitalWrite(EN_PIN, HIGH);
    }
    //* command 0x03 = CLEAR RESET
    else if (buf[0]==3) {
      acc = 0;
      vel = 0;
      driver.VACTUAL(0);
      digitalWrite(EN_PIN, HIGH);
    }
    //* command >= 0x04 = NOP

    // respond with state
    // message msg;
    // msg.unums[0] = micros();
    // msg.nums[1] = acc;
    // msg.nums[2] = pos;
    // msg.nums[3] = vel;
    // msg.nums[4] = (int32_t)top + (int32_t)toprots*16384;
    // msg.nums[5] = topvel/VEL_BUF_LEN;
    // msg.nums[6] = (int32_t)end + (int32_t)endrots*16384;
    // msg.nums[7] = endvel/VEL_BUF_LEN;
    // uint8_t msg[32];
    // msg[0] = (n >> 24) & 0xFF;
    // msg[1] = (n >> 16) & 0xFF;
    // msg[2] = (n >> 8) & 0xFF;
    // msg[3] = n & 0xFF;

    // Serial.write()


    // Serial.write(msg.bytes, 32);
    Serial.printf("[%u, %d, %d, %d, %d, %d, %d, %d]\n", micros(), acc, pos, vel, (int32_t)top + (int32_t)toprots*16384, topvel/VEL_BUF_LEN, (int32_t)end + (int32_t)endrots*16384, endvel/VEL_BUF_LEN);
  }
  
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPIMODE));
  digitalWrite(CS_PIN, LOW);
  uint16_t topread = SPI.transfer16(read_angle);
  uint16_t endread = SPI.transfer16(read_angle);
  digitalWrite(CS_PIN, HIGH);
  if ((topread & 0b0100000000000000) > 0) { clear_error_flag(); return; }
  if ((endread & 0b0100000000000000) > 0) { clear_error_flag(); return; }
  oldtop = top;
  oldend = end;
  top = (int16_t)(topread & angle_bitmask);
  end = (int16_t)(endread & angle_bitmask);
  // SPI.endTransaction();

  // use previous loop's dt, since SPI encoder data is behind by one loop
  uint32_t dt = curtime-oldtime;
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


  uint32_t notimeissues = micros();
  finepos = last_driver_update_pos + ((int64_t)(notimeissues-last_driver_update_time)*(int64_t)vel);
  pos = (int32_t)(finepos/1000000);
  if (notimeissues-last_driver_update_time < DRIVER_UPDATE_INTERVAL) { return; }
  
  add_vel(
    (int32_t)top + (int32_t)toprots*16384 - old_top_full_for_vel, 
    (int32_t)end + (int32_t)endrots*16384 - old_end_full_for_vel,
    notimeissues-last_driver_update_time
  );
  old_top_full_for_vel = (int32_t)top + (int32_t)toprots*16384;
  old_end_full_for_vel = (int32_t)end + (int32_t)endrots*16384;

  last_driver_update_time = notimeissues;
  last_driver_update_pos = finepos;
  bool dir = vel>0;
  if (acc != 0) {
    vel = accsetvel + ((int32_t)(notimeissues-accsettime)*acc)/1000000;
  }
  if (dir != (vel>0)) {
    driver.shaft(vel>0);
    delayMicroseconds(20);
  }
  driver.VACTUAL(vel);
}