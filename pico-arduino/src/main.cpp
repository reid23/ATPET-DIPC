#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/stdlib.h"
#include <SPI.h>

// pins for encoders
#define CS_PIN 5
#define MOSI_PIN 3
#define MISO_PIN 4
#define SCK_PIN 2


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
uint16_t toperr = 0;
uint16_t enderr = 0;


SPISettings settings = SPISettings(100000, MSBFIRST, SPI_MODE1);
// SPISettings settings(5000000, MSBFIRST, SPI_MODE1);

void clear_error_flag() {
  // Serial.println("clearing error flag");
  SPI.beginTransaction(settings);
  //tell it to clear errors
  digitalWrite(CS_PIN, LOW);
  // SPI.transfer16(clear_errors);
  SPI.transfer16(clear_errors);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  // then request angle again so other code still works
  digitalWrite(CS_PIN, LOW);
  // toperr = SPI.transfer16(read_diag);
  enderr = SPI.transfer16(read_diag);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS_PIN, LOW);
  // uint16_t topdiag = SPI.transfer16(read_angle);
  uint16_t enddiag = SPI.transfer16(read_angle);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  Serial.println("diag:");
  // Serial.println(topdiag, BIN);
  Serial.println(enddiag, BIN);
  Serial.println(enderr, BIN);
  Serial.println();
  // Serial.println(toperr);
  // Serial.println(enderr);
}
//1111111101110111

//1110111011101110


void setup() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  // SPI.notUsingInterrupt();
  // SPI.setCS(CS_PIN);
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);
  // SPI.setDataMode(SPI_MODE1);
  // SPI.setBitOrder(MSBFIRST);
  // SPI.setClockDivider(32); //you can chose faster SPI frequency 
  SPI.begin();
  // SPI.setDataMode(SPI_MODE1);
  // SPI.setBitOrder(MSBFIRST);
  
  Serial.begin(115200);
  clear_error_flag();
  delayMicroseconds(1);
  SPI.beginTransaction(settings);
  digitalWrite(CS_PIN, LOW);
  // (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS_PIN, LOW);
  // top = (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  end = (int16_t)(SPI.transfer16(read_angle) & angle_bitmask);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}
uint16_t topread;
uint16_t endread;

void update_encoder_data() {

  SPI.beginTransaction(settings);
  delayMicroseconds(1);
  // spi_write16_read16_blocking(SPI.)
  digitalWrite(CS_PIN, LOW);
  // topread = SPI.transfer16(read_angle);
  endread = SPI.transfer16(read_angle);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);
  // if ((topread & (uint16_t)0b0100000000000000) > 0) { 
    // clear_error_flag(); 
    // topread = top; 
  // }
  if ((endread & (uint16_t)0b0100000000000000) > 0) { 
    clear_error_flag();
    // endread = end; 
  }
  // oldtop = top;
  oldend = end;
  // top = (int16_t)(topread & angle_bitmask);
  end = (int16_t)(endread & angle_bitmask);
}

void loop() {
  clear_error_flag();
  update_encoder_data();
  // Serial.print("toperr:");
  // Serial.println(toperr);
  // Serial.println(toperr, BIN);
  Serial.print("top:");
  Serial.print(top);
  Serial.print(",end:");
  Serial.println(end);
}