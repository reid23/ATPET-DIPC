#include <Arduino.h>
#include <SPI.h>
#include "pico/stdlib.h"
#include "pico/stdlib.h"


#define SPIMODE SPI_MODE1

// pins for encoders
#define CS_PIN 13
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 10

SPISettings settings(10000000, MSBFIRST, SPIMODE);
const uint16_t read_angle = 0xFFFF;
const uint16_t angle_bitmask = 0b0011111111111111;
uint16_t topread = 0;
uint16_t endread = 0;
void setup() {
  Serial.begin(250000);
  pinMode(CS_PIN, OUTPUT);
//   pinMode(MISO_PIN, INPUT);
//   pinMode(MOSI_PIN, OUTPUT);
//   pinMode(SCK_PIN, OUTPUT);

  digitalWrite(CS_PIN, HIGH);

  SPI1.setCS(CS_PIN);
  SPI1.setRX(MISO_PIN);
  SPI1.setTX(MOSI_PIN);
  SPI1.setSCK(SCK_PIN);
  SPI1.begin(false);
}

void loop() {
    SPI1.beginTransaction(settings);
    digitalWrite(CS_PIN, LOW);
    topread = SPI1.transfer16(read_angle);
    // digitalWrite(CS_PIN, HIGH);
    // delayMicroseconds(10);
    // digitalWrite(CS_PIN, LOW);
    endread = SPI1.transfer16(read_angle);
    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(10);
    SPI1.endTransaction();
    Serial.print("TOP: ");
    Serial.print(topread, BIN);
    Serial.print(" END: ");
    Serial.print(endread, BIN);
    Serial.print(" TRUE: ");
    Serial.println(topread&angle_bitmask);
}