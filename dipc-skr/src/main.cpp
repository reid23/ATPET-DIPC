#include <Arduino.h>
// #include <SPI.h>
#include <Configuration.h>
// #include <TMCStepper.h>
// #include <SoftwareSPI.h>

// TMC5160Stepper stepper = TMC5160Stepper(CS_PIN, RSENSE, TMC_SW_MISO, TMC_SW_MOSI, TMC_SW_SCK);

void setup() {
  // stepper.begin();
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); 	// Enable driver in hardware
  Serial.begin(115200);
  // stepper.rms_current(600);
  // stepper.RAMPMODE(1);
  // stepper.VMAX(0);
  // stepper.VSTART(10);
  // stepper.VSTOP(10);
  // stepper.microsteps(USTEPS);
  // stepper.GLOBAL_SCALER(30);
}
// String readstring;

void loop() {
  if (Serial.available()) {
    String readstring = Serial.readStringUntil('\n');
    if (readstring.startsWith("s")) {
      // stepper.VMAX(0);
      // stepper.AMAX((uint16_t)(MAX_ACC*STEPS_PER_MM*ACC_UNIT_CONVERSION));
      return;
    }
    long acc = readstring.toInt();
    // stepper.RAMPMODE(acc>0 ? 1 : 2);
    // stepper.AMAX((uint16_t)(acc*STEPS_PER_MM*ACC_UNIT_CONVERSION));
    // stepper.VMAX((uint32_t)(MAX_VEL*STEPS_PER_MM*VEL_UNIT_CONVERSION));
  }
  
}