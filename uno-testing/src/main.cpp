// #include <Arduino.h>

#include <TMCStepper.h>
// #include <Arduino.h>
// #include "pico/stdlib.h"
// #include "pico/stdlib.h"
// #include <SPI.h>

// pins for encoders
#define CS_PIN_MOTOR 10
#define RSENSE 0.022F
#define DIAG_PIN 4
#define EN_PIN 3
// #define MOSI_PIN 11
// #define MISO_PIN 12
// #define SCK_PIN 13

#define CS_PIN_ENCODER 0

#define STEPS_PER_MM 1.25
#define USTEPS 64.0



#define ACC_UNIT_CONVERSION 0.015270994830222222
#define VEL_UNIT_CONVERSION 1.3981013333333334

#define ENCODER_TICKS 16384
#define RADS_PER_TICK PI/(ENCODER_TICKS/2)

#define MAX_VEL 160.0
#define MAX_ACC 20000.0
#define HOMING_POS 800.0 // position after hitting limit switch
#define HOMING_SPEED 80.0
#define HOMING_ACC 500.0



//1111111101110111

//1110111011101110


void setup() {
  Serial.begin(115200);

  pinMode(DIAG_PIN, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(CS_PIN_ENCODER, OUTPUT);
  pinMode(CS_PIN_MOTOR, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(CS_PIN_ENCODER, HIGH);
  digitalWrite(CS_PIN_MOTOR, HIGH);

  Serial.print("finished first half!");
  pinMode(11, OUTPUT);
  pinMode(12, INPUT);
  pinMode(13, OUTPUT);
  TMC5160Stepper motor = TMC5160Stepper(CS_PIN_MOTOR, RSENSE, 11, 12, 13, -1);
  motor.setSPISpeed(1000000);

  Serial.println(motor.test_connection());


  // motor.toff(0);                           //clear status bits in driver
  digitalWrite(EN_PIN, LOW);                //enable the driver so that we can send the initial register values

  // /*Initial settings for basic SPI command stepper drive no other functions enabled*/ {
  //   motor.begin();                         // start the tmc library

  //   /* base GCONF settings for bare stepper driver operation*/    {
  //     motor.recalibrate(0);                //do not recalibrate the z axis
  //     motor.faststandstill(0);             //fast stand still at 65ms
  //     motor.en_pwm_mode(0);                //no silent step
  //     motor.multistep_filt(0);             //normal multistep filtering
  //     motor.shaft(0);                      //motor direction cw
  //     motor.small_hysteresis(0);           //step hysteresis set 1/16
  //     motor.stop_enable(0);                //no stop motion inputs
  //     motor.direct_mode(0);                //normal driver operation
  //   }

  //   /* Set operation current limits */
  //   motor.rms_current(600, 1);    //set Irun and Ihold for the drive

  //   // /* short circuit monitoring */    {
  //   //   motor.diss2vs(0);                    //driver monitors for short to supply
  //   //   motor.s2vs_level(6);                 //lower values set drive to be very sensitive to low side voltage swings
  //   //   motor.diss2g(0);                     //driver to monitor for short to ground
  //   //   motor.s2g_level(6);                  //lower values set drive to be very sensitive to high side voltage swings
  //   // }

  //   /* minimum settings to to get a motor moving using SPI commands */{
  //     motor.tbl(2);                          //set blanking time to 24
  //     motor.toff(8);                 //pwm off time factor
  //     // motor.pwm_freq(1);                     //pwm at 35.1kHz
  //   }
  //   /* Reseting drive faults and re-enabling drive */ {
  //     digitalWrite(EN_PIN, HIGH);             //disable drive to clear any start up faults
  //     delay(1000);                            //give the drive some time to clear faults
  //     digitalWrite(EN_PIN, LOW);              //re-enable drive, to start loading in parameters
  //     motor.GSTAT(7);                        //clear gstat faults
  //   }
  // }
  motor.begin();
	motor.toff(3);
	motor.rms_current(800);
	motor.en_pwm_mode(true);

	motor.a1(1000);
	motor.v1(50000);
	motor.AMAX(500);
	motor.VMAX(200000);
	motor.DMAX(700);
	motor.d1(1400);
	motor.VSTOP(10);
	motor.RAMPMODE(0);
	motor.XTARGET(-51200);


	while(true) {
		delay(1000);

		auto xactual = motor.XACTUAL();
		auto xtarget = motor.XTARGET();

		char buffer[256];
		sprintf(buffer, "ioin=%#-10lx xactual=%7ld\n",
			motor.IOIN(),
			xactual
			);
		Serial.print(buffer);
    Serial.println(motor.test_connection());

		if (xactual == xtarget) {
			motor.XTARGET(-xactual);
		}
	}  
  // motor.begin();
  Serial.print("sd mode: ");
  Serial.println(motor.sd_mode());
  motor.en_pwm_mode(0);
  motor.rms_current(600, 0.5);
  motor.TZEROWAIT(0);
  motor.microsteps((uint16_t)(USTEPS));
  motor.diag0_stall(false);
  motor.VSTART(10);
  motor.VSTOP(10);
  motor.XACTUAL(0);
  motor.VMAX(0);
  motor.toff(5);
  motor.RAMPMODE(1);
  motor.en_softstop(true);
  Serial.print("sd mode: ");
  Serial.println(motor.sd_mode());

  Serial.print("setup done!");
}


void loop() { }


