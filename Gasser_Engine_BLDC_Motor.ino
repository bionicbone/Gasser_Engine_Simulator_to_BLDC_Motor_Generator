/*
 Name:		Gasser_Engine_BLDC_Motor.ino
 Created:	3/31/2024 12:30:36 PM
 Author:	Kevin Guest (AKA The Bionicbone)
*/

#include <PID_v1.h>
#include <ESP32_ESC_High_Resolution_Driver.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

ESP32_ESC_HIGH_RESOLUTION_DRIVER ESC_Control = ESP32_ESC_HIGH_RESOLUTION_DRIVER();
byte ESC_pin = 4;
int ESC_Frequency = 50;
byte RPM_pin = 14;
int RPM_counter = 0;
unsigned long RPM_timer = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  vTaskDelay(500);

  Serial.println("ESP32_ESC_HIGH_RESOLUTION_DRIVER - Example 0 to 25% Motor RPM \n");

  // Set up MCPWM
  ESC_Control.ESC_attach(ESC_pin, ESC_Frequency);

  // Pause to allow the ESC to activate (MUST BE CALIBRATED !!)
  ESC_Control.ESC_set_us(1000);
  Serial.println("Pause for ESC Activation...");
  vTaskDelay(7000);

 // Uncomment to Calibrate ESC to ESP32_ESC_HIGH_RESOLUTION_DRIVER outputs
 // then follow instructions on Serial output 
 // ESC_Control.ESC_calibrate();
 // while(true);

  pinMode(RPM_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_pin), RPM_ISR, RISING);
  RPM_timer = millis();
}

void loop() {

  ESC_Control.ESC_set_us(1200);  //RPM: 1120 = 5k, 1200 = 10k, 1220 = 11.4k, 1240 = 12.2k 
  if (RPM_counter > 100) {
    detachInterrupt(digitalPinToInterrupt(RPM_pin));
    int finalRPM = (RPM_counter / ((float)(millis() - RPM_timer) / 1000)) * 60;
    Serial.print("Final RPM Calculation = ");
    Serial.println(finalRPM);
    RPM_timer = millis();
    RPM_counter = 0;
    attachInterrupt(digitalPinToInterrupt(RPM_pin), RPM_ISR, RISING);
  }
  /*
    Example COde for ESP32_ESC_HIGH_RESOLUTION_DRIVER
    0-25% example, limited to 25% because it is not good to spin a motor with zero load at full speed.
    Alter as necessary, with caution, watch out for flying propellers or motor cans pulling off the main motor body !!
    Ask me how I know, USE WITH CAUTION !!!
  */


  //// increment by us for RC ESC for Max Resolution of 1000 steps
  //for (uint16_t i = 1000; i < 1250; i++) {
  //  ESC_Control.ESC_set_us(i);
  //  float PWM = ESC_Control.convert_us_to_PWM(i);
  //  uint16_t us = ESC_Control.convert_PWM_to_us(PWM);
  //  Serial.print("PWM "); Serial.print(PWM); Serial.print("% = "); Serial.print(us); Serial.println("us");
  //  vTaskDelay(100);
  //}
  //// decrement by 0.005 PWM % for RC ESC for Max Resolution of 1000 steps
  //for (float i = 6.250; i > 5.100; i -= 0.005) {
  //  ESC_Control.ESC_setPWM(i);
  //  uint16_t us = ESC_Control.convert_PWM_to_us(i);
  //  float PWM = ESC_Control.convert_us_to_PWM(us);
  //  Serial.print("PWM "); Serial.print(PWM); Serial.print("% = "); Serial.print(us); Serial.println("us");
  //  vTaskDelay(100);
  //}
}


void RPM_ISR() {
  RPM_counter++;
}