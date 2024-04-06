/*
 Name:		Gasser_Engine_BLDC_Motor.ino
 Created:	3/31/2024 12:30:36 PM
 Author:	Kevin Guest (AKA The Bionicbone)
*/

#include <ESP32_ESC_High_Resolution_Driver.h>

ESP32_ESC_HIGH_RESOLUTION_DRIVER ESC_Control = ESP32_ESC_HIGH_RESOLUTION_DRIVER();
const byte ESC_pin = 4;
const int ESC_Frequency = 50;
const byte RPM_pin = 14;
int RPM_counter = 0;
unsigned long RPM_timer = 0;

// Governer Setting
int ESC_us = 1120;            // Spin up value, often to first 100us does nothing
float GOV_factor = 0.004;     // Used to control the us increases
int GOV_timing_us = 500;      // us between RPM checks and governer changes
int GOV_RPM = 12000;          // The desired RPM
byte GOV_RPM_Zero_Count = 0;  // Safety, if RPM sensor fails this will stop the motor
bool GOV_Fail = false;        // True if we should skip motor driving signals

// Tempertures
const byte rectifier_TemperaturePin = 34;
const byte BEC_TemperaturePin = 35;
const byte protectionDiode_TemperaturePin = 32;
byte rectifier_Temperature = 0;
byte BEC_Temperature = 0;
byte protectionDiode_Temperature = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  vTaskDelay(500);

  Serial.println("Gasser Charging Circuit Tester \n");

  // Set up MCPWM
  ESC_Control.ESC_attach(ESC_pin, ESC_Frequency);

  // Pause to allow the ESC to activate (MUST BE CALIBRATED !!)
  ESC_Control.ESC_set_us(1000);
  Serial.println("Pause for ESC Activation...");
  vTaskDelay(5000);

 // Uncomment to Calibrate ESC to ESP32_ESC_HIGH_RESOLUTION_DRIVER outputs
 // then follow instructions on Serial output 
 // ESC_Control.ESC_calibrate();
 // while(true);

  pinMode(RPM_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_pin), RPM_ISR, RISING);
  RPM_timer = millis();

  ESC_Control.ESC_set_us(ESC_us); //RPM (no load): 1120 = 5k, 1200 = 10k, 1220 = 11.4k, 1240 = 12.2k 
  delay(2000);                    // Give time for the motor to start up before governing to 12,000 RPM              


  // Temperature Pins
  pinMode(rectifier_TemperaturePin, INPUT);
  pinMode(BEC_TemperaturePin, INPUT);
  pinMode(protectionDiode_TemperaturePin, INPUT);
}

void loop() {
  // check RPM every second
  // RPM_counter = 200 = 12000 RPM
  if (millis() >= RPM_timer + GOV_timing_us) {
    detachInterrupt(digitalPinToInterrupt(RPM_pin));
    int currentRPM = (RPM_counter / ((float)(millis() - RPM_timer) / 1000)) * 60;
    Serial.print("Current RPM Calculation = "); Serial.println(currentRPM);
    
    // Only update motor if RPM is in a good state
    if (GOV_Fail == false) {

      // Governer Checks and new us calculations
      if (abs(GOV_RPM - currentRPM) > 250) {
        ESC_us = ESC_us + int((GOV_RPM - currentRPM) * GOV_factor);
      }

      // Set the ESC PWM to the new us calculated 
      ESC_Control.ESC_set_us(ESC_us);
    }

    // Get the temperatures
    rectifier_Temperature = analogReadMilliVolts(rectifier_TemperaturePin) / 10;
    Serial.printf("    Rectifier Temperature = %dc \n", rectifier_Temperature);
    BEC_Temperature = analogReadMilliVolts(BEC_TemperaturePin) / 10;
    Serial.printf("          BEC Temperature = %dc \n", BEC_Temperature);
    protectionDiode_Temperature = analogReadMilliVolts(protectionDiode_TemperaturePin) / 10;
    Serial.printf("Protect Diode Temperature = %dc \n", protectionDiode_Temperature);
    Serial.println();
    
    // RPM saftey checks
    if (currentRPM == 0) {
      GOV_RPM_Zero_Count++;
      if (GOV_RPM_Zero_Count >= 5) {
        ESC_Control.ESC_set_us(1000);
        Serial.println("RPM Sensor Failure, Emergancy STOP !");
        GOV_Fail = true; // signal no more motor updates
      }
    }
    if (currentRPM >= 16000) {
      ESC_Control.ESC_set_us(1000);
      Serial.println("RPM Too High Failure, Emergancy STOP !");
      GOV_Fail = true; // signal no more motor updates
    }
    
    // Reset for the next loop
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