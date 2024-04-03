/*
 Name:		Gasser_Engine_BLDC_Motor.ino
 Created:	3/31/2024 12:30:36 PM
 Author:	Kevin Guest (AKA The Bionicbone)
*/

#include <PID_v1.h>
#include <ESP32_ESC_High_Resolution_Driver.h>

ESP32_ESC_HIGH_RESOLUTION_DRIVER ESC_Control = ESP32_ESC_HIGH_RESOLUTION_DRIVER();
const byte ESC_pin = 4;
const int ESC_Frequency = 50;
const byte RPM_pin = 14;
int RPM_counter = 0;
unsigned long RPM_timer = 0;

//Define PID control Variables we'll be connecting to
double PID_setpoint, PID_input, PID_output;
int PID_mappedOutput = 0;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&PID_input, &PID_output, &PID_setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
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

  ESC_Control.ESC_set_us(1200);  //RPM: 1120 = 5k, 1200 = 10k, 1220 = 11.4k, 1240 = 12.2k 
  delay(2000);

  //initialize the PID variables we're linked to
  PID_input = 0;        // 0 because the motor is not yet spinning.
  PID_setpoint = 200;   // 12000 RPM / 60 = 200 Revolutions per Second

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // check RPM every second
  // RPM_counter = 200 = 12000 RPM
  if (millis() >= RPM_timer + 1000) {   
    detachInterrupt(digitalPinToInterrupt(RPM_pin));
    int finalRPM = (RPM_counter / ((float)(millis() - RPM_timer) / 1000)) * 60;
    PID_input = RPM_counter;
    Serial.print("Final RPM Calculation = ");
    Serial.println(finalRPM);
    
    // PID Calculation
    myPID.Compute();
    PID_mappedOutput = map(PID_output, 0, 255, 1000, 2000);
    Serial.print("PID_output = ");
    Serial.println(PID_output);
    Serial.print("PID_input = ");
    Serial.println(PID_input);
    Serial.print("PID_mappedOutput = ");
    Serial.println(PID_mappedOutput);
    ESC_Control.ESC_set_us(PID_mappedOutput);
    
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