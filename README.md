Good Results using 3K0 resistor on the 7v pins of the YEP 20A SBEC - this supports my original work, even though initially I went with a 1k2 resistor.

Bench Test Powered with 1000kv Generator Motor

Bench test circuit is powered by a Turnigy D2830/11 1000KV BLDC motor and one of the 4 test rectifiers. 
The 1000KV generator motor will be driven by a Turnigy Typhoon 600H Heli Motor 1100KV (600 class) at 12,000 RPM to simulate the Gasser 23cc engine in normal flight.

1000KV Generator Motor, YEP 20A BEC with 3K0 resistor across 7v pins, a test rectifier, LiFePO4 2s 2100mAH battery .
NOTE 1: Output Voltage (after diode) would be the Servo and Flight Controller Voltage so very important not be too high as 6v Servos will be used.
NOTE 2: The 1100kv RC Heli 600 sized motor is being powered by a fully charged 6S  5000mAH battery
NOTE 3: The 1100kv RC Heli 600 sized motor is connected to and ESC, which is being controlled automatically by and ESP32 and governed to 12,000RPM. 
NOTE 4: The code for the ESP32 can be found at https://github.com/bionicbone/Gasser_Engine_Simulator_to_BLDC_Motor_Generator
NOTE 5: Ambient temperature is 19c, inside the canopy I would expect to find temperatures of around 45c and this should be considered.
NOTE 6: Each test is run for 10 minutes.

Bench Test Circuit

Bench Test Results
![image](https://github.com/bionicbone/Gasser_Engine_Simulator_to_BLDC_Motor_Generator/assets/7845867/856cc2d9-024b-4b5a-b1cf-7aac6255ea8f)

![image](https://github.com/bionicbone/Gasser_Engine_Simulator_to_BLDC_Motor_Generator/assets/7845867/9785b250-e3a8-4e9e-a97e-0ee2b136d658)
