# ATtiny85 programm for IOT weather station

This is a small programm for my small IOT weather station. The weatherstation consists of the following parts:

- ATtiny85 and LT1763-3.3 for power control
- TP4056 to charge the li-ion battery with solar
- BME280 to measure temperature, humidity and pressure
- ESP8266 to connect to WiFi and send the reedings via MQTT

This scetch is the part of the ATtiny85. This program will measure its input voltage VCC which is also the li-ion battery voltage. If it is too low the uC will go to sleep. If it is high enough, it will enable 3.3V supply and start to listen to I2C. It will measure VCC every second to ensure that the battery will not be damaged. If the voltage is too low it will go to sleep immediately. Going to sleep means, that the 3.3V supply will be disabled. The I2C master can send the uC to sleep as well by editing its second register. The register will tell the uC how long to sleep. The register needs to be multiplied with 8s to get the correct time. With register 3 and 4 the last converted value for VCC can be read. Register 5 to X will save the 1.1 voltage reference and the voltage borders. The first bit of register 1 is 1 if the VCC measurement was sucessfull. THe second bit will be 1 if the voltage reference or the voltage limits were conditioned via I2C.

The internal 1.1V reference is used to measure the VCC. This reference can vary between 1.0V and 1.2V. To get accurate results you should change the correct registers to the actual reference voltage of the IC you are using.
