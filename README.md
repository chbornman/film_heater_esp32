# Film Development Water Bath Heater
ESP32 project that controls the temperature of a water bath with an SSR powered heating coil and a thermocouple for feedback. Electronics schematic will be included in the future. 

### Usage
* Download VSCode
* Install PlatformIO plugin
* Add project to PlatformIO
* Upload to ESP32 board

### Manual
* Light switch acts as ON/OFF for the microcontroller as well as the output power outlet
* Output Power Outlet has one 120VAC passthrough (top plug) and one Solid State Relay (SSR) controlled (bottom plug)
* Turning encoder will change the heating setpoint, encoder press will lock/unlock setpoint
* Display shows current setpoint, temperature, and the PWM percentage going to the SSR