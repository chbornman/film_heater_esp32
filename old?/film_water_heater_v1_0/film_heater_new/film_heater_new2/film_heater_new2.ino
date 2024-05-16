#include <max6675.h>

// Define pins
#define THERMO_CLK 8
#define THERMO_CS 9
#define THERMO_SO 10
#define RELAY_PIN 3

// Define target temperature and deadband
#define TARGET_TEMP 102.0
#define DEADBAND 1.0  // Adjust as needed

// Create thermocouple interface instance
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_SO);

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Make sure relay is off during startup
  
  Serial.begin(9600); 
  delay(500); // Allow thermocouple some setup time
}

void loop() {
  float currentTemp = thermocouple.readFahrenheit();
  
  // Print current temperature (optional)
  Serial.print("Current Temperature: ");
  Serial.println(currentTemp);
  
  if (currentTemp < TARGET_TEMP - DEADBAND) {
    digitalWrite(RELAY_PIN, HIGH);  // Turn ON heating coil
  } else if (currentTemp >= TARGET_TEMP) {
    digitalWrite(RELAY_PIN, LOW);   // Turn OFF heating coil
  }
  
  delay(1000); // Delay for 1 second (can be adjusted)
}
