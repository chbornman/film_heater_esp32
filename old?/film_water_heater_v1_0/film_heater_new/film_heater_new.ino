#include <TFT_eSPI.h>
#include <SPI.h>
#include <max6675.h>

#define THERMO_DO 12
#define THERMO_CS 13
#define THERMO_CLK 14
#define RELAY_PIN 2

MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// Define TFT display
TFT_eSPI tft = TFT_eSPI();

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Initialize Relay as OFF
  Serial.begin(9600); // Start serial communication at 9600 baud rate

  // Initialize TFT display
  tft.init();
  tft.setRotation(1); // Use landscape format
  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
}

void loop() {
  double temp;
  temp = thermocouple.readFahrenheit(); // Read temperature in Fahrenheit

  if (isnan(temp)) {
    Serial.println("Failed to read from thermocouple!");
    tft.drawString("Error reading temp!", 0, 0);
  } else {
    Serial.println(temp);
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Temp: " + String(temp) + " F", 0, 0);
    
    if (temp < 102.0) {
      digitalWrite(RELAY_PIN, HIGH); // Turn ON Relay
    } else {
      digitalWrite(RELAY_PIN, LOW); // Turn OFF Relay
    }
  }
  
  delay(1000); // Wait for 1 second before the next reading
}
