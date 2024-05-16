#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX6675.h>
#include <Encoder.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SCL, SDA pins)
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX6675 setup
int thermoDO = 9;
int thermoCS = 10;
int thermoCLK = 11;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Encoder setup
int encA = 14; // Rotary encoder A pin
int encB = 15; // Rotary encoder B pin
int encButton = 16; // Rotary encoder button pin
Encoder myEnc(encA, encB);

// Relay setup
int relayPin = 25; // Digital pin connected to the relay module

// Variables for encoder and temperature control
long oldPosition = -999;
int setpoint = 25; // Default setpoint
bool setpointLocked = false;

void setup() {
  Serial.begin(115200);
  pinMode(relayPin, OUTPUT);
  pinMode(encButton, INPUT_PULLUP); // Setup encoder button with pull-up resistor

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1); // Set text size to 1 for more space
  display.setTextColor(SSD1306_WHITE); // Set text color to white
  display.setCursor(0, 0);
  display.println("FILM TEMP KEEPER"); // Display constant title at the top

  //THERMOCOUPLE POWER
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  //ENCODER POWER
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
}

void loop() {
  long newPosition = myEnc.read()/4; // Assuming encoder increments 4 steps per notch
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    if (!setpointLocked) {
      setpoint = newPosition;
      setpoint = constrain(setpoint, 0, 100); // Constrain setpoint to realistic values
    }
  }

  if (digitalRead(encButton) == LOW) {
    setpointLocked = !setpointLocked; // Toggle lock state when button is pressed
    delay(300); // Debounce delay
  }

  double currentTemp = thermocouple.read(); // Read current temperature
  if (currentTemp < setpoint) {
    digitalWrite(relayPin, HIGH); // Turn on heating coil
  } else {
    digitalWrite(relayPin, LOW); // Turn off heating coil
  }

  // Update display
  display.clearDisplay();
  display.setTextSize(1); // Set text size to 1 for more space
  display.setTextColor(SSD1306_WHITE); // Set text color to white
  display.setCursor(0, 0);
  display.println("FILM TEMP KEEPER"); // Display constant title at the top

  display.setCursor(0, 10); // Move the cursor down to not overlap with the title
  display.print("Set: ");
  display.print(setpoint);
  display.print("C ");
  display.print(setpointLocked ? "*" : ""); // Show * when locked

  display.setCursor(0, 20); // Move the cursor further down for the current temperature
  display.print("Temp: ");
  display.print(currentTemp);
  display.println("C");

  display.display();

  delay(100); // Small delay to reduce flickering
}
