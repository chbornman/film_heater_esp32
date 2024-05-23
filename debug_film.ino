// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #include "max6675.h"
// #include <Encoder.h>
// #include <Bounce2.h>

// #define BUTTON_PIN 9  // Change as per your connection

// // Declaration for an SSD1306 display connected to I2C (SCL, SDA pins)
// #define SCREEN_WIDTH 128  // OLED display width, in pixels
// #define SCREEN_HEIGHT 64  // OLED display height, in pixels
// #define OLED_RESET -1     // Reset pin # (or -1 if sharing Arduino reset pin)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// // MAX6675 setup
// int thermoDO = 16;
// int thermoCS = 17;
// int thermoCLK = 18;
// MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// // Encoder and Button
// int encoderPinA = 10;  // Change as per your connection
// int encoderPinB = 11;  // Change as per your connection
// Encoder myEnc(encoderPinA, encoderPinB);
// Bounce2::Button encoderButton = Bounce2::Button();

// // relay pin
// int relayPin = 6;  // Digital pin connected to the relay module

// // Variables
// bool setpointLocked = false;
// long oldPosition = -999;
// float currentTemp = 22;
// int setpoint = 22;  // Default setpoint
// volatile long encoderPosition = 0;
// long lastEncoderPosition = 0;
// bool displayNeedsUpdate = true;  // Initialize to true to update display initially
// bool relayIsOn = false;
// bool prevTempTooLow = false;

// void setup() {
//   //ENCODER POWER
//   pinMode(14, OUTPUT);
//   digitalWrite(14, LOW);
//   pinMode(15, OUTPUT);
//   digitalWrite(15, HIGH);

//   //DISPLAY POWER
//   pinMode(13, OUTPUT);
//   pinMode(12, OUTPUT);
//   digitalWrite(13, LOW);
//   digitalWrite(12, HIGH);
//   delay(1000);

//   //encoder button
//   encoderButton.attach(BUTTON_PIN, INPUT_PULLUP);  // Use external pull-up
//   encoderButton.interval(5);                       // Debounce interval in milliseconds
//   encoderButton.setPressedState(LOW);              // High state corresponds to physically pressing the button
//     // Attach interrupts
//   attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

//   // Initialize the OLED display
//   if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x64
//     Serial.println(F("SSD1306 allocation failed"));
//     for (;;)
//       ;  // Don't proceed, loop forever
//   }
//   Serial.begin(9600);
//   Serial.println("MAX6675 test");
//   // wait for MAX chip to stabilize
//   delay(500);
//   Serial.println("Setup complete - starting main loop");
// }

// void updateEncoder() {
//   static int lastEncoded = 0;
//   static int changeCount = 0;  // Add a counter to track changes

//   int MSB = digitalRead(encoderPinA);      // MSB = most significant bit
//   int LSB = digitalRead(encoderPinB);      // LSB = least significant bit
//   int encoded = (MSB << 1) | LSB;          // converting the 2 pin value to single number
//   int sum = (lastEncoded << 2) | encoded;  // adding it to the previous encoded value

//   // Increment changeCount on any change, but only update position every fourth change
//   if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011 || sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
//     changeCount++;
//     if (changeCount >= 4) {  // Update position only every fourth change
//       if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
//         encoderPosition--;  // Swap increment and decrement to reverse direction
//       }
//       if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
//         encoderPosition++;  // Swap increment and decrement to reverse direction
//       }
//       changeCount = 0;  // Reset change count after updating
//     }
//   }

//   lastEncoded = encoded;  // Store this value for next time
// }

// void handleSetpointLock() {
//   // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
//   encoderButton.update();

//   if (encoderButton.pressed()) {
//     setpointLocked = !setpointLocked;  // Toggle lock state
//     displayNeedsUpdate = true;         // Set flag to update display
//     if (setpointLocked) {
//       Serial.println("Setpoint locked.");
//     } else {
//       Serial.println("Setpoint unlocked.");
//     }
//   }
// }

// void handleSetpointChange() {
//   long currentEncoderPosition;

//   // Temporarily disable interrupts to safely read encoderPosition
//   noInterrupts();
//   currentEncoderPosition = encoderPosition;
//   interrupts();

//   if (currentEncoderPosition != lastEncoderPosition && !setpointLocked) {
//     // Calculate new setpoint based on encoder position
//     // This assumes each encoder step changes the setpoint by 1 unit
//     setpoint += (currentEncoderPosition - lastEncoderPosition);
//     lastEncoderPosition = currentEncoderPosition;  // Update last position

//     // Optionally, add bounds to setpoint
//     setpoint = max(0, min(setpoint, 100));  // Adjust min/max as needed

//     Serial.print("Setpoint changed to: ");
//     Serial.println(setpoint);

//     displayNeedsUpdate = true;  // Set flag to update display
//   }
// }

// void readTemp() {
//   float newTemp = thermocouple.readCelsius();
//   if (newTemp != currentTemp) {
//     currentTemp = newTemp;
//     displayNeedsUpdate = true;  // Set flag to update display if temperature changes
//   }
// }

// void setRelay() {
//   bool tempTooLow = (currentTemp < setpoint);

//   if (tempTooLow)
//   {
//     digitalWrite(relayPin, HIGH);
//     relayIsOn = true;
//   }
//   else 
//   {
//     digitalWrite(relayPin, LOW);
//     relayIsOn = false;
//   }

//   if (prevTempTooLow != tempTooLow)
//   {
//     displayNeedsUpdate = true; 
//   }

//   prevTempTooLow = tempTooLow;
// }

// void updateDisplay() {
//   if (!displayNeedsUpdate) {
//     return;  // If no update is needed, just return
//   }

//   display.clearDisplay();
//   display.setTextSize(2);
//   display.setTextColor(SSD1306_WHITE);  // Set text color to white
//   display.setCursor(0, 0);
//   display.print("Set: ");
//   display.print(setpoint);
//   display.print("C ");
//   if (setpointLocked) {
//     display.print("*");  // Add an asterisk if setpoint is locked
//   }

//   display.setCursor(0, 20);
//   display.print("Temp: ");
//   display.print((int)round(currentTemp));
//   display.print("C");
  
//   display.setCursor(100, 40);
//   if (relayIsOn) {
//     display.print("~");  // Add [ON] if the relay is active
//   }
//   display.display();

//   displayNeedsUpdate = false;  // Reset update flag
// }

// void loop() {
//   handleSetpointLock();
//   handleSetpointChange();
//   readTemp();
//   setRelay();
//   updateDisplay();
// }
