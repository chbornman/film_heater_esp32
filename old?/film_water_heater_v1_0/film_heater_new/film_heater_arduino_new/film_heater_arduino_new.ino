#include <max6675.h>

// Define pins
#define THERMO_CLK 8
#define THERMO_CS 9
#define THERMO_SO 10
#define RELAY_PIN 3
#define BUZZER_PIN 4
#define GND_PIN 5
#define ENCODER_CLK 11
#define ENCODER_DT 12
#define BUTTON_PIN 13

// Define target temperatures, modes, and other constants
enum Modes {BW, C41, ECN2, MANUAL};
Modes currentMode = BW;

int lastStateCLK;
int lastStateDT;
float targetTemps[] = {72.0, 102.0, 106.0, 90.0};
float currentTemp = 120.0;
float TARGET_TEMP = targetTemps[BW];  // Initial value
const float DEADBAND = 1.0;
unsigned long lastPrintTime = 0;
const long printDelay = 1000;  // Print every 1000ms or 1s


// Create thermocouple interface instance
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_SO);

int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const long debounceDelay = 50;

int encoderState, lastEncoderState;
int encoderPinA, encoderPinB;

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(GND_PIN, OUTPUT);
  digitalWrite(GND_PIN, LOW);  // Force pin 5 to be LOW
  
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(RELAY_PIN, LOW);  
  digitalWrite(BUZZER_PIN, LOW);  

  lastStateCLK = digitalRead(ENCODER_CLK);

  Serial.begin(9600);
  delay(500);
}

void beepBuzzer(int mode) {
  switch (mode) {
    case BW:
      digitalWrite(BUZZER_PIN, HIGH);
      delay(150);
      digitalWrite(BUZZER_PIN, LOW);
      delay(150);
      break;
    case C41:
      for (int i = 0; i < 2; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(150);
        digitalWrite(BUZZER_PIN, LOW);
        delay(150);
      }
      break;
    case ECN2:
      for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(150);
        digitalWrite(BUZZER_PIN, LOW);
        delay(150);
      }
      break;
    case MANUAL:
      for (int i = 0; i < 6; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(50);
        digitalWrite(BUZZER_PIN, LOW);
        delay(50);
      }
      break;
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastPrintTime >= printDelay) {
    currentTemp = thermocouple.readFahrenheit();
    Serial.print("Current Temperature: ");
    Serial.println(currentTemp);
    lastPrintTime = currentMillis;  // Reset the last print time
  }


/********* SWITCH MODES ***********************************/
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW) {
      currentMode = static_cast<Modes>((currentMode + 1) % 4);
      switch (currentMode) {
        case BW: 
          beepBuzzer(BW); 
          Serial.println("Mode: B+W");
          TARGET_TEMP = targetTemps[BW];
          break;
        case C41: 
          beepBuzzer(C41);
          Serial.println("Mode: C41");
          TARGET_TEMP = targetTemps[C41];
          break;
        case ECN2: 
          beepBuzzer(ECN2);
          Serial.println("Mode: ECN2");
          TARGET_TEMP = targetTemps[ECN2];
          break;
        case MANUAL: 
          beepBuzzer(MANUAL);
          Serial.println("Mode: Manual");
          TARGET_TEMP = targetTemps[MANUAL];
          break;
      }
      Serial.print("New Target Temperature: ");
      Serial.println(TARGET_TEMP);
    }
  }
  lastButtonState = reading;


/********* MANUAL MODE ADJUST TEMP ***********************************/
int currentStateCLK = digitalRead(ENCODER_CLK);
int currentStateDT = digitalRead(ENCODER_DT);

if (currentStateCLK != lastStateCLK && currentStateCLK == HIGH) { // Check for a rising edge on CLK
    if (currentStateDT != currentStateCLK) {  // CW rotation
        TARGET_TEMP++;
    } else {  // CCW rotation
        TARGET_TEMP--;
    }
    Serial.println(TARGET_TEMP);
}

lastStateCLK = currentStateCLK;
lastStateDT = currentStateDT;


/********* COMMAND RELAY ***********************************/
  if (currentTemp < TARGET_TEMP - DEADBAND) {
    digitalWrite(RELAY_PIN, HIGH);
  } else if (currentTemp >= TARGET_TEMP) {
    digitalWrite(RELAY_PIN, LOW);
  }

  //delay(1000);
}
