#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"
#include <Encoder.h>
#include <Bounce2.h>
#include <math.h>
#include <PID_v1.h>

#define BUTTON_PIN 9
#define READ_TEMP_MS 750
#define RELAY_SINK_DEADBAND 0.5

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int thermoDO = 16;
int thermoCS = 17;
int thermoCLK = 18;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

int encoderPinA = 10;
int encoderPinB = 11;
Encoder myEnc(encoderPinA, encoderPinB);
Bounce2::Button encoderButton = Bounce2::Button();

int relayPin = 6;

bool setpointLocked = false;
long oldPosition = -999;
float currentTemp = 22;
int setpoint = 22;
volatile long encoderPosition = 0;
long lastEncoderPosition = 0;
bool displayNeedsUpdate = true;
bool relayIsOn = false;
bool prevRelayIsOn = false;
bool prevTempTooLow = false;

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

// Kalman filter variables
float Q = 0.1;
float R = 0.1;
float P = 1.0;
float K;
float X = 22;

float kalmanFilter(float measurement) {
  P = P + Q;
  K = P / (P + R);
  X = X + K * (measurement - X);
  P = (1 - K) * P;
  return X;
}

// Feedforward variables
float heatLossRate = 0.05; // Adjust this value based on your system's heat loss rate
float feedforwardTerm = 0;

void setup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(13, LOW);
  digitalWrite(12, HIGH);
  delay(1000);
  
  encoderButton.attach(BUTTON_PIN, INPUT_PULLUP);
  encoderButton.interval(5);
  encoderButton.setPressedState(LOW);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  Setpoint = setpoint;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  Serial.begin(9600);
  Serial.println("MAX6675 test");
  delay(500);
  Serial.println("Setup complete - starting main loop");
}

void updateEncoder() {
  static int lastEncoded = 0;
  static int changeCount = 0;
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011 || sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    changeCount++;
    if (changeCount >= 4) {
      if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        encoderPosition--;
      }
      if (sum == 0b1110 || sum == 0b0117 || sum == 0b0001 || sum == 0b1000) {
        encoderPosition++;
      }
      changeCount = 0;
    }
  }

  lastEncoded = encoded;
}

void handleSetpointLock() {
  encoderButton.update();

  if (encoderButton.pressed()) {
    setpointLocked = !setpointLocked;
    displayNeedsUpdate = true;
    if (setpointLocked) {
      Serial.println("Setpoint locked.");
    } else {
      Serial.println("Setpoint unlocked.");
      noInterrupts();
      lastEncoderPosition = encoderPosition;
      interrupts();
    }
  }
}

void handleSetpointChange() {
  long currentEncoderPosition;
  noInterrupts();
  currentEncoderPosition = encoderPosition;
  interrupts();

  if (currentEncoderPosition != lastEncoderPosition && !setpointLocked) {
    setpoint += (currentEncoderPosition - lastEncoderPosition);
    lastEncoderPosition = currentEncoderPosition;
    setpoint = max(0, min(setpoint, 100));

    Serial.print("Setpoint changed to: ");
    Serial.println(setpoint);

    displayNeedsUpdate = true;
  }
}

void readTemp() {
  static unsigned long lastTempReadTime = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastTempReadTime >= READ_TEMP_MS) {
    lastTempReadTime = currentMillis;
    float newTemp = thermocouple.readCelsius();
    currentTemp = kalmanFilter(newTemp);

    if (fabs(newTemp - currentTemp) >= 0.1) {
      Serial.print("Sensor Temp Updated: ");
      Serial.print(currentTemp);
      Serial.println("C");
      displayNeedsUpdate = true;
    }
  }
}

void setRelayWithPID() {
  Input = currentTemp;
  Setpoint = setpoint;
  myPID.Compute();

  // Calculate feedforward term based on heat loss rate
  feedforwardTerm = heatLossRate * (Setpoint - currentTemp);

  // Add feedforward term to PID output
  double totalOutput = Output + feedforwardTerm;
  totalOutput = constrain(totalOutput, 0, 255); // Ensure output is within valid range

  analogWrite(relayPin, totalOutput);

  if (prevTempTooLow != (currentTemp < (setpoint - RELAY_SINK_DEADBAND)) || (prevRelayIsOn != (totalOutput > 0))) {
    displayNeedsUpdate = true;
    prevTempTooLow = (currentTemp < (setpoint - RELAY_SINK_DEADBAND));
    prevRelayIsOn = (totalOutput > 0);
  }
}

void updateDisplay() {
  if (!displayNeedsUpdate) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Set: ");
  display.print(setpoint);
  display.print("C ");
  if (setpointLocked) {
    display.print("*");
  }

  display.setCursor(0, 20);
  display.print("Temp:");
  display.print(currentTemp);

  display.setCursor(100, 40);
  if (relayIsOn) {
    display.print("~");
  }
  display.display();

  displayNeedsUpdate = false;
}

void loop() {
  handleSetpointLock();
  handleSetpointChange();
  readTemp();
  setRelayWithPID();
  updateDisplay();
}