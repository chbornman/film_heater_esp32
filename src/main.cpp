/***************** INCLUDES ****************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"
#include <Encoder.h>
#include <Bounce2.h>
#include <math.h>
#include <PID_v1.h>
#include <Preferences.h>  // Include the Preferences library for NVS

/**************** MACROS ******************/
#define BUTTON_PIN 9
#define READ_TEMP_MS 750
#define RELAY_SINK_DEADBAND 0.5
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define ENCODER_PIN_A 10
#define ENCODER_PIN_B 11
#define RELAY_PIN 6
#define THERMO_DO 16
#define THERMO_CS 17
#define THERMO_CLK 18
// Tunable parameters
#define HEAT_LOSS_RATE 1
#define P_GAIN 20
#define I_GAIN 0
#define D_GAIN 0

/*************** GLOBALS *****************/
// Display and sensor objects
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);
Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);
Bounce2::Button encoderButton = Bounce2::Button();
Preferences preferences;  // Create a Preferences object

// Variables
bool setpointLocked = false;
long oldPosition = -999;
float currentTemp = 22;
int setpoint = 22;
float previousPwmPercentage = -1.0;
volatile long encoderPosition = 0;
long lastEncoderPosition = 0;
bool displayNeedsUpdate = true;
bool relayIsOn = false;
bool prevRelayIsOn = false;
bool prevTempTooLow = false;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, P_GAIN, I_GAIN, D_GAIN, DIRECT);

// Kalman filter variables
float Q = 0.1;
float R = 0.1;
float P = 1.0;
float K;
float X = 22;

// Feedforward variable
float feedforwardTerm = 0;

float kalmanFilter(float measurement) {
    P = P + Q;
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;
    return X;
}

/***************** FUNCTION DECLARATIONS ***************/
void setup();
void updateEncoder();
void handleSetpointLock();
void handleSetpointChange();
void readTemp();
void setRelayWithPID();
void updateDisplay();
void loop();

/**************** PUBLIC FUNCTIONS ********************/
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
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }

    // Initialize NVS
    if (!preferences.begin("my-app", false)) {
        Serial.println("Failed to initialize NVS");
        for (;;);
    }

    // Retrieve the setpoint and lock state from NVS
    setpoint = preferences.getInt("setpoint", 22);  // Default setpoint is 22 if not set
    setpointLocked = preferences.getBool("setpointLocked", false);  // Default lock state is false if not set

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
    int MSB = digitalRead(ENCODER_PIN_A);
    int LSB = digitalRead(ENCODER_PIN_B);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011 || 
        sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        changeCount++;
        if (changeCount >= 4) {
            if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
                encoderPosition--;
            }
            if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
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

        // Save the setpoint lock state to NVS
        preferences.putBool("setpointLocked", setpointLocked);

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

        // Save the setpoint to NVS
        preferences.putInt("setpoint", setpoint);

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
    feedforwardTerm = HEAT_LOSS_RATE * (Setpoint - currentTemp);

    // Add feedforward term to PID output
    double totalOutput = Output + feedforwardTerm;
    totalOutput = constrain(totalOutput, 0, 255); // Ensure output is within valid range

    analogWrite(RELAY_PIN, totalOutput);

    // Convert PWM percentage to integer to display only full percent values
    int currentPwmPercentage = static_cast<int>((totalOutput / 255.0) * 100);

    // Update display if the percentage changes by more than 1% or if it reaches 0% or 100%
    if (abs(currentPwmPercentage - static_cast<int>(previousPwmPercentage)) > 1 || currentPwmPercentage == 0 || currentPwmPercentage == 100) {
        displayNeedsUpdate = true;
        previousPwmPercentage = currentPwmPercentage;
    }

    // Update display if the relay state changes
    if (prevRelayIsOn != (totalOutput > 0)) {
        displayNeedsUpdate = true;
        prevRelayIsOn = (totalOutput > 0);
    }
}

void updateDisplay() {
    if (!displayNeedsUpdate) {
        return;  // If no update is needed, just return
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);  // Set text color to white
    display.setCursor(0, 0);
    display.print("Set: ");
    display.print(setpoint);
    display.print("C ");
    if (setpointLocked) {
        display.print("*");  // Add an asterisk if setpoint is locked
    }

    display.setCursor(0, 20);
    display.print("Temp:");
    display.print(currentTemp); 

    display.setCursor(0, 40);
    display.print("PWM: ");
    display.print(static_cast<int>(previousPwmPercentage));  // Display only full percent values
    display.print("%");

    display.display();

    displayNeedsUpdate = false;  // Reset update flag
}

void loop() {
    handleSetpointLock();
    handleSetpointChange();
    readTemp();
    setRelayWithPID();
    updateDisplay();
}
