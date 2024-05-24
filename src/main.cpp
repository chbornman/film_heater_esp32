/***************** INCLUDES ****************/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"
#include <Encoder.h>
#include <Bounce2.h>
#include <math.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <ESP32_FTPClient.h>

/**************** MACROS ******************/
// Tunable parameters
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
#define HEAT_LOSS_RATE 0.05

// WiFi credentials
const char* ssid = "your_SSID";
const char* password = "your_WIFI_password";

// FTP credentials
const char* ftp_server = "192.168.0.7";
const char* ftp_user = "caleb";
const char* ftp_pass = "Omas0321!";
const char* ftp_path = "/homes/caleb/esp32_logs/";

/*************** GLOBALS *****************/
// Display and sensor objects
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);
Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);
Bounce2::Button encoderButton = Bounce2::Button();
ESP32_FTPClient ftp(ftp_server, ftp_user, ftp_pass, 21);

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
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

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
void connectToWiFi();
void logDataToFTP();
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

    Setpoint = setpoint;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);

    Serial.begin(9600);
    Serial.println("MAX6675 test");
    delay(500);
    Serial.println("Setup complete - starting main loop");

    connectToWiFi();
}

void connectToWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected!");
}

void logDataToFTP() {
    // Create a timestamp
    String timestamp = String(year()) + "-" + String(month()) + "-" + String(day()) + "_" + String(hour()) + "-" + String(minute()) + "-" + String(second());

    // Create the filename with the timestamp
    String filename = "film_heater_" + timestamp + ".csv";

    // Create the log data
    String logData = String(millis()) + "," + String(setpoint) + "," + String(currentTemp) + "\n";

    // Connect to FTP
    ftp.OpenConnection();
    
    // Change to the directory
    ftp.ChangeWorkDir(ftp_path);
    
    // Create a new file
    ftp.InitFile("Type A");
    ftp.NewFile(filename.c_str());
    ftp.WriteData(logData.c_str(), logData.length());
    ftp.CloseFile();
    
    // Close FTP connection
    ftp.CloseConnection();
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
    feedforwardTerm = HEAT_LOSS_RATE * (Setpoint - currentTemp);

    // Add feedforward term to PID output
    double totalOutput = Output + feedforwardTerm;
    totalOutput = constrain(totalOutput, 0, 255