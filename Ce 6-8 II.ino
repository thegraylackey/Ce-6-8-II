#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <limits.h> // Include for ULONG_MAX
#include <math.h>   // Include for sin function
#include <map>      // Include for std::map

// **********************************
// IO Pin Assignment
// **********************************

constexpr int MOTOR_FORWARD = 26;
constexpr int MOTOR_REVERSE = 25;
constexpr int DRIVE_RPM_PIN = 34;
constexpr int UNDRIVEN_RPM_PIN = 39;
constexpr int TRACK_CODE_BINARY_PIN_0 = 36;
constexpr int TRACK_CODE_BINARY_PIN_1 = 4;
constexpr int TRACK_CODE_BINARY_PIN_2 = 5;
constexpr int TRACK_CODE_BINARY_PIN_3 = 19;
constexpr int TRACK_CODE_BINARY_PIN_4 = 21;
constexpr int TRACK_CODE_BINARY_PIN_5 = 7;
constexpr int TRACK_CODE_IR_LED_PIN = 8;
constexpr int FWD_LED = 12;
constexpr int REVERSE_LED = 13;
constexpr int TRACK_CODE_MAGNET_PIN_R = 27;
constexpr int TRACK_CODE_MAGNET_PIN_L = 33;
constexpr int BUTTON_A = 15;
constexpr int BUTTON_B = 32;
constexpr int BUTTON_C = 14;

// **********************************
// Drive Control Parameters
// **********************************

constexpr double DRIVEN_WHEEL_DIAMETER = 30.5;
constexpr double UNDRIVEN_WHEEL_DIAMETER = 21.6;
constexpr double GEAR_RATIO = 0.0889;
constexpr double DRIVEN_WHEEL_CONVERSION = (PI * DRIVEN_WHEEL_DIAMETER) / 60.0 * GEAR_RATIO;
constexpr double UNDRIVEN_WHEEL_CONVERSION = (PI * UNDRIVEN_WHEEL_DIAMETER) / 60.0;

constexpr int PWM_FREQUENCY = 200;
constexpr int PWM_RESOLUTION = 10; // 10bit 0-1023
constexpr int jogSpeedIncrement = 100; // mm/s
constexpr int PIDDeadband = 125;

// Command/Setpoint variables
int cs_cmdSpeed = 0; // Commanded Speed (desired motor speed mm/s)
int ca_cmdAccel = 100; // Commanded Acceleration (desired motor acceleration mm/s^2)
float ss_setSpeed = 0.0;  // Set Speed (desired motor speed mm/s)
float sa_setAccel = 0.0; // Set Acceleration (desired motor acceleration mm/s^2)
float ws_wheelSpeed = 0.0; // Motor Speed (measured motor speed mm/s)
float ms_motorSpeed = 0.0;  // Wheel Speed (measured wheel speed mm/s)

// PID parameters
constexpr float Kp = 0.5;
constexpr float Ki = 0.05;
constexpr float Kd = 0.2;
float integral = 0.0;
float derivative = 0.0;

// PWM result
int pwmValue = 0;

// Storage for encoder intervals
constexpr int pulseTimesSize1 = 2;
volatile unsigned long pulseTimes[pulseTimesSize1] = { 0 };
volatile int pulseIndex = 0;
unsigned long lastPulseTime = 0;
constexpr unsigned int slots = 4;

constexpr int pulseTimesSize2 = 3;
volatile unsigned long pulseTimes2[pulseTimesSize2] = { 0 };
volatile int pulseIndex2 = 0;
unsigned long lastPulseTime2 = 0;
constexpr unsigned int slots2 = 10;

// **********************************
// System Control Parameters
// **********************************

constexpr int DisplayUpdateInterval = 100;
constexpr int ControlUpdateInterval = 5;

// Button states
int lastButtonStateA = HIGH;
int lastButtonStateB = HIGH;
int lastButtonStateC = HIGH;
int buttonStateA = HIGH;
int buttonStateB = HIGH;
int buttonStateC = HIGH;

// Time tracking
unsigned long lastDisplayUpdateTime = 0;
unsigned long lastControlUpdateTime = 0;
unsigned long lastSetPointUpdate = 0;

// Display
Adafruit_SH1107 display(64, 128, &Wire);
char rotatingSymbols[] = { '-', '\\', '|', '/' };
int symbolIndex = 0;
bool magneticSensorTriggered = false;
unsigned long lastBreathTime = 0;
constexpr unsigned long breathInterval = 2000;
bool trackCodeSwitchEnabled = true;

String binaryString;
int binaryNumber;

// Forward declarations
void readButtonStates();
void handleTrackCode(int code);
void readBinarySensors();
double calculateMotorSpeed();
double calculateWheelSpeed();
int calculatePWM(float measuredSpeed);
void updateSetPoint(float dt);
void IRAM_ATTR recordPulse(unsigned long pulseDuration);
void IRAM_ATTR recordPulse2(unsigned long pulseDuration);
void IRAM_ATTR count_pulse_motor_ISR();
void IRAM_ATTR count_pulse_undriven_ISR();

// **********************************
// Setup
// **********************************

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");

    display.begin(0x3C, true);
    display.setRotation(1);
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println(F("Ready"));
    display.display();

    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);

    ledcAttach(MOTOR_FORWARD, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_REVERSE, PWM_FREQUENCY, PWM_RESOLUTION);

    pinMode(DRIVE_RPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(DRIVE_RPM_PIN), count_pulse_motor_ISR, CHANGE);

    pinMode(UNDRIVEN_RPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(UNDRIVEN_RPM_PIN), count_pulse_undriven_ISR, CHANGE);

    pinMode(TRACK_CODE_MAGNET_PIN_R, INPUT);
    pinMode(TRACK_CODE_MAGNET_PIN_L, INPUT);

    pinMode(TRACK_CODE_IR_LED_PIN, OUTPUT);
    digitalWrite(TRACK_CODE_IR_LED_PIN, HIGH);

    pinMode(TRACK_CODE_BINARY_PIN_0, INPUT);
    pinMode(TRACK_CODE_BINARY_PIN_1, INPUT);
    pinMode(TRACK_CODE_BINARY_PIN_2, INPUT);
    pinMode(TRACK_CODE_BINARY_PIN_3, INPUT);
    pinMode(TRACK_CODE_BINARY_PIN_4, INPUT);
    pinMode(TRACK_CODE_BINARY_PIN_5, INPUT);

    pinMode(FWD_LED, OUTPUT);
    pinMode(REVERSE_LED, OUTPUT);

    ledcAttach(REVERSE_LED, 4000, 8);

    lastDisplayUpdateTime = millis();
    lastControlUpdateTime = millis();
    lastSetPointUpdate = millis();
    Serial.println("Setup Finished");
}

// **********************************
// Main Loop
// **********************************

void loop() {
    readButtonStates();

    unsigned long now = millis();

    // Check magnetic switch flags
    if (digitalRead(TRACK_CODE_MAGNET_PIN_R) == LOW && cs_cmdSpeed >= 0) {
        digitalWrite(TRACK_CODE_IR_LED_PIN, LOW);
        readBinarySensors();
        handleTrackCode(binaryNumber);
        digitalWrite(TRACK_CODE_IR_LED_PIN, HIGH);
    }
    if (digitalRead(TRACK_CODE_MAGNET_PIN_L) == LOW && cs_cmdSpeed <= 0) {
        digitalWrite(TRACK_CODE_IR_LED_PIN, LOW);
        readBinarySensors();
        handleTrackCode(binaryNumber);
        digitalWrite(TRACK_CODE_IR_LED_PIN, HIGH);
    }

    // Update control logic
    if (now - lastControlUpdateTime >= ControlUpdateInterval) {
        lastControlUpdateTime = now;

        // Update sp from cs using acceleration
        float dt = (now - lastSetPointUpdate) / 1000.0f;
        lastSetPointUpdate = now;

        updateSetPoint(dt);

        ms_motorSpeed = calculateMotorSpeed();
        ws_wheelSpeed = calculateWheelSpeed();
        pwmValue = calculatePWM(ms_motorSpeed);

        if (ss_setSpeed == 0) {
            ledcWrite(MOTOR_FORWARD, 0);
            ledcWrite(MOTOR_REVERSE, 0);
            digitalWrite(FWD_LED, LOW);
            handleBreathingEffect();
        }
        else if (ss_setSpeed > 0) {
            ledcWrite(MOTOR_FORWARD, pwmValue);
            ledcWrite(MOTOR_REVERSE, 0);
            digitalWrite(FWD_LED, HIGH);
            ledcWrite(REVERSE_LED, 0);
        }
        else { // sp < 0
            ledcWrite(MOTOR_FORWARD, 0);
            ledcWrite(MOTOR_REVERSE, abs(pwmValue));
            digitalWrite(FWD_LED, LOW);
            ledcWrite(REVERSE_LED, 255);
        }

        // Check for encoder timeouts
        if (now - lastPulseTime > 50) {
            recordPulse(ULONG_MAX);
        }
        if (now - lastPulseTime2 > 100) {
            recordPulse2(ULONG_MAX);
        }

    }

    // Update display
    if (now - lastDisplayUpdateTime >= DisplayUpdateInterval) {
        lastDisplayUpdateTime = now;

        updateDisplay(ms_motorSpeed, ws_wheelSpeed, ss_setSpeed, pwmValue, binaryString, binaryNumber);
    }
}

// **********************************
// Subroutines
// **********************************

// Gradually move set point sp to commanded speed cs
void updateSetPoint(float dt) {
    sa_setAccel = (float)ca_cmdAccel;
    float delta = sa_setAccel * dt;

    if (ss_setSpeed < cs_cmdSpeed) {
        ss_setSpeed += delta;
        if (ss_setSpeed > cs_cmdSpeed) ss_setSpeed = cs_cmdSpeed;
    }
    else if (ss_setSpeed > cs_cmdSpeed) {
        ss_setSpeed -= delta;
        if (ss_setSpeed < cs_cmdSpeed) ss_setSpeed = cs_cmdSpeed;
    }
}

void updateDisplay(float measuredSpeed, float measuredSpeed2, float targetSpeed, int pwmVal, const String& binaryString, int binaryNumber) {
    display.clearDisplay();
    display.setCursor(0, 0);

    // Display Commanded Speed and Acceleration
    display.print(F("CS: "));
    display.print(round(cs_cmdSpeed), 0);
    display.print(F(" CA: "));
    display.println(round(ca_cmdAccel), 0);

    // Display Set Speed and Acceleration
    display.print(F("SS: "));
    display.print(round(ss_setSpeed), 0);
    display.print(F(" SA: "));
    display.println(round(sa_setAccel), 0);

    // Display Motor Speed
    display.print(F("MS: "));
    display.println(round(ms_motorSpeed), 0);

    // Display Wheel Speed
    display.print(F("WS: "));
    display.println(round(ws_wheelSpeed), 0);

    // Display binary string in 2x3 grid at the top right
    int startX = display.width() - 18;
    int startY = 0;

    display.setCursor(startX - 6, startY);
    display.print(trackCodeSwitchEnabled ? F("ENAB") : F("DISA"));

    display.setCursor(startX, startY + 8);
    display.print(binaryString[0]);
    display.setCursor(startX + 6, startY + 8);
    display.print(binaryString[1]);

    display.setCursor(startX-6, startY + 16);
    if (digitalRead(TRACK_CODE_MAGNET_PIN_L) == LOW && cs_cmdSpeed <= 0) {
        display.print(F("L"));
    }
    else {
        display.print(F("0"));
    }

    display.setCursor(startX, startY + 16);
    display.print(binaryString[2]);
    display.setCursor(startX + 6, startY + 16);
    display.print(binaryString[3]);

    if (digitalRead(TRACK_CODE_MAGNET_PIN_R) == LOW && cs_cmdSpeed >= 0) {
        display.print(F("R"));
    } else {
        display.print(F("0"));
    }

    display.setCursor(startX, startY + 24);
    display.print(binaryString[4]);
    display.setCursor(startX + 6, startY + 24);
    display.print(binaryString[5]);

    display.setCursor(startX, startY + 32);
    display.print(binaryNumber < 10 ? "0" + String(binaryNumber) : String(binaryNumber));

    // Display rotating symbol
    display.setCursor(0, display.height() - 8);
    display.print(rotatingSymbols[symbolIndex]);
    display.print(F(" i"));
    display.print(integral, 0);
    display.print(F(" d"));
    display.print(derivative, 1);

    display.setCursor(0, display.height() - 16);
    display.print(F("PWM:"));
    display.print(pwmVal);

    symbolIndex = (symbolIndex + 1) % 4;

    display.display();
}

void handleBreathingEffect() {
    unsigned long currentTime = millis();
    float phase = (currentTime % breathInterval) / (float)breathInterval;
    int brightness = (sin(phase * 2 * PI) + 1) * 255;
    brightness = map(brightness, 0, 255, 0, 32);
    ledcWrite(REVERSE_LED, brightness);
}


void handleTrackCode(int code) {
    
    if (!trackCodeSwitchEnabled) {
        Serial.println(code);
        return;
    }    

    switch (code) {
    case 0:
        cs_cmdSpeed = (cs_cmdSpeed > 0) ? -200 : 200; // 100 mm/s, reverse
        break;
    case 1:
        //cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 50 : -50; // 50 mm/s
        break;
    case 2:
        //cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 200 : -200; // 200 mm/s
        break;
    default:
        //cs_cmdSpeed = 0; // Stop
        break;
    }
}

void readButtonStates() {
    buttonStateA = digitalRead(BUTTON_A);
    buttonStateB = digitalRead(BUTTON_B);
    buttonStateC = digitalRead(BUTTON_C);

    static unsigned long lastButtonBPressTime = 0;
    static bool buttonBPressedOnce = false;

    if (buttonStateA == LOW && lastButtonStateA == HIGH) {
        cs_cmdSpeed += jogSpeedIncrement;
    }

    if (buttonStateB == LOW && lastButtonStateB == HIGH) {
        unsigned long currentTime = millis();
        if (buttonBPressedOnce && (currentTime - lastButtonBPressTime) < 500) {
            trackCodeSwitchEnabled = !trackCodeSwitchEnabled;
            buttonBPressedOnce = false;
        }
        else {
            buttonBPressedOnce = true;
            lastButtonBPressTime = currentTime;
			cs_cmdSpeed = 0;
			integral = 0.0;
        }
    }

    if (buttonStateC == LOW && lastButtonStateC == HIGH) {
        cs_cmdSpeed -= jogSpeedIncrement;
    }

    lastButtonStateA = buttonStateA;
    lastButtonStateB = buttonStateB;
    lastButtonStateC = buttonStateC;
}

// Read binary sensors and convert to string
void readBinarySensors() {
    binaryNumber = 0;
    binaryNumber |= (!digitalRead(TRACK_CODE_BINARY_PIN_0)) << 0;
    binaryNumber |= (!digitalRead(TRACK_CODE_BINARY_PIN_1)) << 1;
    binaryNumber |= (!digitalRead(TRACK_CODE_BINARY_PIN_2)) << 2;
    binaryNumber |= (!digitalRead(TRACK_CODE_BINARY_PIN_3)) << 3;
    binaryNumber |= (!digitalRead(TRACK_CODE_BINARY_PIN_4)) << 4;
    binaryNumber |= (!digitalRead(TRACK_CODE_BINARY_PIN_5)) << 5;
    binaryString = String(!digitalRead(TRACK_CODE_BINARY_PIN_0)) +
        String(!digitalRead(TRACK_CODE_BINARY_PIN_1)) +
        String(!digitalRead(TRACK_CODE_BINARY_PIN_2)) +
        String(!digitalRead(TRACK_CODE_BINARY_PIN_3)) +
        String(!digitalRead(TRACK_CODE_BINARY_PIN_4)) +
        String(!digitalRead(TRACK_CODE_BINARY_PIN_5));
}

// Interrupt for main encoder
void IRAM_ATTR count_pulse_motor_ISR() {
    unsigned long currentTime = millis();
    unsigned long pulseDuration = currentTime - lastPulseTime;
    lastPulseTime = currentTime;
    recordPulse(pulseDuration);
}

// Interrupt for un-driven wheel encoder
void IRAM_ATTR count_pulse_undriven_ISR() {
    unsigned long currentTime = millis();
    unsigned long pulseDuration = currentTime - lastPulseTime2;
    lastPulseTime2 = currentTime;
    recordPulse2(pulseDuration);
}

// Record pulse duration for main encoder
void IRAM_ATTR recordPulse(unsigned long pulseDuration) {
    pulseTimes[pulseIndex] = pulseDuration;
    pulseIndex = (pulseIndex + 1) % pulseTimesSize1;
}

void IRAM_ATTR recordPulse2(unsigned long pulseDuration) {
    pulseTimes2[pulseIndex2] = pulseDuration;
    pulseIndex2 = (pulseIndex2 + 1) % pulseTimesSize2;
}

// Calculate speed for main encoder in mm/s
double calculateMotorSpeed() {
    unsigned long totalDuration = 0;
    int validPulses = 0;
    for (int i = 0; i < pulseTimesSize1; i++) {
        if (pulseTimes[i] != ULONG_MAX) {
            totalDuration += pulseTimes[i];
            validPulses++;
        }
    }
    if (validPulses == 0) {
        return 0;
    }
    double averageDuration = totalDuration / (double)validPulses;
    if (averageDuration == 0) {
        return 0;
    }
    double rpm = (60000.0 / averageDuration) / slots;
    double linearSpeed = rpm * DRIVEN_WHEEL_CONVERSION;
    // Use sign of sp to indicate direction
    return (ss_setSpeed > 0) ? fabs(linearSpeed) : (ss_setSpeed < 0 ? -fabs(linearSpeed) : 0);
}

// Calculate speed for second sensor in mm/s
double calculateWheelSpeed() {
    unsigned long totalDuration = 0;
    int validPulses = 0;
    for (int i = 0; i < pulseTimesSize2; i++) {
        if (pulseTimes2[i] != ULONG_MAX) {
            totalDuration += pulseTimes2[i];
            validPulses++;
        }
    }
    if (validPulses == 0) {
        return 0;
    }
    double averageDuration = totalDuration / (double)validPulses;
    if (averageDuration == 0) {
        return 0;
    }
    double rpm = (60000.0 / averageDuration) / slots2;
    return rpm * UNDRIVEN_WHEEL_CONVERSION;
}

// Calculate PWM value based on PID control
int calculatePWM(float measuredSpeed) {
    static float lastError = 0.0;
    float error = ss_setSpeed - measuredSpeed;

    // Apply decay factor to integral term
    constexpr float decayFactor = 0.99; // Adjust this value as needed
    integral *= decayFactor;

    // Update integral term
    integral += error;

    // Limit the integral term to prevent windup
    constexpr int integralLimit = 20000; // Adjust this value as needed
    if (integral > integralLimit) {
        integral = integralLimit;
    }
    else if (integral < -integralLimit) {
        integral = -integralLimit;
    }

    // Calculate derivative term
    derivative = error - lastError;
    lastError = error;

    // Calculate PID output
    float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Apply deadband mapping
    int pwmValue;
    if (pidOutput > 0) {
        pwmValue = map(pidOutput, 0, 1023, PIDDeadband, 1023);
    }
    else if (pidOutput < 0) {
        pwmValue = map(pidOutput, -1023, 0, -1023, -PIDDeadband);
    }
    else {
        pwmValue = 0;
    }

    // Constrain 10-bit output
    if (ss_setSpeed > 0) {
        pwmValue = constrain(pwmValue, PIDDeadband, 1023);
    }
    else {
        pwmValue = constrain(pwmValue, -1023, -PIDDeadband);
    }

    // Force to 0 if exactly equal to +/- PIDDeadband
    if (pwmValue == PIDDeadband || pwmValue == -PIDDeadband) {
        pwmValue = 0;
    }

    return pwmValue;
}



