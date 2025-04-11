#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <limits.h> // Include for ULONG_MAX
#include <math.h>   // Include for sin function
#include <map>      // Include for std::map

// **********************************
// IO Pin Assignment
// **********************************

    // Motor Control Pins
    constexpr int MOTOR_FORWARD = 26;
    constexpr int MOTOR_REVERSE = 25;

    // Encoder Pins
    constexpr int DRIVE_RPM_PIN = 34;
    constexpr int UNDRIVEN_RPM_PIN = 39;

    // Track Code Pins
    constexpr int TRACK_CODE_BINARY_PIN_0 = 36;
    constexpr int TRACK_CODE_BINARY_PIN_1 = 4;
    constexpr int TRACK_CODE_BINARY_PIN_2 = 5;
    constexpr int TRACK_CODE_BINARY_PIN_3 = 19;
    constexpr int TRACK_CODE_BINARY_PIN_4 = 21;
    constexpr int TRACK_CODE_BINARY_PIN_5 = 7;

    // Track Code IR LED Pin
    constexpr int TRACK_CODE_IR_LED_PIN = 8;

    // Pin 37 - NC

    // Front LEDs
    constexpr int FWD_LED = 12;
    constexpr int REVERSE_LED = 13;

    // Magnetic Switch Pins
    constexpr int TRACK_CODE_MAGNET_PIN_R = 27;
    constexpr int TRACK_CODE_MAGNET_PIN_L = 33;

    // Button Pins
    constexpr int BUTTON_A = 15;
    constexpr int BUTTON_B = 32;
    constexpr int BUTTON_C = 14;

    // SCL 20
    // SDA 22

// **********************************
// Drive Control Parameters
// **********************************

    // Drive Information
    constexpr double DRIVEN_WHEEL_DIAMETER = 30.5;
    constexpr double UNDRIVEN_WHEEL_DIAMETER = 21.6;
    constexpr double GEAR_RATIO = 0.0889;
    constexpr double DRIVEN_WHEEL_CONVERSION = (PI * DRIVEN_WHEEL_DIAMETER) / 60.0 * GEAR_RATIO;
    constexpr double UNDRIVEN_WHEEL_CONVERSION = (PI * UNDRIVEN_WHEEL_DIAMETER) / 60.0;

    // Drive control initilization 
    constexpr int PWM_FREQUENCY = 200;
    constexpr int PWM_RESOLUTION = 8;
    double motorSpeed = 0;
    constexpr int speedIncrement = 25;
    float targetSpeed = 0.0;
    constexpr float Kp = 0.1;
    constexpr float Ki = 0.02;
    float integral = 0.0;

    // Driven wheel encoder
    constexpr int pulseTimesSize1 = 3;
    volatile unsigned long pulseTimes[pulseTimesSize1] = { 0 };
    volatile int pulseIndex = 0;
    unsigned long lastPulseTime = 0;
    constexpr unsigned int slots = 4;

    // Undriven wheel encoder
    constexpr int pulseTimesSize2 = 20;
    volatile unsigned long pulseTimes2[pulseTimesSize2] = { 0 };
    volatile int pulseIndex2 = 0;
    unsigned long lastPulseTime2 = 0;
    constexpr unsigned int slots2 = 10;

// **********************************
// System Control Parameters
// **********************************

    // UI parameters 
    constexpr int DisplayUpdateInterval = 20;
    constexpr int DriveUpdateInterval = 1;

    // Display Info
    Adafruit_SH1107 display(64, 128, &Wire);

    // Button states
    int lastButtonStateA = HIGH;
    int lastButtonStateB = HIGH;
    int lastButtonStateC = HIGH;
    int buttonStateA = HIGH;
    int buttonStateB = HIGH;
    int buttonStateC = HIGH;


    // Time tracking
    unsigned long lastTime = 0; // Initialize lastTime

    // Rotating symbol states
    char rotatingSymbols[] = { '-', '\\', '|', '/' };
    int symbolIndex = 0;

    bool magneticSensorTriggered = false;

    // Breathing effect variables
    unsigned long lastBreathTime = 0;
    constexpr unsigned long breathInterval = 2000; // 2 seconds for a full breath cycle


// **********************************
// Setup
// **********************************

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");

	// Initialize display
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
    attachInterrupt(digitalPinToInterrupt(DRIVE_RPM_PIN), countPulse, CHANGE);

    pinMode(UNDRIVEN_RPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(UNDRIVEN_RPM_PIN), countPulse2, CHANGE);

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

    lastTime = millis();
    Serial.println("Setup Finished");
}

// **********************************
// Main Loop
// **********************************

void loop() {
    readButtonStates();

    if (millis() - lastTime >= DisplayUpdateInterval) {
        lastTime = millis();
        double actualSpeed = calculateSpeed();
        double actualSpeed2 = calculateSpeed2();
        motorSpeed = calculatePWM(actualSpeed);

        String binaryString;
        int binaryNumber;
        readBinarySensors(binaryString, binaryNumber);

        updateDisplay(actualSpeed, actualSpeed2, targetSpeed, static_cast<int>(motorSpeed), binaryString, binaryNumber);

    }

    if (targetSpeed == 0) {
        ledcWrite(MOTOR_FORWARD, 0);
        ledcWrite(MOTOR_REVERSE, 0);
        motorSpeed = 0;
        integral = 0;
        digitalWrite(FWD_LED, LOW);
        handleBreathingEffect();
    }
    else if (targetSpeed > 0) {
        ledcWrite(MOTOR_FORWARD, motorSpeed);
        ledcWrite(MOTOR_REVERSE, 0);
        digitalWrite(FWD_LED, HIGH);
        ledcWrite(REVERSE_LED, 0);
    }
    else {
        ledcWrite(MOTOR_FORWARD, 0);
        ledcWrite(MOTOR_REVERSE, abs(motorSpeed));
        digitalWrite(FWD_LED, LOW);
        ledcWrite(REVERSE_LED, 255);
    }

    // Check if no new pulse is recorded
    if (millis() - lastPulseTime > 50) {
        recordPulse(ULONG_MAX);
    }
    if (millis() - lastPulseTime2 > 100) {
        recordPulse2(ULONG_MAX);
    }

    // Check magnetic switch based on direction of motion
    if (targetSpeed > 0) { // Moving forward
        if (digitalRead(TRACK_CODE_MAGNET_PIN_R) == LOW) {
            digitalWrite(TRACK_CODE_IR_LED_PIN, LOW);
            if (!magneticSensorTriggered) {
                magneticSensorTriggered = true;

                String binaryString;
                int binaryNumber;
                readBinarySensors(binaryString, binaryNumber);

                handleTrackCode(binaryNumber); // Handle the detected track code

                double actualSpeed = calculateSpeed();
                double actualSpeed2 = calculateSpeed2();
                motorSpeed = calculatePWM(actualSpeed);
                updateDisplay(actualSpeed, actualSpeed2, targetSpeed, static_cast<int>(motorSpeed), binaryString, binaryNumber);
            }
        }
        else {
            magneticSensorTriggered = false;
            digitalWrite(TRACK_CODE_IR_LED_PIN, HIGH);
        }
    }
    else { // Moving backward
        if (digitalRead(TRACK_CODE_MAGNET_PIN_L) == LOW) {
            digitalWrite(TRACK_CODE_IR_LED_PIN, LOW);
            if (!magneticSensorTriggered) {
                magneticSensorTriggered = true;

                String binaryString;
                int binaryNumber;
                readBinarySensors(binaryString, binaryNumber);

                handleTrackCode(binaryNumber); // Handle the detected track code

                double actualSpeed = calculateSpeed();
                double actualSpeed2 = calculateSpeed2();
                motorSpeed = calculatePWM(actualSpeed);
                updateDisplay(actualSpeed, actualSpeed2, targetSpeed, static_cast<int>(motorSpeed), binaryString, binaryNumber);
            }
        }
        else {
            magneticSensorTriggered = false;
            digitalWrite(TRACK_CODE_IR_LED_PIN, HIGH);
        }
    }
}


void updateDisplay(float measuredSpeed, float measuredSpeed2, float targetSpeed, int pwmValue, const String& binaryString, int binaryNumber) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("T: "));
    display.print(targetSpeed);
    display.println(F(" mm/s"));

    display.print(F("M: "));
    display.print(measuredSpeed);
    display.println(F(" mm/s"));

    // Show second sensor speed
    display.print(F("U: "));
    display.print(measuredSpeed2);
    display.println(F(" mm/s"));

    display.print(F("PWM: "));
    display.print(pwmValue);
    display.println(F(" /255"));

    display.print(F("Sensors: "));
    display.print(binaryString);
    display.print(F(" "));
    display.println(binaryNumber);

    // Show rotating symbol
    display.setCursor(display.width() - 6, display.height() - 8);
    display.print(rotatingSymbols[symbolIndex]);
    symbolIndex = (symbolIndex + 1) % 4;

    display.display();
}

void readButtonStates() {
    buttonStateA = digitalRead(BUTTON_A);
    buttonStateB = digitalRead(BUTTON_B);
    buttonStateC = digitalRead(BUTTON_C);

    static unsigned long lastButtonBPressTime = 0;
    static bool buttonBPressedOnce = false;

    if (buttonStateA == LOW && lastButtonStateA == HIGH) {
        targetSpeed += speedIncrement;
    }
    if (buttonStateB == LOW && lastButtonStateB == HIGH) {
        unsigned long currentTime = millis();
        if (buttonBPressedOnce && (currentTime - lastButtonBPressTime) < 500) {
            // Toggle direction if Button B is pressed twice within 500ms
            targetSpeed = -targetSpeed; // Adjust targetSpeed based on direction
            buttonBPressedOnce = false;
        }
        else {
            // Stop the motor if Button B is pressed once
            targetSpeed = 0;
            buttonBPressedOnce = true;
            lastButtonBPressTime = currentTime;
        }
    }
    if (buttonStateC == LOW && lastButtonStateC == HIGH) {
        targetSpeed -= speedIncrement;
    }

    lastButtonStateA = buttonStateA;
    lastButtonStateB = buttonStateB;
    lastButtonStateC = buttonStateC;
}

void readBinarySensors(String& binaryString, int& binaryNumber) {
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


void handleBreathingEffect() {
    unsigned long currentTime = millis();
    float phase = (currentTime % breathInterval) / (float)breathInterval;
    int brightness = (sin(phase * 2 * PI) + 1) * 255;
    brightness = map(brightness, 0, 255, 0, 32);
    ledcWrite(REVERSE_LED, brightness);
}

void handleTrackCode(int code) {
    switch (code) {
    case 1:
        targetSpeed = 50;
        break;
    case 2:
        targetSpeed = -50;
        break;
    case 0:
        targetSpeed = (targetSpeed > 0) ? -100 : 100;
        break;
    default:
        Serial.print("Unknown track code detected: ");
        Serial.println(code);
        return;
    }
    Serial.print("Track code detected: ");
    Serial.print(code);
    Serial.print(", setting target speed to: ");
    Serial.println(targetSpeed);
}

// Interrupt for main encoder
void countPulse() {
    unsigned long currentTime = millis();
    unsigned long pulseDuration = currentTime - lastPulseTime;
    lastPulseTime = currentTime;
    recordPulse(pulseDuration);
}

// Interrupt for un-driven wheel encoder
void countPulse2() {
    unsigned long currentTime = millis();
    unsigned long pulseDuration = currentTime - lastPulseTime2;
    lastPulseTime2 = currentTime;
    recordPulse2(pulseDuration);
}

void recordPulse(unsigned long pulseDuration) {
    pulseTimes[pulseIndex] = pulseDuration;
    pulseIndex = (pulseIndex + 1) % pulseTimesSize1;
}

// Record pulse for second sensor
void recordPulse2(unsigned long pulseDuration) {
    pulseTimes2[pulseIndex2] = pulseDuration;
    pulseIndex2 = (pulseIndex2 + 1) % pulseTimesSize2;
}

// Calculate speed for main encoder in mm/s
double calculateSpeed() {
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
    return (targetSpeed > 0) ? abs(linearSpeed) : (targetSpeed < 0 ? -abs(linearSpeed) : 0);
}

// Calculate speed for second sensor in mm/s
double calculateSpeed2() {
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
    double linearSpeed = rpm * UNDRIVEN_WHEEL_CONVERSION;
    return linearSpeed;
}

// Calculate PWM based on main encoder speed
int calculatePWM(float measuredSpeed) {
    float error = targetSpeed - measuredSpeed;
    integral += error;
    int pwmValue;
    if (targetSpeed > 0) {
        pwmValue = constrain(Kp * error + Ki * integral, 0, 255);
    }
    else {
        pwmValue = constrain(Kp * error + Ki * integral, -255, 0);
    }
    return pwmValue;
}
