#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <limits.h> // Include for ULONG_MAX

// Define the I2C address and reset pin
Adafruit_SH1107 display(64, 128, &Wire);

// Define button pins
#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14

// Define motor control pins
#define MOTOR_FORWARD 26
#define MOTOR_REVERSE 25

// Define photointerrupter pin
#define PHOTO_PIN 34

// Define LED pin
#define LED_PIN 4

// Define PWM frequency and resolution
#define PWM_FREQUENCY 100
#define PWM_RESOLUTION 8

// Button states
int lastButtonStateA = HIGH;
int lastButtonStateB = HIGH;
int lastButtonStateC = HIGH;
int buttonStateA = HIGH;
int buttonStateB = HIGH;
int buttonStateC = HIGH;

// Motor speed
double motorSpeed = 0;

// Update Interval
const double updateInterval = 50;

// Speed increment
const int speedIncrement = 30;

// Encoder settings
volatile unsigned long pulseTimes[16] = { 0 };
volatile int pulseIndex = 0;
unsigned long lastPulseTime = 0;
const unsigned int slots = 16; // Number of fins in the encoder wheel

// PI control settings
float targetRPM = 0.0; // Target RPM
float Kp = 0.01; // Proportional gain
float Ki = 0.01; // Integral gain
float integral = 0.0; // Integral term

// Debounce delay
volatile unsigned long debounceDelay = 25; // Initial debounce delay

// Time tracking
unsigned long lastTime = 0; // Initialize lastTime

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("Starting...");

    // Initialize the display
    display.begin(0x3C, true);
    display.setRotation(1);

    // Clear the buffer
    display.clearDisplay();
    display.display(); // Ensure the display is updated with the cleared buffer
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println(F("Ready"));
    display.display();

    // Initialize button pins
    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);

    // Initialize motor control pins
    ledcAttach(MOTOR_FORWARD, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_REVERSE, PWM_FREQUENCY, PWM_RESOLUTION);

    // Initialize photointerrupter pin
    pinMode(PHOTO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), countPulse, CHANGE);

    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize lastTime
    lastTime = millis();

    Serial.println("Setup Finished");
}

void updateDisplay(float measuredSpeed, float targetSpeed, int pwmValue, float error, float integral) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("T: "));
    display.print(targetSpeed);
    display.println(F(" RPM"));
    display.print(F("M: "));
    display.print(measuredSpeed);
    display.println(F(" RPM"));
    display.print(F("PWM: "));
    display.print(pwmValue);
    display.println(F(" /255"));
    display.print(F("Err: "));
    display.println(error);
    display.print(F("Int: "));
    display.print(integral);
    display.display();
}

void loop() {
    buttonStateA = digitalRead(BUTTON_A);
    buttonStateB = digitalRead(BUTTON_B);
    buttonStateC = digitalRead(BUTTON_C);

    if (buttonStateA == LOW && lastButtonStateA == HIGH) {
        targetRPM += speedIncrement; // Increase target RPM
    }

    if (buttonStateB == LOW && lastButtonStateB == HIGH) {
        targetRPM = 0; // Stop motor
    }

    if (buttonStateC == LOW && lastButtonStateC == HIGH) {
        targetRPM -= speedIncrement; // Decrease target RPM
    }

    lastButtonStateA = buttonStateA;
    lastButtonStateB = buttonStateB;
    lastButtonStateC = buttonStateC;

    // Update speed display every updateInterval
    if (millis() - lastTime >= updateInterval) {
        lastTime = millis();
        double actualSpeed = calculateSpeed();
        float error = targetRPM - actualSpeed;
        motorSpeed = calculatePWM(actualSpeed);
        updateDisplay(actualSpeed, targetRPM, motorSpeed, error, integral);
    }

    // Set motor speed
    if (targetRPM == 0) {
        // Stop motor
        ledcWrite(MOTOR_FORWARD, 0);
        ledcWrite(MOTOR_REVERSE, 0);
        motorSpeed = 0;
        integral = 0;
    }
    else if (targetRPM > 0) {
        // Spin motor forward
        ledcWrite(MOTOR_FORWARD, motorSpeed);
        ledcWrite(MOTOR_REVERSE, 0);
    }
    else {
        // Spin motor reverse
        ledcWrite(MOTOR_FORWARD, 0);
        ledcWrite(MOTOR_REVERSE, abs(motorSpeed));
    }

    // Set LED_PIN based on the state of PHOTO_PIN
    if (digitalRead(PHOTO_PIN) == HIGH) {
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
    }

    // Check if no new pulse is recorded after 250ms
    if (millis() - lastPulseTime > 250) {
        recordPulse(ULONG_MAX); // Use ULONG_MAX to indicate no new pulse
    }
}

void countPulse() {
    unsigned long currentTime = millis();
    unsigned long pulseDuration = currentTime - lastPulseTime;
    if (pulseDuration > debounceDelay) {
        lastPulseTime = currentTime;
        // Record pulse duration in the array
        recordPulse(pulseDuration);
    }
}

void recordPulse(unsigned long pulseDuration) {
    pulseTimes[pulseIndex] = pulseDuration;
    pulseIndex = (pulseIndex + 1) % 16;
}

double calculateSpeed() {
    // Calculate the average pulse duration
    unsigned long totalDuration = 0;
    int validPulses = 0;
    for (int i = 0; i < 16; i++) {
        if (pulseTimes[i] != ULONG_MAX) {
            totalDuration += pulseTimes[i];
            validPulses++;
        }
    }

    if (validPulses == 0) {
        return 0; // No valid pulses, return 0 RPM
    }

    double averageDuration = totalDuration / (double)validPulses;

    // Adjust debounce delay based on average pulse duration
    debounceDelay = averageDuration / 4; // Example: set debounce delay to 1/2 of the average duration

    // Calculate RPM based on the average pulse duration
    double rpm = (60000.0 / averageDuration) / slots;

    if (targetRPM > 0) {
        // If the motor is spinning forward, the RPM should be positive
        rpm = abs(rpm);
    }
    else if (targetRPM < 0) {
        // If the motor is spinning in reverse, the RPM should be negative
        rpm = -abs(rpm);
    }

    return rpm;
}

int calculatePWM(float measuredSpeed) {
    float error = targetRPM - measuredSpeed;
    integral += error; // Accumulate the error for the integral term  
    int pwmValue;
    if (targetRPM > 0) {
        pwmValue = constrain(Kp * error + Ki * integral, 0, 255);
    }
    else {
        pwmValue = constrain(Kp * error + Ki * integral, -255, 0);
    }
    return pwmValue;
}
