#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <limits.h> // Include for ULONG_MAX
#include <math.h>   // Include for sin function

// Define the I2C address and reset pin
Adafruit_SH1107 display(64, 128, &Wire);

// Define button pins
constexpr int BUTTON_A = 15;
constexpr int BUTTON_B = 32;
constexpr int BUTTON_C = 14;

// Define motor control pins
constexpr int MOTOR_FORWARD = 26;
constexpr int MOTOR_REVERSE = 25;

// Define photointerrupter pin
constexpr int PHOTO_PIN = 34;

// Define magnetic switch pin and output switch pin
constexpr int MAGNETIC_SWITCH_PIN_R = 37;
constexpr int BINARY_LED_ENABLE_PIN = 8;

// Define binary sensor pins
constexpr int BINARY_PIN_0 = 36;
constexpr int BINARY_PIN_1 = 4;
constexpr int BINARY_PIN_2 = 5;
constexpr int BINARY_PIN_3 = 19;
constexpr int BINARY_PIN_4 = 21;
constexpr int BINARY_PIN_5 = 7;

// Define status LED pins
constexpr int RUN_LED = 13;
constexpr int FWD_LED = 27;
constexpr int REVERSE_LED = 33;

// Define PWM frequency and resolution
constexpr int PWM_FREQUENCY = 125;
constexpr int PWM_RESOLUTION = 8;

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
constexpr double updateInterval = 50;

// Speed increment
constexpr int speedIncrement = 30;

// Encoder settings
constexpr int pulseTimesSize = 12;
volatile unsigned long pulseTimes[pulseTimesSize] = { 0 };
volatile int pulseIndex = 0;
unsigned long lastPulseTime = 0;
constexpr unsigned int slots = 16; // Number of fins in the encoder wheel

// PI control settings
float targetRPM = 0.0; // Target RPM
constexpr float Kp = 0.01; // Proportional gain
constexpr float Ki = 0.01; // Integral gain
float integral = 0.0; // Integral term

// Debounce delay
volatile unsigned long debounceDelay = 25; // Initial debounce delay

// Time tracking
unsigned long lastTime = 0; // Initialize lastTime

// Rotating symbol states
char rotatingSymbols[] = { '-', '\\', '|', '/' };
int symbolIndex = 0;

// Adjustable delay for magnetic sensor
constexpr unsigned long magneticSensorDelay = 25;
unsigned long magneticSensorLastTime = 0;
bool magneticSensorTriggered = false;

// Breathing effect variables
unsigned long lastBreathTime = 0;
constexpr unsigned long breathInterval = 2000; // 2 seconds for a full breath cycle

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

    // Initialize magnetic switch pin and output switch pin
    pinMode(MAGNETIC_SWITCH_PIN_R, INPUT);
    pinMode(BINARY_LED_ENABLE_PIN, OUTPUT);
    digitalWrite(BINARY_LED_ENABLE_PIN, HIGH); // Ensure the LED is off initially

    // Initialize binary sensor pins
    pinMode(BINARY_PIN_0, INPUT);
    pinMode(BINARY_PIN_1, INPUT);
    pinMode(BINARY_PIN_2, INPUT);
    pinMode(BINARY_PIN_3, INPUT);
    pinMode(BINARY_PIN_4, INPUT);
    pinMode(BINARY_PIN_5, INPUT);

    // Initialize status LED pins
    pinMode(RUN_LED, OUTPUT);
    pinMode(FWD_LED, OUTPUT);
    pinMode(REVERSE_LED, OUTPUT);
    digitalWrite(RUN_LED, HIGH); // Turn on the RUN_LED initially

    // Attach REVERSE_LED to PWM
    ledcAttach(REVERSE_LED, 4000, 8);

    // Initialize lastTime
    lastTime = millis();

    Serial.println("Setup Finished");
}

void updateDisplay(float measuredSpeed, float targetSpeed, int pwmValue, const String& binaryString, int binaryNumber) {
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

    // Display binary sensor values as a string of 0s and 1s
    display.print(F("Sensors: "));
    display.print(binaryString);
    display.print(F(" "));
    display.println(binaryNumber);

    // Display rotating symbol in the bottom right corner
    display.setCursor(display.width() - 6, display.height() - 8);
    display.print(rotatingSymbols[symbolIndex]);
    symbolIndex = (symbolIndex + 1) % 4;

    display.display();
}

void readButtonStates() {
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
}

void readBinarySensors(String& binaryString, int& binaryNumber) {
    binaryNumber = 0;
    binaryNumber |= digitalRead(BINARY_PIN_0) << 0;
    binaryNumber |= digitalRead(BINARY_PIN_1) << 1;
    binaryNumber |= digitalRead(BINARY_PIN_2) << 2;
    binaryNumber |= digitalRead(BINARY_PIN_3) << 3;
    binaryNumber |= digitalRead(BINARY_PIN_4) << 4;
    binaryNumber |= digitalRead(BINARY_PIN_5) << 5;
    binaryString = String(digitalRead(BINARY_PIN_0)) +
        String(digitalRead(BINARY_PIN_1)) +
        String(digitalRead(BINARY_PIN_2)) +
        String(digitalRead(BINARY_PIN_3)) +
        String(digitalRead(BINARY_PIN_4)) +
        String(digitalRead(BINARY_PIN_5));
}

void handleBreathingEffect() {
    unsigned long currentTime = millis();
    float phase = (currentTime % breathInterval) / (float)breathInterval;
    int brightness = (sin(phase * 2 * PI) + 1) * 255; // Sine wave from 0 to 255
    brightness = map(brightness, 0, 255, 0, 32); // Map to 0-25% brightness
    ledcWrite(REVERSE_LED, brightness);
}

void loop() {
    readButtonStates();

    // Update speed display every updateInterval
    if (millis() - lastTime >= updateInterval) {
        lastTime = millis();
        double actualSpeed = calculateSpeed();
        float error = targetRPM - actualSpeed;
        motorSpeed = calculatePWM(actualSpeed);

        // Read binary sensor values
        String binaryString;
        int binaryNumber;
        readBinarySensors(binaryString, binaryNumber);

        updateDisplay(actualSpeed, targetRPM, motorSpeed, binaryString, binaryNumber);
    }

    // Set motor speed and control direction LEDs
    if (targetRPM == 0) {
        // Stop motor
        ledcWrite(MOTOR_FORWARD, 0);
        ledcWrite(MOTOR_REVERSE, 0);
        motorSpeed = 0;
        integral = 0;
        digitalWrite(FWD_LED, LOW);

        // Breathing effect for REVERSE_LED
        handleBreathingEffect();
    }
    else if (targetRPM > 0) {
        // Spin motor forward
        ledcWrite(MOTOR_FORWARD, motorSpeed);
        ledcWrite(MOTOR_REVERSE, 0);
        digitalWrite(FWD_LED, HIGH);
        ledcWrite(REVERSE_LED, 0);
    }
    else {
        // Spin motor reverse
        ledcWrite(MOTOR_FORWARD, 0);
        ledcWrite(MOTOR_REVERSE, abs(motorSpeed));
        digitalWrite(FWD_LED, LOW);
        ledcWrite(REVERSE_LED, 255);
    }

    // Check if no new pulse is recorded after 250ms
    if (millis() - lastPulseTime > 250) {
        recordPulse(ULONG_MAX); // Use ULONG_MAX to indicate no new pulse
    }

    // Check magnetic switch state and control LED enable pin
    if (digitalRead(MAGNETIC_SWITCH_PIN_R) == LOW) {
        digitalWrite(BINARY_LED_ENABLE_PIN, LOW); // Turn on the LED

        if (!magneticSensorTriggered) {
            magneticSensorTriggered = true;
            magneticSensorLastTime = millis();
        }
        if (millis() - magneticSensorLastTime >= magneticSensorDelay) {
            

            // Read binary sensor values
            String binaryString;
            int binaryNumber;
            readBinarySensors(binaryString, binaryNumber);

            // Update display with new sensor values
            double actualSpeed = calculateSpeed();
            float error = targetRPM - actualSpeed;
            motorSpeed = calculatePWM(actualSpeed);
            updateDisplay(actualSpeed, targetRPM, motorSpeed, binaryString, binaryNumber);
        }
    }
    else {
        magneticSensorTriggered = false;
        digitalWrite(BINARY_LED_ENABLE_PIN, HIGH); // Turn off the LED
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
    pulseIndex = (pulseIndex + 1) % pulseTimesSize;
}

double calculateSpeed() {
    // Calculate the average pulse duration
    unsigned long totalDuration = 0;
    int validPulses = 0;
    for (int i = 0; i < pulseTimesSize; i++) {
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
    debounceDelay = constrain(averageDuration / 4, 10, 50); // Example: set debounce delay to 1/2 of the average duration

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
