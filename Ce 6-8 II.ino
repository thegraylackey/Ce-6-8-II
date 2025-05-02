#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <limits.h>
#include <math.h>
#include <map>
#include "driver/pcnt.h"

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
constexpr int slots = 2;
constexpr double UNDRIVEN_WHEEL_DIAMETER = 21.6;
constexpr int slots2 = 5;
constexpr double GEAR_RATIO = 0.0889;
constexpr double DRIVEN_WHEEL_CONVERSION = (PI * DRIVEN_WHEEL_DIAMETER) / 60.0 * GEAR_RATIO;
constexpr double UNDRIVEN_WHEEL_CONVERSION = (PI * UNDRIVEN_WHEEL_DIAMETER) / 60.0;

constexpr int PWM_FREQUENCY = 200;
constexpr int PWM_RESOLUTION = 10; // 10bit 0-1023
constexpr int jogSpeedIncrement = 100; // mm/s
constexpr int PIDDeadband = 150;

// Command/Setpoint variables
int cs_cmdSpeed = 0; // Commanded Speed (desired motor speed mm/s)
int ca_cmdAccel = 999; // Commanded Acceleration (desired motor acceleration mm/s^2)
float ss_setSpeed = 0.0;  // Set Speed (desired motor speed mm/s)
float sa_setAccel = 0.0; // Set Acceleration (desired motor acceleration mm/s^2)
float ws_wheelSpeed = 0.0; // Motor Speed (measured motor speed mm/s)
float ms_motorSpeed = 0.0;  // Wheel Speed (measured wheel speed mm/s)

// PID parameters
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.1;
float integral = 0.0;
float derivative = 0.0;

// PWM result
int pwmValue = 0;

// PCNT configuration
constexpr int PCNT_H_LIM = 32767;
constexpr int PCNT_L_LIM = -32768;

//Wheel Slip
int slipThreshold = 50;
int slipCounter = 0;
int speedDeadband = 25;
bool slipDetected = false;

void setupPCNT(pcnt_unit_t unit, int pulsePin) {
	pcnt_config_t pcntConfig;

	pcntConfig.pulse_gpio_num = pulsePin;
	pcntConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED;
	pcntConfig.channel = PCNT_CHANNEL_0;
	pcntConfig.unit = unit;
	pcntConfig.pos_mode = PCNT_COUNT_INC;  // Count rising edges
	pcntConfig.neg_mode = PCNT_COUNT_INC;  // Count falling edges (count both edges)
	pcntConfig.lctrl_mode = PCNT_MODE_KEEP;
	pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
	pcntConfig.counter_h_lim = PCNT_H_LIM;
	pcntConfig.counter_l_lim = PCNT_L_LIM;

	pcnt_unit_config(&pcntConfig);

	// Set a filter to debounce input signal
	pcnt_set_filter_value(unit, 100);  // Filter pulses shorter than 100 clock cycles
	pcnt_filter_enable(unit);

	pcnt_counter_pause(unit);
	pcnt_counter_clear(unit);
	pcnt_counter_resume(unit);
}

// **********************************
// System Control Parameters
// **********************************

// Time tracking
constexpr int DisplayUpdateInterval = 200;
unsigned long lastDisplayUpdateTime = 0;

constexpr int ControlUpdateInterval = 20;
unsigned long lastControlUpdateTime = 0;

constexpr int TrackCodeReadInterval = 1;
unsigned long lastTrackCodeReadTime = 0;

unsigned long lastSetPointUpdate = 0;

// Button states
int lastButtonStateA = HIGH;
int lastButtonStateB = HIGH;
int lastButtonStateC = HIGH;
int buttonStateA = HIGH;
int buttonStateB = HIGH;
int buttonStateC = HIGH;

// Display
Adafruit_SH1107 display(64, 128, &Wire);
char rotatingSymbols[] = { '-', '\\', '|', '/' };
int symbolIndex = 0;
bool magneticSensorTriggered = false;
unsigned long lastBreathTime = 0;
constexpr unsigned long breathInterval = 2000;

// Track code
bool trackCodeSwitchEnabled = true;
bool magnetLatchR = false;
bool magnetLatchL = false;
constexpr int MAX_TRACK_CODES = 256;
int trackCodeArray[MAX_TRACK_CODES];
int trackCodeIndex = 0;

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

	// Initialize PCNT for main and un-driven encoders
	setupPCNT(PCNT_UNIT_0, DRIVE_RPM_PIN);
	setupPCNT(PCNT_UNIT_1, UNDRIVEN_RPM_PIN);

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

	// Update track code logic
	if (now - lastTrackCodeReadTime >= TrackCodeReadInterval) {
		lastTrackCodeReadTime = now;
		updateTrackCode();
	}

	// Update control logic
	if (now - lastControlUpdateTime >= ControlUpdateInterval) {
		lastControlUpdateTime = now;
		updateControlLogic(now);
	}

	// Update display logic directly
	if (now - lastDisplayUpdateTime >= DisplayUpdateInterval) {
		lastDisplayUpdateTime = now;
		updateDisplay(ms_motorSpeed, ws_wheelSpeed, ss_setSpeed, pwmValue, binaryString, binaryNumber);
	}

}


// **********************************
// Subroutines
// **********************************

void updateTrackCode() {
	// Process right magnetic switch
	if (digitalRead(TRACK_CODE_MAGNET_PIN_R) == LOW && cs_cmdSpeed >= 0) {
		processMagneticSwitch(magnetLatchR, TRACK_CODE_MAGNET_PIN_R);
	}
	else if (magnetLatchR) {
		processMagneticSwitchEnd(magnetLatchR);
	}

	// Process left magnetic switch
	if (digitalRead(TRACK_CODE_MAGNET_PIN_L) == LOW && cs_cmdSpeed <= 0) {
		processMagneticSwitch(magnetLatchL, TRACK_CODE_MAGNET_PIN_L);
	}
	else if (magnetLatchL) {
		processMagneticSwitchEnd(magnetLatchL);
	}
}

void processMagneticSwitch(bool& magnetLatch, int magnetPin) {
	digitalWrite(TRACK_CODE_IR_LED_PIN, LOW);

	if (!magnetLatch) {
		magnetLatch = true;
		trackCodeIndex = 0;
	}

	// Read and store binary numbers while the magnet is low
	if (trackCodeIndex < MAX_TRACK_CODES) {
		readBinarySensors();
		trackCodeArray[trackCodeIndex++] = binaryNumber;
		Serial.print(binaryNumber);
	}
}

void processMagneticSwitchEnd(bool& magnetLatch) {
	digitalWrite(TRACK_CODE_IR_LED_PIN, HIGH);
	magnetLatch = false;

	Serial.println("\n--- Filtering Process Start ---");

	// Step 1: Log the original array
	Serial.print("Original Array: ");
	for (int i = 0; i < trackCodeIndex; i++) {
		Serial.print(trackCodeArray[i]);
		Serial.print(" ");
	}
	Serial.println();

	// Step 2: Check if the array is all zeros
	bool allZeros = true;
	for (int i = 0; i < trackCodeIndex; i++) {
		if (trackCodeArray[i] != 0) {
			allZeros = false;
			break;
		}
	}

	Serial.print("Array is all zeros: ");
	Serial.println(allZeros ? "Yes" : "No");

	// Step 3: Find the largest continuous section with centrality weighting
	int middleIndex = trackCodeIndex / 2;
	int bestStartIndex = -1;
	int bestEndIndex = -1;
	int bestLength = 0;
	int bestValue = 0;
	float bestScore = 0.0;

	// Only search for non-zero sequences if the array isn't all zeros
	if (!allZeros) {
		// Simple single-pass search from left to right
		for (int startPos = 0; startPos < trackCodeIndex; startPos++) {
			if (trackCodeArray[startPos] == 0) continue; // Skip zeros

			int currentValue = trackCodeArray[startPos];
			int endPos = startPos;
			int nonZeroCount = 0;

			// Extend the sequence as far as possible UNTIL we hit a zero or different value
			while (endPos < trackCodeIndex && trackCodeArray[endPos] == currentValue) {
				nonZeroCount++;
				endPos++;
			}

			// Calculate score with centrality weighting only
			int seqMiddle = (startPos + endPos - 1) / 2;
			float distanceFromMiddle = abs(middleIndex - seqMiddle);
			float centralityWeight = 1.0 - (distanceFromMiddle / (float)trackCodeIndex);

			// Calculate final score (no value weighting)
			float score = nonZeroCount * (centralityWeight * 2);

			Serial.print("Candidate: value=");
			Serial.print(currentValue);
			Serial.print(", count=");
			Serial.print(nonZeroCount);
			Serial.print(", centrality=");
			Serial.print(centralityWeight, 2);
			Serial.print(", score=");
			Serial.println(score, 2);

			if (score > bestScore) {
				bestStartIndex = startPos;
				bestEndIndex = endPos - 1;
				bestLength = nonZeroCount;
				bestValue = currentValue;
				bestScore = score;
			}

			// Jump to the end of this sequence for next iteration
			startPos = endPos - 1;
		}
	}

	// If the array is all zeros or no significant sequence found
	if (allZeros || bestLength == 0) {
		Serial.println("Using mode of entire array");

		// Calculate the mode of the entire array
		std::map<int, int> frequencyMap;
		for (int i = 0; i < trackCodeIndex; i++) {
			frequencyMap[trackCodeArray[i]]++;
		}

		int mode = 0;
		int maxFrequency = 0;
		for (const auto& entry : frequencyMap) {
			if (entry.second > maxFrequency) {
				maxFrequency = entry.second;
				mode = entry.first;
			}
		}

		Serial.print("Mode: ");
		Serial.print(mode);
		Serial.print(" (Frequency: ");
		Serial.print(maxFrequency);
		Serial.println(")");

		handleTrackCode(mode);
	}
	else {
		// Found a significant continuous sequence
		Serial.print("Found best sequence of ");
		Serial.print(bestValue);
		Serial.print(" from index ");
		Serial.print(bestStartIndex);
		Serial.print(" to ");
		Serial.print(bestEndIndex);
		Serial.print(" (Length: ");
		Serial.print(bestLength);
		Serial.print(", ");
		Serial.print((float)bestLength / trackCodeIndex * 100.0, 1);
		Serial.print("% of array, Score: ");
		Serial.print(bestScore, 2);
		Serial.println(")");

		// Use the value of the sequence as the track code
		handleTrackCode(bestValue);
	}

	Serial.println("--- Filtering Process End ---\n");
}



void updateControlLogic(unsigned long now) {
	// Update sp from cs using acceleration
	float dt = (now - lastSetPointUpdate) / 1000.0f;
	lastSetPointUpdate = now;

	updateSetPoint(dt);

	// Read encoder values
	ms_motorSpeed = calculateMotorSpeed();
	ws_wheelSpeed = calculateWheelSpeed();

	// Detect wheel slip
	float speedDifference = fabs(ws_wheelSpeed - ms_motorSpeed);

	// Avoid division by zero
	float percentageDifference = (ms_motorSpeed != 0) ? (speedDifference / fabs(ms_motorSpeed)) * 100.0 : 0.0;

	if (fabs(ms_motorSpeed) < speedDeadband && fabs(ws_wheelSpeed) < speedDeadband) {
		// Speeds are too small to detect slip
		slipDetected = false;
		slipCounter = 0;
	}
	else if (percentageDifference >= slipThreshold) {
		slipDetected = true;
		slipCounter++;
	}
	else {
		slipDetected = false;
		slipCounter = 0;
	}

	// PID control
	pwmValue = calculatePWM(ms_motorSpeed);

	// Update PWM output
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
}

double calculateMotorSpeed() {
	static double filteredSpeed = 0.0;
	static const float filterCoeff = 0.1; // Lower value = more filtering (0.0-1.0)

	int16_t count = 0;
	pcnt_get_counter_value(PCNT_UNIT_0, &count);
	pcnt_counter_clear(PCNT_UNIT_0);

	// Calculate raw speed - convert count to RPM
	double countPerSecond = count * (1000.0 / ControlUpdateInterval);
	double revolutionsPerSecond = countPerSecond / (double)(slots * 2); // Multiply by 2 for both edges
	double rpm = revolutionsPerSecond * 60.0;
	double linearSpeed = rpm * DRIVEN_WHEEL_CONVERSION;

	// Apply sign based on direction
	double signedSpeed = (ss_setSpeed > 0) ? fabs(linearSpeed) : (ss_setSpeed < 0 ? -fabs(linearSpeed) : 0);

	// Apply low-pass filter to smooth readings
	filteredSpeed = filterCoeff * signedSpeed + (1.0 - filterCoeff) * filteredSpeed;

	return filteredSpeed;
}

double calculateWheelSpeed() {
	static double filteredSpeed = 0.0;
	static const float filterCoeff = 0.05;

	int16_t count = 0;
	pcnt_get_counter_value(PCNT_UNIT_1, &count);
	pcnt_counter_clear(PCNT_UNIT_1);

	double countPerSecond = count * (1000.0 / ControlUpdateInterval);
	double revolutionsPerSecond = countPerSecond / (double)(slots2 * 2);
	double rpm = revolutionsPerSecond * 60.0;
	double linearSpeed = rpm * UNDRIVEN_WHEEL_CONVERSION;

	double signedSpeed = (ss_setSpeed > 0) ? fabs(linearSpeed) : (ss_setSpeed < 0 ? -fabs(linearSpeed) : 0);
	filteredSpeed = filterCoeff * signedSpeed + (1.0 - filterCoeff) * filteredSpeed;

	return filteredSpeed;
}

void updateSetPoint(float dt) {
	sa_setAccel = (float)ca_cmdAccel;
	float delta = sa_setAccel * dt;

	if (slipDetected) {
		// Reduce the setpoint speed progressively based on the slip counter
		float reductionFactor = 1.0 - (0.01 * slipCounter);
		ss_setSpeed *= reductionFactor;
	}
	else {
		// Normal behavior: Gradually adjust setpoint toward commanded speed
		if (ss_setSpeed < cs_cmdSpeed) {
			ss_setSpeed += delta;
			if (ss_setSpeed > cs_cmdSpeed) ss_setSpeed = cs_cmdSpeed;
		}
		else if (ss_setSpeed > cs_cmdSpeed) {
			ss_setSpeed -= delta;
			if (ss_setSpeed < cs_cmdSpeed) ss_setSpeed = cs_cmdSpeed;
		}
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

	if (slipDetected) {
		display.print(F("SLIP"));
	}


	// Display binary string in 2x3 grid at the top right
	int startX = display.width() - 18;
	int startY = 0;

	display.setCursor(startX - 6, startY);
	display.print(trackCodeSwitchEnabled ? F("ENAB") : F("DISA"));

	display.setCursor(startX, startY + 8);
	display.print(binaryString[0]);
	display.setCursor(startX + 6, startY + 8);
	display.print(binaryString[1]);

	display.setCursor(startX - 6, startY + 16);
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
	}
	else {
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

	Serial.println(String("-") + code);

	// Check if track code switch is enabled and run away
	if (!trackCodeSwitchEnabled) {
		return;
	}

	switch (code) {
	case 0:
		cs_cmdSpeed = (cs_cmdSpeed > 0) ? -200 : 200; // 100 mm/s, reverse
		break;
	case 1:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 100 : -100;
		break;
	case 2:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 200 : -200;
		break;
	case 3:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 300 : -300;
		break;
	case 4:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 400 : -400;
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

void readBinarySensors() {
	binaryNumber = 0;
	binaryNumber |= (~digitalRead(TRACK_CODE_BINARY_PIN_0) & 0x01) << 0;
	binaryNumber |= (~digitalRead(TRACK_CODE_BINARY_PIN_1) & 0x01) << 1;
	binaryNumber |= (~digitalRead(TRACK_CODE_BINARY_PIN_2) & 0x01) << 2;
	binaryNumber |= (~digitalRead(TRACK_CODE_BINARY_PIN_3) & 0x01) << 3;
	binaryNumber |= (~digitalRead(TRACK_CODE_BINARY_PIN_4) & 0x01) << 4;
	binaryNumber |= (~digitalRead(TRACK_CODE_BINARY_PIN_5) & 0x01) << 5;
	binaryString = String(~digitalRead(TRACK_CODE_BINARY_PIN_0) & 0x01) +
		String(~digitalRead(TRACK_CODE_BINARY_PIN_1) & 0x01) +
		String(~digitalRead(TRACK_CODE_BINARY_PIN_2) & 0x01) +
		String(~digitalRead(TRACK_CODE_BINARY_PIN_3) & 0x01) +
		String(~digitalRead(TRACK_CODE_BINARY_PIN_4) & 0x01) +
		String(~digitalRead(TRACK_CODE_BINARY_PIN_5) & 0x01);
}

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



