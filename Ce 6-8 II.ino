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

constexpr int PWM_FREQUENCY = 17500;
constexpr int PWM_RESOLUTION = 10; // 10bit 0-1023
constexpr int jogSpeedIncrement = 100; // mm/s
constexpr int PIDDeadband = 590;

// Command/Setpoint variables
int cs_cmdSpeed = 0; // Commanded Speed (desired motor speed mm/s)
int ca_cmdAccel = 100; // Commanded Acceleration (desired motor acceleration mm/s^2)
float ss_setSpeed = 0.0;  // Set Speed (desired motor speed mm/s)
float sa_setAccel = 0.0; // Set Acceleration (desired motor acceleration mm/s^2)
float ws_wheelSpeed = 0.0; // Motor Speed (measured motor speed mm/s)
float ms_motorSpeed = 0.0;  // Wheel Speed (measured wheel speed mm/s)
float as_avgSpeed = 0.0; // Average Speed (average of motor and wheel speed mm/s)
int percentThrottle = 0; // Percent of max power (0-100%)

// PID parameters
float Kp = 0.3;
float Ki = 0.2;
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

// Status message buffer
constexpr int STATUS_MSG_COUNT = 3;
String statusMessages[STATUS_MSG_COUNT] = {"", "", ""};

// Forward declarations
void readButtonStates();
void handleTrackCode(int code);
void readBinarySensors();
double calculateMotorSpeed();
double calculateWheelSpeed();
int calculatePWM(float measuredSpeed);
void updateSetPoint(float dt);
void addStatusMessage(const String& msg);
void drawDirectionArrow(int x, int y, int w, int h);

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
	if (now - lastDisplayUpdateTime >= DisplayUpdateInterval 
		&& digitalRead(TRACK_CODE_MAGNET_PIN_R) == HIGH) {
		lastDisplayUpdateTime = now;
		updateDisplay();
	}

}


void addStatusMessage(const String& msg) {
    // Shift messages down
    for (int i = STATUS_MSG_COUNT - 1; i > 0; i--) {
        statusMessages[i] = statusMessages[i - 1];
    }
    statusMessages[0] = msg;
}

void drawDirectionArrow(int x, int y, int w, int h) {
    int thickness = 3; // Thickness for arrow lines
    if (ss_setSpeed > 0) {
        // Up arrow, thick lines
        for (int t = -thickness/2; t <= thickness/2; t++) {
            display.drawLine(x + w/2 + t, y + h, x + w/2 + t, y, SH110X_WHITE);
        }
        display.fillTriangle(
            x + w/2, y,
            x, y + h/2,
            x + w, y + h/2,
            SH110X_WHITE
        );
    } else if (ss_setSpeed < 0) {
        // Down arrow, thick lines
        for (int t = -thickness/2; t <= thickness/2; t++) {
            display.drawLine(x + w/2 + t, y, x + w/2 + t, y + h, SH110X_WHITE);
        }
        display.fillTriangle(
            x + w/2, y + h,
            x, y + h/2,
            x + w, y + h/2,
            SH110X_WHITE
        );
    } else {
        // Filled octagon for stopped
        int ox[8], oy[8];
        int r = w / 2 - 1;
        int cx = x + w / 2;
        int cy = y + h / 2;
        for (int i = 0; i < 8; i++) {
            float angle = PI / 4 * i + PI / 8;
            ox[i] = cx + (int)(r * cos(angle));
            oy[i] = cy + (int)(r * sin(angle));
        }
        // Fill octagon using triangles from center to each edge
        for (int i = 0; i < 8; i++) {
            int next = (i + 1) % 8;
            display.fillTriangle(cx, cy, ox[i], oy[i], ox[next], oy[next], SH110X_WHITE);
        }
        // Draw a black rectangle at the center
        int rectW = w / 2.5;
        int rectH = h / 4.5;
        int rectX = cx - rectW / 2;
        int rectY = cy - rectH / 2;
        display.fillRect(rectX, rectY, rectW, rectH, SH110X_BLACK);
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
	}
}

void processMagneticSwitchEnd(bool& magnetLatch) {
	digitalWrite(TRACK_CODE_IR_LED_PIN, HIGH);
	magnetLatch = false;

	// Step 1: Log the original array
	Serial.println();
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
			float score = nonZeroCount * (centralityWeight * 4);

			//Serial.print("Candidate: value=");
			//Serial.print(currentValue);
			//Serial.print(", count=");
			//Serial.print(nonZeroCount);
			//Serial.print(", centrality=");
			//Serial.print(centralityWeight, 2);
			//Serial.print(", score=");
			//Serial.println(score, 2);

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
		Serial.println(bestValue);
		Serial.println();

		// Use the value of the sequence as the track code
		handleTrackCode(bestValue);
		updateDisplay();
	}

}

void updateControlLogic(unsigned long now) {
	// Update sp from cs using acceleration
	float dt = (now - lastSetPointUpdate) / 1000.0f;
	lastSetPointUpdate = now;

	updateSetPoint(dt);

	// Read encoder values
	ms_motorSpeed = calculateMotorSpeed();
	ws_wheelSpeed = calculateWheelSpeed();
	as_avgSpeed = (ms_motorSpeed + ws_wheelSpeed) / 2.0;

	// Detect wheel slip
	float speedDifference = fabs(ws_wheelSpeed - ms_motorSpeed);

	// Avoid division by zero
	float percentageDifference = (ms_motorSpeed != 0) ? (speedDifference / fabs(ms_motorSpeed)) * 100.0 : 0.0;

	if (fabs(ms_motorSpeed) < speedDeadband && fabs(ws_wheelSpeed) < speedDeadband) {
		// Speeds are too small to detect slip
		slipDetected = false;
		slipCounter = 0;
	}
	else if (percentageDifference >= slipThreshold  && trackCodeSwitchEnabled) {
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

void updateDisplay() {
    display.clearDisplay();
    // Top left: direction arrow/box (3 lines tall)
    int arrowX = 0;
    int arrowY = 0;
    int arrowW = 18;
    int arrowH = 24;
    drawDirectionArrow(arrowX, arrowY, arrowW, arrowH);

    // Set speed (right of arrow, one line, rounded, no units)
    display.setCursor(arrowX + arrowW + 2, arrowY);
    display.setTextSize(1);
    display.print((int)round(ss_setSpeed));
    display.print(" mm/s");

    // Average speed (two lines high, rounded, no units)
    display.setTextSize(2);
    display.setCursor(arrowX + arrowW + 2, arrowY + 10);
    display.print((int)round(as_avgSpeed));
    display.setTextSize(1);

    // Top right: throttle percent, two lines high, no label
    display.setTextSize(2);
    int throttleChars = String(percentThrottle).length();
    int charWidth = 12; // Approximate width per character in text size 2
    int throttleX = display.width() - (throttleChars * charWidth) - 12; // 2px padding from right
    display.setCursor(throttleX, 0);
    display.print(percentThrottle);
    if (percentThrottle < 100) {
        display.print("%");
    }
    display.setTextSize(1);

    // Bottom: spinning symbol and slip indicator above three lines of status messages
    int statusY = display.height() - 8 * STATUS_MSG_COUNT - 8; // 8px for symbol line
    display.setCursor(0, statusY);
    display.print(rotatingSymbols[symbolIndex]);
    // Print "Slip!" to the right if slipDetected
    if (slipDetected) {
        display.setCursor(20, statusY); // 20px to the right of symbol
        display.print("Slip!");
    }
    symbolIndex = (symbolIndex + 1) % 4;

    // Now print the three status messages below the symbol
    for (int i = 0; i < STATUS_MSG_COUNT; i++) {
        display.setCursor(0, statusY + 8 + i * 8);
        display.print(statusMessages[i]);
    }
    display.display();
}

void handleBreathingEffect() {
	unsigned long currentTime = millis();
	float phase = (currentTime % breathInterval) / (float)breathInterval;
	int brightness = (sin(phase * 2 * PI) + 1) * 255;
	brightness = map(brightness, 0, 255, 0, 127);
	ledcWrite(REVERSE_LED, brightness);
}

void handleTrackCode(int code) {

	// Check if track code switch is enabled and run away
	if (!trackCodeSwitchEnabled) {
		return;
	}

	switch (code) {
	case 0:
		cs_cmdSpeed = (cs_cmdSpeed > 0) ? -100 : 100; // 100 mm/s, reverse
		addStatusMessage("Code " + String(code) + " CS: " + String(cs_cmdSpeed));
		break;
	case 1:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 100 : -100;
		addStatusMessage("Code " + String(code) + " CS: " + String(cs_cmdSpeed));
		break;
	case 2:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 200 : -200;
		addStatusMessage("Code " + String(code) + " CS: " + String(cs_cmdSpeed));
		break;
	case 3:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 300 : -300;
		addStatusMessage("Code " + String(code) + " CS: " + String(cs_cmdSpeed));
		break;
	case 4:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 400 : -400;
		addStatusMessage("Code " + String(code) + " CS: " + String(cs_cmdSpeed));
		break;
	case 5:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 500 : -500;
		addStatusMessage("Code " + String(code) + " CS: " + String(cs_cmdSpeed));
		break;
	case 6:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 600 : -600;
		addStatusMessage("Code " + String(code) + " CS: " + String(cs_cmdSpeed));
		break;
	case 7:
		cs_cmdSpeed = (cs_cmdSpeed >= 0) ? 700 : -700;
		addStatusMessage("Code " + String(code) + " CS: " + String(cs_cmdSpeed));
		break;
	default:
		cs_cmdSpeed = 0;
		addStatusMessage("Code Unknown CS: " + String(cs_cmdSpeed));
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
		addStatusMessage("CS " + String(cs_cmdSpeed));
    }

    if (buttonStateB == LOW && lastButtonStateB == HIGH) {
        unsigned long currentTime = millis();
        if (buttonBPressedOnce && (currentTime - lastButtonBPressTime) < 500) {
            trackCodeSwitchEnabled = !trackCodeSwitchEnabled;
            buttonBPressedOnce = false;
            addStatusMessage(trackCodeSwitchEnabled ? "Track code ON" : "Track code OFF");
        }
        else {
            buttonBPressedOnce = true;
            lastButtonBPressTime = currentTime;
            cs_cmdSpeed = 0;
            integral = 0.0;
            addStatusMessage("Stop");
        }
    }

    if (buttonStateC == LOW && lastButtonStateC == HIGH) {
        cs_cmdSpeed -= jogSpeedIncrement;
        addStatusMessage("CS " + String(cs_cmdSpeed));
    }

    lastButtonStateA = buttonStateA;
    lastButtonStateB = buttonStateB;
    lastButtonStateC = buttonStateC;
}

void readBinarySensors() {
	// Read raw binary values from pins
	int bit0 = (~digitalRead(TRACK_CODE_BINARY_PIN_0) & 0x01);
	int bit1 = (~digitalRead(TRACK_CODE_BINARY_PIN_1) & 0x01);
	int bit2 = (~digitalRead(TRACK_CODE_BINARY_PIN_2) & 0x01);
	int bit3 = (~digitalRead(TRACK_CODE_BINARY_PIN_3) & 0x01);
	int bit4 = (~digitalRead(TRACK_CODE_BINARY_PIN_4) & 0x01);
	int bit5 = (~digitalRead(TRACK_CODE_BINARY_PIN_5) & 0x01);

	// If moving backward (negative speed), rotate 180 degrees by reversing bit order
	if (cs_cmdSpeed < 0) {
		// Swap bit positions to rotate 180 degrees
		int temp0 = bit0;
		int temp1 = bit1;
		int temp2 = bit2;

		bit0 = bit5;
		bit1 = bit4;
		bit2 = bit3;
		bit3 = temp2;
		bit4 = temp1;
		bit5 = temp0;
	}

	// Assemble the binary number
	binaryNumber = 0;
	binaryNumber |= bit0 << 0;
	binaryNumber |= bit1 << 1;
	binaryNumber |= bit2 << 2;
	binaryNumber |= bit3 << 3;
	binaryNumber |= bit4 << 4;
	binaryNumber |= bit5 << 5;

	// Create the binary string representation
	binaryString = String(bit0) + String(bit1) + String(bit2) +
		String(bit3) + String(bit4) + String(bit5);

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

	percentThrottle = map(abs(pwmValue), PIDDeadband, 1023, 0, 100);
	if (percentThrottle < 0)
	{
		percentThrottle = 0;
	}

	return pwmValue;
}
