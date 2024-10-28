#define PHOTO_PIN 34
#define LED_PIN 39

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("Starting...");

    // Initialize photointerrupter pin
    pinMode(PHOTO_PIN, INPUT);

    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Additional debug statement to ensure setup completes
    Serial.println("Setup complete");
}

void loop() {
    // Set LED_PIN based on the state of PHOTO_PIN
    if (digitalRead(PHOTO_PIN) == HIGH) {
        digitalWrite(LED_PIN, HIGH);
    }
    else {
        digitalWrite(LED_PIN, LOW);
    }
}
