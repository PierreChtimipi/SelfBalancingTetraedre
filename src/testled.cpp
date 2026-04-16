#include <Arduino.h>

const int LED_PINS[] = {2, 4, 5};
const size_t LED_PIN_COUNT = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
const int BLINK_MS = 400;

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("Test LED ESP32 (pins 2, 4, 5)");
}

void loop() {
    for (size_t i = 0; i < LED_PIN_COUNT; i++) {
        int pin = LED_PINS[i];
        pinMode(pin, OUTPUT);

        // Test actif HIGH (HIGH allume, LOW eteint)
        Serial.printf("GPIO %d - test actif HIGH\n", pin);
        for (int n = 0; n < 4; n++) {
            digitalWrite(pin, HIGH);
            delay(BLINK_MS);
            digitalWrite(pin, LOW);
            delay(BLINK_MS);
        }
        delay(400);

        // Test actif LOW (LOW allume, HIGH eteint)
        Serial.printf("GPIO %d - test actif LOW\n", pin);
        for (int n = 0; n < 4; n++) {
            digitalWrite(pin, LOW);
            delay(BLINK_MS);
            digitalWrite(pin, HIGH);
            delay(BLINK_MS);
        }

        // Etat neutre
        digitalWrite(pin, LOW);
        delay(800);
    }

    Serial.println("Cycle termine, reprise...");
}