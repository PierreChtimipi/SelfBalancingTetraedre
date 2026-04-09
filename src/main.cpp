#include <Arduino.h>

const int LED_PIN = 2; // GPIO pin for LED (adjust if needed)

void setup() {
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);   // Turn LED on
    delay(1000);                    // Wait 1 second
    digitalWrite(LED_PIN, LOW);    // Turn LED off
    delay(1000);                    // Wait 1 second
}