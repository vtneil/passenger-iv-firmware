#include <Arduino.h>

// Blink
#define LED_PIN PC13

void setup() {
    delay(1000);
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    Serial.println("Hello test!");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
}
