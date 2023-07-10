#include <Arduino.h>

// Blink
#define LED_PIN PA13

//HardwareSerial Serial1(USART1);

void setup() {
    delay(1000);
    Serial.begin(115200);
//    Serial1.begin(115200);
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    Serial.println("Hello test!");
//    Serial1.println("Hello test!");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
}
