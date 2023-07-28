#include <Arduino.h>
#include "Servo.h"

#define PIN_CTRL  2
#define PIN_SERVO 3

Servo servo;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_CTRL, INPUT);
    pinMode(PIN_SERVO, OUTPUT);
    servo.attach(PIN_SERVO);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    servo.writeMicroseconds(0);
    delay(5000);
    digitalWrite(LED_BUILTIN, LOW);
    servo.writeMicroseconds(1500);
    delay(5000);
//    if (digitalRead(PIN_CTRL)) {
//        digitalWrite(LED_BUILTIN, HIGH);
//        servo.writeMicroseconds(0);
//    } else {
//        digitalWrite(LED_BUILTIN, LOW);
//        servo.writeMicroseconds(1500);
//    }
}
