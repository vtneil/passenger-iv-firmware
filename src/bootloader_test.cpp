#include <Arduino.h>
//#include "vt_bme280"

// Blink
#define LED_PIN PA13

//using namespace vt;

//HardwareSerial Serial1(USART1);
//bme280_t bme280;

void setup() {
    delay(1000);
    Serial.begin(115200);
//    Serial1.begin(115200);
    pinMode(LED_PIN, OUTPUT);
//    Wire.setSCL(PB6);
//    Wire.setSDA(PB7);
//    bme280.begin(0x77);
}

void loop() {
//    Serial1.println(bme280.read_temperature_c());
//    Serial1.println(bme280.read_pressure());
//    Serial1.println(bme280.read_humidity());

    Serial.println("Hello test!");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
}
